import cohere
from typing import List, Dict, Any, Optional
from app.config import settings
import logging


# Define the Table of Contents
BOOK_TOC = """
1. Introduction to Physical AI
2. Foundations of Robotics
3. Sensors and Perception
4. Actuators and Control
5. Embodied Intelligence
6. Humanoid Robot Design
7. Learning for Robotics
8. Human–Robot Interaction
9. Ethics and Safety
10. Future of Physical AI
"""


def answer_with_toc(question: str) -> str:
    """
    Generate a response using the Table of Contents when no specific content is found
    """
    return f"""
Based on the textbook structure:

{BOOK_TOC}

Your question: "{question}"

Here is a clear explanation aligned with the book content.
"""


class GenerationService:
    def __init__(self):
        self.cohere_client = cohere.Client(settings.COHERE_API_KEY)

    def generate_response(self, query: str, context_chunks: List[Dict[str, Any]],
                         include_citations: bool = True,
                         user_software_background: Optional[str] = None,
                         user_hardware_background: Optional[str] = None) -> Dict[str, Any]:
        """
        Generate a response based on the query and retrieved context chunks

        Args:
            query: The user's question
            context_chunks: List of relevant content chunks with metadata
            include_citations: Whether to include citations in the response
            user_software_background: User's software background for personalization
            user_hardware_background: User's hardware background for personalization

        Returns:
            Dictionary with response text, citations, and confidence score
        """
        try:
            # Check if we have relevant context chunks
            if not context_chunks or len(context_chunks) == 0:
                # Use TOC fallback when no content is retrieved
                response_text = answer_with_toc(query)
                return {
                    "response": response_text,
                    "citations": [],
                    "confidence": 0.3  # Lower confidence for fallback response
                }

            # Format the context for the LLM
            context_str = "\n\n".join([
                f"Source: {chunk.get('chapter', 'Unknown')} - {chunk.get('section', 'Unknown')} (Page {chunk.get('page', 'N/A')})\nContent: {chunk['content']}"
                for chunk in context_chunks
            ])

            # Create the prompt for the AI tutor
            prompt = f"""
You are an AI Tutor for the book "Physical AI & Humanoid Robotics".

Rules:
1. Prefer retrieved textbook content.
2. If no content is retrieved, answer using the Table of Contents.
3. NEVER say "Failed to fetch" or mention backend errors.
4. If the user asks for Table of Contents, always share it.

Book Table of Contents:
{BOOK_TOC}

Context:
{context_str}

Question:
{query}
Answer:"""

            # Add personalization based on user background if available
            if user_software_background or user_hardware_background:
                background_info = []
                if user_software_background:
                    background_info.append(f"User's software background: {user_software_background}")
                if user_hardware_background:
                    background_info.append(f"User's hardware background: {user_hardware_background}")

                background_str = " ".join(background_info)
                prompt += f"""
User Background: {background_str}
When answering, try to relate the concepts to the user's background where appropriate to make the explanation more relatable and easier to understand.
"""

            # Generate response using Cohere
            response = self.cohere_client.chat(
                message=query,
                preamble=prompt,
                model=settings.COHERE_GENERATION_MODEL,
                temperature=0.1,  # Low temperature for more consistent responses
                max_tokens=500
            )

            # Extract the response text
            response_text = response.text.strip()

            # Calculate confidence based on the context relevance
            # For now, using a simple heuristic based on context quality
            avg_score = sum([chunk.get("score", 0) for chunk in context_chunks]) / len(context_chunks) if context_chunks else 0
            confidence = min(avg_score, 1.0)  # Ensure confidence is between 0 and 1

            # Extract citations
            citations = []
            if include_citations:
                for chunk in context_chunks:
                    if chunk.get("source") == "selected_text":
                        citations.append({
                            "source": "selected_text",
                            "context": chunk.get("context", "")
                        })
                    else:
                        citations.append({
                            "chapter": chunk.get("chapter"),
                            "section": chunk.get("section"),
                            "page": chunk.get("page"),
                            "url": chunk.get("url")
                        })

            return {
                "response": response_text,
                "citations": citations,
                "confidence": confidence
            }
        except Exception as e:
            logging.error(f"Error during response generation: {str(e)}")
            # Return a graceful fallback response instead of an error
            return {
                "response": answer_with_toc(query),
                "citations": [],
                "confidence": 0.2
            }

    def generate_selected_text_response(self, query: str, selected_text: str) -> Dict[str, Any]:
        """
        Generate a response based only on the user-selected text
        """
        try:
            # Create a prompt that focuses only on the selected text
            prompt = f"""
You are an AI assistant for an AI textbook. Answer the user's question based ONLY on the provided selected text. Do not use any other knowledge or information outside of this selected text.
If the selected text does not contain the answer, respond with: "This information is not available in the provided text."

Selected Text:
{selected_text}

Question: {query}
Answer:"""

            # Generate response using Cohere
            response = self.cohere_client.chat(
                message=query,
                preamble=prompt,
                model=settings.COHERE_GENERATION_MODEL,
                temperature=0.1,
                max_tokens=500
            )

            # Extract the response text
            response_text = response.text.strip()

            # For selected text queries, we return a high confidence if we got a response
            confidence = 0.9 if "not available in the provided text" not in response_text.lower() else 0.0

            # Create citation for selected text
            citation = {
                "source": "selected_text",
                "context": selected_text[:200] + "..." if len(selected_text) > 200 else selected_text
            }

            return {
                "response": response_text,
                "citations": [citation],
                "confidence": confidence
            }
        except Exception as e:
            logging.error(f"Error during selected text response generation: {str(e)}")
            return {
                "response": "This information is not available in the provided text.",
                "citations": [],
                "confidence": 0.0
            }