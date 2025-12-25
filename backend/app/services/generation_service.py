import cohere
from typing import List, Dict, Any
from app.config import settings
import logging


class GenerationService:
    def __init__(self):
        self.cohere_client = cohere.Client(settings.COHERE_API_KEY)

    def generate_response(self, query: str, context_chunks: List[Dict[str, Any]],
                         include_citations: bool = True) -> Dict[str, Any]:
        """
        Generate a response based on the query and retrieved context chunks

        Args:
            query: The user's question
            context_chunks: List of relevant content chunks with metadata
            include_citations: Whether to include citations in the response

        Returns:
            Dictionary with response text, citations, and confidence score
        """
        try:
            if not context_chunks:
                # No relevant content found
                return {
                    "response": "This information is not available in the book.",
                    "citations": [],
                    "confidence": 0.0
                }

            # Format the context for the LLM
            context_str = "\n\n".join([
                f"Source: {chunk.get('chapter', 'Unknown')} - {chunk.get('section', 'Unknown')} (Page {chunk.get('page', 'N/A')})\nContent: {chunk['content']}"
                for chunk in context_chunks
            ])

            # Create the prompt for zero hallucination
            prompt = f"""
You are an AI assistant for an AI textbook. Answer the user's question based ONLY on the provided context from the book. Do not use any prior knowledge or external information.
If the context does not contain the answer, respond with: "This information is not available in the book."

Context:
{context_str}

"""

            if include_citations:
                prompt += "Cite your sources with chapter/section and page number where possible.\n"

            prompt += f"""
Question: {query}
Answer:"""

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
            return {
                "response": "This information is not available in the book.",
                "citations": [],
                "confidence": 0.0
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