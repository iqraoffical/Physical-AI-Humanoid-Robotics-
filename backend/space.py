"""
Hugging Face Space entry point
This file serves as the entry point for Hugging Face Spaces
"""
import gradio as gr
import os
import sys
from typing import Dict, Any, Optional

# Add the parent directory to sys.path so we can import from app
sys.path.append(os.path.join(os.path.dirname(__file__)))

# Initialize database tables on startup
from init_db import init_db
init_db()

from app.config import settings
from app.services.retrieval_service import RetrievalService
from app.services.generation_service import GenerationService
from app.services.session_service import SessionService
from app.models.query import UserQuery
from app.models.api_responses import QueryResponse
from app.db import get_db
from sqlalchemy.orm import Session


class RAGChatbotInterface:
    """
    Interface class to connect Gradio UI with the existing backend services
    """
    def __init__(self):
        self.retrieval_service = RetrievalService()
        self.generation_service = GenerationService()
        self.session_service = SessionService()
        self.db: Optional[Session] = None

    def get_db(self):
        """Get database session"""
        if self.db is None:
            self.db = next(get_db())
        return self.db

    def query(self, user_input: str, session_id: Optional[str] = None, include_citations: bool = True):
        """
        Process a query using the existing backend services
        """
        try:
            # Validate input
            if len(user_input) > settings.MAX_QUERY_LENGTH:
                return {
                    "response": f"Query exceeds maximum length of {settings.MAX_QUERY_LENGTH} characters",
                    "citations": [],
                    "session_id": session_id or "",
                    "confidence": 0.0,
                    "query_id": ""
                }

            # Create UserQuery object
            user_query = UserQuery(
                query=user_input,
                session_id=session_id,
                include_citations=include_citations
            )

            # Get database session
            db = self.get_db()

            # Create or retrieve session
            if not user_query.session_id:
                session = SessionService.create_session(db)
                session_id = session.session_id
            else:
                session = SessionService.get_session(db, user_query.session_id)
                if not session:
                    # Create a new session if the provided session_id doesn't exist
                    session = SessionService.create_session(db)
                    session_id = session.session_id
                else:
                    session_id = session.session_id
                    # Update last access time
                    SessionService.update_session_last_access(db, session_id)

            # Retrieve relevant content
            context_chunks = self.retrieval_service.search(
                query_text=user_query.query,
                top_k=3,
                threshold=0.5
            )

            # Generate response
            result = self.generation_service.generate_response(
                query=user_query.query,
                context_chunks=context_chunks,
                include_citations=user_query.include_citations
            )

            # Add query-response to session history
            SessionService.add_query_to_session(
                db,
                session_id,
                user_query.query,
                result["response"],
                result["citations"]
            )

            # Create response object
            response = QueryResponse(
                response=result["response"],
                citations=result["citations"],
                session_id=session_id,
                confidence=result["confidence"],
                query_id="temp_id"  # In a real implementation, you'd have a proper query ID
            )

            return {
                "response": response.response,
                "citations": response.citations,
                "session_id": response.session_id,
                "confidence": response.confidence,
                "query_id": response.query_id
            }

        except Exception as e:
            return {
                "response": f"Error processing query: {str(e)}",
                "citations": [],
                "session_id": session_id or "",
                "confidence": 0.0,
                "query_id": ""
            }


def create_gradio_interface():
    """
    Create the Gradio interface for the RAG Chatbot
    """
    chatbot_interface = RAGChatbotInterface()

    def chat_response(message, history, include_citations=True):
        """
        Process chat message and return response
        """
        result = chatbot_interface.query(message, include_citations=include_citations)
        response = result["response"]

        if result["citations"] and include_citations:
            citations_text = "\n\nCitations:\n" + "\n".join([f"- {cit}" for cit in result["citations"]])
            response += citations_text

        return response

    # Create Gradio interface
    with gr.Blocks(title="RAG Chatbot for AI Textbook") as demo:
        gr.Markdown("# RAG Chatbot for AI Textbook")
        gr.Markdown("Ask questions about the AI textbook content")

        # Checkbox for citations
        with gr.Row():
            include_citations = gr.Checkbox(label="Include Citations", value=True)

        # Chat interface
        chatbot = gr.Chatbot(
            label="Chat History",
            bubble_full_width=False,
            height=500
        )
        msg = gr.Textbox(label="Enter your question about the textbook")
        clear = gr.Button("Clear Chat")

        # Event handling
        def respond(message, chat_history, include_citations):
            bot_message = chat_response(message, chat_history, include_citations)
            chat_history.append((message, bot_message))
            return "", chat_history

        msg.submit(respond, [msg, chatbot, include_citations], [msg, chatbot])
        clear.click(lambda: None, None, chatbot, queue=False)

    return demo


# Create the Gradio app
interface = create_gradio_interface()


if __name__ == "__main__":
    interface.launch()
else:
    # This is for Hugging Face Spaces to pick up the interface
    app = interface