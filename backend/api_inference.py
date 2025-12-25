"""
API inference file for Hugging Face Spaces
This file provides an interface to use the existing backend API
"""
import os
import sys
from typing import Dict, Any, Optional
import uuid

# Add the parent directory to sys.path so we can import from app
sys.path.append(os.path.join(os.path.dirname(__file__)))

from app.config import settings
from app.services.retrieval_service import RetrievalService
from app.services.generation_service import GenerationService
from app.services.session_service import SessionService
from app.models.query import UserQuery
from app.models.api_responses import QueryResponse
from app.db import get_db
from sqlalchemy.orm import Session


class HuggingFaceAPI:
    """
    API class for Hugging Face Spaces inference
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
        This method is designed for Hugging Face inference API
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
                query_id=str(uuid.uuid4())  # Generate a proper query ID
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
    
    def __call__(self, user_input: str, session_id: Optional[str] = None, include_citations: bool = True):
        """
        Callable method for Hugging Face inference API
        """
        return self.query(user_input, session_id, include_citations)


# Initialize the API instance
api_instance = HuggingFaceAPI()


def inference(user_input: str, session_id: Optional[str] = None, include_citations: bool = True):
    """
    Main inference function for Hugging Face Spaces
    """
    return api_instance.query(user_input, session_id, include_citations)


# For Hugging Face Inference API
if __name__ == "__main__":
    # Example usage
    result = inference("What is artificial intelligence?", include_citations=True)
    print(result)