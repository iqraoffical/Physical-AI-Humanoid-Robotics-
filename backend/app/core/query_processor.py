from typing import Dict, Any, List
from app.services.retrieval_service import RetrievalService
from app.services.generation_service import GenerationService
from app.services.session_service import SessionService
from app.utils.citation_utils import extract_citations_from_response
from sqlalchemy.orm import Session
import uuid


class QueryProcessor:
    def __init__(self):
        self.retrieval_service = RetrievalService()
        self.generation_service = GenerationService()
    
    def process_query(
        self,
        db: Session,
        query_text: str,
        session_id: str,
        include_citations: bool = True
    ) -> Dict[str, Any]:
        """
        Process a general book query end-to-end
        """
        # Retrieve relevant content
        context_chunks = self.retrieval_service.search(
            query_text=query_text,
            top_k=3,
            threshold=0.5
        )
        
        # Generate response
        result = self.generation_service.generate_response(
            query=query_text,
            context_chunks=context_chunks,
            include_citations=include_citations
        )
        
        # Add query-response to session history
        SessionService.add_query_to_session(
            db,
            session_id,
            query_text,
            result["response"],
            result["citations"]
        )
        
        # Create response object
        response = {
            "response": result["response"],
            "citations": result["citations"],
            "session_id": session_id,
            "confidence": result["confidence"],
            "query_id": str(uuid.uuid4())
        }
        
        return response
    
    def process_selected_text_query(
        self,
        db: Session,
        query_text: str,
        selected_text: str,
        session_id: str
    ) -> Dict[str, Any]:
        """
        Process a selected text query end-to-end
        """
        # Generate response based only on selected text
        result = self.generation_service.generate_selected_text_response(
            query=query_text,
            selected_text=selected_text
        )
        
        # Add query-response to session history
        SessionService.add_query_to_session(
            db,
            session_id,
            f"Query: {query_text}, Selected Text: {selected_text[:100]}...",
            result["response"],
            result["citations"]
        )
        
        # Create response object
        response = {
            "response": result["response"],
            "citations": result["citations"],
            "session_id": session_id,
            "confidence": result["confidence"],
            "query_id": str(uuid.uuid4())
        }
        
        return response