from fastapi import APIRouter, Depends, HTTPException, status
from sqlalchemy.orm import Session
from typing import Optional
import uuid
import logging

from app.db import get_db
from app.models.query import UserQuery
from app.models.api_responses import QueryResponse
from app.services.session_service import SessionService
from app.services.retrieval_service import RetrievalService
from app.services.generation_service import GenerationService
from app.config import settings


router = APIRouter()


@router.post("/query", response_model=QueryResponse)
async def query_endpoint(
    user_query: UserQuery,
    db: Session = Depends(get_db)
):
    """
    Submit a general question about book content to get accurate,
    context-specific answers derived solely from the book's content.
    """
    try:
        # Validate input
        if len(user_query.query) > settings.MAX_QUERY_LENGTH:
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail=f"Query exceeds maximum length of {settings.MAX_QUERY_LENGTH} characters"
            )

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

        # Initialize services
        retrieval_service = RetrievalService()
        generation_service = GenerationService()

        # Retrieve relevant content
        context_chunks = retrieval_service.search(
            query_text=user_query.query,
            top_k=3,
            threshold=0.5
        )

        # Generate response
        result = generation_service.generate_response(
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
            query_id=str(uuid.uuid4())
        )

        return response

    except HTTPException:
        # Re-raise HTTP exceptions
        raise
    except Exception as e:
        logging.error(f"Error processing query: {str(e)}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Internal server error occurred while processing the query"
        )