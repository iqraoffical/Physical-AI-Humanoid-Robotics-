from fastapi import APIRouter, Depends, HTTPException, status
from sqlalchemy.orm import Session
from typing import Optional
import uuid
import logging

from app.db import get_db
from app.models.query import UserQuery
from app.models.api_responses import SelectedTextQueryResponse
from app.services.session_service import SessionService
from app.services.retrieval_service import RetrievalService
from app.services.generation_service import GenerationService
from app.config import settings


router = APIRouter()


@router.post("/selected-text-query", response_model=SelectedTextQueryResponse)
async def selected_text_query_endpoint(
    user_query: UserQuery,
    db: Session = Depends(get_db)
):
    """
    Submit a question about user-selected text to get answers based only
    on the provided text, ignoring the broader book content.
    """
    try:
        # Validate input
        if not user_query.selected_text:
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail="Selected text is required for this endpoint"
            )

        if len(user_query.query) > settings.MAX_QUERY_LENGTH:
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail=f"Query exceeds maximum length of {settings.MAX_QUERY_LENGTH} characters"
            )

        if len(user_query.selected_text) > settings.MAX_SELECTED_TEXT_LENGTH:
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail=f"Selected text exceeds maximum length of {settings.MAX_SELECTED_TEXT_LENGTH} characters"
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

        # Generate response based only on selected text
        result = generation_service.generate_selected_text_response(
            query=user_query.query,
            selected_text=user_query.selected_text
        )

        # Add query-response to session history
        SessionService.add_query_to_session(
            db,
            session_id,
            f"Query: {user_query.query}, Selected Text: {user_query.selected_text[:100]}...",
            result["response"],
            result["citations"]
        )

        # Create response object
        response = SelectedTextQueryResponse(
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
        logging.error(f"Error processing selected text query: {str(e)}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Internal server error occurred while processing the selected text query"
        )