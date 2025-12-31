from fastapi import APIRouter, Depends, HTTPException, status
from sqlalchemy.orm import Session
from typing import Optional
import logging
from pydantic import BaseModel

from app.db import get_db
from app.config import settings


router = APIRouter()

class TranslateRequest(BaseModel):
    content: str
    targetLanguage: str

class PersonalizeRequest(BaseModel):
    content: str
    userBackground: dict

@router.post("/translate")
async def translate_content(
    request: TranslateRequest,
    db: Session = Depends(get_db)
):
    """
    Translate content to the specified language
    """
    try:
        # In a real implementation, this would call a translation API like Google Translate
        # For now, we'll return the original content as a placeholder
        # since we don't have a translation service configured
        
        # Log the request for debugging
        logging.info(f"Translation request for language: {request.targetLanguage}")
        
        # Placeholder response - in a real implementation, you would call a translation API
        return {
            "translatedContent": f"[TRANSLATED TO {request.targetLanguage.upper()}]: {request.content[:100]}...",
            "targetLanguage": request.targetLanguage,
            "originalContentLength": len(request.content)
        }

    except Exception as e:
        logging.error(f"Error translating content: {str(e)}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Internal server error occurred while translating content"
        )

@router.post("/personalize")
async def personalize_content(
    request: PersonalizeRequest,
    db: Session = Depends(get_db)
):
    """
    Personalize content based on user background
    """
    try:
        # In a real implementation, this would use AI to customize the content
        # based on the user's software and hardware background
        # For now, we'll return a personalized version as a placeholder
        
        # Extract user background
        software_bg = request.userBackground.get('software', 'general programming')
        hardware_bg = request.userBackground.get('hardware', 'general hardware')
        
        # Log the request for debugging
        logging.info(f"Personalization request for software: {software_bg}, hardware: {hardware_bg}")
        
        # Create personalized content
        personalized_content = f"""
        <div style="border: 2px solid #4CAF50; padding: 1rem; border-radius: 8px; background-color: #f9f9f9;">
          <h3>Personalized Content</h3>
          <p>Based on your background in {software_bg} and {hardware_bg},
          here's a customized version of this content:</p>
          <div>{request.content}</div>
          <div style="margin-top: 1rem; padding: 0.5rem; background-color: #e8f5e9; border-radius: 4px;">
            <strong>Note:</strong> This content has been tailored to your experience level.
          </div>
        </div>
        """
        
        return {
            "personalizedContent": personalized_content,
            "userBackground": request.userBackground
        }

    except Exception as e:
        logging.error(f"Error personalizing content: {str(e)}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Internal server error occurred while personalizing content"
        )