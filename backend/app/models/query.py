from pydantic import BaseModel, Field
from typing import Optional
import uuid
import re


class UserQuery(BaseModel):
    """
    A question or text input from the user along with context and source type
    """
    query: str = Field(..., min_length=1, max_length=1000)
    selected_text: Optional[str] = Field(None, max_length=5000)
    session_id: Optional[str] = None
    user_id: Optional[str] = None  # For personalization based on user background
    query_type: str = Field(default="general", pattern="^(general|selected_text)$")
    include_citations: bool = True

    class Config:
        from_attributes = True