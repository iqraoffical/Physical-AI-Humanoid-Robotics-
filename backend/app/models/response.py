from pydantic import BaseModel
from typing import List, Optional
import uuid


class ChatResponse(BaseModel):
    """
    The system's answer to a user query
    """
    response: str
    citations: List[dict]
    session_id: str
    confidence: float
    query_id: str
    
    class Config:
        from_attributes = True