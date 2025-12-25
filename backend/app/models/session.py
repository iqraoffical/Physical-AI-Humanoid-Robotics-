from pydantic import BaseModel
from typing import List, Optional
from datetime import datetime
import uuid


class QuerySession(BaseModel):
    """
    Represents a user's interaction with the chatbot system
    """
    session_id: str
    created_at: datetime
    last_access: datetime
    query_history: List[dict]  # List of query-response pairs
    
    class Config:
        # Enable ORM mode for compatibility with database models
        from_attributes = True