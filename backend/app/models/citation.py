from pydantic import BaseModel, Field
from typing import Optional


class Citation(BaseModel):
    """
    Information about the source of information in a response
    """
    chapter: Optional[str] = None
    section: Optional[str] = None
    page: Optional[int] = None
    url: Optional[str] = None
    # For selected text citations
    source: Optional[str] = None
    context: Optional[str] = None
    
    class Config:
        from_attributes = True