from pydantic import BaseModel
from typing import List, Optional
from app.models.citation import Citation


class QueryResponse(BaseModel):
    response: str
    citations: List[Citation]
    session_id: str
    confidence: float
    query_id: str


class SelectedTextQueryResponse(BaseModel):
    response: str
    citations: List[Citation]
    session_id: str
    confidence: float
    query_id: str


class HealthResponse(BaseModel):
    status: str
    timestamp: str
    dependencies: dict