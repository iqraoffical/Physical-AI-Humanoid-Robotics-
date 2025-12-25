from sqlalchemy import Column, Integer, String, DateTime, Text, ForeignKey, JSON
from sqlalchemy.dialects.postgresql import UUID
from sqlalchemy.sql import func
from app.db import Base
import uuid


class SessionDB(Base):
    """
    Database model for user sessions
    """
    __tablename__ = "sessions"

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    session_id = Column(String(255), unique=True, nullable=False, index=True)
    created_at = Column(DateTime(timezone=True), server_default=func.now())
    last_access = Column(DateTime(timezone=True), server_default=func.now(), onupdate=func.now())
    query_history = Column(JSON)  # Store as JSONB


class QueryDB(Base):
    """
    Database model for storing queries and responses
    """
    __tablename__ = "queries"

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    session_id = Column(String(255), ForeignKey("sessions.session_id", ondelete="CASCADE"), index=True)
    query_text = Column(Text, nullable=False)
    response_text = Column(Text, nullable=False)
    created_at = Column(DateTime(timezone=True), server_default=func.now())
    citations = Column(JSON)  # Store as JSONB