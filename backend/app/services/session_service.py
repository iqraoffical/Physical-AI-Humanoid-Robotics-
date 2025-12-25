from sqlalchemy.orm import Session
from sqlalchemy import and_
from app.models.db_models import SessionDB, QueryDB
from app.models.session import QuerySession
from datetime import datetime, timedelta
import uuid


class SessionService:
    @staticmethod
    def create_session(db: Session) -> QuerySession:
        """
        Create a new session with a unique session ID
        """
        session_id = str(uuid.uuid4())
        db_session = SessionDB(
            session_id=session_id,
            query_history=[]
        )
        db.add(db_session)
        db.commit()
        db.refresh(db_session)
        
        return QuerySession(
            session_id=db_session.session_id,
            created_at=db_session.created_at,
            last_access=db_session.last_access,
            query_history=db_session.query_history
        )
    
    @staticmethod
    def get_session(db: Session, session_id: str) -> QuerySession:
        """
        Retrieve an existing session by session ID
        """
        db_session = db.query(SessionDB).filter(SessionDB.session_id == session_id).first()
        if db_session:
            return QuerySession(
                session_id=db_session.session_id,
                created_at=db_session.created_at,
                last_access=db_session.last_access,
                query_history=db_session.query_history
            )
        return None
    
    @staticmethod
    def update_session_last_access(db: Session, session_id: str):
        """
        Update the last access time for a session
        """
        db_session = db.query(SessionDB).filter(SessionDB.session_id == session_id).first()
        if db_session:
            db_session.last_access = datetime.utcnow()
            db.commit()
    
    @staticmethod
    def add_query_to_session(db: Session, session_id: str, query_text: str, response_text: str, citations: list):
        """
        Add a query-response pair to the session's query history
        """
        # Create query record
        query_record = QueryDB(
            session_id=session_id,
            query_text=query_text,
            response_text=response_text,
            citations=citations
        )
        db.add(query_record)
        
        # Update session history
        db_session = db.query(SessionDB).filter(SessionDB.session_id == session_id).first()
        if db_session:
            # Limit history to 50 most recent entries
            history = db_session.query_history or []
            history.append({
                "query": query_text,
                "response": response_text,
                "timestamp": datetime.utcnow().isoformat()
            })
            # Keep only the most recent 50 entries
            db_session.query_history = history[-50:]
        
        db.commit()
    
    @staticmethod
    def cleanup_old_sessions(db: Session, hours: int = 24):
        """
        Remove sessions that haven't been accessed in the specified number of hours
        """
        cutoff_time = datetime.utcnow() - timedelta(hours=hours)
        old_sessions = db.query(SessionDB).filter(SessionDB.last_access < cutoff_time).all()
        
        for session in old_sessions:
            db.delete(session)
        
        db.commit()