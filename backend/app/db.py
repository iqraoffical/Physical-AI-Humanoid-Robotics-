from sqlalchemy import create_engine
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.orm import sessionmaker
from app.config import settings


# Create the database engine
engine = create_engine(
    settings.NEON_DATABASE_URL,
    pool_pre_ping=True,  # Verify connections before use
    pool_recycle=300,    # Recycle connections every 5 minutes
    echo=False           # Set to True for SQL query logging
)

# Create a configured "SessionLocal" class
SessionLocal = sessionmaker(
    autocommit=False,
    autoflush=False,
    bind=engine
)

# Create a Base class for declarative models
Base = declarative_base()


def get_db():
    """
    Dependency function that yields database sessions
    """
    db = SessionLocal()
    try:
        yield db
    finally:
        db.close()