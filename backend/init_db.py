"""
Database initialization for Hugging Face Spaces deployment
"""
from app.db import engine, Base
from app.config import settings
import logging

def init_db():
    """
    Initialize the database by creating all tables
    """
    try:
        # Create all tables in the database
        Base.metadata.create_all(bind=engine)
        print("Database tables created successfully!")
    except Exception as e:
        print(f"Error initializing database: {e}")
        raise

if __name__ == "__main__":
    init_db()