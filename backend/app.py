"""
Hugging Face Space app.py file for the RAG Chatbot API
This file creates the FastAPI app instance for deployment
"""

from app.main import app

# This allows the Hugging Face Space to properly load the application
# The app instance is created in app.main and imported here