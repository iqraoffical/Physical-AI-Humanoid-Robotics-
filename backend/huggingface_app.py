"""
Hugging Face Space app.py file for the RAG Chatbot API
This file creates the FastAPI app instance for deployment
"""

from app.main import app

# This allows the Hugging Face Space to properly load the application
# The app instance is created in app.main and imported here

# For Hugging Face Spaces, the app instance should be available at the module level
# This file serves as the entry point for the Hugging Face Space

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)