# RAG Chatbot for AI Textbook - Backend

This is the backend for a RAG (Retrieval-Augmented Generation) chatbot that answers questions based on AI textbook content. It can be deployed on Hugging Face Spaces using Gradio.

## Features

- Question answering based on textbook content
- Support for citations
- Session management
- API endpoints for integration

## Deployment on Hugging Face Spaces

This application is configured for deployment on Hugging Face Spaces with Gradio. The main entry point is `space.py`.

### Environment Variables

You need to set the following environment variables in your Hugging Face Space settings:

- `COHERE_API_KEY`: Your Cohere API key
- `QDRANT_URL`: Your Qdrant instance URL
- `QDRANT_API_KEY`: Your Qdrant API key
- `NEON_DATABASE_URL`: Your Neon PostgreSQL database URL

### Configuration

The application uses the following configuration values (set via environment variables):
- `COHERE_GENERATION_MODEL`: Cohere model for generation (default: "command-r-plus")
- `COHERE_EMBEDDING_MODEL`: Cohere model for embeddings (default: "command-r")
- `QDRANT_COLLECTION_NAME`: Name of the Qdrant collection (default: "book_content_chunks")
- `APP_ENV`: Application environment (default: "development")
- `LOG_LEVEL`: Logging level (default: "info")
- `ALLOWED_ORIGINS`: Comma-separated list of allowed origins for CORS (default: "*")

## Local Development

To run the application locally:

```bash
# Install dependencies
pip install -r requirements.txt

# Set environment variables
export COHERE_API_KEY=your_cohere_api_key
export QDRANT_URL=your_qdrant_url
export QDRANT_API_KEY=your_qdrant_api_key
export NEON_DATABASE_URL=your_neon_database_url

# Run the application
python space.py
```

## API Endpoints

The application also provides a FastAPI backend with the following endpoints:
- `/api/v1/query` - General question answering
- `/api/v1/selected-text-query` - Question answering for selected text
- `/api/v1/health` - Health check

## Architecture

- `app/` - Main application code
- `app/api/` - API endpoints
- `app/services/` - Business logic services
- `app/models/` - Data models
- `app/config.py` - Configuration settings