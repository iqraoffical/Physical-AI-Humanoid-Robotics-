# Deployment Guide: RAG Chatbot on Hugging Face Spaces

This guide provides instructions for deploying the RAG Chatbot for AI Textbook on Hugging Face Spaces.

## Prerequisites

1. A Hugging Face account
2. API keys for the required services:
   - Cohere API key
   - Qdrant instance (URL and API key)
   - Neon PostgreSQL database URL

## Deployment Steps

### Option 1: Direct Hugging Face Spaces Creation

1. Create a new Space on Hugging Face:
   - Go to [Hugging Face Spaces](https://huggingface.co/spaces)
   - Click "Create new Space"
   - Choose the following settings:
     - **Repository**: Use your repository with the backend code
     - **SDK**: Gradio
     - **Hardware**: Choose appropriate hardware (CPU basic is usually sufficient)
     - **Visibility**: Public or Private as needed

2. Add Environment Variables:
   - Go to your Space settings
   - Go to the "Secrets" tab
   - Add the following environment variables:
     - `COHERE_API_KEY`: Your Cohere API key
     - `QDRANT_URL`: Your Qdrant instance URL
     - `QDRANT_API_KEY`: Your Qdrant API key
     - `NEON_DATABASE_URL`: Your Neon PostgreSQL database URL

### Option 2: Git-based Deployment

1. Push your code to a Hugging Face Hub repository
2. Create a Space that uses your repository
3. Add the required environment variables in the Space settings

## Required Files

The following files are essential for Hugging Face Spaces deployment:

- `app.py` - Main entry point for the Gradio application
- `requirements.txt` - Python dependencies
- `README.md` - Documentation for the Space
- `runtime.txt` - Python version specification (optional)
- `.env.example` - Example environment variables

## Configuration

The application uses environment variables for configuration:

- `COHERE_API_KEY`: Required for text generation and embeddings
- `QDRANT_URL`: Required for vector database operations
- `QDRANT_API_KEY`: Required for vector database operations
- `NEON_DATABASE_URL`: Required for session management
- `ALLOWED_ORIGINS`: For CORS configuration (default: "*")

## Architecture Notes

- The application uses Gradio for the UI
- FastAPI is available for API endpoints
- All services (Cohere, Qdrant, Neon) need to be accessible from the Space
- The application automatically uses the PORT environment variable provided by Hugging Face Spaces

## Troubleshooting

If your Space fails to build or run:

1. Check the Space logs for error messages
2. Ensure all required environment variables are set
3. Verify that your external services (Cohere, Qdrant, Neon) are accessible
4. Make sure all dependencies in requirements.txt are compatible

## Scaling Considerations

- For high traffic, consider upgrading the Space hardware
- Monitor external service usage (Cohere, Qdrant, Neon) for costs and rate limits
- Sessions might not persist across Space restarts if using temporary storage

## Updating the Space

To update your Space:
1. Push changes to your repository
2. The Space will automatically rebuild, or you can manually trigger a rebuild in the Space settings