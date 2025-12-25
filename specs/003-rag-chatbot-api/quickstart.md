# Quickstart Guide: RAG Chatbot API for AI Textbook

**Feature**: `003-rag-chatbot-api`
**Created**: 2025-12-17
**Status**: Draft

## Overview

This guide provides instructions for quickly setting up and using the RAG Chatbot API for AI Textbook. The API allows users to ask questions about book content and receive answers derived solely from the book's content with proper citations.

## Prerequisites

- Python 3.9+
- pip package manager
- Access to Cohere API (with provided trial key)
- Qdrant Cloud account with provided credentials
- Neon Postgres account with provided credentials

## Getting Started

### 1. Clone the Repository

```bash
git clone [repository-url]
cd [repository-name]
```

### 2. Install Dependencies

```bash
pip install -r requirements.txt
```

### 3. Set Environment Variables

Create a `.env` file in the project root with the following variables:

```env
COHERE_API_KEY=Nmrb5JUPUhFhOc10lBOVastdMjDRcfdNbe88bbCK
QDRANT_URL=https://dbeba504-b071-44c9-bb0f-638ba28cd9a5.us-east4-0.gcp.cloud.qdrant.io:6333
QDRANT_API_KEY=eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJhY2Nlc3MiOiJtIn0.1qLZaiA4EEwNsk27pnFsdqP4xy1jp_Nm2DzL6DDfKyo
NEON_DATABASE_URL=postgresql://neondb_owner:npg_StxV5LlJ4kYj@ep-empty-heart-a49iz3q3-pooler.us-east-1.aws.neon.tech/neondb?sslmode=require&channel_binding=require
```

### 4. Run the Application

```bash
# Using uvicorn
uvicorn main:app --reload --port 8000

# Or using the run script if available
python -m app.main
```

## API Usage Examples

### Health Check

Check if the API is running:

```bash
curl -X GET http://localhost:8000/health
```

Expected response:
```json
{
  "status": "healthy",
  "timestamp": "2025-12-17T10:30:00Z",
  "dependencies": {
    "qdrant": "reachable",
    "neon": "reachable",
    "cohere": "reachable"
  }
}
```

### General Book Query

Ask a question about the book content:

```bash
curl -X POST http://localhost:8000/query \
  -H "Content-Type: application/json" \
  -d '{
    "query": "What is the main concept of attention in transformers?",
    "session_id": "abc-123"
  }'
```

Expected response:
```json
{
  "response": "The attention mechanism in transformers allows the model to focus on different parts of the input when generating each part of the output. This is achieved through self-attention, where each token computes attention scores with all other tokens.",
  "citations": [
    {
      "chapter": "Attention Mechanisms",
      "section": "Self-Attention",
      "page": 156,
      "url": "https://book.example.com/chapter-attention"
    }
  ],
  "session_id": "abc-123",
  "confidence": 0.92,
  "query_id": "uuid-456"
}
```

### Selected Text Query

Ask a question about specific selected text:

```bash
curl -X POST http://localhost:8000/selected-text-query \
  -H "Content-Type: application/json" \
  -d '{
    "query": "Can you explain what gradient descent is?",
    "selected_text": "Gradient descent is an optimization algorithm used to minimize the loss function in machine learning models. It works by iteratively adjusting the model parameters in the direction of the negative gradient of the loss function.",
    "session_id": "abc-123"
  }'
```

Expected response:
```json
{
  "response": "Gradient descent is an optimization algorithm used to minimize the loss function in machine learning models. It works by iteratively adjusting the model parameters in the direction of the negative gradient of the loss function, effectively moving toward the minimum of the function.",
  "citations": [
    {
      "source": "selected_text",
      "context": "Gradient descent is an optimization algorithm used to minimize the loss function in machine learning models. It works by iteratively adjusting the model parameters in the direction of the negative gradient of the loss function."
    }
  ],
  "session_id": "abc-123",
  "confidence": 0.98,
  "query_id": "uuid-789"
}
```

## Local Development

### Running Tests

```bash
# Run all tests
python -m pytest

# Run with coverage
python -m pytest --cov=app
```

### Building Docker Image

```bash
docker build -t rag-chatbot-api .
docker run -p 8000:8000 rag-chatbot-api
```

### Environment Setup for Development

1. Install pre-commit hooks:
   ```bash
   pip install pre-commit
   pre-commit install
   ```

2. Format code:
   ```bash
   black .
   ```

3. Lint code:
   ```bash
   ruff check .
   ```

## Troubleshooting

### Common Issues

1. **API Keys Not Working**: Ensure your API keys are correctly set in the environment variables.

2. **Qdrant Connection Issues**: Verify the URL and API key are correct and that your account is active.

3. **No Results for Valid Queries**: Check that the book content has been properly vectorized and stored in Qdrant.

### Debugging Tips

- Enable debug logging by setting `LOG_LEVEL=DEBUG` in your environment
- Check the logs for specific error messages
- Verify all service connections using the health endpoint

## Next Steps

1. Explore the full API documentation
2. Set up vectorization pipeline for your book content
3. Customize the response formatting
4. Implement frontend integration