# API Contract: RAG Chatbot API for AI Textbook

**Feature**: `003-rag-chatbot-api`
**Contract Version**: 1.0.0
**Created**: 2025-12-17
**Status**: Draft

## Overview

This document defines the API contracts for the RAG Chatbot API for AI Textbook. The API provides endpoints for querying book content, including both general queries and queries limited to user-selected text.

## Base URL

`https://[your-domain]/api/v1`

## Authentication

No authentication required for API endpoints.

## Endpoints

### GET /health

#### Description
Check the health status of the chatbot API to monitor availability and functionality.

#### Request
No request body required.

#### Response

**Success Response (200 OK)**
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

**Error Response (503 Service Unavailable)**
```json
{
  "status": "unhealthy",
  "timestamp": "2025-12-17T10:30:00Z",
  "dependencies": {
    "qdrant": "unreachable",
    "neon": "reachable",
    "cohere": "reachable"
  },
  "error": "Qdrant vector database is unavailable"
}
```

### POST /query

#### Description
Submit a general question about book content to get accurate, context-specific answers derived solely from the book's content.

#### Request
```json
{
  "query": "string (question about book content)",
  "session_id": "string (optional)",
  "include_citations": "boolean (optional, default: true)"
}
```

**Field Requirements:**
- `query`: Required, non-empty string (1-1000 characters)
- `session_id`: Optional, if provided should be valid UUID format
- `include_citations`: Optional, boolean to determine if citations should be included in response

#### Response

**Success Response (200 OK)**
```json
{
  "response": "string (answer from book content)",
  "citations": [
    {
      "chapter": "string",
      "section": "string", 
      "page": "number",
      "url": "string (optional)"
    }
  ],
  "session_id": "string",
  "confidence": "number (0-1)",
  "query_id": "string (UUID)"
}
```

**Not Found Response (200 OK)**
```json
{
  "response": "This information is not available in the book.",
  "citations": [],
  "session_id": "string", 
  "confidence": 0,
  "query_id": "string (UUID)"
}
```

**Error Response (400 Bad Request)**
```json
{
  "error": "string (error message)",
  "details": "object (optional details)"
}
```

### POST /selected-text-query

#### Description
Submit a question about user-selected text to get answers based only on the provided text, ignoring the broader book content.

#### Request
```json
{
  "query": "string (question about selected text)",
  "selected_text": "string (the text user selected, 1-5000 characters)",
  "session_id": "string (optional)",
  "include_citations": "boolean (optional, default: true)"
}
```

**Field Requirements:**
- `query`: Required, non-empty string (1-1000 characters)
- `selected_text`: Required, non-empty string (1-5000 characters)
- `session_id`: Optional, if provided should be valid UUID format
- `include_citations`: Optional, boolean to determine if citations should be included in response

#### Response

**Success Response (200 OK)**
```json
{
  "response": "string (answer based only on selected text)",
  "citations": [
    {
      "source": "selected_text",
      "context": "string (relevant part of selected text)"
    }
  ],
  "session_id": "string",
  "confidence": "number (0-1)",
  "query_id": "string (UUID)"
}
```

**Not Found Response (200 OK)**
```json
{
  "response": "This information is not available in the provided text.",
  "citations": [],
  "session_id": "string",
  "confidence": 0,
  "query_id": "string (UUID)"
}
```

**Error Response (400 Bad Request)**
```json
{
  "error": "string (error message)",
  "details": "object (optional details)"
}
```

## Error Responses

The API returns standard HTTP status codes:

- `200`: Success (including cases where no content was found)
- `400`: Bad request (malformed input, validation error)
- `408`: Request timeout 
- `429`: Rate limit exceeded
- `500`: Internal server error
- `503`: Service unavailable (dependency failure)

## Citation Format

All responses that contain information from the book must include citations in the following format:

For book content:
```json
{
  "chapter": "Chapter Name",
  "section": "Section Name",
  "page": 123,
  "url": "https://example.com/book/chapter-123"
}
```

For user-selected text:
```json
{
  "source": "selected_text",
  "context": "Relevant excerpt from the selected text..."
}
```

## Validation Rules

1. All responses must be sourced from the book content only (no external knowledge)
2. If no relevant content is found, return the exact string "This information is not available in the book." or "This information is not available in the provided text."
3. Citations must accurately reference the source of information in the response
4. Input queries must be validated for length and format