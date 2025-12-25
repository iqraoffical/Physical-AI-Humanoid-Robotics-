# Data Model: RAG Chatbot API for AI Textbook

**Feature**: `003-rag-chatbot-api`
**Created**: 2025-12-17
**Status**: Draft

## Database Schema (Neon Postgres)

### Sessions Table
```sql
CREATE TABLE sessions (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    session_id VARCHAR(255) NOT NULL UNIQUE,
    created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
    last_access TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
    query_history JSONB,
    INDEX idx_session_id (session_id),
    INDEX idx_last_access (last_access)
);
```

### Queries Table
```sql
CREATE TABLE queries (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    session_id VARCHAR(255) REFERENCES sessions(session_id) ON DELETE CASCADE,
    query_text TEXT NOT NULL,
    response_text TEXT NOT NULL,
    created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
    citations JSONB,
    INDEX idx_session_id (session_id),
    INDEX idx_created_at (created_at)
);
```

## Vector Database Schema (Qdrant Cloud)

### Collection: book_content_chunks

#### Vector Configuration
- Size: 1024 (Cohere command-r embedding dimensions)
- Distance: Cosine

#### Payload Schema
```json
{
  "content": {
    "type": "keyword",
    "description": "The text content of the book chunk"
  },
  "chapter": {
    "type": "keyword",
    "description": "The chapter name/number"
  },
  "section": {
    "type": "keyword",
    "description": "The section name/number within the chapter"
  },
  "page": {
    "type": "integer",
    "description": "The page number in the book"
  },
  "url": {
    "type": "keyword",
    "description": "The URL to the specific page/chapter online"
  },
  "chunk_index": {
    "type": "integer",
    "description": "The sequential index of this chunk within the source"
  }
}
```

## Entity Models

### QuerySession
Represents a user's interaction with the chatbot system

**Fields**:
- `session_id`: Unique identifier for the session (string, UUID format)
- `created_at`: Timestamp when session was created (datetime)
- `last_access`: Timestamp of last interaction (datetime)
- `query_history`: List of query-response pairs in this session (list of objects)

**Validation Rules**:
- session_id must be unique
- created_at and last_access must be valid timestamps
- query_history is limited to 50 most recent entries

### BookContentChunk
Represents a segment of book content in vector format stored in Qdrant

**Fields**:
- `content`: The text content of the book chunk (string)
- `chapter`: The chapter name/number (string)
- `section`: The section name/number within the chapter (string)
- `page`: The page number in the book (integer)
- `url`: The URL to the specific page/chapter online (string, optional)
- `chunk_index`: The sequential index of this chunk within the source (integer)

**Validation Rules**:
- content cannot be empty
- page must be a positive integer
- if URL is provided, it must be a valid URL format

### ChatResponse
The system's answer to a user query

**Fields**:
- `response`: The answer text (string)
- `citations`: List of sources for the response (list of citation objects)
- `session_id`: Associated session ID (string)
- `confidence`: Confidence level of the response (float between 0 and 1)
- `query_id`: Unique identifier for this query (UUID)

**Validation Rules**:
- response must be non-empty when information is available
- if response is "This information is not available in the book", citations should be empty
- confidence must be between 0 and 1
- citations must match the required format

### Citation Object
Information about the source of information in a response

**Fields**:
- `chapter`: The chapter name/number where the information was found (string)
- `section`: The section name/number where the information was found (string) 
- `page`: The page number where the information was found (integer)
- `url`: The URL to the specific page/chapter (string, optional)

**Validation Rules**:
- chapter and section must correspond to actual book content
- page must be a positive integer
- if URL is provided, it must be a valid URL format

### UserQuery
A question or text input from the user

**Fields**:
- `query`: The user's question (string)
- `selected_text`: Optional text selected by the user to limit context (string, optional)
- `session_id`: Associated session ID (string, optional)
- `query_type`: Type of query ("general" or "selected_text") (enum)

**Validation Rules**:
- query must be non-empty
- if selected_text is provided, query_type must be "selected_text"
- selected_text must not exceed 5000 characters