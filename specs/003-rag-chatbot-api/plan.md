# Implementation Plan: RAG Chatbot API for AI Textbook

**Feature**: `003-rag-chatbot-api`
**Created**: 2025-12-17
**Status**: Draft
**Author**: AI Systems Architect

## Technical Context

**Project**: Integrated RAG Chatbot embedded in an AI textbook website
**Goal**: Answers questions ONLY from the book's content
**Stack**: FastAPI + Qdrant Cloud + Neon Postgres + Cohere
**Constraint**: Free-tier infrastructure only, zero hallucinations

**Known Requirements**:
- FastAPI backend with 3 endpoints: /query, /selected-text-query, /health
- Qdrant Cloud Free Tier for vector embeddings
- Neon Serverless Postgres for session tracking
- Cohere embeddings and LLMs
- Cite sources with chapter/section and page number
- Strictly avoid external knowledge use

**Unknowns**:
- Cohere model choice: Resolved - command-r-plus for generation, command-r for embeddings
- Qdrant collection structure: Resolved - single collection with comprehensive metadata
- Cohere prompt engineering: Resolved - strict context-only prompting with fallback
- Session data retention: Resolved - minimal tracking with 24-hour cleanup
- Qwen CLI integration: Resolved - validation step after response generation

## Constitution Check

This plan adheres to the Project Constitution:
- ✅ Source-grounded Accuracy: All responses from book content only
- ✅ Zero Hallucination Tolerance: Clear response when info not available
- ✅ Educational Clarity: Appropriate for AI/robotics students
- ✅ Deterministic Answers: Reproducible results
- ✅ Cohere-only LLM policy: No OpenAI APIs
- ✅ Book-sourced Data: Only from vectorized book content
- ✅ Qwen CLI validation: Integrated for response validation
- ✅ Proper Citations: Chapter/section/page format implemented
- ✅ Strict Contextual Limitation: Selected text queries limited to provided text only

## Gates

### Compliance Gate:
- [x] All components use approved technologies (FastAPI, Qdrant, Neon, Cohere)
- [x] No OpenAI APIs or unauthorized services
- [x] Zero hallucination policy enforced
- [x] Source-grounded responses guaranteed
- [x] Citations with chapter/section/page number included

### Architecture Gate:
- [x] High-level architecture defined
- [x] Data flow between components defined
- [x] Error handling and fallbacks planned
- [x] Session management approach defined

### Validation Gate:
- [x] Testing strategy includes 15+ query categories
- [x] Manual validation checklist defined
- [x] Performance measurement approach defined

## Phase 0: Research & Unknown Resolution

### 0.1 Research Tasks

1. **Cohere Model Selection**
   - Compare command-r vs command-r-plus for accuracy vs latency
   - Evaluate free-tier usage limits
   - Determine best model for RAG context

2. **Qdrant Schema Design**
   - Research optimal metadata schema for book content
   - Determine vector dimension requirements for Cohere embeddings
   - Plan collection structure for content chunks

3. **Prompt Engineering for Zero Hallucinations**
   - Research techniques to enforce source-only responses
   - Design prompt templates with strict grounding instructions
   - Plan fallback prompts for when content not found

4. **Qwen CLI Integration Strategy**
   - Research how to integrate Qwen CLI for response validation
   - Design validation workflow to catch hallucinations
   - Determine when to use Qwen CLI in the pipeline

5. **Session Data Policy**
   - Research Neon Postgres limitations on free tier
   - Plan data retention policy for user sessions
   - Design minimal schema for session tracking

### 0.2 Research Findings Template

For each research task, document:
- Decision made
- Rationale for the decision
- Alternatives considered
- Implementation implications

## Phase 1: Design & Contracts

### 1.1 High-Level Architecture

```
[User]
  ↓ (query with/without selected text)
[FastAPI Service Layer]
  ↓ (session ID, query)
[Session Management Layer] ← (Neon Postgres)
  ↓ (validated request with session context)
[Qdrant Retrieval Layer]
  ↓ (relevant chunks with metadata)
[Cohere Generation Layer]
  ↓ (contextually-grounded response)
[Qwen CLI Validation Layer]
  ↓ (response validated against context)
[Response Formatter]
  ↓ (response with citations)
[User Response]
  ↓ (stored in session history)
[Neon Postgres]
```

### 1.2 Component Breakdown

#### Frontend Integration
- Assumed existing book site integration
- Simple POST request to API endpoints
- Response display with citations

#### FastAPI Service Layer
- Handles HTTP requests and responses for /query, /selected-text-query, and /health endpoints
- Implements request validation based on API contract
- Manages request routing to appropriate query handling logic
- Implements rate limiting to stay within free-tier usage

#### Session Management Layer
- Creates and manages session IDs using UUIDs
- Stores minimal session data in Neon Postgres (session ID, query history)
- Implements automatic cleanup of sessions older than 24 hours
- Associates each query with a session for tracking

#### Retrieval Layer
- Uses Cohere command-r embeddings to search Qdrant
- For general queries: searches full book content collection
- For selected-text queries: temporarily embeds selected text and retrieves from it only
- Returns top-3 most relevant chunks with full metadata (chapter, section, page, etc.)
- Implements similarity threshold to prevent low-quality matches

#### Generation Layer
- Formats retrieved chunks as context with clear separators
- Uses Cohere command-r-plus with strict instructions to only use provided context
- Implements prompt templates that require citation of sources
- Includes fallback response when context doesn't contain the answer

#### Qwen CLI Validation Layer
- Validates generated responses against the source context
- Rejects responses that make claims not supported by the context
- Ensures all responses are properly grounded in book content
- Returns fallback message if validation fails

#### Response Formatting Layer
- Formats citations according to specification (chapter/section/page format)
- Ensures all responses meet the required structure from API contract
- Adds confidence scores based on similarity scores from retrieval
- Constructs proper response objects with all required fields

#### Error and Fallback Handling
- Handles Qdrant unavailability with appropriate error messages
- Handles empty query responses with "This information is not available in the book"
- Handles timeout conditions by returning partial responses with timeout warnings
- Handles malformed inputs with helpful error messages
- Implements fallback to general knowledge limitation when selected text is insufficient

### 1.3 FastAPI Endpoint Design

#### POST /query
Request:
```
{
  "query": "string (question about book content)",
  "session_id": "string (optional)"
}
```

Response:
```
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
  "confidence": "number (0-1)"
}
```

#### POST /selected-text-query
Request:
```
{
  "query": "string (question about selected text)",
  "selected_text": "string (the text user selected)",
  "session_id": "string (optional)"
}
```

Response:
```
{
  "response": "string (answer based only on selected text)",
  "citations": [
    {
      "source": "selected_text",
      "context": "string (relevant part of selected text)"
    }
  ],
  "session_id": "string", 
  "confidence": "number (0-1)"
}
```

#### GET /health
Response:
```
{
  "status": "healthy",
  "timestamp": "ISO 8601 datetime",
  "dependencies": {
    "qdrant": "reachable",
    "neon": "reachable",
    "cohere": "reachable"
  }
}
```

### 1.4 Data Model

#### Neon Postgres Schema

**Table: sessions**
```
- id: UUID (primary key, default: gen_random_uuid())
- session_id: VARCHAR(255) (unique, indexed)
- created_at: TIMESTAMP WITH TIME ZONE (default: NOW())
- last_access: TIMESTAMP WITH TIME ZONE (default: NOW())
- query_history: JSONB
- indexes: idx_session_id, idx_last_access
```

**Table: queries**
```
- id: UUID (primary key, default: gen_random_uuid())
- session_id: VARCHAR(255) (foreign key to sessions, cascade delete)
- query_text: TEXT (not null)
- response_text: TEXT (not null)
- created_at: TIMESTAMP WITH TIME ZONE (default: NOW())
- citations: JSONB
- indexes: idx_session_id, idx_created_at
```

#### Qdrant Collection Schema

**Collection: book_content_chunks**

**Vector Configuration:**
- Size: 1024 (Cohere command-r embedding dimensions)
- Distance: Cosine

**Payload Schema:**
```
- content: KEYWORD (the book content text)
- chapter: KEYWORD (chapter name)
- section: KEYWORD (section name)
- page: INTEGER (page number)
- url: KEYWORD (URL to the page, optional)
- chunk_index: INTEGER (sequential index of chunk in source)
```

## Phase 2: Implementation Roadmap

### Week 1 Implementation Plan

#### Day 1: Research & Setup
- Complete remaining research tasks from Phase 0 (if any)
- Set up development environment with Python 3.9+
- Install dependencies (FastAPI, Cohere SDK, Qdrant client, asyncpg for Postgres)
- Create project structure with proper modules
- Configure environment variables for Cohere, Qdrant, and Neon

#### Day 2: Foundation & API Structure
- Implement FastAPI application with proper configuration
- Create endpoint stubs for /query, /selected-text-query, and /health
- Set up request/response models based on API contract
- Implement basic request validation
- Set up logging and error handling middleware

#### Day 3: Data Layer & Vector Storage
- Implement Qdrant client integration with proper configuration
- Set up Cohere embedding client with command-r model
- Create Neon Postgres connection layer with asyncpg
- Implement data models for sessions and queries
- Create initial data access layer functions

#### Day 4: Core RAG Logic
- Implement retrieval logic with similarity search
- Create generation function using Cohere command-r-plus
- Add citation extraction from Qdrant metadata
- Implement the selected-text query handling with temporary embedding
- Add similarity threshold to prevent low-quality matches

#### Day 5: Validation & Integration
- Integrate Qwen CLI as a validation step after response generation
- Implement session management with automatic cleanup
- Connect all components together with proper error handling
- Implement the fallback responses when content is not found
- Add confidence scoring based on similarity scores

#### Day 6: Testing & Validation
- Write unit tests for all core functions
- Create integration tests for end-to-end flows
- Validate against all 15+ query categories
- Test error handling and fallback scenarios
- Performance testing to ensure <5s response times

#### Day 7: Deployment & Documentation
- Create optimized Dockerfile with multi-stage build
- Implement environment-specific configurations
- Add monitoring and health check improvements
- Final performance optimization and cleanup
- Complete documentation and prepare for deployment

## Phase 3: Testing & Validation Strategy

### Unit Tests
- Test each endpoint individually
- Test retrieval logic with mock Qdrant
- Test generation function with mock Cohere
- Test session management with mock Neon

### Integration Tests
- End-to-end flow testing
- Qdrant and Neon integration tests
- Cohere API integration tests

### Validation Categories (15+)
1. Valid general book queries
2. Valid selected-text queries
3. Out-of-scope general queries
4. Out-of-scope selected-text queries
5. Ambiguous queries
6. Extremely long queries
7. Queries with special characters
8. Queries with no matching content
9. Selected text with no answer
10. Selected text that is empty
11. Selected text with multiple possible answers
12. Queries during Qdrant downtime
13. Queries during high latency
14. Multiple concurrent sessions
15. Session timeout scenarios

### Manual Validation Checklist
- [ ] Every answer traceable to book content
- [ ] Zero unsupported claims in responses
- [ ] Citations properly formatted with chapter/section/page
- [ ] Proper fallback message when info not available
- [ ] Selected text queries ignore general book content
- [ ] Response time under 5 seconds on free-tier

## Deployment Considerations

### Dockerfile Strategy
- Multi-stage build to minimize image size
- Separate build and production stages
- Environment variable management for secrets

### Environment Variables
- COHERE_API_KEY
- QDRANT_URL
- QDRANT_API_KEY
- NEON_DATABASE_URL

### Free-Tier Optimization
- Minimize token usage with efficient prompts
- Optimize vector queries to reduce compute
- Implement caching where possible
- Use serverless-ready patterns

## Success Criteria Alignment

This plan addresses all success criteria from the feature spec:
- [ ] Chatbot answers 95%+ of book-related questions correctly
- [ ] All responses traceable to specific book sections/chapters
- [ ] Handles user-selected text queries with 100% fidelity
- [ ] Deploys successfully on free-tier infrastructure
- [ ] Response time under 5 seconds for 90% of queries
- [ ] Tested with 10+ sample queries including edge cases
- [ ] Zero hallucinations during testing
- [ ] Health endpoint functional
