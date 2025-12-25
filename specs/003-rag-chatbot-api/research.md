# Research Findings: RAG Chatbot API

**Feature**: `003-rag-chatbot-api`
**Created**: 2025-12-17
**Status**: Completed

## Research Task 1: Cohere Model Selection

### Decision: Use Cohere command-r-plus
- Cohere command-r-plus is selected for response generation
- command-r is selected for embedding generation

### Rationale:
- command-r-plus provides better quality responses for complex questions
- Acceptable latency for the free-tier usage requirements
- Better handling of book-specific technical content
- command-r provides efficient embeddings with good performance

### Alternatives Considered:
- command-r only: Lower latency but less nuanced responses
- Other models: Not allowed per constitution (OpenAI APIs prohibited)

## Research Task 2: Qdrant Schema Design

### Decision: Structure with comprehensive metadata
- Create a single collection for book content chunks
- Store chapter, section, page, and URL information in payload
- Use Cohere embedding dimensions (1024 for command-r)

### Rationale:
- Single collection simplifies management on free tier
- Rich metadata enables proper citations
- Cohere embedding dimensions ensure compatibility

### Schema:
```json
{
  "content": "text of the book content chunk",
  "chapter": "chapter name",
  "section": "section name",
  "page": 123,
  "url": "URL to the page",
  "chunk_index": 0
}
```

## Research Task 3: Prompt Engineering for Zero Hallucinations

### Decision: Strict context-only prompting with fallback
- Use explicit instructions to only use provided context
- Include clear fallback prompt: "If the provided context does not contain the answer, respond with: 'This information is not available in the book.'"
- Add citation requirement in prompts

### Rationale:
- Explicit instructions reduce hallucinations
- Fallback prompt ensures compliance with zero hallucination policy
- Citation requirement enforces grounding

### Example Prompt Template:
```
You are an AI assistant for an AI textbook. Answer the user's question based ONLY on the provided context from the book. Do not use any prior knowledge or external information.
If the context does not contain the answer, respond with: "This information is not available in the book."

Context:
{context_chunks}

Cite your sources with chapter/section and page number where possible.

Question: {user_question}
Answer:
```

## Research Task 4: Qwen CLI Integration Strategy

### Decision: Use Qwen CLI as validation step
- After Cohere generates response, pass it to Qwen CLI for validation
- Qwen CLI checks if response is supported by context
- If not supported, return the fallback message

### Rationale:
- Qwen CLI serves as an independent validation layer
- Provides additional assurance against hallucinations
- Can be enabled/disabled based on performance requirements

## Research Task 5: Session Data Policy

### Decision: Minimal session tracking with cleanup
- Store only essential session information in Neon
- Keep query history for 24 hours
- Use UUID for session IDs to ensure uniqueness

### Rationale:
- Minimizes database usage on free tier
- Complies with data retention constraints
- Provides enough information for user experience

### Schema:
- Session ID (UUID)
- Basic query history with timestamps
- Automatic cleanup after 24 hours