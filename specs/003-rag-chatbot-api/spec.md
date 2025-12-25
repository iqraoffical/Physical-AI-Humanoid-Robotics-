# Feature Specification: RAG Chatbot API for AI Textbook

**Feature Branch**: `003-rag-chatbot-api`
**Created**: 2025-12-17
**Status**: Draft
**Input**: User description: "Target Audience: AI enthusiasts, students, and developers reading the AI textbook, seeking accurate, context-specific answers without external knowledge. Focus: Retrieval from vectorized book content for accurate responses. Support for user-selected text: Answers must be derived only from the provided text, ignoring the full book if specified. Integration: Embed the chatbot seamlessly into the book/website using FastAPI for API endpoints. Database and Storage: Use Neon Postgres for session tracking/user data, Qdrant for vector embeddings and retrieval. Success Criteria: Chatbot answers 95%+ of book-related questions correctly, traceable to source content. Handles user-selected text queries with 100% fidelity to the provided text. Deploys successfully on free-tier infrastructure with low latency (<5s response time). Includes at least 3 example endpoints in FastAPI: /query (general book query), /selected-text-query (limited to user input), /health (status check). All responses include citations to book sections/chapters. Zero hallucinations: If no relevant content, respond "This information is not available in the book." Tested with 10+ sample queries, including edge cases like ambiguous or out-of-scope questions. Constraints: Use only specified tools: OpenAI Agents/ChatKit SDKs for agent logic, FastAPI for backend, Neon Serverless Postgres for database, Qdrant Cloud Free Tier for vectors. Embeddings and Generation: Use Cohere for embeddings and LLMs (trial key: Nmrb5JUPUhFhOc10lBOVastdMjDRcfdNbe88bbCK). Qdrant Credentials: Cluster ID: dbeba504-b071-44c9-bb0f-638ba28cd9a5, API Key: eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJhY2Nlc3MiOiJtIn0.1qLZaiA4EEwNsk27pnFsdqP4xy1jp_Nm2DzL6DDfKyo, Cluster Endpoint: https://dbeba504-b071-44c9-bb0f-638ba28cd9a5.us-east4-0.gcp.cloud.qdrant.io, Client Init Example: from qdrant_client import QdrantClient; qdrant_client = QdrantClient(url="https://dbeba504-b071-44c9-bb0f-638ba28cd9a5.us-east4-0.gcp.cloud.qdrant.io:6333", api_key="eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJhY2Nlc3MiOiJtIn0.EhuA80SCKHdpdMKLtJc2un-O_Z3odzpQACrgDz6KtEM"); print(qdrant_client.get_collections()). Neon Database Credentials: Connection String: psql 'postgresql://neondb_owner:npg_StxV5LlJ4kYj@ep-empty-heart-a49iz3q3-pooler.us-east-1.aws.neon.tech/neondb?sslmode=require&channel_binding=require'. Format: Code in Python, deployable as a FastAPI app with Dockerfile for easy hosting. Timeline: Complete within 1 week. Budget: Free tiers only; no paid upgrades. Not Building: Full website frontend (assume embedding into existing book site). Advanced authentication beyond basic session tracking in Neon. Multi-user real-time collaboration features. Custom ML training; use pre-built Cohere/OpenAI SDKs. Ethical AI discussions or bias mitigation (focus on functionality)."

## Clarifications

### Session 2025-12-17

- Q: How should system handle Qdrant unavailability? → A: Return appropriate error message
- Q: What level of session tracking is needed? → A: Basic session ID with query history
- Q: How should text selection affect query context? → A: Selected text becomes entire context for query
- Q: What should citation format include? → A: Include chapter/section name and page number
- Q: How to handle query timeouts? → A: Return partial response with timeout warning

## User Scenarios & Testing *(mandatory)*

### User Story 1 - General Book Query (Priority: P1)

As a reader of the AI textbook, I want to ask questions about the book content so that I can get accurate, context-specific answers derived solely from the book's content.

**Why this priority**: This is the core functionality of the chatbot - providing accurate answers from the book without external knowledge. It delivers the primary value of the feature.

**Independent Test**: Can be fully tested by submitting a question about book content and verifying that the response is accurate, sourced from the book, and includes proper citations.

**Acceptance Scenarios**:

1. **Given** I have entered a question about book content, **When** I submit the query, **Then** the chatbot returns an answer based only on book content with proper citations to book sections/chapters.

2. **Given** I ask a question not covered by the book content, **When** I submit the query, **Then** the chatbot responds with "This information is not available in the book."

3. **Given** I have submitted a question, **When** I receive the response, **Then** the response includes citations to specific book sections/chapters where the information was retrieved from.

---

### User Story 2 - User-Selected Text Query (Priority: P2)

As a reader using the book website, I want to select specific text on the page and ask questions about only that text so that I can get detailed explanations focused on the content I'm currently viewing.

**Why this priority**: This provides a specialized interaction mode that enhances the reading experience by allowing context-specific queries on selected content.

**Independent Test**: Can be tested by providing selected text along with a question and verifying that the response is derived only from that provided text, ignoring the broader book content.

**Acceptance Scenarios**:

1. **Given** I have provided selected text and a question about it, **When** I submit the query, **Then** the chatbot returns an answer based only on the provided text (making the selected text the entire context for the query) with no reference to other book content.

2. **Given** I ask a question that cannot be answered from the selected text, **When** I submit the query, **Then** the chatbot responds with "This information is not available in the provided text."

---

### User Story 3 - Health and Status Check (Priority: P3)

As a system administrator or developer, I want to check the health status of the chatbot API so that I can monitor its availability and functionality.

**Why this priority**: Essential for operational monitoring and ensuring the system is functioning as expected.

**Independent Test**: Can be tested by calling the health endpoint and verifying it returns a successful status response.

**Acceptance Scenarios**:

1. **Given** The chatbot API is running, **When** I call the health check endpoint, **Then** I receive a successful status response indicating the system is operational.

2. **Given** The chatbot API is experiencing issues, **When** I call the health check endpoint, **Then** I receive an appropriate error status response.

---

### Edge Cases

- What happens when a query takes longer than 5 seconds to process? The system should return partial response with timeout warning.
- How does system handle extremely long user inputs? The system should truncate or return an error with instruction to shorten the input.
- What happens when Qdrant vector database is unavailable? The system should return an appropriate error message.
- How does system handle ambiguous questions that could refer to multiple book sections? The system should ask for clarification or provide a general response.
- What happens when the system encounters malformed inputs? The system should return a helpful error message.

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST retrieve answers from vectorized book content only, with no external knowledge sources
- **FR-002**: System MUST provide accurate, context-specific answers traceable to book sections/chapters
- **FR-003**: Users MUST be able to submit general questions about book content via the /query endpoint
- **FR-004**: Users MUST be able to submit questions limited to selected text via the /selected-text-query endpoint
- **FR-005**: System MUST return "This information is not available in the book" when no relevant content is found
- **FR-006**: System MUST include citations to book sections/chapters (with chapter/section name and page number) in all responses
- **FR-007**: System MUST track user sessions using Neon Postgres database with basic session ID and query history
- **FR-008**: System MUST store and retrieve vector embeddings using Qdrant
- **FR-009**: System MUST respond to the /health endpoint with operational status
- **FR-010**: System MUST implement zero hallucination policy, not generating content beyond book scope
- **FR-011**: System MUST use Cohere for embeddings and language generation
- **FR-012**: System MUST limit response time to under 5 seconds
- **FR-013**: System MUST handle user-selected text queries with 100% fidelity to the provided text, making the selected text the entire context for the query
- **FR-014**: System MUST be deployable using a Dockerfile on free-tier infrastructure
- **FR-015**: System MUST return appropriate error message when Qdrant vector database is unavailable
- **FR-016**: System MUST return partial response with timeout warning when query takes longer than 5 seconds

### Key Entities

- **QuerySession**: Represents a user's interaction with the chatbot system, including basic session ID, timestamps, and query history
- **BookContentChunk**: Represents a segment of book content in vector format stored in Qdrant, with associated metadata (chapter, section, page number, URL)
- **ChatResponse**: The system's answer to a user query, including content, citations with chapter/section name and page number, and confidence level
- **UserQuery**: A question or text input from the user along with context and source type (full book or selected text)

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Chatbot answers 95%+ of book-related questions correctly when tested against a standard dataset
- **SC-002**: All responses are traceable to specific book sections/chapters in at least 95% of cases
- **SC-003**: System handles user-selected text queries with 100% fidelity to the provided text, ignoring broader book content
- **SC-004**: System deploys successfully on free-tier infrastructure with response time under 5 seconds for 90% of queries
- **SC-005**: System operates correctly when tested with 10+ sample queries including edge cases
- **SC-006**: Zero hallucinations occur during testing with sample queries (all responses are grounded in book content)
- **SC-007**: Health endpoint returns operational status correctly 100% of the time
- **SC-008**: System maintains 99% uptime during a 24-hour monitoring period