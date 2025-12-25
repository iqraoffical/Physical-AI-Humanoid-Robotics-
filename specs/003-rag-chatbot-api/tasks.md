# Task Specification: RAG Chatbot API for AI Textbook

**Feature**: `003-rag-chatbot-api`
**Created**: 2025-12-17
**Status**: Draft

## Feature Overview

Target Audience: AI enthusiasts, students, and developers reading the AI textbook, seeking accurate, context-specific answers without external knowledge. The system will provide a RAG chatbot API that answers questions using only book content with proper citations.

## Implementation Strategy

The feature will be implemented in phases prioritizing User Story 1 as the MVP. Each phase will build upon the previous one and result in independently testable functionality. Key dependencies include FastAPI, Qdrant Cloud, Neon Postgres, and Cohere APIs.

## Dependencies

- User Story 2 (Selected Text Query) depends on foundational components (data models, session management) established in User Story 1
- User Story 3 (Health Check) is independent but benefits from foundational infrastructure
- All stories depend on successful setup and foundational phases

## Parallel Execution Opportunities

- Within each user story, parallel development can occur on models, services, and endpoints (e.g., T020 [P], T021 [P], T022 [P] for US1)
- Database schema creation can run in parallel with API endpoint development
- Docker configuration can be developed in parallel with the core application
- Unit tests can be written in parallel with the implementation code

---

## Phase 1: Setup (Project Initialization)

- [X] T001 Create project structure with proper directories (app/, tests/, docs/, etc.)
- [ ] T002 Set up virtual environment with Python 3.9+
- [X] T003 Create requirements.txt with dependencies (fastapi, uvicorn, cohere, qdrant-client, asyncpg, python-multipart)
- [X] T004 Configure environment variables for Cohere, Qdrant, and Neon
- [X] T005 Set up basic configuration file for service settings
- [X] T006 Initialize Git repository with proper .gitignore
- [X] T007 Create Dockerfile for containerization
- [ ] T008 Set up pre-commit hooks for code formatting and linting

---

## Phase 2: Foundational (Blocking Prerequisites)

- [X] T010 Define core data models for QuerySession, ChatResponse, UserQuery, and BookContentChunk
- [X] T011 Implement database connection and session handling for Neon Postgres
- [X] T012 Create database schema for sessions and queries tables
- [X] T013 Implement Qdrant client connection and initialization
- [X] T014 Set up Cohere client with command-r and command-r-plus models
- [X] T015 Implement basic service layer interfaces (SessionService, QueryService)
- [X] T016 Create API response models based on contract specifications
- [ ] T017 Implement request validation models for all endpoints
- [X] T018 Set up logging and error handling middleware
- [X] T019 Create utility functions for citation formatting and validation

---

## Phase 3: User Story 1 - General Book Query (Priority: P1)

As a reader of the AI textbook, I want to ask questions about the book content so that I can get accurate, context-specific answers derived solely from the book's content.

**Independent Test**: Can be fully tested by submitting a question about book content and verifying that the response is accurate, sourced from the book, and includes proper citations.

**Acceptance Scenarios**:
1. Given I have entered a question about book content, When I submit the query, Then the chatbot returns an answer based only on book content with proper citations to book sections/chapters.
2. Given I ask a question not covered by the book content, When I submit the query, Then the chatbot responds with "This information is not available in the book."
3. Given I have submitted a question, When I receive the response, Then the response includes citations to specific book sections/chapters where the information was retrieved from.

- [X] T020 [P] [US1] Create QuerySession model in app/models/session.py
- [X] T021 [P] [US1] Create UserQuery model in app/models/query.py
- [X] T022 [P] [US1] Create ChatResponse model in app/models/response.py
- [X] T023 [P] [US1] Create Citation model in app/models/citation.py
- [X] T024 [US1] Implement SessionService in app/services/session_service.py
- [X] T025 [US1] Implement retrieval service for Qdrant in app/services/retrieval_service.py
- [X] T026 [US1] Implement generation service with Cohere in app/services/generation_service.py
- [X] T027 [US1] Implement Qwen CLI validation service in app/services/validation_service.py
- [X] T028 [US1] Create /query endpoint in app/api/query_endpoint.py
- [X] T029 [US1] Implement the main query processing logic in app/core/query_processor.py
- [X] T030 [US1] Add citation extraction logic from Qdrant metadata in app/utils/citation_extractor.py
- [ ] T031 [US1] Implement zero hallucination check using fallback prompt
- [ ] T032 [US1] Add response formatting with citations and confidence scores
- [ ] T033 [US1] Integrate session management with Neon Postgres
- [ ] T034 [US1] Implement confidence scoring based on similarity scores
- [ ] T035 [US1] Add error handling for Qdrant unavailability
- [ ] T036 [US1] Create unit tests for US1 components in tests/test_us1_query.py
- [ ] T037 [US1] Create integration tests for US1 flow in tests/integration/test_us1_integration.py
- [ ] T038 [US1] Validate US1 against acceptance criteria

---

## Phase 4: User-Selected Text Query (Priority: P2)

As a reader using the book website, I want to select specific text on the page and ask questions about only that text so that I can get detailed explanations focused on the content I'm currently viewing.

**Independent Test**: Can be tested by providing selected text along with a question and verifying that the response is derived only from that provided text, ignoring the broader book content.

**Acceptance Scenarios**:
1. Given I have provided selected text and a question about it, When I submit the query, Then the chatbot returns an answer based only on the provided text (making the selected text the entire context for the query) with no reference to other book content.
2. Given I ask a question that cannot be answered from the selected text, When I submit the query, Then the chatbot responds with "This information is not available in the provided text."

- [X] T040 [P] [US2] Enhance UserQuery model to support selected text in app/models/query.py
- [ ] T041 [P] [US2] Create SelectedTextQuery model in app/models/selected_text_query.py
- [ ] T042 [US2] Implement temporary embedding functionality for selected text in app/services/embedding_service.py
- [X] T043 [US2] Create /selected-text-query endpoint in app/api/selected_text_endpoint.py
- [ ] T044 [US2] Implement selected text specific processing logic in app/core/selected_text_processor.py
- [X] T045 [US2] Adapt retrieval service for temporary embeddings in app/services/retrieval_service.py
- [X] T046 [US2] Update generation service to handle selected text context in app/services/generation_service.py
- [X] T047 [US2] Implement special citation format for selected text responses
- [ ] T048 [US2] Add validation to ensure selected text is used as exclusive context
- [ ] T049 [US2] Create unit tests for US2 components in tests/test_us2_selected_text.py
- [ ] T050 [US2] Create integration tests for US2 flow in tests/integration/test_us2_integration.py
- [ ] T051 [US2] Validate US2 against acceptance criteria

---

## Phase 5: User Story 3 - Health and Status Check (Priority: P3)

As a system administrator or developer, I want to check the health status of the chatbot API so that I can monitor its availability and functionality.

**Independent Test**: Can be tested by calling the health endpoint and verifying it returns a successful status response.

**Acceptance Scenarios**:
1. Given The chatbot API is running, When I call the health check endpoint, Then I receive a successful status response indicating the system is operational.
2. Given The chatbot API is experiencing issues, When I call the health check endpoint, Then I receive an appropriate error status response.

- [ ] T055 [P] [US3] Create HealthCheck model in app/models/health.py
- [ ] T056 [P] [US3] Create HealthService in app/services/health_service.py
- [X] T057 [US3] Implement GET /health endpoint in app/api/health_endpoint.py
- [X] T058 [US3] Implement dependency health checks for Qdrant, Neon, and Cohere
- [ ] T059 [US3] Create unit tests for US3 components in tests/test_us3_health.py
- [ ] T060 [US3] Validate US3 against acceptance criteria

---

## Phase 6: Polish & Cross-Cutting Concerns

- [ ] T065 Implement rate limiting for API endpoints in app/middleware/rate_limit.py
- [ ] T066 Add comprehensive error handling with proper HTTP status codes
- [ ] T067 Implement timeout handling with partial response return in app/core/timeout_handler.py
- [ ] T068 Add automatic session cleanup after 24 hours in app/tasks/session_cleanup.py
- [ ] T069 Optimize Qdrant queries to stay within free-tier limits
- [ ] T070 Add performance monitoring and metrics collection
- [ ] T071 Implement comprehensive logging for debugging and monitoring
- [ ] T072 Create API documentation with Swagger/OpenAPI
- [ ] T073 Add input validation for extremely long user inputs
- [ ] T074 Handle ambiguous questions that could refer to multiple book sections
- [ ] T075 Address malformed input handling
- [ ] T076 Create comprehensive test suite for edge cases
- [ ] T077 Optimize response times to meet <5s requirement
- [ ] T078 Perform integration testing across all user stories
- [ ] T079 Update Docker configuration with environment-specific settings
- [ ] T080 Document deployment process and requirements
- [ ] T081 Conduct final validation against all success criteria