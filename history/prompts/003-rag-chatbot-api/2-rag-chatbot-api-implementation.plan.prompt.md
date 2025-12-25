---
id: 2
title: "RAG Chatbot API Implementation Plan"
stage: plan
date: 2025-12-17
surface: agent
model: "Qwen Code"
feature: "003-rag-chatbot-api"
branch: "003-rag-chatbot-api"
user: "iqra"
command: "/sp.plan"
labels: ["planning", "architecture", "RAG", "chatbot"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - "specs/003-rag-chatbot-api/plan.md"
  - "specs/003-rag-chatbot-api/research.md"
  - "specs/003-rag-chatbot-api/data-model.md"
  - "specs/003-rag-chatbot-api/contracts/api_contract.md"
  - "specs/003-rag-chatbot-api/quickstart.md"
tests: []
---

# RAG Chatbot API Implementation Plan

## Original Prompt

Execute the implementation planning workflow using the plan template to generate design artifacts.

Role: You are a senior AI systems architect and RAG engineer designing a production-ready, zero-hallucination Retrieval-Augmented Generation chatbot for a published AI textbook.

Context:
This plan MUST strictly follow the previously defined:
- /sp.constitution (governance, grounding, citation, and behavior rules)
- /sp.specify (technical stack, constraints, success criteria)

Project Summary:
Design an Integrated RAG Chatbot embedded in an AI textbook website that answers questions ONLY from the book’s content using:
- FastAPI backend
- Qdrant Cloud Free Tier (vector store with metadata)
- Neon Serverless Postgres (session tracking)
- Cohere embeddings + Cohere LLMs (generation only)
- Qwen CLI for reasoning validation
- OpenAI APIs are strictly prohibited

Primary Objective:
Produce a clear, deterministic, and implementation-ready PLAN that enables zero hallucinations, strict source grounding, selected-text isolation, and free-tier performance.

Deliverables (MANDATORY):
1. High-Level Architecture
   - Provide a text-based architecture diagram (ASCII or structured description)
   - Clearly show data flow between:
     User → FastAPI → Retrieval → Cohere LLM → Validation → Response
   - Include Qdrant, Neon, Cohere, Qwen CLI explicitly

2. Detailed Component Breakdown
   - Frontend integration (assumed existing book site)
   - FastAPI service layer
   - Retrieval layer (Qdrant usage + metadata filtering)
   - Generation layer (Cohere prompt discipline)
   - Validation layer (Qwen CLI)
   - Session management (Neon Postgres)
   - Error and fallback handling

3. Data Flow Explanation
   - Step-by-step flow for:
     a) General book query
     b) Selected-text-only query
     c) No-relevant-content case
   - Explicitly state where hallucination prevention occurs

4. FastAPI Endpoint Design
   Define request/response schemas for:
   - POST /query
   - POST /selected-text-query
   - GET /health
   Include:
   - Input fields
   - Output format
   - Citation structure (Chapter/Section)
   - Exact fallback message:
     "This information is not available in the book."

5. Key Technical Decisions (Explain Tradeoffs)
   - Selected-text handling:
     - In-memory embeddings vs temporary Qdrant collection
     - Decision rationale considering free-tier limits
   - Retrieval strategy:
     - Pure semantic vs hybrid
     - Top-k value
     - Similarity threshold to prevent hallucinations
   - Cohere model choice:
     - command-r vs command-r-plus
     - Latency vs accuracy tradeoff
   - Session storage:
     - Neon schema options
     - What is stored vs what is NOT stored
   - Citation extraction from Qdrant payloads

6. Error Handling & Safety Logic
   - What happens if:
     - No chunks are retrieved
     - Retrieved chunks don’t contain the answer
     - User query is ambiguous
     - Selected text is empty or irrelevant
   - Explicit rejection of speculative answers

7. Testing & Validation Strategy
   - Unit tests (per endpoint)
   - Integration tests (Qdrant + Neon)
   - 15+ test query categories:
     - Valid queries
     - Selected-text queries
     - Out-of-scope queries
     - Ambiguous queries
     - Edge cases
   - Manual validation checklist:
     - Every answer traceable
     - Zero unsupported claims
   - Latency measurement on free-tier infra

8. Implementation Roadmap (1-Week Timeline)
   Organize phases as:
   - Research
   - Foundation
   - Analysis
   - Synthesis
   Include:
   - Daily milestones
   - Parallel development opportunities
   - Risk mitigation points

9. Deployment Considerations
   - Dockerfile strategy (multi-stage build)
   - Environment variable management
   - Free-tier optimization tips
   - Stateless backend constraints

Output Rules:
- Do NOT include any external knowledge
- Do NOT invent APIs, features, or tools not listed
- Be concise but complete
- Use clear headings and bullet points
- Assume production deployment for academic/public use
- The plan must be directly implementable without reinterpretation

## Response Summary

Successfully created a comprehensive implementation plan for the RAG Chatbot API, including:

1. Technical context with all unknowns resolved
2. Constitution compliance verification
3. All compliance, architecture, and validation gates passed
4. Research findings document with decisions and rationale
5. Detailed data model for both Qdrant and Neon Postgres
6. Complete API contract defining all endpoints
7. Quickstart guide for developers
8. High-level architecture and detailed component breakdown
9. Implementation roadmap with day-by-day tasks
10. Comprehensive testing and validation strategy

## Outcome

The implementation plan fully addresses all requirements from the original prompt, including architecture, component breakdown, data flow, endpoint design, technical decisions, error handling, testing strategy, roadmap, and deployment considerations. All unknowns have been researched and resolved, and the plan adheres to the project constitution and feature specification.

## Evaluation

The plan is complete, coherent, and ready for implementation. It addresses all specified deliverables with appropriate detail while maintaining focus on zero hallucinations, strict source grounding, and free-tier performance. The architecture incorporates all required components and follows best practices for RAG systems.