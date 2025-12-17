<!-- SYNC IMPACT REPORT:
Version Change: 1.0.0 -> 1.1.0
Modified Principles: Previous principles replaced with new RAG chatbot principles
Added Sections: New project specifics for RAG chatbot
Removed Sections: Previous Physical AI & Humanoid Robotics content
Templates Requiring Updates:
  - .specify/templates/plan-template.md: ⚠ pending review
  - .specify/templates/spec-template.md: ⚠ pending review
  - .specify/templates/tasks-template.md: ⚠ pending review
  - .specify/templates/commands/*.md: ⚠ pending review
  - README.md: ⚠ pending review
Follow-up TODOs: Update dependent artifacts to align with new principles
-->

# Project Constitution: Integrated RAG Chatbot for AI Textbook Website

## Overview

This constitution governs the development of an integrated retrieval-augmented generation (RAG) chatbot embedded in a published technical book that answers user questions strictly using the book's content.

The chatbot will be built using Spec-Kit Plus methodology and deployed as part of a static documentation website using Docusaurus and GitHub Pages.

## Primary Objectives

1. Build a retrieval-augmented chatbot that answers user questions using only book content
2. Ensure 100% source-grounded accuracy with zero hallucination tolerance
3. Provide educational clarity for AI, robotics, and software engineering readers
4. Maintain deterministic, reproducible answers from the book's content
5. Deploy a production-ready system for academic and public use

## Target Audience

- Students learning AI, robotics, and software engineering
- Researchers seeking specific information from the textbook
- Technical professionals using the book as reference material
- Anyone interested in accurate, book-grounded responses to their questions

## Core Principles

### I. Source-grounded Accuracy (No External Knowledge)
All chatbot responses must be derived exclusively from the book's content. Responses must never incorporate external knowledge or general pre-trained model capabilities that weren't learned from the book itself.

### II. Zero Hallucination Tolerance
The chatbot must never fabricate, imagine, or extrapolate information beyond what is explicitly contained in the book content. If the answer is not present in the book, the system must state this clearly rather than providing potentially inaccurate information.

### III. Educational Clarity for Target Audience
All responses must be clear, educational, and appropriate for AI, robotics, and software engineering readers. Responses should use simple technical language that balances precision with understanding.

### IV. Deterministic and Reproducible Answers
Chatbot responses must be consistent for identical queries and retrievals. The system should provide reproducible results that can be traced back to specific book sections.

### V. Model-agnostic, API-independent Design
The system architecture should be designed to support alternative models and APIs if needed, though the implementation must follow the specified model policy for the current iteration.

## Key Standards

### Model Policy:
- Generation: Cohere LLMs only
- Embeddings: Cohere embeddings only
- Reasoning validation: Qwen CLI
- OpenAI APIs are strictly prohibited

### Data Management:
- Primary data source: Vectorized book content stored in Qdrant Cloud
- Metadata-backed chunks with chapter, section, page, and URL information
- All retrievals must reference proper book sections

### Answer Structure:
- Direct answer first
- Explanation in simple technical language
- Bullet points where helpful
- Citations of section/chapter names when applicable
- Format: (Chapter X – Section Y)

## Content Constraints

### Retrieval Rules:
- All answers MUST be derived from retrieved book chunks
- If no relevant chunks are retrieved, respond with: "This information is not available in the book."
- Never infer, assume, or extrapolate beyond provided content

### Selected Text Constraint:
- If user provides selected text:
  - Retrieval scope is LIMITED to the selected text only
  - Ignore all other book content
  - Answer must reference only that text
  - If answer is not present, explicitly state it

### Prohibited Behaviors:
- Using pretrained general knowledge
- Guessing missing information
- Answering outside book scope
- Fabricating citations or explanations
- Using OpenAI APIs or any other unauthorized services

## Tooling & Workflow Governance

### Qwen CLI:
- Used for quality checks and validation
- Validates answer against retrieved context
- Rejects responses with unsupported claims
- Enforces concise, non-redundant output

### Spec-Kit Plus:
- Governs specification-driven development
- Ensures consistency across components
- Enforces compliance with this constitution

### Development Workflow:
- Follow TDD principles where applicable
- Ensure all changes are tested against quality criteria
- Maintain traceability from requirements to implementation

All specifications, plans, tasks, and generated code must conform to this
constitution. Any violation must be corrected before continuing development.

## Success Criteria

- 100% answers traceable to book content
- Zero hallucinations in evaluation
- User-selected text queries handled correctly
- Production-ready for academic and public deployment
- Response latency optimized for free-tier infrastructure
- Stateless generation with session tracking via database only

## Governance

This constitution is the authoritative, non-negotiable standard governing the
entire RAG chatbot project. Any amendments must be explicitly documented and approved
before use. All AI agents must reference this constitution during every phase
of specification, planning, implementation, and review.

### Versioning
- Version: 1.1.0
- Ratification Date: 2025-12-15
- Last Amended Date: 2025-12-17
- Status: Active
- Amendment Procedure: Changes require explicit documentation and approval before implementation
- Versioning Policy: Follow semantic versioning for all constitution updates
  - MAJOR: Backward incompatible governance/principle removals or redefinitions
  - MINOR: New principle/section added or materially expanded guidance
  - PATCH: Clarifications, wording, typo fixes, non-semantic refinements
- Compliance Review: All project artifacts must be validated against this constitution regularly