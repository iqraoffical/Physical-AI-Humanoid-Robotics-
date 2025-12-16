<!-- SYNC IMPACT REPORT:
Version Change: N/A -> 1.0.0
Modified Principles: N/A (New creation)
Added Sections: All sections added for new constitution
Removed Sections: N/A
Templates Requiring Updates: 
  - .specify/templates/plan-template.md: ✅ to be reviewed
  - .specify/templates/spec-template.md: ✅ to be reviewed
  - .specify/templates/tasks-template.md: ✅ to be reviewed
  - .specify/templates/commands/*.md: ✅ to be reviewed
  - README.md: ⚠ pending review
Follow-up TODOs: None
-->

# Project Constitution: Physical AI & Humanoid Robotics — Unified Academic Book

## Overview

This constitution governs the creation of a unified, graduate-level academic textbook titled:

"Physical AI & Humanoid Robotics: Embodied Intelligence in the Physical World"

The book focuses on AI systems operating in physical environments, covering
ROS 2, Gazebo, Unity, NVIDIA Isaac, Vision-Language-Action (VLA), humanoid
robotics, and sim-to-real deployment.

The book will be written using Claude Code and Spec-Kit Plus, structured as
a modular curriculum, and deployed as a static documentation website using
Docusaurus and GitHub Pages.

## Primary Objectives

1. Produce a complete, structured textbook using an AI/spec-driven workflow
2. Ensure academic rigor suitable for senior undergraduate and graduate students
3. Maintain technical accuracy and reproducibility
4. Publish the book via Docusaurus with long-term maintainability

## Target Audience

- Senior undergraduate students (CS, AI, Robotics)
- Graduate students and researchers
- Robotics and AI engineers
- Readers with a computer science background

## Core Principles

### I. Accuracy through Primary Source Verification
All factual, technical, and architectural claims must be verifiable against
primary sources, including:
- Peer-reviewed papers
- Official documentation (ROS 2, NVIDIA Isaac, Gazebo, OpenAI/Whisper)
- Authoritative standards

No speculative or unsupported claims are allowed.

### II. Clarity for an Academic Book Audience
The writing must be clear, structured, and pedagogical:
- Explain intuition before implementation
- Define all acronyms at first use
- Maintain consistent terminology across chapters
- Assume CS background, but not prior robotics expertise

### III. Reproducibility and Practical Realism
All system architectures, pipelines, and workflows must be described with
sufficient detail to allow:
- Reproduction in simulation
- Understanding of sim-to-real constraints
- Awareness of hardware and performance limitations

### IV. Engineering and Scientific Rigor
The book must:
- Prefer peer-reviewed sources where applicable
- Clearly distinguish research results from engineering practice
- Explicitly state assumptions, limitations, and trade-offs

### V. Traceability of Claims
Every non-trivial factual or technical claim must be traceable to a citation.
Uncited assertions are not permitted.

### VI. Zero-Tolerance Plagiarism Policy
All text must be original.
- No copying or paraphrasing without citation
- Direct quotations must be minimal and clearly marked
- Final content must pass plagiarism detection with 0% unoriginal content
  (excluding properly cited quotes)

## Key Standards

### Citation Format:
- APA style (7th edition)

### Source Requirements:
- Minimum 50% peer-reviewed sources
- Remaining sources may include:
  - Official documentation
  - Technical whitepapers
  - Standards and SDK references

### Writing Quality:
- Flesch–Kincaid grade level: 10–12
- Academic but readable textbook tone

## Content Constraints

### Structure:
- Modular organization aligned with:
  - Physical AI foundations
  - ROS 2 (Robotic Nervous System)
  - Simulation (Gazebo & Unity)
  - NVIDIA Isaac platform
  - Vision-Language-Action systems
  - Humanoid robotics and capstone project

### Chapter Requirements:
Each chapter must include:
- Learning Objectives
- Conceptual Foundations
- System Architecture
- Practical / Engineering Considerations
- Summary

### Format:
- Markdown source files (Docusaurus compatible)
- Final deployed website via GitHub Pages
- Optional PDF export from Markdown

## Tooling & Workflow Governance

### Claude Code:
- Used for structured content generation
- Must strictly follow this constitution
- Must flag any violations before proceeding

### Spec-Kit Plus:
- Governs specification-driven writing
- Ensures consistency across chapters
- Enforces compliance with this constitution

All specifications, plans, tasks, and generated content must conform to this
constitution. Any violation must be corrected before moving forward.

## Success Criteria

- Book is fully deployable via Docusaurus and GitHub Pages
- All claims are verifiable and properly cited
- Zero plagiarism detected
- Consistent academic tone across all modules
- Content is suitable for peer review and classroom adoption

## Governance

This constitution is the authoritative, non-negotiable standard governing the
entire book project. Any amendments must be explicitly documented and approved
before use. All AI agents must reference this constitution during every phase
of specification, planning, writing, and review.

### Versioning
- Version: 1.0.0
- Ratification Date: 2025-12-15
- Status: Active
- Amendment Procedure: Changes require explicit documentation and approval before implementation
- Versioning Policy: Follow semantic versioning for all constitution updates
- Compliance Review: All project artifacts must be validated against this constitution regularly