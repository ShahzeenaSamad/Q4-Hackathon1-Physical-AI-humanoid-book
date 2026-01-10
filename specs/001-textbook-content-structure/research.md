# Research: Physical AI Textbook Content Structure

**Date**: 2025-12-29
**Feature**: 001-textbook-content-structure
**Status**: Complete

## Technology Decisions

### 1. Frontend Framework: Docusaurus 3.x

**Decision**: Use Docusaurus 3.x with TypeScript

**Rationale**:
- Constitution mandate (Principle IV): "The textbook MUST be built using Docusaurus v3+"
- Built-in MDX support for interactive components
- Excellent sidebar navigation for textbook structure
- Dark mode support out of the box
- Fast static site generation for GitHub Pages deployment

**Alternatives Considered**:
- Nextra (Next.js-based): Rejected - smaller community, less documentation features
- GitBook: Rejected - less customizable, paid features for advanced use
- VitePress: Rejected - Vue-based, team prefers React ecosystem

### 2. Backend Framework: FastAPI

**Decision**: Use FastAPI with Python 3.10+

**Rationale**:
- Constitution mandate (Principle III): "Backend: FastAPI with Neon Serverless Postgres and Qdrant Cloud"
- Native async/await support for concurrent RAG queries
- Automatic OpenAPI documentation
- Excellent Python ecosystem integration (OpenAI SDK, Qdrant client)
- Type hints with Pydantic for request/response validation

**Alternatives Considered**:
- Flask: Rejected - no native async, older patterns
- Django: Rejected - too heavy for API-only backend
- Express.js: Rejected - Python required for OpenAI/ML ecosystem

### 3. Vector Database: Qdrant Cloud

**Decision**: Use Qdrant Cloud free tier

**Rationale**:
- Constitution mandate (Principle III): "Qdrant Cloud (free tier)"
- 1GB free tier sufficient for ~1000 embeddings
- Python client with simple API
- Managed service reduces operational overhead
- Good documentation and community support

**Alternatives Considered**:
- Pinecone: Rejected - expensive for free tier limits
- pgvector: Rejected - adds complexity to Postgres, less optimized
- Chroma: Rejected - local-only, no managed option
- Weaviate: Rejected - more complex setup, overkill for project scope

### 4. Relational Database: Neon Serverless PostgreSQL

**Decision**: Use Neon Serverless Postgres

**Rationale**:
- Constitution mandate (Principle III): "Neon Serverless Postgres"
- Generous free tier (500MB storage, 190 compute hours)
- Serverless scales to zero when idle
- Standard PostgreSQL compatibility
- Easy Vercel/Railway integration

**Alternatives Considered**:
- Supabase: Rejected - overkill features, more complex setup
- PlanetScale: Rejected - MySQL-based, team prefers Postgres
- SQLite: Rejected - not suitable for production deployment

### 5. LLM Integration: OpenAI API

**Decision**: Use OpenAI GPT-4 for RAG responses and text-embedding-3-small for embeddings

**Rationale**:
- Constitution mandate (Principle III): "Answer questions about book content using OpenAI Agents/ChatKit SDKs"
- Best-in-class response quality for educational content
- text-embedding-3-small: cost-effective, 1536 dimensions
- Well-documented Python SDK

**Alternatives Considered**:
- Anthropic Claude: Not specified in constitution
- Local LLMs (Llama): Rejected - resource constraints, quality concerns
- Azure OpenAI: Rejected - additional setup complexity

### 6. Authentication: better-auth (Bonus Feature)

**Decision**: Use better-auth library for bonus authentication feature

**Rationale**:
- Constitution mandate (Principle V bonus): "Authentication system using better-auth"
- Lightweight, TypeScript-first
- JWT-based session management
- Easy integration with React frontend

**Alternatives Considered**:
- NextAuth: Rejected - requires Next.js, not Docusaurus compatible
- Auth0: Rejected - overkill, paid features
- Passport.js: Rejected - older patterns, not TypeScript-first

### 7. Embedding Model Selection

**Decision**: OpenAI text-embedding-3-small (1536 dimensions)

**Rationale**:
- Cost: $0.02 per 1M tokens (vs $0.13 for text-embedding-3-large)
- Quality: Sufficient for textbook Q&A use case
- Dimension: 1536 fits Qdrant free tier well
- Performance: Fast inference for real-time queries

**Configuration**:
- Chunk size: 500 tokens with 100 token overlap
- Metadata: module_id, chapter_id, section_title, learning_outcomes
- Estimated embeddings: 500-1000 for 24 chapters

### 8. Deployment Strategy

**Decision**: GitHub Pages (frontend) + Railway (backend)

**Rationale**:
- GitHub Pages: Free, automatic CI/CD with GitHub Actions
- Railway: Free tier ($5 credit/month), easy Docker deployment
- Separation allows independent scaling and updates

**Alternatives Considered**:
- Vercel: Good option, may use for frontend if GitHub Pages limitations arise
- Fly.io: Good alternative to Railway
- Render: Slightly slower cold starts than Railway

## Content Architecture Decisions

### 1. Chapter Structure

**Decision**: Standardized 8-section chapter template

**Sections**:
1. Title + Prerequisites
2. Learning Outcomes (3-5, Bloom's taxonomy)
3. Introduction
4. Core Content (subdivided)
5. Code Examples (2-5 per chapter)
6. Hands-On Exercise (1-2 per chapter)
7. Summary
8. Assessment Questions (5-10 per chapter)

**Rationale**: Aligns with FR-003 in spec, ensures consistency for AI-generated content

### 2. Module Organization

**Decision**: Sequential chapter numbering (1-24, not per-module)

**Rationale**:
- Spec requirement FR-005: "Chapters MUST be numbered sequentially across modules"
- Easier cross-referencing ("See Chapter 6" vs "See Module 1 Chapter 6")
- Simpler URL structure (/docs/module-1-ros2/06-understanding-urdf)

### 3. Code Example Standards

**Decision**: Python 3.10+ with Black formatting, ROS 2 Humble environment

**Standards**:
- Environment comment at top: `# Environment: Ubuntu 22.04 + ROS 2 Humble + Python 3.10`
- Inline comments for non-obvious logic
- Executable without modification (spec FR-008)
- Numbered and captioned (spec FR-019)

## Risk Mitigations

| Risk | Mitigation |
|------|------------|
| OpenAI API costs | Use text-embedding-3-small, cache responses, rate limiting |
| Qdrant free tier limits | Optimize chunk size, deduplicate similar content |
| Content generation time | Parallel chapter generation, prioritize MVP chapters |
| Hackathon deadline | MVP-first strategy (3-4 chapters/module, not all 24) |

## Resolved Clarifications

All NEEDS CLARIFICATION items from Technical Context have been resolved:
- ✅ Language/Version: Python 3.10+, TypeScript
- ✅ Dependencies: Docusaurus, FastAPI, OpenAI, Qdrant, Neon
- ✅ Storage: Neon Postgres + Qdrant Cloud
- ✅ Testing: pytest (backend), Jest/Playwright (frontend)
- ✅ Platform: GitHub Pages + Railway
- ✅ Performance: <3s page load, <2s API response
- ✅ Constraints: Free tier limits, simulation-focused
- ✅ Scale: 24 chapters, 500-1000 embeddings, 100+ concurrent users
