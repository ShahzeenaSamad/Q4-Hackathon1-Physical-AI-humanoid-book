# Implementation Plan: Physical AI Textbook Content Structure

**Branch**: `001-textbook-content-structure` | **Date**: 2025-12-29 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/001-textbook-content-structure/spec.md`

## Summary

Build a comprehensive Physical AI & Humanoid Robotics textbook using Docusaurus with an integrated RAG chatbot. The textbook covers 4 modules (ROS 2, Gazebo/Unity Simulation, NVIDIA Isaac, Vision-Language-Action) across 24 chapters. The system includes a FastAPI backend for the RAG chatbot with Qdrant vector search and OpenAI integration, delivering the base 100-point hackathon requirement with optional bonus features (authentication, personalization, Urdu translation) for an additional 200 points.

## Technical Context

**Language/Version**: Python 3.10+ (backend), TypeScript/JavaScript (frontend)
**Primary Dependencies**:
- Frontend: Docusaurus 3.x, React 18, TypeScript
- Backend: FastAPI, OpenAI SDK, Qdrant Client, SQLAlchemy, Alembic, python-jose (JWT), bcrypt
- Optional: better-auth (bonus authentication feature)

**Storage**:
- Neon Serverless PostgreSQL (user data, chat sessions, personalization)
- Qdrant Cloud free tier (vector embeddings for RAG)
- Markdown files (textbook content)

**Testing**:
- Backend: pytest, pytest-cov
- Frontend: Jest, Playwright (e2e)

**Target Platform**:
- Frontend: GitHub Pages or Vercel (static site)
- Backend: Railway/Fly.io/Render (containerized FastAPI)
- Development: Ubuntu 22.04 (content environment), Windows/Mac (development)

**Project Type**: Web application (frontend + backend)

**Performance Goals**:
- Page load: <3s initial load
- API response: <2s for chatbot queries
- RAG search: <500ms vector similarity search

**Constraints**:
- OpenAI API rate limits (free tier considerations)
- Qdrant Cloud free tier (1GB vectors)
- No physical robot hardware required (simulation-focused)

**Scale/Scope**:
- 24 chapters across 4 modules
- ~500-1000 vector embeddings for RAG
- Target: 100+ concurrent users (hackathon demo)
- 13-week course curriculum

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

| Principle | Status | Compliance Notes |
|-----------|--------|------------------|
| I. AI-Native Content Development | ✅ PASS | Content created using Claude Code with SDD methodology |
| II. Modular Content Architecture | ✅ PASS | 4 modules with clear boundaries (ROS 2, Gazebo/Unity, Isaac, VLA) |
| III. Integrated RAG Chatbot (NON-NEGOTIABLE) | ✅ PASS | FastAPI + Qdrant + OpenAI architecture defined |
| IV. Docusaurus Foundation | ✅ PASS | Docusaurus 3.x with TypeScript, mobile-responsive |
| V. Progressive Enhancement | ✅ PASS | Base (100pts) first, then bonus features |
| VI. Code Quality & Security | ✅ PASS | Input validation, env vars for secrets, pytest testing |
| VII. Pedagogical Excellence | ✅ PASS | Progressive complexity, hands-on exercises, visual aids |
| VIII. Reusable Intelligence | ✅ PASS | Claude Code subagents defined for content generation |

**Gate Status**: ✅ ALL GATES PASS - Proceed to Phase 0

## Project Structure

### Documentation (this feature)

```text
specs/001-textbook-content-structure/
├── plan.md              # This file
├── research.md          # Phase 0 output - technology decisions
├── data-model.md        # Phase 1 output - database entities
├── quickstart.md        # Phase 1 output - developer setup guide
├── contracts/           # Phase 1 output - API specifications
│   └── openapi.yaml     # RAG chatbot API contract
└── tasks.md             # Phase 2 output - 171 implementation tasks
```

### Source Code (repository root)

```text
content/
└── docs/                        # Markdown chapters (24 chapters + appendices)
    ├── module-1-ros2/           # Chapters 1-8: ROS 2 Fundamentals
    ├── module-2-simulation/     # Chapters 9-13: Gazebo & Unity
    ├── module-3-isaac/          # Chapters 14-18: NVIDIA Isaac
    ├── module-4-vla/            # Chapters 19-24: Vision-Language-Action
    ├── appendices/              # Hardware, software, glossary, troubleshooting
    └── _templates/              # Chapter and module templates

frontend/
├── src/
│   ├── components/
│   │   ├── ChatbotWidget/       # RAG chatbot UI
│   │   ├── PersonalizationButton/  # Bonus: content personalization
│   │   ├── TranslationButton/   # Bonus: Urdu translation
│   │   └── AuthProvider/        # Bonus: authentication
│   ├── pages/                   # Custom Docusaurus pages
│   ├── theme/                   # Docusaurus theme overrides
│   └── services/                # API client services
├── tests/
│   └── components/              # Jest component tests
├── docusaurus.config.js         # Site configuration
├── sidebars.js                  # Navigation structure
└── package.json

backend/
├── src/
│   ├── api/
│   │   ├── health.py            # Health check endpoint
│   │   ├── chat.py              # RAG chatbot endpoints
│   │   ├── auth.py              # Bonus: authentication endpoints
│   │   ├── personalization.py   # Bonus: personalization endpoints
│   │   └── translation.py       # Bonus: translation endpoints
│   ├── core/
│   │   ├── config.py            # Environment configuration
│   │   ├── database.py          # Database connection
│   │   └── security.py          # Password hashing, JWT
│   ├── models/
│   │   ├── chat_session.py      # Chat session model
│   │   ├── textbook_content.py  # Content for embeddings
│   │   ├── user.py              # Bonus: user model
│   │   ├── personalization_profile.py  # Bonus
│   │   ├── personalized_content.py     # Bonus
│   │   └── translated_content.py       # Bonus
│   ├── services/
│   │   ├── embedding_service.py # OpenAI embeddings
│   │   ├── vector_store_service.py  # Qdrant operations
│   │   ├── rag_service.py       # RAG query pipeline
│   │   ├── personalization_service.py  # Bonus
│   │   └── translation_service.py      # Bonus
│   └── main.py                  # FastAPI application
├── migrations/
│   └── versions/                # Alembic migrations
├── tests/
│   ├── unit/                    # Unit tests
│   └── integration/             # API integration tests
├── requirements.txt
└── Dockerfile

scripts/
├── validate_content.py          # Chapter structure validator
└── generate_embeddings.py       # Batch embedding generation

static/
├── img/
│   ├── module-1/                # ROS 2 diagrams
│   ├── module-2/                # Simulation screenshots
│   ├── module-3/                # Isaac Sim visuals
│   └── module-4/                # VLA architecture diagrams
└── code-examples/
    ├── ros2/                    # Python ROS 2 examples
    ├── gazebo/                  # SDF world files
    ├── isaac/                   # Isaac Sim configs
    └── vla/                     # Voice/LLM integration code

checklists/
└── chapter-quality.md           # Content quality checklist

.claude/
├── agents/
│   └── chapter-generator.md     # Subagent for content generation
└── skills/
    ├── format-code-example.md   # Code formatting skill
    └── validate-markdown.md     # Validation skill

tests/
└── e2e/
    └── full_user_journey.spec.ts  # Playwright e2e tests
```

**Structure Decision**: Web application structure selected (Option 2) because the project requires:
1. Docusaurus static site frontend for textbook delivery
2. FastAPI backend for RAG chatbot, authentication, and personalization services
3. Separate deployment targets (GitHub Pages vs Railway)

## Complexity Tracking

> No constitution violations requiring justification. Architecture follows prescribed patterns.

| Decision | Rationale | Alternatives Considered |
|----------|-----------|------------------------|
| Separate frontend/backend | Required for dynamic RAG chatbot + static content | Single SPA rejected (Docusaurus is SSG) |
| Qdrant for vectors | Free tier, managed service, Python SDK | Pinecone (pricing), pgvector (complexity) |
| Neon Postgres | Serverless, free tier, Vercel integration | Supabase (overkill), SQLite (not scalable) |

## Phase 0 Research Completed

All technical decisions resolved from constitution and spec:
- Docusaurus 3.x (constitution mandate)
- FastAPI + Qdrant + OpenAI (constitution mandate)
- Neon Postgres (constitution mandate)
- better-auth (constitution bonus feature)

See [research.md](./research.md) for detailed technology rationale.

## Phase 1 Artifacts

- **data-model.md**: Database schema for chat sessions, users, personalization
- **contracts/openapi.yaml**: RESTful API specification for RAG chatbot
- **quickstart.md**: Developer setup guide for local development

## Next Steps

1. Run `/sp.tasks` to generate detailed implementation tasks (already exists: 171 tasks)
2. Begin Phase 1: Setup (T001-T007)
3. Execute MVP critical path: Setup → Foundational → US4 → US1 → RAG Chatbot
