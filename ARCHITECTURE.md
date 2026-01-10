# System Architecture

## Overview

The Physical AI & Humanoid Robotics Textbook is a web-based learning platform consisting of three main components:

1. **Static Content Layer** (Docusaurus) - Interactive textbook interface
2. **RAG Chatbot Backend** (FastAPI) - AI-powered Q&A service
3. **Data Layer** (Qdrant + Neon Postgres) - Vector embeddings and user data

```
┌─────────────────────────────────────────────────────────────┐
│                         User Browser                         │
└──────────────────┬──────────────────────────────────────────┘
                   │
                   │ HTTP/HTTPS
                   │
┌──────────────────▼──────────────────────────────────────────┐
│                  Docusaurus Frontend                         │
│  ┌────────────────────────────────────────────────────────┐ │
│  │  • Static Textbook Pages (24 chapters)                 │ │
│  │  • Sidebar Navigation (4 modules)                      │ │
│  │  • ChatbotWidget Component (React)                     │ │
│  │  • Authentication UI (better-auth)                     │ │
│  └────────────────────────────────────────────────────────┘ │
└──────────────────┬──────────────────────────────────────────┘
                   │
                   │ REST API (axios)
                   │
┌──────────────────▼──────────────────────────────────────────┐
│                    FastAPI Backend                           │
│  ┌────────────────────────────────────────────────────────┐ │
│  │  Endpoints:                                             │ │
│  │  • POST /api/chat/query      - RAG chatbot Q&A         │ │
│  │  • POST /api/auth/signup     - User registration       │ │
│  │  • POST /api/auth/signin     - User authentication     │ │
│  │  • POST /api/personalize     - Content adaptation      │ │
│  │  • POST /api/translate       - Urdu translation        │ │
│  │  • GET  /health              - Health check            │ │
│  └────────────────────────────────────────────────────────┘ │
│  ┌────────────────────────────────────────────────────────┐ │
│  │  Services:                                              │ │
│  │  • EmbeddingService   - Generate text embeddings       │ │
│  │  • RAGService         - Retrieval-Augmented Generation │ │
│  │  • AuthService        - JWT token management           │ │
│  │  • PersonalizationSvc - LLM content rewriting          │ │
│  │  • TranslationService - Urdu translation with OpenAI   │ │
│  └────────────────────────────────────────────────────────┘ │
└───────────────┬──────────────────────┬──────────────────────┘
                │                      │
                │                      │
    ┌───────────▼───────────┐   ┌──────▼────────────┐
    │   Qdrant Cloud        │   │  Neon Serverless  │
    │   (Vector Store)      │   │  Postgres         │
    │                       │   │                   │
    │  • Chapter embeddings │   │  • users          │
    │  • Semantic search    │   │  • chat_sessions  │
    │  • text-embedding-3   │   │  • user_profiles  │
    │    -small (1536-dim)  │   │  • translations   │
    └───────────────────────┘   └───────────────────┘
                │
                │
    ┌───────────▼───────────┐
    │   OpenAI API          │
    │                       │
    │  • text-embedding-3-  │
    │    small (embeddings) │
    │  • gpt-4-turbo        │
    │    (completions)      │
    └───────────────────────┘
```

## Component Details

### 1. Frontend (Docusaurus 3.x)

**Technology Stack**:
- React 18+ with TypeScript
- Docusaurus 3.x (static site generator)
- Axios (API client)
- better-auth (authentication)

**Key Files**:
- `frontend/docusaurus.config.js` - Site configuration
- `frontend/sidebars.js` - Navigation structure
- `frontend/src/components/ChatbotWidget.tsx` - RAG chatbot UI
- `frontend/src/components/Auth/` - Login/signup components
- `content/docs/` - Markdown chapter content

**Responsibilities**:
- Serve static textbook content (24 chapters organized into 4 modules)
- Provide sidebar navigation with chapter prerequisites
- Render ChatbotWidget for AI-powered Q&A
- Handle user authentication state
- Request content personalization and translation

### 2. Backend (FastAPI)

**Technology Stack**:
- Python 3.10+
- FastAPI (async web framework)
- SQLAlchemy (ORM)
- Alembic (migrations)
- OpenAI Python SDK
- Qdrant Client
- python-jose (JWT)
- LangChain (RAG orchestration)

**Key Files**:
- `backend/src/main.py` - FastAPI application entry point
- `backend/src/api/` - REST endpoint handlers
- `backend/src/services/` - Business logic
- `backend/src/models/` - SQLAlchemy models
- `backend/src/core/config.py` - Settings and environment variables
- `backend/migrations/` - Alembic database migrations

**Responsibilities**:
- Process RAG chatbot queries with context retrieval
- Manage user authentication (JWT tokens)
- Personalize content based on user experience level
- Translate chapters to Urdu with technical term preservation
- Store and retrieve chat history

### 3. Data Layer

#### Qdrant Cloud (Vector Store)
- **Purpose**: Semantic search over textbook chapters
- **Data**: Chapter embeddings (text-embedding-3-small, 1536 dimensions)
- **Collection Structure**:
  ```json
  {
    "id": "chapter-01-01",
    "vector": [0.123, -0.456, ...],  // 1536-dim
    "payload": {
      "chapter_id": "01-introduction-physical-ai",
      "module": "module-1-ros2",
      "title": "Introduction to Physical AI",
      "content": "Full chapter text...",
      "metadata": {...}
    }
  }
  ```

#### Neon Serverless Postgres
- **Purpose**: User data, chat history, personalization profiles
- **Tables**:
  - `users` - User accounts (id, email, password_hash, background_questions)
  - `chat_sessions` - Chat history (id, user_id, query, response, timestamp)
  - `user_profiles` - Personalization settings (user_id, experience_level, preferences)
  - `translations` - Cached Urdu translations (chapter_id, translated_content)

## Data Flow

### RAG Chatbot Query Flow

```
1. User types question in ChatbotWidget
   ↓
2. Frontend sends POST /api/chat/query {"query": "What is ROS 2?", "user_id": "..."}
   ↓
3. Backend EmbeddingService generates query embedding (OpenAI text-embedding-3-small)
   ↓
4. Backend queries Qdrant for top-k similar chapter sections (k=3)
   ↓
5. Backend RAGService constructs prompt:
   "Context: [Retrieved chapters...]
    Question: What is ROS 2?
    Answer based on context with citations:"
   ↓
6. Backend calls OpenAI GPT-4 for completion
   ↓
7. Backend stores query/response in Neon Postgres chat_sessions table
   ↓
8. Backend returns response with chapter citations to frontend
   ↓
9. ChatbotWidget displays response with clickable chapter links
```

### Content Personalization Flow (Bonus Feature)

```
1. User clicks "Personalize for me" button on chapter page
   ↓
2. Frontend sends POST /api/personalize {"chapter_id": "...", "user_id": "..."}
   ↓
3. Backend checks user_profiles table for experience_level (beginner/intermediate/advanced)
   ↓
4. Backend constructs LLM prompt:
   "Rewrite this chapter for [experience_level] audience:
    - Adjust technical depth
    - Add/remove prerequisite explanations
    Original: [chapter content]"
   ↓
5. Backend calls OpenAI GPT-4 for rewritten content
   ↓
6. Backend caches personalized version in Postgres
   ↓
7. Frontend displays personalized content (not saved to docs/)
```

### Urdu Translation Flow (Bonus Feature)

```
1. User clicks "Translate to Urdu" button
   ↓
2. Frontend sends POST /api/translate {"chapter_id": "..."}
   ↓
3. Backend checks translations table for cached version
   ↓
4. If not cached:
   - Backend constructs prompt:
     "Translate to Urdu, preserve English for: ROS 2, URDF, VSLAM, Nav2, Isaac Sim, etc."
   - Calls OpenAI GPT-4
   - Caches result
   ↓
5. Backend returns Urdu content
   ↓
6. Frontend displays translated chapter (RTL layout)
```

## Security

### Authentication
- **Library**: better-auth (TypeScript)
- **Strategy**: JWT tokens with httpOnly cookies
- **Flow**:
  1. Signup: POST /api/auth/signup with email, password, background questions
  2. Backend hashes password (bcrypt), stores user
  3. Signin: POST /api/auth/signin
  4. Backend verifies password, generates JWT (python-jose)
  5. Frontend stores token in httpOnly cookie
  6. Subsequent requests include token in Authorization header

### API Security
- CORS configured for `FRONTEND_URL` (localhost:3000 dev, deployed URL prod)
- Environment variables for secrets (`.env` file, never committed)
- Rate limiting on chatbot endpoints (prevent abuse)
- Input validation with Pydantic models

## Deployment

### Development
- Frontend: `npm start` (localhost:3000)
- Backend: `uvicorn src.main:app --reload` (localhost:8000)
- Databases: Qdrant Cloud free tier, Neon Serverless free tier

### Production
- **Frontend**: GitHub Pages / Vercel
  - Build: `npm run build` → static HTML/JS/CSS
  - Deploy: Push to `gh-pages` branch or Vercel CLI
- **Backend**: Railway / Fly.io / Render
  - Containerized with Docker
  - Environment variables set in platform dashboard
  - Auto-scaling based on traffic
- **CI/CD**: GitHub Actions workflows
  - `.github/workflows/deploy-frontend.yml`
  - `.github/workflows/deploy-backend.yml`

## Performance Considerations

### Embedding Generation
- **One-time operation**: Run `python scripts/generate_embeddings.py` after content updates
- Batch processing (10 chapters at a time to avoid rate limits)
- Store embeddings in Qdrant (persistent)

### RAG Query Latency
- **Target**: <2 seconds for chatbot responses
- **Optimizations**:
  - Vector search: ~100ms (Qdrant Cloud)
  - LLM completion: ~1-2s (OpenAI GPT-4)
  - Cache frequent queries in Redis (optional future enhancement)

### Content Delivery
- Static site served from CDN (GitHub Pages, Vercel Edge Network)
- Lazy loading for images and code examples
- Gzip compression for markdown content

## Scalability

### Current Architecture (Hackathon)
- Expected load: ~100 concurrent users
- Qdrant free tier: 1GB storage (~500 chapters with embeddings)
- Neon free tier: 0.5GB storage (~10,000 chat sessions)
- OpenAI API: Pay-per-use (estimate $20-50/month for demo traffic)

### Future Enhancements
- Redis caching layer for chatbot responses
- PostgreSQL read replicas for chat history
- CDN for static assets
- Horizontal scaling of FastAPI backend (Docker + Kubernetes)

## Development Workflow

1. **Content Development**: Author chapters in `content/docs/` markdown files
2. **Embedding Generation**: Run `python scripts/generate_embeddings.py` to update Qdrant
3. **Backend Testing**: `pytest tests/ -v --cov=src`
4. **Frontend Testing**: `npm test` (Jest) + `npx playwright test` (e2e)
5. **Local Preview**: Start both servers, test integration
6. **Deployment**: Merge to `main` branch triggers CI/CD

## Monitoring and Observability

### Health Checks
- `GET /health` endpoint returns backend status
- Qdrant connection check
- Neon Postgres connection check
- OpenAI API key validation

### Logging
- FastAPI request/response logs (structured JSON)
- Error tracking with stack traces
- Chatbot query logs (for quality analysis)

### Metrics (Future)
- RAG query latency (p50, p95, p99)
- OpenAI API usage (tokens, cost)
- User engagement (chapters viewed, chatbot usage)
- Error rates by endpoint

---

**Document Version**: 1.0.0
**Last Updated**: 2025-12-27
**Maintained By**: Development Team
