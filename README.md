# Physical AI & Humanoid Robotics Textbook

**A comprehensive interactive textbook for learning Physical AI, ROS 2, Gazebo, NVIDIA Isaac, and Vision-Language-Action (VLA) systems**

[![Hackathon Project](https://img.shields.io/badge/Hackathon-Panaversity-blue)]()
[![Base Points](https://img.shields.io/badge/Base-100%20pts-green)]()
[![Bonus Points](https://img.shields.io/badge/Bonus-200%20pts-orange)]()

## 🎯 Project Overview

This project delivers an AI-native technical textbook covering a 13-week Physical AI & Humanoid Robotics course, built with:
- **Docusaurus 3.x** - Interactive documentation site
- **RAG Chatbot** - OpenAI-powered Q&A with chapter citations
- **FastAPI Backend** - Vector search and AI services
- **Bonus Features** - Authentication, personalization, Urdu translation

### Hackathon Requirements

**Base Deliverable (100 points)**:
✅ Complete textbook with 4 modules (24 chapters)
✅ Deployed Docusaurus site
✅ Functional RAG chatbot (OpenAI + Qdrant + Neon Postgres)

**Bonus Features (200 additional points)**:
- 🔐 Authentication (50 pts) - better-auth with user background questions
- 🎨 Personalization (50 pts) - Content adapted to user experience level
- 🌐 Urdu Translation (50 pts) - Chapter translation with technical term preservation
- 🤖 Claude Code Agent Skills (50 pts) - Reusable subagents for content generation

---

## 📚 Course Structure

### Module 1: ROS 2 Fundamentals (Weeks 1-5)
8 chapters covering ROS 2 architecture, nodes, topics, services, actions, launch files, URDF, and best practices

### Module 2: Gazebo & Unity Simulation (Weeks 6-7)
5 chapters on robot simulation, SDF world building, sensor simulation, and Unity integration

### Module 3: NVIDIA Isaac Platform (Weeks 8-10)
5 chapters covering Isaac Sim, Isaac ROS, VSLAM, and Nav2 navigation

### Module 4: Vision-Language-Action (Weeks 11-13)
6 chapters on VLA models, Whisper voice-to-action, LLM cognitive planning, multimodal interaction, and capstone project

**Total**: 24 chapters + 5 appendices + instructor guide

---

## 🚀 Quick Start

### Prerequisites

- Node.js 18+ and npm
- Python 3.10+
- Git
- (Optional) Docker and Docker Compose

### Local Development Setup

1. **Clone the repository**
   ```bash
   git clone <repository-url>
   cd "Hackathon 1"
   ```

2. **Set up environment variables**
   ```bash
   cp .env.example .env
   # Edit .env with your API keys:
   # - OPENAI_API_KEY
   # - NEON_DATABASE_URL
   # - QDRANT_API_KEY, QDRANT_CLUSTER_URL
   ```

3. **Backend Setup**
   ```bash
   cd backend
   python -m venv venv
   source venv/bin/activate  # Windows: venv\Scripts\activate
   pip install -r requirements.txt

   # Run database migrations
   alembic upgrade head

   # Generate embeddings (one-time)
   python scripts/generate_embeddings.py

   # Start FastAPI server
   uvicorn src.main:app --reload --port 8000
   ```

4. **Frontend Setup**
   ```bash
   cd frontend
   npm install

   # Start Docusaurus dev server
   npm start
   # Opens http://localhost:3000
   ```

5. **Verify Setup**
   - Frontend: http://localhost:3000
   - Backend API: http://localhost:8000/docs (Swagger UI)
   - Health check: http://localhost:8000/health

---

## 🧪 Running Tests

### Backend Tests
```bash
cd backend
pytest tests/ -v --cov=src --cov-report=term
```

### Frontend Tests
```bash
cd frontend
npm test
```

### End-to-End Tests
```bash
npx playwright test
```

---

## 🏗️ Project Architecture

```
Hackathon 1/
├── content/docs/          # Markdown chapters (24 chapters + appendices)
│   ├── module-1-ros2/
│   ├── module-2-simulation/
│   ├── module-3-isaac/
│   ├── module-4-vla/
│   └── appendices/
├── frontend/              # Docusaurus React application
│   ├── src/
│   │   ├── components/    # ChatbotWidget, Auth, Personalization
│   │   ├── pages/
│   │   └── css/
│   ├── docusaurus.config.js
│   └── sidebars.js
├── backend/               # FastAPI RAG service
│   ├── src/
│   │   ├── api/           # Chat, auth, personalization endpoints
│   │   ├── models/        # SQLAlchemy models
│   │   ├── services/      # Embedding, RAG, vector store
│   │   └── core/          # Config, database, security
│   ├── migrations/        # Alembic database migrations
│   └── tests/
├── static/
│   ├── img/               # Diagrams and screenshots
│   └── code-examples/     # Executable code samples
└── scripts/               # Utility scripts (embeddings, validation)
```

---

## 📖 Key Features

### 1. Interactive Content
- 24 chapters with progressive learning outcomes
- Hands-on exercises for each chapter
- 2-5 code examples per chapter
- Assessment questions with answer keys

### 2. RAG Chatbot
- Ask questions about any textbook content
- AI responses cite specific chapters and sections
- Selected-text queries for contextual help
- Chat history persistence (with authentication)

### 3. Bonus: Authentication
- Signup with user background questions (software/hardware experience)
- JWT-based session management
- Protected chat history

### 4. Bonus: Content Personalization
- Rewrite chapters based on user background (beginner/intermediate/advanced)
- Adaptive complexity and explanations
- Cached personalized versions

### 5. Bonus: Urdu Translation
- On-demand chapter translation
- Technical terms preserved in English (ROS 2, URDF, VSLAM, etc.)
- Cached translations

---

## 🎓 Target Audience

- **Students**: Intermediate programming skills (Python basics, command-line familiarity)
- **Instructors**: Planning 13-week Physical AI courses
- **Self-Paced Learners**: Skipping to advanced topics with clear prerequisites
- **Prerequisites**: Python basics, Linux command-line, basic linear algebra

---

## 🛠️ Technology Stack

**Frontend**:
- Docusaurus 3.x (React 18+, TypeScript)
- better-auth (authentication)
- Axios (API calls)

**Backend**:
- FastAPI (Python 3.10+)
- OpenAI Python SDK (embeddings + completions)
- Qdrant Client (vector search)
- asyncpg / psycopg2 (Neon Postgres)
- python-jose (JWT)
- LangChain (RAG orchestration)

**Infrastructure**:
- Neon Serverless Postgres (user data, chat history)
- Qdrant Cloud (vector embeddings)
- GitHub Pages / Vercel (frontend deployment)
- Railway / Fly.io / Render (backend deployment)

---

## 📦 Deployment

### Frontend (GitHub Pages)
```bash
cd frontend
npm run build
# Deploy via GitHub Actions workflow: .github/workflows/deploy-frontend.yml
```

### Backend (Railway/Fly.io)
```bash
cd backend
# Deploy via GitHub Actions workflow: .github/workflows/deploy-backend.yml
# Or manually:
docker build -t textbook-backend .
# Push to container registry and deploy
```

### Environment Variables (Production)
Set these in your deployment platform:
- `OPENAI_API_KEY`
- `NEON_DATABASE_URL`
- `QDRANT_API_KEY`, `QDRANT_CLUSTER_URL`
- `JWT_SECRET`
- `FRONTEND_URL`, `BACKEND_URL`

---

## 📝 Development Workflow

1. **Content Creation**: Use templates in `content/docs/_templates/`
2. **Validation**: Run `python scripts/validate_content.py` on chapters
3. **Embedding Generation**: Run `python scripts/generate_embeddings.py` after content updates
4. **Testing**: pytest (backend), Jest (frontend), Playwright (e2e)
5. **Deployment**: Merge to `main` triggers CI/CD workflows

---

## 🤝 Contributing

This is a hackathon project developed using:
- **Claude Code** - AI-assisted development
- **Spec-Kit Plus** - Specification-driven development methodology
- **Constitutional AI** - Adherence to project principles

For the spec-driven development workflow, see:
- `specs/001-textbook-content-structure/spec.md` - Feature specification
- `specs/001-textbook-content-structure/plan.md` - Implementation plan
- `specs/001-textbook-content-structure/tasks.md` - Task breakdown

---

## 📄 License

This project is created for the Panaversity Hackathon. All rights reserved.

---

## 👥 Team

- **Developer**: [Your Name]
- **AI Assistant**: Claude Sonnet 4.5
- **Organization**: Panaversity

---

## 🎬 Demo Video

📹 [Demo Video Link](https://youtu.be/YOUR_VIDEO_ID) (<90 seconds)

Demonstrates:
1. Textbook navigation across 4 modules
2. RAG chatbot answering questions with citations
3. Authentication signup with background questions
4. Content personalization based on user profile
5. Urdu translation feature

---

## 📞 Contact

For questions about this hackathon project:
- **Submission Form**: https://forms.gle/CQsSEGM3GeCrL43c8
- **Zoom Presentation**: Nov 30, 2025 at 6:00 PM (Meeting ID: 849 7684 7088)
- **WhatsApp**: [Your Number]

---

## 🏆 Hackathon Scoring

| Category | Points | Status |
|----------|--------|--------|
| Base Textbook + RAG Chatbot | 100 | ✅ Completed |
| Authentication (better-auth) | 50 | ⏳ In Progress |
| Content Personalization | 50 | ⏳ Planned |
| Urdu Translation | 50 | ⏳ Planned |
| Claude Code Agent Skills | 50 | ⏳ Planned |
| **Total Possible** | **300** | |

---

**Built with ❤️ using Claude Code and Spec-Kit Plus**
