# Quickstart Guide: Physical AI Textbook Development

**Date**: 2025-12-29
**Feature**: 001-textbook-content-structure

## Prerequisites

- **Node.js**: v18+ (for Docusaurus frontend)
- **Python**: 3.10+ (for FastAPI backend)
- **Git**: Latest version
- **IDE**: VS Code recommended (with Python and TypeScript extensions)

## Environment Setup

### 1. Clone the Repository

```bash
git clone https://github.com/<your-org>/hackathon-1.git
cd hackathon-1
git checkout 001-textbook-content-structure
```

### 2. Environment Variables

Create `.env` file in project root (copy from `.env.example`):

```bash
cp .env.example .env
```

Edit `.env` with your credentials:

```env
# OpenAI (Required for RAG chatbot)
OPENAI_API_KEY=sk-your-openai-api-key

# Neon Postgres (Required for data persistence)
NEON_DATABASE_URL=postgresql://user:pass@ep-xxx.us-east-2.aws.neon.tech/textbook?sslmode=require

# Qdrant Cloud (Required for vector search)
QDRANT_API_KEY=your-qdrant-api-key
QDRANT_CLUSTER_URL=https://xxx-xxx.us-east-1-0.aws.cloud.qdrant.io:6333

# JWT Authentication (Bonus feature)
JWT_SECRET=your-super-secret-jwt-key-min-32-chars
JWT_ALGORITHM=HS256
JWT_EXPIRE_MINUTES=60

# Frontend URL (for CORS)
FRONTEND_URL=http://localhost:3000
```

### 3. Backend Setup

```bash
# Create Python virtual environment
cd backend
python -m venv venv

# Activate virtual environment
# Windows:
venv\Scripts\activate
# macOS/Linux:
source venv/bin/activate

# Install dependencies
pip install -r requirements.txt

# Run database migrations
alembic upgrade head

# Start development server
uvicorn src.main:app --reload --port 8000
```

Verify backend is running:
```bash
curl http://localhost:8000/api/v1/health
# Expected: {"status":"ok","service":"textbook-rag-api"}
```

### 4. Frontend Setup

```bash
# Open new terminal, navigate to frontend
cd frontend

# Install dependencies
npm install

# Start development server
npm start
```

Verify frontend is running: Open http://localhost:3000

### 5. Generate Embeddings (One-time setup)

After writing chapter content, generate vector embeddings:

```bash
cd scripts
python generate_embeddings.py
```

This processes all markdown files in `content/docs/` and uploads embeddings to Qdrant.

## Project Structure

```
hackathon-1/
├── .env                    # Environment variables (do not commit)
├── .env.example            # Template for environment variables
├── content/docs/           # Markdown textbook chapters
├── frontend/               # Docusaurus React app
├── backend/                # FastAPI Python API
├── scripts/                # Utility scripts
├── static/                 # Images and code examples
└── specs/                  # SDD documentation
```

## Development Workflow

### Writing Content

1. **Create chapter from template**:
   ```bash
   cp content/docs/_templates/chapter-template.md content/docs/module-1-ros2/03-nodes-topics.md
   ```

2. **Fill in chapter content** following the template structure

3. **Validate chapter**:
   ```bash
   python scripts/validate_content.py content/docs/module-1-ros2/03-nodes-topics.md
   ```

4. **Preview in browser**: http://localhost:3000/docs/module-1-ros2/03-nodes-topics

### Backend Development

1. **Add new endpoint**: Create file in `backend/src/api/`
2. **Register in router**: Import in `backend/src/main.py`
3. **Test endpoint**: Use `curl` or Postman
4. **Add tests**: Create in `backend/tests/`

### Running Tests

```bash
# Backend tests
cd backend
pytest tests/ -v --cov=src

# Frontend tests
cd frontend
npm test
```

## Common Tasks

### Reset Database

```bash
cd backend
alembic downgrade base
alembic upgrade head
```

### Rebuild Embeddings

```bash
cd scripts
python generate_embeddings.py --force
```

### Build for Production

```bash
# Frontend
cd frontend
npm run build

# Backend (Docker)
cd backend
docker build -t textbook-backend .
```

## Troubleshooting

### Backend won't start

- Check `.env` has all required variables
- Verify Python 3.10+ is installed
- Ensure virtual environment is activated

### Frontend build fails

- Delete `node_modules` and reinstall: `rm -rf node_modules && npm install`
- Clear Docusaurus cache: `npm run clear`

### Chatbot returns empty responses

- Verify `OPENAI_API_KEY` is valid
- Check embeddings were generated: `python scripts/generate_embeddings.py --check`
- Review Qdrant dashboard for collection status

### Database connection errors

- Verify `NEON_DATABASE_URL` is correct
- Check Neon dashboard for connection limits
- Ensure SSL mode is enabled

## API Quick Reference

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/api/v1/health` | GET | Health check |
| `/api/v1/chat` | POST | Send chat message |
| `/api/v1/chat/sessions/{id}` | GET | Get chat history |
| `/api/v1/auth/signup` | POST | User registration (bonus) |
| `/api/v1/auth/signin` | POST | User login (bonus) |
| `/api/v1/personalize/chapter` | POST | Personalize content (bonus) |
| `/api/v1/translate/chapter` | POST | Translate to Urdu (bonus) |

## Resources

- [Docusaurus Docs](https://docusaurus.io/docs)
- [FastAPI Docs](https://fastapi.tiangolo.com/)
- [Qdrant Docs](https://qdrant.tech/documentation/)
- [OpenAI API Reference](https://platform.openai.com/docs/api-reference)
- [Neon Postgres Docs](https://neon.tech/docs/)
