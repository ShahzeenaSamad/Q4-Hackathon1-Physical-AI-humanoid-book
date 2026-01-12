---
title: Physical AI Textbook API
sdk: docker
app_port: 7860
---

# Physical AI Textbook Backend

FastAPI backend for the Physical AI & Humanoid Robotics Textbook project.

## Deployment to Hugging Face Spaces

This backend can be deployed to Hugging Face Spaces using Docker.

### Requirements
- `requirements.txt` - Python dependencies
- `Dockerfile` - Container configuration
- `app.py` - Application entry point

### Environment Variables
Set these secrets in your Hugging Face Space:
- `DATABASE_URL` - Database connection string
- `QDRANT_URL` - Qdrant vector database URL
- `OPENAI_API_KEY` - OpenAI API key (if using OpenAI)
- `COHERE_API_KEY` - Cohere API key (if using Cohere)
- `OPENROUTER_API_KEY` - OpenRouter API key (if using OpenRouter)

### Local Development
```bash
pip install -r requirements.txt
uvicorn src.main:app --reload --port 8000
```