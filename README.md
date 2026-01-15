---
title: Physical AI & Humanoid Robotics Chatbot
emoji: 🤖
colorFrom: blue
colorTo: yellow
sdk: docker
app_port: 7860
---

# Physical AI & Humanoid Robotics Textbook Chatbot API

This is a FastAPI backend for the Physical AI & Humanoid Robotics textbook project, deployed on Hugging Face Spaces. This version focuses on the chatbot functionality only.

## Features

- RAG (Retrieval Augmented Generation) chatbot for textbook content
- Mock responses for demonstration purposes
- Content API for textbook modules

## Endpoints

- `GET /` - Root endpoint with API information
- `GET /health` - Health check endpoint
- `GET /docs` - Interactive API documentation
- `POST /api/chat/query` - Chat with the textbook content
- `GET /api/content/{module}` - Get content for a specific module
- `POST /api/query` - Query the textbook content

## Deployment Notes

This application is configured for Hugging Face Spaces deployment with:
- Dummy API keys for initialization
- Chatbot functionality only (no authentication)
- Mock responses for demonstration
- Proper app.py entry point

## Local Development

To run locally:
```bash
pip install -r requirements.txt
python -m uvicorn src.main:app --reload
```