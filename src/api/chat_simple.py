"""
Simple Chat API - No database, just RAG
"""

from fastapi import APIRouter, HTTPException
from pydantic import BaseModel
from typing import List, Dict, Optional
import uuid

router = APIRouter()

class ChatRequest(BaseModel):
    question: str
    top_k: int = 3

class ChatResponse(BaseModel):
    session_id: str
    answer: str
    sources: List[Dict]
    context_used: bool
    model_used: str
    
    model_config = {"protected_namespaces": ()}

@router.post("/query-simple", response_model=ChatResponse)
async def chat_query_simple(request: ChatRequest):
    """Simple chatbot without database - just RAG"""
    try:
        from src.services.rag_service import RAGService

        rag = RAGService()
        result = rag.answer_question(
            question=request.question,
            top_k=request.top_k
        )

        return ChatResponse(
            session_id=str(uuid.uuid4()),
            **result
        )
    except Exception as e:
        import traceback
        print(f"\n[ERROR] Simple chat failed:")
        traceback.print_exc()
        raise HTTPException(
            status_code=500,
            detail=f"Error: {str(e)}"
        )
