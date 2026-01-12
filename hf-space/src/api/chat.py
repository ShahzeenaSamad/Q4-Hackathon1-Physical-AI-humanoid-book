"""
Chat API - RAG chatbot endpoints with session persistence
"""

from fastapi import APIRouter, HTTPException, Depends
from pydantic import BaseModel, Field
from typing import List, Optional, Dict, Any
from sqlalchemy.ext.asyncio import AsyncSession
from sqlalchemy import select, delete
import uuid
from datetime import datetime

from src.services.rag_service import RAGService
from src.core.database import get_db
from src.models.chat_session import ChatSession

router = APIRouter()
rag_service = RAGService()


class ChatRequest(BaseModel):
    """Request model for chat endpoint"""
    question: str = Field(..., min_length=3, max_length=500, description="User's question")
    session_id: Optional[uuid.UUID] = Field(None, description="Existing session ID to continue conversation")
    module_filter: Optional[str] = Field(None, description="Optional module name to filter context")
    top_k: int = Field(3, ge=1, le=5, description="Number of relevant chapters to retrieve")
    selected_text: Optional[str] = Field(None, description="Provide if the user has selected specific text to query against")


class ChatResponse(BaseModel):
    """Response model for chat endpoint"""
    session_id: uuid.UUID
    answer: str
    sources: List[Dict]
    context_used: bool
    model_used: str


class RelatedTopicsRequest(BaseModel):
    """Request model for related topics endpoint"""
    question: str = Field(..., min_length=3, max_length=500)
    limit: int = Field(5, ge=1, le=10)


@router.post("/query", response_model=ChatResponse)
async def chat_query(request: ChatRequest):
    """
    Answer a question using RAG over textbook content (session persistence disabled for now)
    """
    try:
        # Get AI answer from RAG service
        result = rag_service.answer_question(
            question=request.question,
            module_filter=request.module_filter,
            top_k=request.top_k,
            selected_text=request.selected_text
        )

        # Generate a session ID (not persisted to database yet)
        session_id = request.session_id or uuid.uuid4()

        return ChatResponse(
            session_id=session_id,
            **result
        )

    except Exception as e:
        import traceback
        error_trace = traceback.format_exc()
        print(f"\n[ERROR] Chat query failed:")
        print(error_trace)
        raise HTTPException(
            status_code=500,
            detail=f"Error processing question: {str(e)}"
        )


@router.get("/sessions/{session_id}", response_model=Dict[str, Any])
async def get_chat_session(session_id: uuid.UUID, db: AsyncSession = Depends(get_db)):
    """Retrieve message history for a chat session"""
    result = await db.execute(select(ChatSession).where(ChatSession.id == session_id))
    session = result.scalar_one_or_none()

    if not session:
        raise HTTPException(status_code=404, detail="Session not found")

    return {
        "session_id": session.id,
        "messages": session.messages,
        "created_at": session.created_at,
        "updated_at": session.updated_at
    }


@router.delete("/sessions/{session_id}", status_code=204)
async def delete_chat_session(session_id: uuid.UUID, db: AsyncSession = Depends(get_db)):
    """Clear chat history for a session"""
    await db.execute(delete(ChatSession).where(ChatSession.id == session_id))
    await db.commit()
    return None


@router.post("/related-topics")
async def get_related_topics(request: RelatedTopicsRequest):
    """Suggest related chapters/topics based on a question"""
    try:
        suggestions = rag_service.suggest_related_topics(
            question=request.question,
            limit=request.limit
        )
        return {"related_topics": suggestions}
    except Exception as e:
        raise HTTPException(
            status_code=500,
            detail=f"Error finding related topics: {str(e)}"
        )


@router.get("/chapter/{chapter_id}")
async def get_chapter_info(chapter_id: str):
    """Get summary information about a specific chapter"""
    try:
        chapter = rag_service.get_chapter_summary(chapter_id)
        if chapter:
            return chapter
        else:
            raise HTTPException(
                status_code=404,
                detail=f"Chapter '{chapter_id}' not found"
            )
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(
            status_code=500,
            detail=f"Error retrieving chapter: {str(e)}"
        )


@router.get("/status")
async def get_rag_status():
    """Get RAG system status"""
    try:
        # Since qdrant_service doesn't have get_collection_info, we check the collection direct
        from src.services.qdrant_service import qdrant_service
        info = qdrant_service.client.get_collection(qdrant_service.collection_name)

        return {
            "status": "operational",
            "vector_store": {
                "name": qdrant_service.collection_name,
                "points_count": info.points_count,
                "status": info.status
            },
            "embedding_model": rag_service.cohere.embedding_model,
            "completion_model": rag_service.completion_model
        }
    except Exception as e:
        raise HTTPException(
            status_code=500,
            detail=f"Error getting system status: {str(e)}"
        )
