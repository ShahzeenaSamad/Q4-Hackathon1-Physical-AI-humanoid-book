from sqlalchemy import Column, String, DateTime, ForeignKey, UUID
from sqlalchemy.dialects.postgresql import JSONB
from sqlalchemy.sql import func
import uuid
from src.core.database import Base

class ChatSession(Base):
    """
    Stores chat history for RAG chatbot sessions.
    Each session contains a list of messages.
    """
    __tablename__ = "chat_sessions"

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    user_id = Column(UUID(as_uuid=True), nullable=True, index=True) # Optional for now until auth is fully integrated
    messages = Column(JSONB, nullable=False, server_default='[]')

    created_at = Column(DateTime(timezone=True), server_default=func.now())
    updated_at = Column(DateTime(timezone=True), onupdate=func.now(), server_default=func.now())

    def __repr__(self):
        return f"<ChatSession(id={self.id}, user_id={self.user_id}, created_at={self.created_at})>"
