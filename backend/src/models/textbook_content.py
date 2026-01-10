from sqlalchemy import Column, String, Integer, DateTime, UUID, TEXT, ARRAY
from sqlalchemy.sql import func
from src.core.database import Base
import uuid

class TextbookContent(Base):
    """
    Stores metadata and content for textbook chapters used in RAG.
    Connects with the vector store via embedding_id.
    """
    __tablename__ = "textbook_content"

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    module_id = Column(Integer, nullable=False, index=True)
    chapter_id = Column(Integer, nullable=False, index=True)
    section_title = Column(String(255), nullable=False)
    content = Column(TEXT, nullable=False)
    embedding_id = Column(String(100), nullable=False, unique=True, index=True)

    # Optional metadata
    content_type = Column(String(50), nullable=True) # introduction, core, exercise, assessment
    learning_outcomes = Column(ARRAY(String), nullable=True)

    created_at = Column(DateTime(timezone=True), server_default=func.now())

    def __repr__(self):
        return f"<TextbookContent(chapter={self.chapter_id}, section='{self.section_title}')>"
