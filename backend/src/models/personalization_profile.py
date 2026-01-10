from sqlalchemy import Column, String, DateTime, ForeignKey, UUID, TEXT, ARRAY, Enum
from sqlalchemy.sql import func
from sqlalchemy.orm import relationship
import uuid
import enum
from src.core.database import Base

class SoftwareBackground(str, enum.Enum):
    BEGINNER = "beginner"
    INTERMEDIATE = "intermediate"
    ADVANCED = "advanced"

class HardwareBackground(str, enum.Enum):
    NONE = "none"
    HOBBYIST = "hobbyist"
    PROFESSIONAL = "professional"

class ComplexityPreference(str, enum.Enum):
    SIMPLIFIED = "simplified"
    STANDARD = "standard"
    DETAILED = "detailed"

class PersonalizationProfile(Base):
    """
    Stores background questions for user-based content personalization.
    """
    __tablename__ = "personalization_profiles"

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    user_id = Column(UUID(as_uuid=True), ForeignKey("users.id"), unique=True, nullable=False)

    # Background fields
    software_background = Column(String(50), nullable=False, default=SoftwareBackground.BEGINNER)
    hardware_background = Column(String(50), nullable=False, default=HardwareBackground.NONE)
    learning_goals = Column(ARRAY(TEXT), nullable=True)
    preferred_complexity = Column(String(50), nullable=False, default=ComplexityPreference.STANDARD)

    created_at = Column(DateTime(timezone=True), server_default=func.now())
    updated_at = Column(DateTime(timezone=True), onupdate=func.now(), server_default=func.now())

    # Relationships
    user = relationship("User", back_populates="profile")

    def __repr__(self):
        return f"<PersonalizationProfile(user_id={self.user_id}, software={self.software_background})>"
