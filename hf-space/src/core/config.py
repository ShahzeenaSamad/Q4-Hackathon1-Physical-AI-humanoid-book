"""
Configuration settings loaded from environment variables
Uses pydantic-settings for type validation
"""

from pydantic_settings import BaseSettings
from typing import Optional


class Settings(BaseSettings):
    """Application settings from environment variables"""

    # AI API Configuration
    COHERE_API_KEY: str
    OPENROUTER_API_KEY: Optional[str] = None

    # Database Configuration
    NEON_DATABASE_URL: str

    # Qdrant Vector Store Configuration
    QDRANT_API_KEY: str
    QDRANT_CLUSTER_URL: str

    # Authentication (Bonus Feature)
    JWT_SECRET: Optional[str] = None
    JWT_ALGORITHM: str = "HS256"
    JWT_EXPIRATION_HOURS: int = 24

    # Application Configuration
    ENVIRONMENT: str = "development"
    FRONTEND_URL: str = "http://localhost:3000"
    BACKEND_URL: str = "http://localhost:8000"

    # RAG Configuration
    COMPLETION_MODEL: str = "command-r-plus-08-2024"
    EMBEDDING_MODEL: str = "embed-english-v3.0"
    EMBEDDING_DIMENSION: int = 1024
    TOP_K_RESULTS: int = 5

    class Config:
        # env_file = "../.env"  # .env is in project root, not backend folder
        case_sensitive = True


# Global settings instance
settings = Settings()
