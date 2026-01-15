"""
Configuration settings loaded from environment variables
Uses pydantic-settings for type validation
"""

from pydantic_settings import BaseSettings
from typing import Optional


class Settings(BaseSettings):
    """Application settings from environment variables"""

    # AI API Configuration - Making these optional for Hugging Face deployment
    COHERE_API_KEY: Optional[str] = "dummy-key-for-hf-deployment"
    OPENROUTER_API_KEY: Optional[str] = None

    # Database Configuration - Making this optional for Hugging Face deployment
    NEON_DATABASE_URL: Optional[str] = None

    # Qdrant Vector Store Configuration - Making these optional for Hugging Face deployment
    QDRANT_API_KEY: Optional[str] = "dummy-key-for-hf-deployment"
    QDRANT_CLUSTER_URL: Optional[str] = "dummy-url-for-hf-deployment"

    # Authentication (Bonus Feature)
    JWT_SECRET: Optional[str] = "secret-key-for-hf-deployment"  # Default for demo
    JWT_ALGORITHM: str = "HS256"
    JWT_EXPIRATION_HOURS: int = 24

    # Application Configuration
    ENVIRONMENT: str = "production"  # Changed for Hugging Face deployment
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
