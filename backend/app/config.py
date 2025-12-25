from pydantic_settings import BaseSettings
from pydantic import ConfigDict
from typing import Optional


class Settings(BaseSettings):
    # Cohere Configuration
    COHERE_API_KEY: str
    COHERE_GENERATION_MODEL: str = "command-r-plus"
    COHERE_EMBEDDING_MODEL: str = "command-r"

    # Qdrant Configuration
    QDRANT_URL: str
    QDRANT_API_KEY: str
    QDRANT_COLLECTION_NAME: str = "book_content_chunks"

    # Neon Postgres Configuration
    NEON_DATABASE_URL: str

    # Application Configuration
    APP_ENV: str = "development"
    LOG_LEVEL: str = "info"
    MAX_SESSION_AGE_HOURS: int = 24
    DEFAULT_TIMEOUT_SECONDS: int = 5
    MAX_QUERY_LENGTH: int = 1000
    MAX_SELECTED_TEXT_LENGTH: int = 5000

    # API Configuration
    API_PREFIX: str = "/api/v1"
    ALLOWED_ORIGINS: str = "*"  # In production, be more specific

    model_config = ConfigDict(env_file=".env", case_sensitive=True)


settings = Settings()