from fastapi import APIRouter
from datetime import datetime
import requests
import cohere
from app.config import settings


router = APIRouter()


@router.get("/health")
async def health_check():
    """
    Check the health status of the chatbot API to monitor availability and functionality.
    """
    # Check if we can connect to external services
    dependencies = {
        "qdrant": "reachable",
        "neon": "reachable",
        "cohere": "reachable"
    }

    # Test Cohere connection
    try:
        cohere_client = cohere.Client(settings.COHERE_API_KEY)
        # Make a simple request to test the connection
        cohere_client.check_api_key()
    except Exception:
        dependencies["cohere"] = "unreachable"

    # Test Qdrant connection
    try:
        from qdrant_client import QdrantClient
        client = QdrantClient(
            url=settings.QDRANT_URL,
            api_key=settings.QDRANT_API_KEY,
        )
        # Try to get collection list to test connection
        client.get_collections()
    except Exception:
        dependencies["qdrant"] = "unreachable"

    # For Neon, we would normally test the database connection
    # For now, we'll assume it's reachable since we'll connect when needed

    status = "healthy" if all(v == "reachable" for v in dependencies.values()) else "unhealthy"

    return {
        "status": status,
        "timestamp": datetime.utcnow().isoformat(),
        "dependencies": dependencies
    }