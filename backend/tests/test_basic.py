"""
Basic tests for the RAG Chatbot API
"""
import pytest
from fastapi.testclient import TestClient
from app.main import app


client = TestClient(app)


def test_health_endpoint():
    """Test the health endpoint"""
    response = client.get("/api/v1/health")
    assert response.status_code == 200
    data = response.json()
    assert "status" in data
    assert "timestamp" in data
    assert "dependencies" in data


def test_query_endpoint_exists():
    """Test that the query endpoint exists"""
    # Send a minimal query request
    response = client.post(
        "/api/v1/query",
        json={"query": "test", "session_id": "test-session"}
    )
    # Expect either a successful response or a validation error (not a 404)
    assert response.status_code in [200, 422]  # 422 is validation error


def test_selected_text_endpoint_exists():
    """Test that the selected text query endpoint exists"""
    # Send a minimal selected text query request
    response = client.post(
        "/api/v1/selected-text-query",
        json={"query": "test", "selected_text": "test text", "session_id": "test-session"}
    )
    # Expect either a successful response or a validation error (not a 404)
    assert response.status_code in [200, 422]  # 422 is validation error