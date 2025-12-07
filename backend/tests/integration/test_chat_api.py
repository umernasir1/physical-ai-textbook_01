import pytest
from fastapi.testclient import TestClient
from backend.src.main import app
from backend.src.api.v1.auth import user_repository  # Import the FakeUserRepository
from backend.src.services.auth import create_access_token
from datetime import timedelta

client = TestClient(app)


@pytest.fixture(autouse=True)
def clean_and_mock_repos():
    """
    Cleans the fake user repository before each test.
    """
    user_repository.fake_users_db.clear()
    yield


def get_auth_header(email: str, password: str, profile_data: dict = None):
    """
    Helper function to signup a user and return their auth header.
    """
    client.post(
        "/v1/auth/signup",
        json={"email": email, "password": password, "profile_data": profile_data or {}},
    )
    login_response = client.post(
        "/v1/auth/token", data={"username": email, "password": password}
    )
    token = login_response.json()["access_token"]
    return {"Authorization": f"Bearer {token}"}


def test_chat_endpoint_unauthenticated():
    """
    Test that accessing the chat endpoint without authentication fails.
    """
    response = client.post("/v1/chat/", json={"text": "Hello, chat!"})
    assert response.status_code == 401
    assert response.json()["detail"] == "Not authenticated"


def test_chat_endpoint_with_valid_token_and_personalization():
    """
    Test that accessing the chat endpoint with a valid token and user profile
    returns a personalized response.
    """
    email = "test_user@example.com"
    password = "testpassword"
    profile_data = {"experience_level": "intermediate", "hardware": "NVIDIA GPU"}
    auth_headers = get_auth_header(email, password, profile_data)

    response = client.post(
        "/v1/chat/", json={"text": "What is AI?"}, headers=auth_headers
    )

    assert response.status_code == 200
    response_data = response.json()
    assert response_data["user_email"] == email
    assert "personalization_recommendations" in response_data
    assert "response" in response_data

    # Check personalization recommendations
    recs = response_data["personalization_recommendations"]
    assert recs["content_level"] == "intermediate"
    assert recs["hardware_focus"] == "GPU"
    assert "advanced_algorithms" in recs["recommended_modules"]
    assert "practical_applications" in recs["recommended_modules"]

    # Check personalized response text
    assert "intermediate" in response_data["response"]
    assert "GPU" in response_data["response"]


def test_chat_endpoint_with_different_profile():
    """
    Test chat personalization with a different user profile (beginner, edge AI).
    """
    email = "beginner_edge@example.com"
    password = "securepassword"
    profile_data = {"experience_level": "beginner", "hardware": "Raspberry Pi"}
    auth_headers = get_auth_header(email, password, profile_data)

    response = client.post(
        "/v1/chat/", json={"text": "How to get started with AI?"}, headers=auth_headers
    )

    assert response.status_code == 200
    response_data = response.json()
    assert response_data["user_email"] == email
    recs = response_data["personalization_recommendations"]
    assert recs["content_level"] == "introductory"
    assert recs["hardware_focus"] == "Edge AI"
    assert "introduction" in recs["recommended_modules"]
    assert "edge_deployment" in recs["recommended_modules"]

    assert "introductory" in response_data["response"]
    assert "Edge AI" in response_data["response"]
