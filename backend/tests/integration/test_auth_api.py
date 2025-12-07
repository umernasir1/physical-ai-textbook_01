import pytest
from fastapi.testclient import TestClient
from backend.src.main import app  # Assuming main.py is in backend/src

client = TestClient(app)


# Use a fixture to ensure a clean state for the FakeUserRepository for each test
@pytest.fixture(autouse=True)
def clean_fake_user_repository():
    # This assumes FakeUserRepository is a global instance or can be reset
    # For a more robust solution, dependency injection should be used for the repository
    from backend.src.api.v1.auth import user_repository

    user_repository.fake_users_db.clear()
    yield


def test_signup_success():
    response = client.post(
        "/v1/auth/signup",  # Updated path to /v1/auth/signup
        json={
            "email": "test@example.com",
            "password": "testpassword",
            "profile_data": {"level": "beginner"},
        },
    )
    assert response.status_code == 200
    data = response.json()
    assert "access_token" in data
    assert data["token_type"] == "bearer"


def test_signup_duplicate_email():
    client.post(
        "/v1/auth/signup",
        json={"email": "test@example.com", "password": "testpassword"},
    )
    response = client.post(
        "/v1/auth/signup",
        json={"email": "test@example.com", "password": "anotherpassword"},
    )
    assert response.status_code == 400
    assert response.json()["detail"] == "Email already registered"


def test_login_success():
    # First signup a user
    client.post(
        "/v1/auth/signup",
        json={"email": "test@example.com", "password": "testpassword"},
    )
    # Then attempt to login
    response = client.post(
        "/v1/auth/token",  # Updated path to /v1/auth/token for OAuth2PasswordRequestForm
        data={"username": "test@example.com", "password": "testpassword"},
    )
    assert response.status_code == 200
    data = response.json()
    assert "access_token" in data
    assert data["token_type"] == "bearer"


def test_login_invalid_credentials():
    # First signup a user
    client.post(
        "/v1/auth/signup",
        json={"email": "test@example.com", "password": "testpassword"},
    )
    # Attempt to login with wrong password
    response = client.post(
        "/v1/auth/token",
        data={"username": "test@example.com", "password": "wrongpassword"},
    )
    assert response.status_code == 401
    assert response.json()["detail"] == "Incorrect email or password"


def test_login_non_existent_user():
    # Attempt to login with a user that doesn't exist
    response = client.post(
        "/v1/auth/token",
        data={"username": "nonexistent@example.com", "password": "testpassword"},
    )
    assert response.status_code == 401
    assert response.json()["detail"] == "Incorrect email or password"


def test_protected_route_with_valid_token():
    # Signup and login to get a valid token
    client.post(
        "/v1/auth/signup",
        json={"email": "protected@example.com", "password": "securepassword"},
    )
    login_response = client.post(
        "/v1/auth/token",
        data={"username": "protected@example.com", "password": "securepassword"},
    )
    token = login_response.json()["access_token"]

    # Access a protected route (e.g., chat endpoint)
    response = client.post(
        "/v1/chat/",
        json={"text": "Hello, chat!"},
        headers={"Authorization": f"Bearer {token}"},
    )
    assert response.status_code == 200
    assert "user_email" in response.json()
    assert response.json()["user_email"] == "protected@example.com"


def test_protected_route_without_token():
    # Attempt to access a protected route without a token
    response = client.post("/v1/chat/", json={"text": "Hello, chat!"})
    assert response.status_code == 401
    assert (
        response.json()["detail"] == "Not authenticated"
    )  # This is the default from FastAPI's OAuth2PasswordBearer


def test_protected_route_with_invalid_token():
    # Attempt to access a protected route with an invalid token
    response = client.post(
        "/v1/chat/",
        json={"text": "Hello, chat!"},
        headers={"Authorization": "Bearer invalid_token"},
    )
    assert response.status_code == 401
    assert response.json()["detail"] == "Could not validate credentials"
