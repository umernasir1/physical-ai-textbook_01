import pytest
from fastapi.testclient import TestClient
from backend.src.main import app

client = TestClient(app)


def test_translation_endpoint_success():
    """
    Test that the translation endpoint successfully translates text.
    """
    response = client.post(
        "/v1/translation/translate",
        json={"text": "Hello world", "target_language": "ur"},
    )
    assert response.status_code == 200
    data = response.json()
    assert "translated_text" in data
    assert (
        "ہیلو، یہ ایک نقلی ترجمہ ہے" in data["translated_text"]
    )  # Check for Urdu part of mock response


def test_translation_endpoint_unsupported_language():
    """
    Test that the translation endpoint handles unsupported languages gracefully.
    """
    response = client.post(
        "/v1/translation/translate",
        json={
            "text": "Hello world",
            "target_language": "xyz",
        },  # 'xyz' is unsupported in mock
    )
    assert response.status_code == 200
    data = response.json()
    assert "translated_text" in data
    assert "[xyz translation of: 'Hello world']" in data["translated_text"]


def test_translation_endpoint_invalid_request_body_missing_text():
    """
    Test that the translation endpoint returns an error for a missing 'text' field.
    """
    response = client.post("/v1/translation/translate", json={"target_language": "es"})
    assert (
        response.status_code == 422
    )  # Unprocessable Entity for Pydantic validation error


def test_translation_endpoint_invalid_request_body_missing_language():
    """
    Test that the translation endpoint returns an error for a missing 'target_language' field.
    """
    response = client.post("/v1/translation/translate", json={"text": "Hello world"})
    assert (
        response.status_code == 422
    )  # Unprocessable Entity for Pydantic validation error
