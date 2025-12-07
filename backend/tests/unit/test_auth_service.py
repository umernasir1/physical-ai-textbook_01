import pytest
from datetime import datetime, timedelta
from unittest.mock import patch

from jose import jwt, JWTError  # Used for direct decoding in tests
from backend.src.services.auth import (
    verify_password,
    get_password_hash,
    create_access_token,
    decode_access_token,
    ALGORITHM,
    ACCESS_TOKEN_EXPIRE_MINUTES,
)
from backend.src.core.config import settings


# Mock the SECRET_KEY for consistent testing
@pytest.fixture(autouse=True)
def mock_settings_secret_key():
    with patch.object(settings, "SECRET_KEY", "super-secret-test-key") as mock_key:
        yield mock_key
    with patch.object(
        settings, "ACCESS_TOKEN_EXPIRE_MINUTES", 1
    ) as mock_expire:  # Shorten for testing expiry
        yield mock_expire


def test_get_password_hash():
    """
    Test that get_password_hash returns a valid hash.
    """
    password = "testpassword"
    hashed_password = get_password_hash(password)
    assert hashed_password is not None
    assert isinstance(hashed_password, str)
    assert len(hashed_password) > 0
    assert hashed_password != password  # Hash should not be the same as plain password


def test_verify_password_correct():
    """
    Test that verify_password returns True for a correct password.
    """
    password = "testpassword"
    hashed_password = get_password_hash(password)
    assert verify_password(password, hashed_password) is True


def test_verify_password_incorrect():
    """
    Test that verify_password returns False for an incorrect password.
    """
    password = "testpassword"
    hashed_password = get_password_hash(password)
    assert verify_password("wrongpassword", hashed_password) is False


def test_create_access_token_default_expiry():
    """
    Test that create_access_token creates a token with default expiry.
    """
    data = {"sub": "test@example.com"}
    token = create_access_token(data)
    assert token is not None
    assert isinstance(token, str)

    # Decode to verify contents and expiry
    decoded_payload = jwt.decode(token, settings.SECRET_KEY, algorithms=[ALGORITHM])
    assert decoded_payload["sub"] == "test@example.com"

    # Check that expiry is within a reasonable range (e.g., around ACCESS_TOKEN_EXPIRE_MINUTES)
    # Allow for slight time differences in test execution
    expected_exp_min = (
        datetime.utcnow() + timedelta(minutes=ACCESS_TOKEN_EXPIRE_MINUTES - 1)
    ).timestamp()
    expected_exp_max = (
        datetime.utcnow() + timedelta(minutes=ACCESS_TOKEN_EXPIRE_MINUTES + 1)
    ).timestamp()
    assert expected_exp_min < decoded_payload["exp"] < expected_exp_max


def test_create_access_token_custom_expiry():
    """
    Test that create_access_token creates a token with custom expiry.
    """
    data = {"sub": "custom@example.com"}
    expires_delta = timedelta(minutes=5)
    token = create_access_token(data, expires_delta=expires_delta)
    assert token is not None

    decoded_payload = jwt.decode(token, settings.SECRET_KEY, algorithms=[ALGORITHM])
    assert decoded_payload["sub"] == "custom@example.com"

    # Check expiry is around 5 minutes from now
    expected_exp_min = (datetime.utcnow() + timedelta(minutes=4)).timestamp()
    expected_exp_max = (datetime.utcnow() + timedelta(minutes=6)).timestamp()
    assert expected_exp_min < decoded_payload["exp"] < expected_exp_max


def test_decode_access_token_valid():
    """
    Test that decode_access_token successfully decodes a valid token.
    """
    data = {"sub": "valid@example.com"}
    token = create_access_token(
        data, expires_delta=timedelta(minutes=1)
    )  # Short expiry for test
    payload = decode_access_token(token)
    assert payload is not None
    assert payload["sub"] == "valid@example.com"


def test_decode_access_token_invalid_signature():
    """
    Test that decode_access_token returns None for a token with invalid signature.
    """
    # Create a token with a valid payload but signed with a different key
    invalid_secret_key = "wrong-secret-key"
    to_encode = {
        "sub": "hacker@example.com",
        "exp": (datetime.utcnow() + timedelta(minutes=1)).timestamp(),
    }
    invalid_token = jwt.encode(to_encode, invalid_secret_key, algorithm=ALGORITHM)

    payload = decode_access_token(invalid_token)
    assert payload is None


def test_decode_access_token_expired():
    """
    Test that decode_access_token returns None for an expired token.
    """
    data = {"sub": "expired@example.com"}

    # Manually create an expired token
    to_encode = data.copy()
    expire_time = datetime.utcnow() - timedelta(minutes=1)  # 1 minute in the past
    to_encode.update({"exp": expire_time.timestamp()})

    expired_token = jwt.encode(to_encode, settings.SECRET_KEY, algorithm=ALGORITHM)

    payload = decode_access_token(expired_token)
    assert payload is None


def test_decode_access_token_malformed():
    """
    Test that decode_access_token returns None for a malformed token.
    """
    malformed_token = "malformed.token.string"
    payload = decode_access_token(malformed_token)
    assert payload is None
