import pytest
from uuid import UUID, uuid4
from datetime import datetime
from pydantic import ValidationError

from backend.src.models.user import User


def test_user_model_valid_instantiation():
    """
    Test that a User model can be instantiated with valid data.
    """
    user_data = {
        "email": "test@example.com",
        "hashed_password": "supersecretpasswordhash",
        "profile_data": {"experience_level": "intermediate"},
    }
    user = User(**user_data)

    assert isinstance(user.user_id, UUID)
    assert user.email == "test@example.com"
    assert user.hashed_password == "supersecretpasswordhash"
    assert user.profile_data == {"experience_level": "intermediate"}
    assert isinstance(user.created_at, datetime)


def test_user_model_default_values():
    """
    Test that default values are correctly assigned when not provided.
    """
    user_data = {
        "email": "default@example.com",
        "hashed_password": "defaultpasswordhash",
    }
    user = User(**user_data)

    assert isinstance(user.user_id, UUID)
    assert user.profile_data == {}  # Should be default empty dict
    assert isinstance(user.created_at, datetime)


def test_user_model_invalid_email():
    """
    Test that instantiation fails with an invalid email format.
    """
    user_data = {"email": "invalid-email", "hashed_password": "somehash"}
    with pytest.raises(ValidationError):
        User(**user_data)


def test_user_model_missing_required_fields():
    """
    Test that instantiation fails if required fields are missing.
    """
    with pytest.raises(ValidationError):
        User(email="missing@example.com")  # Missing hashed_password

    with pytest.raises(ValidationError):
        User(hashed_password="somehash")  # Missing email
