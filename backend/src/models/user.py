from datetime import datetime
from typing import Dict, Any
from uuid import UUID, uuid4

from pydantic import BaseModel, Field


class User(BaseModel):
    user_id: UUID = Field(default_factory=uuid4)
    email: str
    hashed_password: str
    profile_data: Dict[str, Any] = Field(default_factory=dict)
    created_at: datetime = Field(default_factory=datetime.utcnow)

    class Config:
        json_schema_extra = {
            "example": {
                "user_id": "123e4567-e89b-12d3-a456-426614174000",
                "email": "test@example.com",
                "hashed_password": "supersecurehash",
                "profile_data": {
                    "hardware": "rtx_4090",
                    "experience_level": "beginner",
                },
                "created_at": "2023-01-01T12:00:00Z",
            }
        }
