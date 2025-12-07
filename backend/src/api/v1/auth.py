from datetime import timedelta
from typing import Optional
from uuid import UUID

from fastapi import APIRouter, Depends, HTTPException, status
from fastapi.security import OAuth2PasswordBearer, OAuth2PasswordRequestForm
from pydantic import BaseModel, EmailStr
from ...core.config import settings
from ...models.user import User
from ...services.auth import (
    verify_password,
    create_access_token,
    get_user,
)
from ...services.neon_db import (
    get_db_connection,
)  # Assuming this will be used for persistence

router = APIRouter()

oauth2_scheme = OAuth2PasswordBearer(tokenUrl="v1/auth/token")


class Token(BaseModel):
    access_token: str
    token_type: str


class UserCreate(BaseModel):
    email: EmailStr
    password: str
    profile_data: Optional[dict] = None


class UserInDB(User):
    hashed_password: str


# Placeholder for database operations
# In a real application, this would interact with a database
# to store and retrieve user data.
class FakeUserRepository:
    fake_users_db = {}  # In-memory store for demonstration

    async def get_user_by_email(self, email: str) -> Optional[UserInDB]:
        user_data = self.fake_users_db.get(email)
        if user_data:
            return UserInDB(**user_data)
        return None

    async def create_user(self, user: User) -> User:
        user_in_db = UserInDB(
            **user.model_dump(), hashed_password=get_password_hash(user.password)
        )
        self.fake_users_db[user.email] = user_in_db.model_dump()
        return user_in_db


user_repository = FakeUserRepository()


@router.post("/signup", response_model=Token)
async def signup(user_create: UserCreate):
    db = get_db_connection()  # Placeholder for actual DB session
    # For now, just a print to show db connection is obtained.
    # In a real app, you'd pass this db session to the user_repository
    # and perform operations.
    print(f"Obtained DB connection: {db}")
    if db:
        db.close()

    existing_user = await user_repository.get_user_by_email(user_create.email)
    if existing_user:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail="Email already registered",
        )

    # Hash the password and store the user
    hashed_password = get_password_hash(user_create.password)
    new_user = User(
        email=user_create.email,
        hashed_password=hashed_password,
        profile_data=user_create.profile_data or {},
    )
    # The `User` model currently doesn't have a `password` field,
    # but `UserCreate` does. For the sake of demonstration, I'm passing
    # `user_create.password` to `get_password_hash`.
    # In a real scenario, the `User` model might have `password` field to hash before saving,
    # or the `UserCreate` model would be directly used to hash.
    # To simplify, I'll update the UserCreate and UserInDB models a bit
    # UserCreate will now also have the hashed password.

    # Corrected User creation for FakeUserRepository
    user_data_for_repo = user_create.model_dump()
    user_data_for_repo["hashed_password"] = hashed_password
    del user_data_for_repo[
        "password"
    ]  # Remove plain password before passing to UserInDB

    user_in_db = UserInDB(**user_data_for_repo)  # Pass the data to UserInDB directly
    user_repository.fake_users_db[user_in_db.email] = user_in_db.model_dump()

    access_token_expires = timedelta(minutes=settings.ACCESS_TOKEN_EXPIRE_MINUTES)
    access_token = create_access_token(
        data={"sub": new_user.email}, expires_delta=access_token_expires
    )
    return {"access_token": access_token, "token_type": "bearer"}


@router.post("/token", response_model=Token)
async def login_for_access_token(form_data: OAuth2PasswordRequestForm = Depends()):
    db = get_db_connection()  # Placeholder
    if db:
        db.close()

    user = await user_repository.get_user_by_email(form_data.username)
    if not user or not verify_password(form_data.password, user.hashed_password):
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Incorrect email or password",
            headers={"WWW-Authenticate": "Bearer"},
        )
    access_token_expires = timedelta(minutes=settings.ACCESS_TOKEN_EXPIRE_MINUTES)
    access_token = create_access_token(
        data={"sub": user.email}, expires_delta=access_token_expires
    )
    return {"access_token": access_token, "token_type": "bearer"}


# Dependency to get the current user from the token
async def get_current_user(token: str = Depends(oauth2_scheme)):
    credentials_exception = HTTPException(
        status_code=status.HTTP_401_UNAUTHORIZED,
        detail="Could not validate credentials",
        headers={"WWW-Authenticate": "Bearer"},
    )
    try:
        payload = create_access_token(
            data={"sub": "dummy"}
        )  # Use create_access_token to get a payload structure, but it's not decoding
        # This is a mistake, should use decode_access_token
        # Corrected:
        from ...services.auth import decode_access_token

        payload = decode_access_token(token)
        if payload is None:
            raise credentials_exception
        email: str = payload.get("sub")
        if email is None:
            raise credentials_exception
    except Exception:
        raise credentials_exception
    user = await user_repository.get_user_by_email(email)
    if user is None:
        raise credentials_exception
    return user
