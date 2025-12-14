from datetime import datetime, timedelta
from typing import Optional

from jose import jwt, JWTError
from passlib.context import CryptContext

from ..core.config import settings

# Password hashing configuration
# Using Argon2 - more secure and modern than bcrypt
pwd_context = CryptContext(schemes=["argon2"], deprecated="auto")

# JWT configuration
ALGORITHM = "HS256"
ACCESS_TOKEN_EXPIRE_MINUTES = 30  # For example, tokens expire in 30 minutes


def verify_password(plain_password: str, hashed_password: str) -> bool:
    """
    Verifies a plain password against a hashed password.
    """
    return pwd_context.verify(plain_password, hashed_password)


def get_password_hash(password: str) -> str:
    """
    Generates a hash for the given password.
    """
    return pwd_context.hash(password)


def create_access_token(data: dict, expires_delta: Optional[timedelta] = None):
    """
    Creates a JWT access token.
    """
    to_encode = data.copy()
    if expires_delta:
        expire = datetime.utcnow() + expires_delta
    else:
        expire = datetime.utcnow() + timedelta(minutes=ACCESS_TOKEN_EXPIRE_MINUTES)
    to_encode.update({"exp": expire})
    encoded_jwt = jwt.encode(to_encode, settings.SECRET_KEY, algorithm=ALGORITHM)
    return encoded_jwt


def decode_access_token(token: str) -> Optional[dict]:
    """
    Decodes and verifies a JWT access token.
    """
    try:
        payload = jwt.decode(token, settings.SECRET_KEY, algorithms=[ALGORITHM])
        return payload
    except JWTError:
        return None
def get_user(username: str):
    """
    Retrieves a user from the database by username.
    You need to implement this based on your database model.
    """
    # TODO: Replace with your actual database query
    # Example for SQLAlchemy:
    # from ..models.user import User
    # from ..db.session import SessionLocal
    # 
    # db = SessionLocal()
    # user = db.query(User).filter(User.username == username).first()
    # db.close()
    # return user
    
    # For now, return a placeholder
    return None