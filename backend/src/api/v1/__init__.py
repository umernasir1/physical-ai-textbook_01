from fastapi import APIRouter
from . import chat
from . import auth
from . import translation

api_router = APIRouter()

api_router.include_router(chat.router, prefix="/chat", tags=["chat"])
api_router.include_router(auth.router, prefix="/auth", tags=["auth"])
api_router.include_router(
    translation.router, prefix="/translation", tags=["translation"]
)
