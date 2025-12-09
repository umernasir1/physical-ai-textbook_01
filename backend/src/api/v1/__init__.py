from fastapi import APIRouter

router = APIRouter(prefix="/v1")

# Import and include other routers here
from . import chat, auth, translation
router.include_router(chat.router)
router.include_router(auth.router)
router.include_router(translation.router)