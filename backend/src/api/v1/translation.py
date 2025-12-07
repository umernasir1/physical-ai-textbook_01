from fastapi import APIRouter
from pydantic import BaseModel
from backend.src.services.translation import translate_text

router = APIRouter()


class TranslateRequest(BaseModel):
    text: str
    target_language: str


class TranslateResponse(BaseModel):
    translated_text: str


@router.post("/translate", response_model=TranslateResponse)
async def translate_content(request: TranslateRequest):
    """
    Endpoint to translate text using the translation service.
    """
    translated = await translate_text(request.text, request.target_language)
    return TranslateResponse(translated_text=translated["translated_text"])


@router.get("/")
async def read_translation_status():
    return {"message": "Translation endpoint is ready."}
