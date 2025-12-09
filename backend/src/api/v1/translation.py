# src/api/v1/translation.py

from fastapi import APIRouter, HTTPException
from ...services.translation import translate_text  # absolute import

router = APIRouter()

@router.post("/translate")
async def translate_text_endpoint(text: str, target_language: str):
    try:
        result = await translate_text(text, target_language)
        return result
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))
