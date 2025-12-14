# src/api/v1/translation.py

from fastapi import APIRouter, HTTPException, Request
from pydantic import BaseModel, ValidationError
from ...services.translation import translate_text  # absolute import
import logging

logger = logging.getLogger(__name__)
router = APIRouter()


class TranslationRequest(BaseModel):
    content: str
    target_language: str


@router.post("/translate")
async def translate_text_endpoint(raw_request: Request):
    try:
        # Get raw body for debugging
        body = await raw_request.json()
        logger.info(f"Received translation request: {body}")

        # Validate with Pydantic
        request = TranslationRequest(**body)

        # Perform translation
        result = await translate_text(request.content, request.target_language)
        return result
    except ValidationError as e:
        logger.error(f"Validation error: {e}")
        raise HTTPException(status_code=422, detail=str(e))
    except Exception as e:
        logger.error(f"Translation error: {e}")
        raise HTTPException(status_code=500, detail=str(e))
