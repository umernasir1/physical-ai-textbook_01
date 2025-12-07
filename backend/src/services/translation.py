from typing import Dict
from fastapi import APIRouter, HTTPException
from ..services.translation import translate_text  # FIXED IMPORT

router = APIRouter()

@router.post("/translate")
async def translate_text_endpoint(text: str, target_language: str):
    try:
        result = await translate_text(text, target_language)  # Note: await added
        return result
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

async def translate_text(text: str, target_language: str) -> Dict[str, str]:
    """
    Mocks an external translation service to translate text to a target language.

    In a real application, this would make an API call to a service like
    Google Translate, DeepL, or another translation provider.

    Args:
        text (str): The text to translate.
        target_language (str): The language code to translate to (e.g., "es", "fr", "ur").

    Returns:
        Dict[str, str]: A dictionary containing the translated text.
                        Example: {"translated_text": "Hola Mundo"}
    """
    # Simple mock translation logic
    if target_language == "es":
        translated_text = f"Hola, esto es una traducción simulada de: '{text}'"
    elif target_language == "fr":
        translated_text = f"Bonjour, ceci est une traduction simulée de: '{text}'"
    elif target_language == "ur":
        translated_text = f"ہیلو، یہ ایک نقلی ترجمہ ہے: '{text}'"
    else:
        translated_text = f"[{target_language} translation of: '{text}']"

    return {"translated_text": translated_text}


if __name__ == "__main__":
    import asyncio

    async def main():
        print(await translate_text("Hello World", "es"))
        print(await translate_text("Hello World", "fr"))
        print(await translate_text("Hello World", "ur"))
        print(await translate_text("Hello World", "de"))

    asyncio.run(main())
