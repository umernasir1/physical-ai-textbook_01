# src/services/translation.py

from typing import Dict
from .openai import get_openai_client
import logging

logger = logging.getLogger(__name__)

# Language code to name mapping
LANGUAGE_NAMES = {
    "ur": "Urdu",
    "es": "Spanish",
    "fr": "French",
    "de": "German",
    "ar": "Arabic",
    "zh": "Chinese",
    "ja": "Japanese",
    "hi": "Hindi",
    "en": "English"
}

async def translate_text(text: str, target_language: str) -> Dict[str, str]:
    """
    Translates text to a target language using Groq AI.
    """
    try:
        client = get_openai_client()
        language_name = LANGUAGE_NAMES.get(target_language, target_language)

        # Use Groq to translate the text
        response = client.chat.completions.create(
            model="llama-3.3-70b-versatile",
            messages=[
                {
                    "role": "system",
                    "content": f"You are a professional translator. Translate the following text to {language_name}. ONLY return the translated text, nothing else. Do not add explanations, notes, or commentary. Preserve the structure and formatting of the original text."
                },
                {
                    "role": "user",
                    "content": text
                }
            ],
            temperature=0.3,
            max_tokens=8000,
        )

        translated_text = response.choices[0].message.content.strip()

        logger.info(f"Successfully translated {len(text)} characters to {language_name}")
        return {"translated_content": translated_text}

    except Exception as e:
        logger.error(f"Translation error: {str(e)}")
        # Fallback to mock translation on error
        return {"translated_content": f"Translation to {LANGUAGE_NAMES.get(target_language, target_language)} failed: {str(e)}"}
