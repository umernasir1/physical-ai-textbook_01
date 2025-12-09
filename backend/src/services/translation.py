# src/services/translation.py

from typing import Dict

async def translate_text(text: str, target_language: str) -> Dict[str, str]:
    """
    Mocks an external translation service to translate text to a target language.
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
