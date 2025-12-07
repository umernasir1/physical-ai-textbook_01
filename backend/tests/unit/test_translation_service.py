import pytest
import asyncio
from backend.src.services.translation import translate_text


@pytest.mark.asyncio
async def test_translate_text_spanish():
    """
    Test translation to Spanish.
    """
    text = "Hello World"
    target_language = "es"
    result = await translate_text(text, target_language)
    assert "translated_text" in result
    assert (
        result["translated_text"]
        == f"Hola, esto es una traducción simulada de: '{text}'"
    )


@pytest.mark.asyncio
async def test_translate_text_french():
    """
    Test translation to French.
    """
    text = "Hello World"
    target_language = "fr"
    result = await translate_text(text, target_language)
    assert "translated_text" in result
    assert (
        result["translated_text"]
        == f"Bonjour, ceci est une traduction simulée de: '{text}'"
    )


@pytest.mark.asyncio
async def test_translate_text_urdu():
    """
    Test translation to Urdu.
    """
    text = "Hello World"
    target_language = "ur"
    result = await translate_text(text, target_language)
    assert "translated_text" in result
    assert result["translated_text"] == f"ہیلو، یہ ایک نقلی ترجمہ ہے: '{text}'"


@pytest.mark.asyncio
async def test_translate_text_unsupported_language():
    """
    Test translation to an unsupported language.
    """
    text = "Hello World"
    target_language = "de"  # German, not explicitly supported by mock
    result = await translate_text(text, target_language)
    assert "translated_text" in result
    assert result["translated_text"] == f"[{target_language} translation of: '{text}']"


@pytest.mark.asyncio
async def test_translate_text_empty_text():
    """
    Test translation with empty text.
    """
    text = ""
    target_language = "es"
    result = await translate_text(text, target_language)
    assert "translated_text" in result
    assert result["translated_text"] == f"Hola, esto es una traducción simulada de: ''"
