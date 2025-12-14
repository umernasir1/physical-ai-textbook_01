from groq import Groq
from ..core.config import settings

def get_openai_client():
    """Returns a Groq client (formerly OpenAI client)"""
    client = Groq(
        api_key=settings.GROQ_API_KEY,
    )
    return client