from openai import OpenAI
from core.config import settings

def get_openai_client():
    client = OpenAI(
        api_key=settings.OPENAI_API_KEY,
    )
    return client