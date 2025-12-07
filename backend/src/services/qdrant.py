from qdrant_client import QdrantClient
from qdrant_client.http.models import Distance, VectorParams

from core.config import settings

def get_qdrant_client():
    client = QdrantClient(
        url=settings.QDRANT_URL,
        api_key=settings.QDRANT_API_KEY,
    )
    return client

def recreate_collection(collection_name: str, vector_size: int = 1536):
    client = get_qdrant_client()
    client.recreate_collection(
        collection_name=collection_name,
        vectors_config=VectorParams(size=vector_size, distance=Distance.COSINE),
    )
    return client