import os
from .rag import get_text_splitter, get_embeddings
from ..services.qdrant import get_qdrant_client
from qdrant_client.http.models import PointStruct, UpdateStatus


def index_docs():
    qdrant_client = get_qdrant_client()
    text_splitter = get_text_splitter()

    docs_path = "frontend/docs"
    for root, _, files in os.walk(docs_path):
        for file in files:
            if file.endswith(".md"):
                with open(os.path.join(root, file), "r", encoding="utf-8") as f:
                    text = f.read()
                    chunks = text_splitter.split_text(text)
                    embeddings = get_embeddings(chunks)
                    qdrant_client.upsert(
                        collection_name="textbook",
                        wait=True,
                        points=[
                            PointStruct(id=i, vector=vector, payload={"text": chunk})
                            for i, (chunk, vector) in enumerate(zip(chunks, embeddings))
                        ],
                    )
