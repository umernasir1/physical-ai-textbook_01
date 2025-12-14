import os
from pathlib import Path
from .rag import get_text_splitter, get_embeddings
from ..services.qdrant import get_qdrant_client, recreate_collection
from qdrant_client.http.models import PointStruct, UpdateStatus


def index_docs():
    """
    Index all markdown documentation files into Qdrant for RAG retrieval.
    """
    try:
        qdrant_client = get_qdrant_client()
        text_splitter = get_text_splitter()

        # Recreate the collection to ensure a clean state
        print("Recreating 'textbook' collection...")
        recreate_collection("textbook")

        # Find the docs directory - handle different execution contexts
        backend_dir = Path(__file__).parent.parent.parent
        docs_path = backend_dir.parent / "frontend" / "docs"

        if not docs_path.exists():
            print(f"Warning: Docs path not found at {docs_path}")
            print("Attempting alternate paths...")
            # Try alternate path (if running from root)
            docs_path = Path("Hackaton/frontend/docs")
            if not docs_path.exists():
                docs_path = Path("frontend/docs")

        if not docs_path.exists():
            raise FileNotFoundError(f"Could not find docs directory. Tried: {docs_path}")

        print(f"Indexing documents from: {docs_path}")

        id_counter = 0
        indexed_files = 0

        for root, _, files in os.walk(docs_path):
            for file in files:
                if file.endswith(".md"):
                    file_path = os.path.join(root, file)
                    try:
                        with open(file_path, "r", encoding="utf-8") as f:
                            text = f.read()

                            # Skip empty files
                            if not text.strip():
                                continue

                            chunks = text_splitter.split_text(text)

                            if not chunks:
                                continue

                            embeddings = get_embeddings(chunks)
                            qdrant_client.upsert(
                                collection_name="textbook",
                                wait=True,
                                points=[
                                    PointStruct(
                                        id=id_counter + i,
                                        vector=vector,
                                        payload={
                                            "text": chunk,
                                            "file": file,
                                            "path": str(Path(root).relative_to(docs_path) / file)
                                        }
                                    )
                                    for i, (chunk, vector) in enumerate(zip(chunks, embeddings))
                                ],
                            )
                            id_counter += len(chunks)
                            indexed_files += 1
                            print(f"Indexed: {file} ({len(chunks)} chunks)")
                    except Exception as e:
                        print(f"Error indexing {file}: {e}")
                        continue

        print(f"[OK] Indexing complete! Indexed {indexed_files} files with {id_counter} total chunks.")

    except Exception as e:
        print(f"Error during indexing: {e}")
        print("Indexing failed but server will continue to start.")
