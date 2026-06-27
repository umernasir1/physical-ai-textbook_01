from ..services.openai import get_openai_client
from ..services.qdrant import get_qdrant_client

from langchain_text_splitters import RecursiveCharacterTextSplitter
from fastembed import TextEmbedding

# Initialize embedding model. FastEmbed runs the same all-MiniLM-L6-v2 model
# via ONNX (384-dim output, no PyTorch), so it stays light enough for small
# instances while producing vectors compatible with the existing collection.
embedding_model = TextEmbedding(model_name="sentence-transformers/all-MiniLM-L6-v2")


def get_text_splitter():
    return RecursiveCharacterTextSplitter(
        chunk_size=1000,
        chunk_overlap=200,
        length_function=len,
    )


def get_embeddings(texts):
    """Generate 384-dim embeddings using FastEmbed (ONNX, open-source)."""
    return [embedding.tolist() for embedding in embedding_model.embed(list(texts))]


def rag_chain(query: str):
    qdrant_client = get_qdrant_client()
    groq_client = get_openai_client()  # Now returns Groq client

    query_embedding = get_embeddings([query])[0]

    search_result = qdrant_client.query_points(
        collection_name="textbook",
        query=query_embedding,
        limit=3,
    ).points

    context = " ".join([hit.payload["text"] for hit in search_result])

    response = groq_client.chat.completions.create(
        model="llama-3.3-70b-versatile",  # Groq's fast LLM model
        messages=[
            {
                "role": "system",
                "content": "You are a helpful assistant for a Physical AI and Humanoid Robotics Textbook. Your name is C-3PO and you are a protocol droid. You should answer questions based on the context provided. If the answer is not in the context, you should say that you are not able to answer the question.",
            },
            {"role": "user", "content": f"Context: {context}\n\nQuestion: {query}"},
        ],
    )

    return response.choices[0].message.content
