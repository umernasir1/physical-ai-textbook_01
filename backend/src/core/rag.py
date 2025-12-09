from ..services.openai import get_openai_client
from ..services.qdrant import get_qdrant_client

from langchain_text_splitters import RecursiveCharacterTextSplitter


def get_text_splitter():
    return RecursiveCharacterTextSplitter(
        chunk_size=1000,
        chunk_overlap=200,
        length_function=len,
    )


def get_embeddings(texts):
    client = get_openai_client()
    response = client.embeddings.create(input=texts, model="text-embedding-ada-002")
    return [embedding.embedding for embedding in response.data]


def rag_chain(query: str):
    qdrant_client = get_qdrant_client()
    openai_client = get_openai_client()

    query_embedding = get_embeddings([query])[0]

    search_result = qdrant_client.search(
        collection_name="textbook",
        query_vector=query_embedding,
        limit=3,
    )

    context = " ".join([hit.payload["text"] for hit in search_result])

    response = openai_client.chat.completions.create(
        model="gpt-3.5-turbo",
        messages=[
            {
                "role": "system",
                "content": "You are a helpful assistant for a Physical AI and Humanoid Robotics Textbook. Your name is C-3PO and you are a protocol droid. You should answer questions based on the context provided. If the answer is not in the context, you should say that you are not able to answer the question.",
            },
            {"role": "user", "content": f"Context: {context}\n\nQuestion: {query}"},
        ],
    )

    return response.choices[0].message.content
