from fastapi import APIRouter, Depends
from pydantic import BaseModel
from ...models.user import User
from ...core.personalization import get_personalization_recommendations
from .auth import get_current_user
from ...core.rag import rag_chain

router = APIRouter()


# Define a request model for the chat message
class ChatMessage(BaseModel):
    text: str


# User Story 2: RAG Chatbot
@router.post("/chat", response_model=dict)
def chat(message: ChatMessage):
    """
    Chat endpoint that provides a response from the RAG chain.
    """
    response = rag_chain(message.text)
    return {"response": response}


# User Story 3: Bonus Feature - Personalization
@router.post("/chat/personalized", response_model=dict)
def chat_with_personalization(
    message: ChatMessage, current_user: User = Depends(get_current_user)
):
    """
    Chat endpoint that provides a personalized response based on the user's profile.
    """
    # Get personalization recommendations
    personalization_recs = get_personalization_recommendations(
        current_user.profile_data
    )

    # Get response from RAG chain
    rag_response = rag_chain(message.text)

    # Augment the response with personalization
    # (This is a simple example, a real implementation would be more sophisticated)
    augmented_response = f"Based on your profile (content level: {personalization_recs.get('content_level')}), here is your answer:\n\n{rag_response}"

    return {
        "user_email": current_user.email,
        "personalization_recommendations": personalization_recs,
        "response": augmented_response,
    }


@router.get("/")
def read_chat():
    return {"message": "Chat endpoint is ready."}
