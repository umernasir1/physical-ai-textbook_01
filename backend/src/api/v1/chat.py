from fastapi import APIRouter, Depends, HTTPException
from pydantic import BaseModel
from ...models.user import User
from ...core.personalization import get_personalization_recommendations
from .auth import get_current_user
from ...core.rag import rag_chain
from ...services.personalization import personalize_content

router = APIRouter()


# Define a request model for the chat message
class ChatMessage(BaseModel):
    text: str


# Request model for content personalization
class PersonalizeRequest(BaseModel):
    content: str
    skill_level: str


# User Story 2: RAG Chatbot
@router.post("/chat", response_model=dict)
def chat(message: ChatMessage):
    """
    Chat endpoint that provides a response from the RAG chain.
    """
    try:
        response = rag_chain(message.text)
        return {"response": response}
    except Exception as e:
        import traceback
        traceback.print_exc() # Log the full traceback
        raise HTTPException(status_code=500, detail=f"RAG chain failed: {e}")


# User Story 3: Bonus Feature - Personalization
@router.post("/chat/personalized", response_model=dict)
def chat_with_personalization(
    message: ChatMessage, current_user: User = Depends(get_current_user)
):
    """
    Chat endpoint that provides a personalized response based on the user's profile.
    """
    try:
        # Get personalization recommendations
        personalization_recs = get_personalization_recommendations(
            current_user.profile_data
        )
    except Exception as e:
        import traceback
        traceback.print_exc()
        raise HTTPException(status_code=500, detail=f"Personalization failed: {e}")

    try:
        # Get response from RAG chain
        rag_response = rag_chain(message.text)
    except Exception as e:
        import traceback
        traceback.print_exc()
        raise HTTPException(status_code=500, detail=f"RAG chain failed: {e}")

    # Augment the response with personalization
    # (This is a simple example, a real implementation would be more sophisticated)
    augmented_response = f"Based on your profile (content level: {personalization_recs.get('content_level')}), here is your answer:\n\n{rag_response}"

    return {
        "user_email": current_user.email,
        "personalization_recommendations": personalization_recs,
        "response": augmented_response,
    }


@router.post("/personalize")
async def personalize_content_endpoint(request: PersonalizeRequest):
    """
    Personalizes content based on the user's skill level.
    """
    try:
        result = await personalize_content(request.content, request.skill_level)
        return result
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@router.get("/")
def read_chat():
    return {"message": "Chat endpoint is ready."}
