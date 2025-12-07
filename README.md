# Physical AI Textbook Project

This project aims to create an AI-Native Textbook for Physical AI and Humanoid Robotics. It consists of a Docusaurus-based frontend for content delivery and a FastAPI backend providing RAG chatbot, personalization, and translation services.

## Project Structure

-   `frontend/`: Docusaurus-based textbook content, user authentication UI, and translation features.
-   `backend/`: FastAPI application for the RAG chatbot, authentication, personalization logic, and translation services.
-   `specs/`: Detailed system specifications, data models, and quickstart guide.

## Quickstart

This guide provides the basic steps to set up and run the frontend and backend components of this project locally.

### Prerequisites

-   Node.js (>=20.0) and npm (for the frontend)
-   Python 3.11+ and pip (for the backend)
-   Account credentials for Qdrant Cloud, Neon DB, and OpenAI.

### Setup

1.  **Clone the Repository**:
    ```bash
    git clone [repository-url]
    cd [repository-name]
    ```

2.  **Configure Environment Variables**:
    -   Create a `.env` file in the root directory.
    -   Add the following keys with your credentials:
        ```
        OPENAI_API_KEY=your_openai_api_key
        QDRANT_URL=your_qdrant_url
        QDRANT_API_KEY=your_qdrant_api_key
        NEON_DATABASE_URL=your_neon_database_url
        SECRET_KEY=a_super_secret_key_for_jwt_signing
        ```
        *Note: The `SECRET_KEY` should be a long, randomly generated string. You can generate one using `openssl rand -hex 32`.*

### Running the Backend (FastAPI)

1.  **Navigate to the backend directory**:
    ```bash
    cd backend
    ```

2.  **Install dependencies**:
    ```bash
    pip install -r requirements.txt
    ```

3.  **Run the server**:
    ```bash
    uvicorn src.main:app --reload
    ```
    The API will be available at `http://127.0.0.1:8000`.

#### Backend API Features

-   **Authentication & User Management**:
    -   `POST /v1/auth/signup`: Register a new user.
    -   `POST /v1/auth/token`: Log in and receive an access token.
-   **Personalized RAG Chatbot**:
    -   `POST /v1/chat/`: Interact with the chatbot, which provides personalized responses based on your user profile (requires authentication).
-   **Translation Service**:
    -   `POST /v1/translation/translate`: Translate text to various languages.

### Running the Frontend (Docusaurus)

1.  **Navigate to the frontend directory**:
    ```bash
    cd frontend
    ```

2.  **Install dependencies**:
    ```bash
    npm install
    ```

3.  **Run the development server**:
    ```bash
    npm start
    ```
    The Docusaurus website will be available at `http://localhost:3000`.

#### Frontend UI Features

-   **Authentication UI**: Access login/signup via the `/auth` route. Successful authentication stores a token and enables personalized content.
-   **Personalized Content Display**: Authenticated users with profile data will see a message indicating personalized content on documentation pages.
-   **Translation UI**: A "Show Translator" button on documentation pages allows on-the-fly translation of page titles and descriptions.

## Building for Production

-   **Frontend**: Run `npm run build` in the `frontend` directory. The output will be in the `frontend/build` directory.
-   **Backend**: The backend is a Python application that can be deployed using a production-grade server like Gunicorn behind a reverse proxy, typically in a containerized environment.

## Further Documentation

For more in-depth details on the system architecture, data models, and specific task implementations, please refer to the `specs/` directory.