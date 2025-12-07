# Quickstart Guide for the Physical AI Textbook Project

This guide provides the basic steps to set up and run the frontend and backend components of this project locally.

## Prerequisites

-   Node.js and npm (for the frontend)
-   Python 3.11+ and pip (for the backend)
-   Account credentials for Qdrant Cloud, Neon DB, and OpenAI.

## Setup

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
    -   The frontend does not require a `.env` file unless specific keys are needed for client-side features.

## Running the Backend (FastAPI)

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

4.  **Run Tests**:
    ```bash
    pytest
    ```
    (Ensure `pytest` is installed via `pip install pytest`)
    
5.  **Run Linters**:
    ```bash
    black .
    flake8 .
    ```

### Backend API Usage

**Authentication & User Management**
-   **Signup**: `POST /v1/auth/signup`
    -   Body: `{"email": "user@example.com", "password": "securepassword", "profile_data": {"experience_level": "beginner", "hardware": "RTX 4090"}}`
    -   Returns: `{"access_token": "...", "token_type": "bearer"}`
-   **Login**: `POST /v1/auth/token`
    -   Body (form-data/x-www-form-urlencoded): `username=user@example.com&password=securepassword`
    -   Returns: `{"access_token": "...", "token_type": "bearer"}`
    *   Use the `access_token` in the `Authorization: Bearer <token>` header for authenticated endpoints.

**Personalized Chat**
-   **Chat**: `POST /v1/chat/` (requires authentication)
    -   Body: `{"text": "Your question here"}`
    -   Returns: Personalized response based on your `profile_data` (set during signup).
    *   The backend integrates personalization logic (`backend/src/core/personalization.py`) to tailor responses.

**Translation**
-   **Translate**: `POST /v1/translation/translate`
    -   Body: `{"text": "Text to translate", "target_language": "ur"}` (e.g., 'es', 'fr', 'ur')
    -   Returns: `{"translated_text": "..."}`

## Running the Frontend (Docusaurus)

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

### Frontend UI Features

-   **Authentication UI**: Access the login/signup page via `/auth`.
    -   Login/Signup forms will set an `access_token` in local storage upon successful authentication.
    -   Authenticated users will see a personalized welcome message on content pages.
-   **Translation UI**: On any content page, click the "Show Translator" button to activate the translation tool.
    -   The translator will translate the page's title and description to the selected language.

4.  **Run Linters**:
    ```bash
    npm run lint
    ```

## Building for Production

-   **Frontend**: Run `npm run build` in the `frontend` directory. The output will be in the `frontend/build` directory.
-   **Backend**: The backend is a Python application that can be deployed using a production-grade server like Gunicorn behind a reverse proxy, typically in a containerized environment.