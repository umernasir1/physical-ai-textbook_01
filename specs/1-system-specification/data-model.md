# Data Model for Physical AI Textbook

This document outlines the key data entities for the project.

## Core Entities

### 1. User (Optional)

This entity is only required if the bonus features for authentication and personalization are implemented.

-   **`user_id`**: `UUID` (Primary Key) - A unique identifier for the user.
-   **`email`**: `String` (Unique) - The user's email address, used for login.
-   **`hashed_password`**: `String` - The salted and hashed password for the user.
-   **`profile_data`**: `JSONB` - A flexible field to store user-specific information, such as the software and hardware background collected at signup for personalization.
    -   *Example: `{ "hardware": "rtx_4090", "experience_level": "beginner" }`
-   **`created_at`**: `Timestamp` - The timestamp when the user account was created.

### 2. Book Content & Embeddings (Conceptual)

This is not a direct database model but represents the data flow for the RAG pipeline.

-   **Content Source**: The source of truth is the collection of Markdown (`.md`) files within the `frontend/docs` directory of the Docusaurus project.
-   **Content Chunks**: For the RAG pipeline, the content of the Markdown files will be programmatically split into smaller, indexed chunks of text.
-   **Embeddings**: Each content chunk will be converted into a vector embedding using an OpenAI model.
-   **Vector Storage**: The generated embeddings, along with a reference to their source chunk (e.g., file path and chunk ID), will be stored and indexed in the **Qdrant Cloud** vector database.

## Relationships

-   A **User** is an independent entity.
-   The **Book Content** is conceptually linked to its **Embeddings** stored in Qdrant. There is no direct relational link between the User table in Postgres and the content in Qdrant, although application logic can link user interactions to specific content.
