from pydantic_settings import BaseSettings, SettingsConfigDict

class Settings(BaseSettings):
    OPENAI_API_KEY: str
    QDRANT_URL: str
    QDRANT_API_KEY: str
    NEON_DATABASE_URL: str
    SECRET_KEY: str # For JWT signing

    model_config = SettingsConfigDict(env_file=".env", extra="ignore")

settings = Settings()