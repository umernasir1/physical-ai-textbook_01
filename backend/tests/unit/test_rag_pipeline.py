import unittest
from unittest.mock import patch, MagicMock
from src.core.rag import get_text_splitter, get_embeddings
from src.core import config  # Import the config module


class TestRagPipeline(unittest.TestCase):
    @patch.object(config, "settings")
    def test_get_text_splitter(self, mock_settings):
        mock_settings.OPENAI_API_KEY = "mock_openai_key"
        mock_settings.QDRANT_URL = "http://mock-qdrant:6333"
        mock_settings.QDRANT_API_KEY = "mock_qdrant_key"
        mock_settings.NEON_DATABASE_URL = (
            "postgresql://user:password@host:port/database"
        )

        text_splitter = get_text_splitter()
        self.assertIsNotNone(text_splitter)
        text = "This is a test text."
        chunks = text_splitter.split_text(text)
        self.assertEqual(chunks, ["This is a test text."])

    @patch.object(config, "settings")
    @patch("src.core.rag.get_openai_client")
    def test_get_embeddings(self, mock_get_openai_client, mock_settings):
        mock_settings.OPENAI_API_KEY = "mock_openai_key"
        mock_settings.QDRANT_URL = "http://mock-qdrant:6333"
        mock_settings.QDRANT_API_KEY = "mock_qdrant_key"
        mock_settings.NEON_DATABASE_URL = (
            "postgresql://user:password@host:port/database"
        )

        mock_openai_client = MagicMock()
        mock_embedding = MagicMock()
        mock_embedding.embedding = [0.1, 0.2, 0.3]
        mock_openai_client.embeddings.create.return_value.data = [mock_embedding]
        mock_get_openai_client.return_value = mock_openai_client

        embeddings = get_embeddings(["test"])
        self.assertEqual(len(embeddings), 1)
        self.assertEqual(embeddings[0], [0.1, 0.2, 0.3])


if __name__ == "__main__":
    unittest.main()
