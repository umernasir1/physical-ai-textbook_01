import unittest
from unittest.mock import patch, MagicMock
from src.services.qdrant import get_qdrant_client
from src.core import config  # Import the config module


class TestQdrantService(unittest.TestCase):
    @patch.object(config, "settings")
    @patch("src.services.qdrant.QdrantClient")
    def test_get_qdrant_client(self, mock_qdrant_client, mock_settings):
        # Configure mock_settings before calling get_qdrant_client
        mock_settings.OPENAI_API_KEY = "mock_openai_key"
        mock_settings.QDRANT_URL = "http://mock-qdrant:6333"
        mock_settings.QDRANT_API_KEY = "mock_qdrant_key"
        mock_settings.NEON_DATABASE_URL = (
            "postgresql://user:password@host:port/database"
        )

        # Create a mock instance of the QdrantClient
        mock_client_instance = MagicMock()
        mock_qdrant_client.return_value = mock_client_instance

        # Call the function that creates the client
        client = get_qdrant_client()

        # Assert that the QdrantClient was called with the correct arguments
        mock_qdrant_client.assert_called_once_with(
            url="http://mock-qdrant:6333",
            api_key="mock_qdrant_key",
        )
        self.assertEqual(client, mock_client_instance)


if __name__ == "__main__":
    unittest.main()
