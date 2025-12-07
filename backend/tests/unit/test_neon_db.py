import pytest
from unittest.mock import patch, MagicMock

from backend.src.services.neon_db import get_db_connection
from backend.src.core.config import settings


@patch("backend.src.services.neon_db.psycopg2")
def test_get_db_connection_success(mock_psycopg2):
    """
    Test that get_db_connection returns a connection object on success.
    """
    mock_conn = MagicMock()
    mock_psycopg2.connect.return_value = mock_conn

    # Temporarily set a mock database URL for the test
    original_db_url = settings.NEON_DATABASE_URL
    settings.NEON_DATABASE_URL = "postgresql://user:password@host:port/dbname"

    conn = get_db_connection()

    mock_psycopg2.connect.assert_called_once_with(settings.NEON_DATABASE_URL)
    assert conn == mock_conn

    # Restore original database URL
    settings.NEON_DATABASE_URL = original_db_url


@patch("backend.src.services.neon_db.psycopg2")
def test_get_db_connection_failure(mock_psycopg2):
    """
    Test that get_db_connection raises an exception on connection failure.
    """
    mock_psycopg2.connect.side_effect = Exception("Connection failed")

    # Temporarily set a mock database URL for the test
    original_db_url = settings.NEON_DATABASE_URL
    settings.NEON_DATABASE_URL = "postgresql://invalid:invalid@invalid:1111/invalid"

    with pytest.raises(Exception, match="Connection failed"):
        get_db_connection()

    mock_psycopg2.connect.assert_called_once_with(settings.NEON_DATABASE_URL)

    # Restore original database URL
    settings.NEON_DATABASE_URL = original_db_url
