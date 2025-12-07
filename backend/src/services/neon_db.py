import psycopg2
from ..core.config import settings

def get_db_connection():
    """
    Establishes a connection to the Neon Postgres database.
    """
    try:
        conn = psycopg2.connect(settings.NEON_DATABASE_URL)
        return conn
    except Exception as e:
        print(f"Error connecting to the database: {e}")
        raise


if __name__ == "__main__":
    # This block is for testing the connection directly
    print("Attempting to connect to the Neon Postgres database...")
    try:
        conn = get_db_connection()
        print("Connection successful!")
        # You can add a simple query here to test further
        # cursor = conn.cursor()
        # cursor.execute("SELECT version();")
        # print(f"PostgreSQL version: {cursor.fetchone()}")
        conn.close()
        print("Connection closed.")
    except Exception as e:
        print(f"Failed to connect: {e}")
