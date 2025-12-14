# Vercel entry point
# This file is required for Vercel serverless deployment

from src.main import app

# Vercel will use this as the ASGI app
handler = app
