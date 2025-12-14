# Vercel entry point
# This file is required for Vercel serverless deployment

from src.main import app as fastapi_app

# Vercel expects the variable to be named 'app'
app = fastapi_app
