# Vercel entry point for FastAPI backend
# This file redirects to the actual backend implementation

import sys
import os
from pathlib import Path

# Get the root directory (parent of api/)
root_dir = Path(__file__).parent.parent
backend_path = root_dir / 'backend'

# Add backend directory to Python path
sys.path.insert(0, str(backend_path))

# Import the FastAPI app from backend
try:
    from src.main import app as fastapi_app
    # Explicitly assign to 'app' for Vercel
    app = fastapi_app
except ImportError as e:
    # Fallback: create a minimal FastAPI app if import fails
    from fastapi import FastAPI
    app = FastAPI()

    @app.get("/")
    async def root():
        return {
            "error": "Backend import failed",
            "message": str(e),
            "backend_path": str(backend_path),
            "sys_path": sys.path[:3]
        }

# Ensure app is exported for Vercel
__all__ = ['app']
