# Vercel entry point for FastAPI backend
# This file redirects to the actual backend implementation

import sys
import os

# Add backend directory to Python path
backend_path = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'backend')
sys.path.insert(0, backend_path)

# Import the FastAPI app from backend
from src.main import app

# Vercel expects the variable to be named 'app'
# The imported app is already configured and ready to use
