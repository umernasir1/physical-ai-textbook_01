from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware

from .api.v1 import router as api_router
from .core.indexer import index_docs # Import index_docs

app = FastAPI()

origins = [
    "http://localhost",
    "http://localhost:3000", # Default Docusaurus port
]

app.add_middleware(
    CORSMiddleware,
    allow_origins=origins,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

app.include_router(api_router, prefix="/api")

@app.on_event("startup")
async def startup_event():
    print("Application startup: Indexing documents for RAG...")
    index_docs()
    print("Application startup: Document indexing complete.")

@app.get("/")
async def read_root():
    return {"message": "Welcome to the FastAPI Backend!"}