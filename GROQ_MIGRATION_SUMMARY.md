# Groq API Migration Summary

**Date:** December 10, 2025
**Migration:** OpenAI API → Groq API

## Overview

Successfully migrated the Physical AI & Humanoid Robotics Textbook project from OpenAI to Groq AI for faster and more cost-effective LLM inference.

## Changes Made

### 1. Backend Configuration
**File:** `backend/src/core/config.py`
- **Line 4:** Changed `OPENAI_API_KEY: str` to `GROQ_API_KEY: str`
- **Impact:** All environment variable loading now uses `GROQ_API_KEY`

### 2. Service Layer
**File:** `backend/src/services/openai.py`
- **Line 1:** Changed `from openai import OpenAI` to `from groq import Groq`
- **Lines 4-8:** Updated client initialization to use Groq client with `GROQ_API_KEY`
- **Function name:** Kept as `get_openai_client()` for backward compatibility (returns Groq client)

### 3. RAG Pipeline
**File:** `backend/src/core/rag.py`
- **Added:** `from sentence-transformers import SentenceTransformer` (Line 5)
- **Added:** Open-source embedding model initialization using `all-MiniLM-L6-v2` (Line 8)
- **Modified:** `get_embeddings()` function to use SentenceTransformers instead of OpenAI embeddings
- **Modified:** `rag_chain()` function:
  - Line 27: Renamed `openai_client` to `groq_client`
  - Line 40: Changed model from `gpt-3.5-turbo` to `llama-3.3-70b-versatile` (Groq's fast LLM)

**Why SentenceTransformers?**
- Groq doesn't provide embedding APIs
- `all-MiniLM-L6-v2` is a proven open-source model (384 dimensions)
- No additional API calls required for embeddings
- Runs locally for faster processing

### 4. Dependencies
**File:** `backend/requirements.txt`
- **Removed:** `openai>=1.3.0`, `langchain-openai>=0.0.2`, `langchain-community>=0.0.10`
- **Added:** `groq>=0.4.0`, `sentence-transformers>=2.2.0`
- **Kept:** Core dependencies (FastAPI, Qdrant, langchain-text-splitters, etc.)

### 5. Documentation Updates

#### README.md
- Line 5: Updated description to mention "Groq AI" instead of "OpenAI"
- Line 13: Changed feature description to "Groq LLaMA 3.3 + vector search"
- Line 77: Updated API key link to Groq console
- Line 88: Changed environment variable from `OPENAI_API_KEY` to `GROQ_API_KEY`
- Line 119: Updated embedding model description

#### SETUP_COMPLETE.md
- Line 13: Updated "RAG chatbot with Groq AI + Qdrant integration"
- Line 20: Added "Switched from OpenAI to Groq for faster inference"
- Line 203: Updated points breakdown to reflect Groq AI
- Line 230-232: Updated troubleshooting for Groq API key

## Technical Benefits

### Performance
- **Groq's LLaMA 3.3-70B:** Up to 10x faster inference than OpenAI GPT-3.5
- **Local Embeddings:** No API latency for embedding generation
- **Cost Reduction:** Groq offers competitive pricing with generous free tier

### Model Comparison

| Feature | OpenAI (Before) | Groq (After) |
|---------|----------------|--------------|
| LLM Model | GPT-3.5 Turbo | LLaMA 3.3 70B Versatile |
| Embedding | text-embedding-ada-002 | all-MiniLM-L6-v2 (local) |
| Inference Speed | Baseline | ~10x faster |
| Cost | $0.001/1K tokens | Lower (check current pricing) |
| API Calls | 2 per query (embed + chat) | 1 per query (chat only) |

## Environment Variables

### Updated .env Format
```bash
# Old (OpenAI)
OPENAI_API_KEY=sk-...

# New (Groq)
GROQ_API_KEY=gsk_...
```

### Full .env Template
```bash
GROQ_API_KEY=your_groq_api_key_here
QDRANT_URL=your_qdrant_url_here
QDRANT_API_KEY=your_qdrant_api_key_here
NEON_DATABASE_URL=your_neon_database_url_here
SECRET_KEY=$(openssl rand -hex 32)
```

## Migration Checklist

- [x] Update config.py to use GROQ_API_KEY
- [x] Replace OpenAI client with Groq client
- [x] Implement local embedding generation
- [x] Update requirements.txt
- [x] Update README documentation
- [x] Update SETUP_COMPLETE documentation
- [ ] Install new dependencies (`pip install groq sentence-transformers`)
- [ ] Test backend with Groq API
- [ ] Re-index documents (embeddings will be different)
- [ ] Test chatbot functionality end-to-end

## Testing Plan

### 1. Backend Startup Test
```bash
cd backend
python -m uvicorn src.main:app --reload
```
**Expected:** Server starts without errors, documents indexed successfully

### 2. Embedding Generation Test
```python
from backend.src.core.rag import get_embeddings

test_texts = ["What is ROS 2?", "Explain Physical AI"]
embeddings = get_embeddings(test_texts)
print(f"Generated {len(embeddings)} embeddings of dimension {len(embeddings[0])}")
# Expected: 2 embeddings, 384 dimensions each
```

### 3. RAG Query Test
```python
from backend.src.core.rag import rag_chain

response = rag_chain("What is Physical AI?")
print(response)
# Expected: Relevant answer based on textbook content
```

### 4. Frontend Integration Test
1. Start backend: `uvicorn src.main:app --reload`
2. Open chatbot widget in frontend
3. Ask: "What is ROS 2?"
4. **Expected:** Fast response from Groq LLaMA 3.3 model

## Important Notes

### Embedding Dimension Change
- **Old:** OpenAI text-embedding-ada-002 (1536 dimensions)
- **New:** all-MiniLM-L6-v2 (384 dimensions)

**Action Required:** You must **re-create the Qdrant collection** with the new dimension size or re-index all documents. The existing embeddings are incompatible.

### Re-indexing Documents
The backend automatically indexes documents on startup. To force re-indexing:
1. Delete the Qdrant collection via dashboard
2. Restart the backend
3. Documents will be re-embedded with the new model

### Model Performance Notes
- **LLaMA 3.3 70B** may have different output style than GPT-3.5
- Test chatbot responses to ensure quality meets expectations
- Adjust system prompts if needed (backend/src/core/rag.py:41-44)

## Rollback Plan

If issues arise, revert with:

```bash
# 1. Restore requirements.txt
git checkout HEAD -- backend/requirements.txt

# 2. Restore code files
git checkout HEAD -- backend/src/core/config.py
git checkout HEAD -- backend/src/services/openai.py
git checkout HEAD -- backend/src/core/rag.py

# 3. Reinstall dependencies
pip install -r backend/requirements.txt

# 4. Update .env
# Change GROQ_API_KEY back to OPENAI_API_KEY

# 5. Re-index documents
# Restart backend to regenerate OpenAI embeddings
```

## Support & Resources

- **Groq Documentation:** https://console.groq.com/docs
- **Groq API Keys:** https://console.groq.com/keys
- **SentenceTransformers:** https://www.sbert.net/
- **Model Card:** https://huggingface.co/sentence-transformers/all-MiniLM-L6-v2

## Next Steps

1. Complete dependency installation
2. Test backend startup and document indexing
3. Verify Groq API connectivity
4. Test chatbot end-to-end
5. Monitor performance and adjust if needed
6. Update project screenshots/demo video if needed for hackathon submission

---

**Migration completed by:** Claude Code
**Status:** ✅ Code changes complete, awaiting testing
