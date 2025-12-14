---
description: Manage RAG system - reindex documents, test queries, optimize retrieval, and monitor vector database
---

## User Input

```text
$ARGUMENTS
```

You **MUST** consider the user input before proceeding (if not empty).

## Outline

### Purpose
This subagent manages the RAG (Retrieval-Augmented Generation) chatbot system, ensuring optimal document indexing and query performance.

### Capabilities
1. **Reindex Documents**: Rebuild vector database from documentation
2. **Test Queries**: Validate retrieval quality with sample questions
3. **Optimize Retrieval**: Tune search parameters and chunking strategy
4. **Monitor Performance**: Check Qdrant health and embedding quality
5. **Debug Issues**: Diagnose and fix RAG pipeline problems

### Execution Steps

1. **Understand Request**:
   - Parse action: reindex | test | optimize | monitor | debug
   - If no action, perform health check

2. **Reindex Documents**:
   - Connect to backend API at `http://localhost:8000`
   - Trigger `/api/v1/indexer/reindex` endpoint (if exists)
   - OR manually trigger indexing:
     - Read all `.md` files from `frontend/docs/`
     - Count total documents to index
     - Monitor indexing progress
     - Verify vector count in Qdrant
   - Report: Documents indexed, vectors created, collection status

3. **Test Queries**:
   - Run predefined test queries covering all modules:
     - "What is ROS 2?" (Module 1)
     - "How do I set up Gazebo simulation?" (Module 2)
     - "Explain NVIDIA Isaac Sim" (Module 3)
     - "What are Vision-Language-Action models?" (Module 4)
   - For each query:
     - Send POST to `/api/v1/chat`
     - Capture response and retrieved context
     - Verify relevant content was retrieved
     - Check response quality (coherent, accurate, relevant)
   - Report: Query success rate, avg retrieval relevance, response quality

4. **Optimize Retrieval**:
   - Test different search parameters:
     - Top K results (5, 10, 20)
     - Score threshold (0.5, 0.6, 0.7)
     - Embedding model performance
   - Suggest optimal parameters based on results
   - Test chunking strategies:
     - Current chunk size
     - Overlap settings
     - Boundary detection (headers, paragraphs)
   - Recommend improvements

5. **Monitor Performance**:
   - Check Qdrant connection: `GET Qdrant status`
   - Verify collection exists and vector count
   - Test embedding generation speed
   - Measure query latency (p50, p95, p99)
   - Check for:
     - Missing documents
     - Duplicate vectors
     - Stale embeddings
   - Report health status: Healthy | Degraded | Offline

6. **Debug Issues**:
   - Common problems to check:
     - Backend not running (port 8000)
     - Qdrant connection failed
     - API key invalid
     - Documents not indexed
     - Poor retrieval quality
     - Slow query responses
   - For each issue:
     - Diagnose root cause
     - Provide fix instructions
     - Test fix if auto-repairable
   - Report: Issues found, fixes applied, manual steps needed

7. **Report Summary**:
   - RAG system status: Operational | Issues Detected | Offline
   - Vector database: X documents, Y vectors
   - Query performance: avg Xms latency
   - Retrieval quality: X% relevant
   - Issues: N found, M fixed
   - Recommendations for improvement

### Usage Examples

```bash
# Health check
/hackathon.rag

# Reindex all documents
/hackathon.rag reindex

# Test query quality
/hackathon.rag test

# Optimize retrieval
/hackathon.rag optimize

# Debug issues
/hackathon.rag debug "queries returning no results"
```

### Success Criteria
- All documents indexed in Qdrant
- Queries return relevant context
- Response latency < 2 seconds
- Retrieval accuracy > 80%
- No errors in RAG pipeline
