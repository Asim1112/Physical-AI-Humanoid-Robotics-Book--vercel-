# Quickstart: RAG Content Ingestion Pipeline

**Feature**: 001-rag-content-ingestion
**Date**: 2025-12-25
**Audience**: Developers setting up the RAG ingestion pipeline

## Prerequisites

### Required Tools

- **Python**: 3.11 or higher
- **UV**: Ultra-fast Python package manager ([installation](https://github.com/astral-sh/uv))
- **Node.js**: 18+ (for Vercel CLI)
- **Git**: For version control

### Required Accounts

1. **Cohere API**:
   - Sign up at https://dashboard.cohere.com/
   - Create API key from dashboard
   - Free tier: 100 API calls/min

2. **Qdrant Cloud**:
   - Sign up at https://cloud.qdrant.io/
   - Create free tier cluster
   - Note cluster URL and API key

3. **Vercel** (for deployment):
   - Sign up at https://vercel.com/
   - Install Vercel CLI: `npm install -g vercel`

## Setup Instructions

### Step 1: Install UV Package Manager

```bash
# macOS/Linux
curl -LsSf https://astral.sh/uv/install.sh | sh

# Windows (PowerShell)
powershell -c "irm https://astral.sh/uv/install.ps1 | iex"

# Verify installation
uv --version
```

### Step 2: Initialize Backend Project

```bash
# Navigate to repo root
cd /path/to/Humanoid-Robotics-Book

# Create backend directory
mkdir backend
cd backend

# Initialize UV project
uv init

# Add dependencies
uv add cohere qdrant-client tiktoken python-dotenv markdown-it-py requests

# Add dev dependencies
uv add --dev pytest pytest-cov
```

### Step 3: Configure Environment Variables

Create `.env` file in `backend/` directory:

```bash
# backend/.env

# Cohere API
COHERE_API_KEY=your-cohere-api-key-here

# Qdrant Cloud
QDRANT_URL=https://your-cluster-id.cloud.qdrant.io:6333
QDRANT_API_KEY=your-qdrant-api-key-here

# Qdrant Collection Name
QDRANT_COLLECTION_NAME=humanoid-robotics-textbook

# Vercel (optional - CLI will prompt)
VERCEL_TOKEN=your-vercel-token-here

# Frontend Path (relative to backend/)
FRONTEND_PATH=../frontend

# Docs Path (relative to frontend/)
DOCS_PATH=docs
```

**Security**: Add `.env` to `.gitignore` to prevent committing secrets.

```bash
# Add to .gitignore
echo "backend/.env" >> .gitignore
```

### Step 4: Project Structure

After setup, your structure should look like:

```
Humanoid-Robotics-Book/
├── backend/
│   ├── .env                 # Environment variables (not committed)
│   ├── pyproject.toml       # UV project config
│   ├── uv.lock              # Locked dependencies
│   ├── main.py              # Pipeline orchestration (to be created)
│   ├── chunking.py          # Chunking logic (to be created)
│   ├── embedding.py         # Cohere integration (to be created)
│   ├── storage.py           # Qdrant operations (to be created)
│   ├── deployment.py        # Vercel deployment (to be created)
│   └── tests/
│       ├── test_chunking.py
│       ├── test_embedding.py
│       └── test_integration.py
├── frontend/
│   ├── docs/                # Textbook markdown content
│   ├── docusaurus.config.js
│   └── ...
└── specs/
    └── 001-rag-content-ingestion/
        ├── spec.md
        ├── plan.md
        ├── research.md
        ├── data-model.md
        ├── contracts/
        └── quickstart.md (this file)
```

## Running the Pipeline

### Step 5: Verify Setup

```bash
cd backend

# Test environment variables
uv run python -c "import os; from dotenv import load_dotenv; load_dotenv(); print('Cohere:', os.getenv('COHERE_API_KEY')[:10] + '...')"

# Test Cohere API
uv run python -c "
import cohere, os
from dotenv import load_dotenv
load_dotenv()
co = cohere.Client(os.getenv('COHERE_API_KEY'))
resp = co.embed(texts=['test'], model='embed-english-v3.0', input_type='search_document')
print(f'Embedding dimension: {len(resp.embeddings[0])}')
"

# Test Qdrant connection
uv run python -c "
from qdrant_client import QdrantClient
import os
from dotenv import load_dotenv
load_dotenv()
client = QdrantClient(url=os.getenv('QDRANT_URL'), api_key=os.getenv('QDRANT_API_KEY'))
print(f'Qdrant collections: {client.get_collections()}')
"
```

Expected output:
```
Cohere: sk_test_ab...
Embedding dimension: 1024
Qdrant collections: CollectionsResponse(collections=[])
```

### Step 6: Run Ingestion Pipeline

```bash
cd backend

# Run full pipeline (after implementation)
uv run python main.py

# Expected output:
# [INFO] Starting RAG ingestion pipeline
# [INFO] Deploying frontend to Vercel...
# [INFO] Deployment successful: https://your-app.vercel.app
# [INFO] Discovering markdown files in frontend/docs/...
# [INFO] Found 50 markdown files
# [INFO] Chunking files...
# [INFO] Created 350 chunks (avg 7 chunks/file)
# [INFO] Generating embeddings...
# [INFO] Embedded 350 chunks in 4 batches (3.2s)
# [INFO] Uploading to Qdrant...
# [INFO] Uploaded 350 vectors in 4 batches (2.1s)
# [INFO] Verifying storage...
# [INFO] Sample query: "ROS 2 introduction"
# [INFO] Top result: module-1-ros2/intro.md (score: 0.92)
# [INFO] Pipeline completed successfully!
```

### Step 7: Verify Deployment and Storage

```bash
# Check Vercel deployment
curl -I https://your-app.vercel.app
# Expected: HTTP/2 200

# Query Qdrant (Python)
uv run python -c "
from qdrant_client import QdrantClient
import os, cohere
from dotenv import load_dotenv
load_dotenv()

# Generate query embedding
co = cohere.Client(os.getenv('COHERE_API_KEY'))
query_emb = co.embed(
    texts=['What is ROS 2?'],
    model='embed-english-v3.0',
    input_type='search_query'
).embeddings[0]

# Search Qdrant
client = QdrantClient(url=os.getenv('QDRANT_URL'), api_key=os.getenv('QDRANT_API_KEY'))
results = client.search(
    collection_name=os.getenv('QDRANT_COLLECTION_NAME'),
    query_vector=query_emb,
    limit=3,
    with_payload=True
)

for r in results:
    print(f'Score: {r.score:.2f} | Doc: {r.payload[\"document_id\"]}')
    print(f'Text preview: {r.payload[\"text\"][:100]}...')
    print()
"
```

Expected output:
```
Score: 0.92 | Doc: module-1-ros2/intro.md
Text preview: # Introduction to ROS 2

ROS 2 is the next generation Robot Operating System...

Score: 0.88 | Doc: module-1-ros2/architecture.md
Text preview: ## ROS 2 Architecture

The architecture of ROS 2 is designed for...

Score: 0.85 | Doc: module-1-ros2/concepts.md
Text preview: ### Core Concepts

ROS 2 introduces several key concepts...
```

## Testing

### Run Unit Tests

```bash
cd backend

# Run all tests
uv run pytest tests/ -v

# Run with coverage
uv run pytest tests/ --cov=. --cov-report=html

# Run specific test file
uv run pytest tests/test_chunking.py -v
```

### Run Integration Tests

```bash
# Integration tests require API keys
uv run pytest tests/test_integration.py -v --log-cli-level=INFO
```

## Troubleshooting

### Issue: "Invalid Cohere API key"

**Solution**:
1. Verify API key in Cohere dashboard
2. Check `.env` file has correct key (no quotes)
3. Ensure `load_dotenv()` is called before using key

### Issue: "Qdrant connection refused"

**Solution**:
1. Verify cluster URL includes port `:6333`
2. Check API key is correct
3. Ensure cluster is active (not paused on free tier)

### Issue: "Token count exceeds 512"

**Solution**:
1. Check chunking logic splits at proper boundaries
2. Verify `tiktoken` encoding matches Cohere's tokenization
3. Reduce target chunk size to 700 tokens (buffer for safety)

### Issue: "Rate limit exceeded"

**Solution**:
1. Reduce batch size (currently 96 chunks)
2. Add delay between batches (600ms recommended)
3. Check Cohere dashboard for usage limits

### Issue: "Vercel deployment failed"

**Solution**:
1. Ensure Vercel CLI is authenticated: `vercel login`
2. Check frontend builds locally: `cd frontend && npm run build`
3. Review Vercel logs: `vercel logs <deployment-url>`

## Next Steps

After successful setup and verification:

1. **Run full ingestion**: Process all ~50 markdown files
2. **Monitor usage**: Check Cohere and Qdrant dashboards
3. **Verify quality**: Test semantic search with various queries
4. **Document results**: Capture deployment URL and sample queries
5. **Proceed to next spec**: RAG query pipeline (Spec 2)

## Resources

- **Cohere Docs**: https://docs.cohere.com/
- **Qdrant Docs**: https://qdrant.tech/documentation/
- **Vercel Docs**: https://vercel.com/docs
- **UV Docs**: https://github.com/astral-sh/uv
- **Tiktoken**: https://github.com/openai/tiktoken

## Support

For issues or questions:
1. Check this quickstart guide
2. Review spec.md and research.md for decisions
3. Consult API contracts in contracts/
4. File issue in project repository
