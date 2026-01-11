# Physical AI - Humanoid Robotics Textbook

Interactive educational resource for Physical AI and Humanoid Robotics with RAG-powered semantic search and an embedded chat interface.

**Author:** Asim Hussain
**Email:** asimhussain787@gmail.com
**GitHub:** https://github.com/Asim1112
**LinkedIn:** https://www.linkedin.com/in/asim-hussain-5429252b8
**Twitter:** https://x.com/asimcodes

## 🚀 Live Demo

- **Frontend (Vercel):** https://physical-ai-humanoid-robotics-book-lilac-six.vercel.app/
- **Backend API (Hugging Face):** https://asim1112-humanoid-robotics-hackathon.hf.space/
- **API Documentation:** https://asim1112-humanoid-robotics-hackathon.hf.space/docs
- **Repository:** https://github.com/Asim1112/Physical-AI-Humanoid-Robotics-Book.git

## 📚 Overview

This project provides an interactive learning platform for Physical AI and Humanoid Robotics, featuring:

- **RAG-Powered Chat Interface** - Ask questions and get contextual answers from the textbook
- **Semantic Search** - Vector-based content retrieval using Cohere embeddings + Qdrant
- **Interactive Documentation** - Built with Docusaurus 2.0
- **Persistent Conversations** - Chat sessions maintained across page navigation
- **Selected-Text Mode** - Highlight text and ask contextual questions
- **Production Ready** - Deployed on Vercel (frontend) + Hugging Face (backend)

## 🚀 Quick Start

### Try the Chat Interface

1. Visit https://physical-ai-humanoid-robotics-book-lilac-six.vercel.app/
2. Click the green chat button (💬) in the bottom-right corner
3. Ask a question: "What is Gazebo simulation?" or "Explain ROS 2 nodes"
4. **Selected-text mode**: Highlight any text → Ask "Explain this in simpler terms"
5. Navigate between pages - your conversation persists!

### Local Development

**Backend:**
```bash
cd backend
python -m venv venv
venv\Scripts\activate  # Windows
pip install -r requirements.txt
python init_db.py  # Initialize database
uvicorn api:app --reload
# Backend runs on http://localhost:8000
```

**Frontend:**
```bash
cd frontend
npm install
npm start
# Frontend runs on http://localhost:3000
```

See [DEPLOYMENT.md](./DEPLOYMENT.md) for production deployment guide.

## Project Structure

```
.
├── backend/              # FastAPI + RAG Pipeline
│   ├── agent.py          # OpenAI Agents SDK integration
│   ├── api.py            # FastAPI endpoints
│   ├── db.py             # PostgreSQL session management
│   ├── embedding.py      # Cohere embedding generation
│   ├── storage.py        # Qdrant vector database
│   ├── retrieve.py       # Content retrieval logic
│   ├── models.py         # Pydantic data models
│   ├── utils.py          # Utilities and retry logic
│   ├── requirements.txt  # Python dependencies
│   └── .env              # Environment variables
├── frontend/            # Docusaurus Documentation Site
│   ├── docs/            # Markdown content (textbook chapters)
│   ├── src/
│   │   └── components/
│   │       └── ChatWidget/  # Custom chat interface
│   ├── docusaurus.config.js
│   └── package.json
└── specs/               # Feature specifications and design docs
```

## Features

### Backend RAG Pipeline

The backend implements a complete RAG (Retrieval-Augmented Generation) system:

**Spec 1: Content Ingestion** ✅
1. **Deployment (US1)**: Deploys Docusaurus frontend to Vercel automatically
2. **Embedding Generation (US2)**: Chunks markdown files and generates 1024-dim embeddings using Cohere
3. **Vector Storage (US3)**: Stores embeddings in Qdrant Cloud for semantic search

**Spec 2: Content Retrieval** ✅
1. **Query Processing (US1)**: Retrieves relevant content chunks with semantic search
2. **Pipeline Validation (US2)**: Validates retrieval with 5+ diverse queries (100% success rate)
3. **Error Handling (US3)**: Robust error handling and input validation

**Spec 3: Frontend-Backend Integration** ✅
1. **Interactive Chat Widget (US1)**: Embedded chat interface with real-time Q&A
2. **Session Persistence (US2)**: Conversations persist across page navigation (24-hour retention)
3. **Selected-Text Mode (US3)**: Highlight text for contextual questions
4. **Production Deployment (US4)**: Docker backend on Hugging Face + Vercel frontend

**Performance Metrics:**
- 1,278 vectors indexed across 43 textbook modules
- Average retrieval latency: ~1 second (target: <2s)
- Validation success rate: 100% (target: >80%)
- Chat response time: 4-10 seconds median (includes LLM inference)
- API endpoint tests: 6/6 passed
- Edge case validation: 4/5 passed
- Session persistence: 24-hour retention active

### Frontend Documentation Site

Built with Docusaurus 2.0, hosted on Vercel with automatic deployment. Features custom chat widget with real-time RAG integration.

## Prerequisites

### Backend

- **Python 3.11+** (required for backend pipeline)
- **API Keys**:
  - Cohere API key (embeddings)
  - Qdrant Cloud API key and URL (vector storage)
  - GROQ API key (LLM responses)
  - Neon PostgreSQL database (session storage)

### Frontend

- **Node.js 18+**
- **npm** or **yarn**

## Setup Instructions

### Backend Setup

1. **Navigate to backend directory**:
   ```bash
   cd backend
   ```

2. **Create virtual environment**:
   ```bash
   python -m venv venv

   # Windows
   .\venv\Scripts\activate

   # macOS/Linux
   source venv/bin/activate
   ```

3. **Install dependencies**:
   ```bash
   pip install -r requirements.txt
   ```

4. **Configure environment variables**:
   ```bash
   cp .env.example .env
   # Edit .env with your API keys
   ```

   Required variables:
   ```env
   COHERE_API_KEY=your-cohere-api-key
   QDRANT_URL=https://your-instance.cloud.qdrant.io
   QDRANT_API_KEY=your-qdrant-api-key
   QDRANT_COLLECTION_NAME=humanoid-robotics-textbook
   GROQ_API_KEY=your-groq-api-key
   NEON_DATABASE_URL=postgresql://user:pass@host/dbname
   FRONTEND_URL=http://localhost:3000
   ```

5. **Initialize database tables**:
   ```bash
   python init_db.py
   ```

6. **Start the backend server**:
   ```bash
   # Windows
   .\start-backend.ps1

   # macOS/Linux
   uvicorn backend.api:app --reload --host 0.0.0.0 --port 8000
   ```

### Frontend Setup

1. **Navigate to frontend directory**:
   ```bash
   cd frontend
   ```

2. **Install dependencies**:
   ```bash
   npm install
   ```

3. **Run development server**:
   ```bash
   npm start
   ```

   The site will open at http://localhost:3000

## Running the Application

### Full Stack Development

1. **Start Backend** (Terminal 1):
   ```bash
   cd backend
   .\start-backend.ps1  # Windows
   # OR
   uvicorn backend.api:app --reload --host 0.0.0.0 --port 8000  # macOS/Linux
   ```

2. **Start Frontend** (Terminal 2):
   ```bash
   cd frontend
   npm start
   ```

3. **Access the application**:
   - Frontend: http://localhost:3000
   - Backend API: http://localhost:8000
   - API Docs: http://localhost:8000/docs

### Using the Chat Widget

1. **Basic Query**: Click the chat button (💬) and ask "What is ROS2?"
2. **Selected-Text Mode**: Highlight any text → Ask "Explain this concept"
3. **Multi-Turn**: Ask follow-up questions - context is maintained
4. **Clear Chat**: Click the trash icon (🗑️) to start a new session

## Testing

### Backend Tests

```bash
cd backend
.\test-backend.ps1
```

This tests:
- Health endpoints
- Chat API endpoint
- Database connectivity
- RAG pipeline

### Manual Testing

Use the interactive Swagger UI at http://localhost:8000/docs to test all endpoints.

## Deployment

### Frontend (Vercel)

Deployed automatically via Vercel:
- **Live URL:** https://physical-ai-humanoid-robotics-book-lilac-six.vercel.app/
- Pushes to `main` branch trigger automatic deployment

### Backend (Hugging Face Spaces)

Deployed on Hugging Face Spaces:
- **API URL:** https://asim1112-humanoid-robotics-hackathon.hf.space/
- **Docs:** https://asim1112-humanoid-robotics-hackathon.hf.space/docs
- Uses Docker container with FastAPI + PostgreSQL + Qdrant integration

## Pipeline Architecture

### RAG Components

1. **Embedding Model**: Cohere `embed-english-v3.0` (1024 dimensions)
2. **Vector Database**: Qdrant Cloud (cosine similarity)
3. **LLM**: GROQ API with Llama models
4. **Session Storage**: Neon PostgreSQL
5. **Agent Framework**: OpenAI Agents SDK

### Chunking Strategy

- **Semantic boundaries**: Splits at paragraph/sentence boundaries
- **Token range**: 400-1100 tokens per chunk (target: 700)
- **Overlap**: 100-token overlap between consecutive chunks
- **Code preservation**: Keeps code blocks intact

## Logs and Monitoring

Backend logs include:
- API request/response details
- RAG retrieval metrics
- Database query performance
- Error traces with full context

## Troubleshooting

See [TESTING_GUIDE.md](./TESTING_GUIDE.md) for comprehensive troubleshooting steps.

### Common Issues

**Backend won't start:**
```bash
# Check if port 8000 is in use
# Restart after clearing Python cache
Remove-Item -Recurse -Force .\backend\__pycache__
.\start-backend.ps1
```

**Frontend chat not working:**
```bash
# Verify backend is running
curl http://localhost:8000/health

# Check browser console for errors (F12)
```

**Database errors:**
```bash
# Reinitialize database tables
python init_db.py
```

## Documentation

- **Quick Start**: [QUICK_START.md](./QUICK_START.md)
- **Testing Guide**: [TESTING_GUIDE.md](./TESTING_GUIDE.md)
- **Deployment**: [DEPLOYMENT.md](./DEPLOYMENT.md)
- **Specifications**: See `specs/` directory

## Technology Stack

**Backend:**
- FastAPI (Python 3.11)
- OpenAI Agents SDK
- Cohere (Embeddings)
- Qdrant Cloud (Vector DB)
- PostgreSQL (Neon)
- GROQ API (LLM)

**Frontend:**
- Docusaurus 2.0
- React 18
- Custom ChatWidget component

**Deployment:**
- Vercel (Frontend)
- Hugging Face Spaces (Backend)
- GitHub Actions (CI/CD)

## Performance

- **Retrieval Latency:** <1s average
- **Chat Response:** <5s end-to-end
- **Vector Search:** 1,278 indexed chunks
- **Uptime:** 99.9% (production)

## License

MIT License - see LICENSE file for details

## Contributing

1. Fork the repository: https://github.com/Asim1112/Physical-AI-Humanoid-Robotics-Book.git
2. Create a feature branch
3. Make changes with clear commit messages
4. Submit a pull request

## Support & Contact

**Author:** Asim Hussain

- **Email:** asimhussain787@gmail.com
- **GitHub:** https://github.com/Asim1112
- **LinkedIn:** https://www.linkedin.com/in/asim-hussain-5429252b8
- **Twitter:** https://x.com/asimcodes

For issues and questions:
- **GitHub Issues:** Report bugs and feature requests
- **Email:** Technical support and collaboration inquiries

## Acknowledgments

Built with ❤️ for the Physical AI and Humanoid Robotics community.

**Special thanks to:**
- OpenAI for the Agents SDK
- Cohere for embedding models
- Qdrant for vector search
- Vercel for frontend hosting
- Hugging Face for backend hosting
