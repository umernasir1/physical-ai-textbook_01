# Physical AI & Humanoid Robotics Textbook

**🤖 An AI-Native Interactive Textbook for Learning Physical AI and Humanoid Robotics**

This hackathon project delivers a comprehensive, interactive textbook on Physical AI built with Docusaurus, featuring an embedded RAG chatbot powered by Groq AI, Qdrant, and Neon Postgres.

[![Deploy to GitHub Pages](https://github.com/umernasir1/physical-ai-textbook/actions/workflows/deploy.yml/badge.svg)](https://github.com/umernasir1/physical-ai-textbook/actions/workflows/deploy.yml)

## 🎯 Features

### Core Features (100 points)
- ✅ **Comprehensive Textbook**: 4 modules covering ROS 2, Gazebo/Unity, NVIDIA Isaac, and Vision-Language-Action
- ✅ **RAG Chatbot**: Intelligent question-answering using Groq LLaMA 3.3 + vector search
- ✅ **GitHub Pages Deployment**: Automated CI/CD pipeline

### Bonus Features (up to 200 points)
- ✅ **Authentication**: Signup/signin with better-auth.com + background questionnaire
- ✅ **Content Personalization**: Tailored content based on user software/hardware background
- ✅ **Urdu Translation**: Translate chapters to Urdu on demand
- 🚧 **Claude Code Subagents**: Reusable intelligence for development workflow

## 📚 Course Content

### Module 1: The Robotic Nervous System (ROS 2)
- Introduction to Physical AI
- ROS 2 Nodes, Topics, and Services
- Python Agents and rclpy
- URDF for Humanoids

### Module 2: The Digital Twin (Gazebo & Unity)
- Physics Simulation and Environment Building
- High-Fidelity Rendering and Human-Robot Interaction
- Simulating Sensors (LIDAR, Depth Cameras, IMUs)

### Module 3: The AI-Robot Brain (NVIDIA Isaac™)
- NVIDIA Isaac Sim: Photorealistic Simulation
- Advanced Perception and Training
- Isaac ROS: VSLAM and Navigation
- Nav2: Path Planning for Bipedal Movement

### Module 4: Vision-Language-Action (VLA)
- The Convergence of LLMs and Robotics
- Voice-to-Action with OpenAI Whisper
- Cognitive Planning: Natural Language to ROS 2 Actions
- Capstone Project: The Autonomous Humanoid

## 🏗️ Project Structure

```
Hackaton/
├── frontend/               # Docusaurus website
│   ├── docs/              # Textbook content (Markdown)
│   ├── src/
│   │   ├── components/    # React components (Chatbot, Auth, Translator)
│   │   ├── services/      # API clients
│   │   └── theme/         # Docusaurus theme customizations
│   └── docusaurus.config.js
├── backend/               # FastAPI application
│   ├── src/
│   │   ├── api/v1/       # API endpoints (chat, auth, translation)
│   │   ├── core/         # RAG pipeline, indexer, personalization
│   │   ├── services/     # External service clients (Qdrant, OpenAI, Neon)
│   │   └── models/       # Pydantic data models
│   └── requirements.txt
├── specs/                 # System specifications and planning docs
└── .github/workflows/     # CI/CD automation

```

## 🚀 Quick Start

### Prerequisites

- **Node.js** ≥20.0 and npm
- **Python** 3.11+
- **API Keys**:
  - Groq API key ([get one here](https://console.groq.com/keys))
  - Qdrant Cloud account ([free tier](https://cloud.qdrant.io/))
  - Neon Postgres database ([create one](https://neon.tech/))

### 1. Clone and Setup

```bash
cd Hackaton

# Create .env file in both root and backend/
cat > .env <<EOF
GROQ_API_KEY=your_groq_api_key_here
QDRANT_URL=your_qdrant_url_here
QDRANT_API_KEY=your_qdrant_api_key_here
NEON_DATABASE_URL=your_neon_database_url_here
SECRET_KEY=$(openssl rand -hex 32)
EOF

cp .env backend/.env
```

### 2. Run Backend

```bash
cd backend

# Create virtual environment
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate

# Install dependencies
pip install -r requirements.txt

# Run FastAPI server
uvicorn src.main:app --reload
```

Backend will start at `http://localhost:8000`

**On startup**, the backend automatically:
1. Connects to Qdrant and creates the `textbook` collection
2. Indexes all markdown files from `frontend/docs/`
3. Generates embeddings using SentenceTransformers `all-MiniLM-L6-v2` (open-source model)

### 3. Run Frontend

```bash
cd frontend

# Install dependencies
npm install

# Start development server
npm start
```

Frontend will open at `http://localhost:3000`

## 🧪 Testing the Features

### Test RAG Chatbot
1. Navigate to any documentation page
2. Ask a question in the chatbot widget (e.g., "What is ROS 2?")
3. The chatbot retrieves relevant content and generates an answer

### Test Text Selection
1. Select text on any page
2. Ask a question about the selected text
3. The chatbot will answer based only on that selection

### Test Authentication
1. Click "Sign In" and create an account
2. Answer the software/hardware background questions
3. Your responses are saved for personalization

### Test Translation
1. Click the "Translate to Urdu" button at the top of any chapter
2. Content is translated using the translation API

## 📦 Building for Production

### Frontend (GitHub Pages)

```bash
cd frontend
npm run build
```

The GitHub Actions workflow (`.github/workflows/deploy.yml`) automatically deploys to GitHub Pages on push to `main` or `master`.

### Backend Deployment

For production, deploy the FastAPI backend to:
- **Railway** / **Render** / **Fly.io** (recommended for quick deployment)
- **AWS ECS** / **Google Cloud Run** (containerized)
- **DigitalOcean App Platform** (straightforward Python support)

Update `frontend/src/services/chat_api.js` with your production backend URL.

## 🔧 Configuration

### Update GitHub Pages Settings

Edit `frontend/docusaurus.config.js`:

```javascript
url: 'https://YOUR_GITHUB_USERNAME.github.io',
baseUrl: '/YOUR_REPO_NAME/',
organizationName: 'YOUR_GITHUB_USERNAME',
projectName: 'YOUR_REPO_NAME',
```

### Environment Variables

| Variable | Description | Example |
|----------|-------------|---------|
| `OPENAI_API_KEY` | OpenAI API key for embeddings and chat | `sk-proj-...` |
| `QDRANT_URL` | Qdrant Cloud instance URL | `https://xxx.gcp.cloud.qdrant.io` |
| `QDRANT_API_KEY` | Qdrant API key | `eyJhbG...` |
| `NEON_DATABASE_URL` | Neon Postgres connection string | `postgresql+psycopg://...` |
| `SECRET_KEY` | JWT signing secret (generate with `openssl rand -hex 32`) | Random 64-char hex |

## 📖 API Documentation

Once the backend is running, visit:
- **Interactive API docs**: http://localhost:8000/docs
- **Alternative docs**: http://localhost:8000/redoc

### Key Endpoints

- `POST /api/v1/chat` - Chat with RAG bot
- `POST /api/v1/auth/signup` - Create account
- `POST /api/v1/auth/token` - Login
- `POST /api/v1/translation/translate` - Translate text

## 🛠️ Development

### Project Constitution

See `.specify/memory/constitution.md` for:
- Code quality standards
- Testing requirements
- Security guidelines
- Performance targets

### Specifications

Detailed specs in `specs/1-system-specification/`:
- `spec.md` - Feature requirements and user stories
- `plan.md` - Technical architecture and design decisions
- `tasks.md` - Implementation task breakdown
- `data-model.md` - Database schemas and API contracts

## 🤝 Contributing

This is a hackathon submission project. For questions or suggestions:
1. Check the `specs/` directory for design decisions
2. Review the constitution for development guidelines
3. Open an issue for bugs or feature requests

## 📄 License

This project is created for the Panaversity Physical AI Hackathon.

---

**Built with**: Docusaurus, FastAPI, OpenAI, Qdrant, Neon Postgres, better-auth.com