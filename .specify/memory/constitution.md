# Physical AI Textbook Project Constitution

## Core Principles

### I. AI-Native Content First
All textbook content must be designed for AI-assisted learning:
- Content is structured for retrieval-augmented generation (RAG)
- Clear, concise explanations optimized for embedding and search
- Practical examples that can be understood by both humans and AI agents
- Modular design enabling personalization and adaptive learning

### II. User-Centered Experience
The learning experience must prioritize accessibility and engagement:
- Responsive design that works across all devices
- Interactive chatbot for instant question answering
- Personalized content based on user background and learning style
- Multi-language support (starting with English and Urdu)
- Text selection for contextual assistance

### III. Code Quality and Testing
Code must be maintainable, tested, and documented:
- Backend: FastAPI with type hints, comprehensive error handling
- Frontend: React components with clear separation of concerns
- Unit tests for core functionality (RAG pipeline, auth, personalization)
- Integration tests for API endpoints
- Linting enforced (ESLint for JS/TS, Flake8/Black for Python)

### IV. Technical Excellence
The stack must leverage industry-standard technologies:
- **Frontend**: Docusaurus (React-based static site generator)
- **Backend**: FastAPI (Python async web framework)
- **Database**: Neon Serverless Postgres (relational data)
- **Vector Store**: Qdrant Cloud (embeddings and similarity search)
- **AI**: OpenAI GPT models and embeddings
- **Auth**: better-auth.com (modern authentication)

### V. Performance and Scalability
The system must be performant within free-tier constraints:
- Chatbot responses begin streaming within 3 seconds
- Page loads complete within 2 seconds
- Efficient embedding strategy (chunking, caching)
- Respect rate limits and free-tier quotas
- Lazy loading for non-critical features

### VI. Security and Privacy
User data must be protected:
- Environment variables for all secrets (never commit credentials)
- JWT tokens for authentication with secure expiration
- HTTPS for all production deployments
- SQL injection prevention via parameterized queries
- XSS protection via React's built-in escaping

## Development Workflow

### Version Control
- Feature branches for all changes
- Descriptive commit messages following conventional commits
- Pull requests require passing tests before merge
- Main branch always deployable

### Documentation Standards
- README.md with quickstart instructions
- API endpoints documented via OpenAPI/Swagger
- Inline code comments for complex logic
- Architecture decisions recorded in ADRs when significant

### Content Creation
- All course modules follow the structure defined in Hackathon requirements
- Content written in Markdown for Docusaurus compatibility
- Each chapter includes learning objectives and practical examples
- Technical accuracy verified through research and testing

## Deployment and Operations

### CI/CD Pipeline
- GitHub Actions for automated deployment
- Deploy to GitHub Pages on merge to main
- Backend deployed separately (containerized environment)
- Environment-specific configurations

### Monitoring
- Error logging for backend API failures
- User interaction tracking (anonymized)
- Performance metrics for chatbot response times
- Uptime monitoring for external services

## Governance

This constitution defines the standards for the Physical AI Textbook Project. All development must align with these principles:

- **Constitution Supersedes**: In case of conflict, constitution principles override individual preferences
- **Amendment Process**: Changes require documentation and team approval
- **Compliance Verification**: All PRs must demonstrate adherence to code quality, testing, and security standards
- **Continuous Improvement**: Regular retrospectives to refine processes and update constitution as needed

**Version**: 1.0.0 | **Ratified**: 2025-12-09 | **Last Amended**: 2025-12-09
