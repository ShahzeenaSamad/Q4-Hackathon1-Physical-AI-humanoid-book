<!--
Sync Impact Report:
- Version: 0.0.0 → 1.0.0 (Initial constitution creation)
- Modified Principles: N/A (new creation)
- Added Sections: All core principles, Technical Standards, Development Workflow, Governance
- Removed Sections: None
- Templates Status:
  ✅ plan-template.md - aligned with AI-native development principles
  ✅ spec-template.md - aligned with textbook structure requirements
  ✅ tasks-template.md - aligned with modular implementation approach
  ⚠ Review command files after initial implementation
- Follow-up TODOs: Monitor template consistency as project evolves
-->

# Physical AI Textbook Project Constitution

## Core Principles

### I. AI-Native Content Development

All textbook content MUST be created using AI-assisted tools (Claude Code with Spec-Kit Plus) following spec-driven development methodology. Content generation follows this workflow:

- Specification created first defining learning outcomes and module structure
- Implementation plan designed with clear architectural decisions
- Tasks broken down into testable, incremental units
- Each module independently reviewed for pedagogical quality

**Rationale**: Ensures systematic, high-quality content creation aligned with modern AI-native publishing practices and Panaversity's mission.

### II. Modular Content Architecture

The textbook MUST be structured into four discrete modules with clear boundaries:

1. **Module 1**: ROS 2 (Robot Operating System) - Weeks 1-5
2. **Module 2**: Gazebo & Unity Simulation - Weeks 6-7
3. **Module 3**: NVIDIA Isaac Platform - Weeks 8-10
4. **Module 4**: Vision-Language-Action (VLA) - Weeks 11-13

Each module MUST:
- Stand independently while building on prerequisites
- Include hands-on examples and exercises
- Provide clear learning outcomes
- Reference specific hardware/software requirements

**Rationale**: Modular structure enables progressive learning, flexible teaching schedules, and maintainable content updates.

### III. Integrated RAG Chatbot (NON-NEGOTIABLE)

The published textbook MUST include an embedded Retrieval-Augmented Generation (RAG) chatbot with the following capabilities:

- Answer questions about book content using OpenAI Agents/ChatKit SDKs
- Support context-aware queries on user-selected text passages
- Backend: FastAPI with Neon Serverless Postgres and Qdrant Cloud (free tier)
- Real-time response with citation to relevant textbook sections

**Rationale**: Interactive AI assistance enhances learning outcomes and demonstrates practical AI agent integration - core to the hackathon requirements.

### IV. Docusaurus Foundation

The textbook MUST be built using Docusaurus v3+ and deployed to GitHub Pages or Vercel with:

- Mobile-responsive design
- Fast page load times (<3s initial load)
- Markdown-based content for easy maintenance
- Integrated search functionality
- Dark mode support

**Rationale**: Docusaurus provides a production-ready documentation framework optimized for technical content and modern web standards.

### V. Progressive Enhancement Strategy

Base functionality (100 points) MUST be completed first before implementing bonus features:

**Base Requirements**:
- Complete textbook with all 4 modules
- Deployed Docusaurus site
- Functional RAG chatbot

**Bonus Features** (implement in priority order):
1. Authentication system using better-auth (signup with background questions)
2. Personalized content based on user background (software/hardware experience)
3. Content personalization button per chapter
4. Urdu translation feature per chapter

**Rationale**: Ensures core deliverables are functional before investing in enhancements; aligns with hackathon scoring structure.

### VI. Code Quality & Security Standards

All code (backend, frontend, chatbot) MUST adhere to:

- **Security**: Input validation, SQL injection prevention, XSS protection, secure API key management
- **Testing**: Unit tests for critical functions, integration tests for RAG pipeline
- **Documentation**: README with setup instructions, API documentation, deployment guide
- **Version Control**: Semantic versioning, clear commit messages, feature branches
- **Performance**: API response times <2s, database query optimization, efficient vector search

**Rationale**: Professional code standards ensure maintainability, security, and scalability for production deployment.

### VII. Pedagogical Excellence

Content MUST prioritize teaching effectiveness:

- **Progressive Complexity**: Start with foundations, build to advanced topics
- **Practical Examples**: Real-world robot control scenarios, simulation exercises
- **Visual Learning**: Diagrams for ROS 2 architecture, screenshots of Isaac Sim, video demonstrations
- **Assessment Alignment**: Learning outcomes map to assessments and capstone project
- **Industry Relevance**: Current tools (ROS 2 Humble/Iron, Isaac Sim 2024+, latest GPT models)

**Rationale**: Educational content quality directly impacts learning outcomes and Panaversity's teaching mission.

### VIII. Reusable Intelligence & Agent Skills

Project SHOULD leverage Claude Code's agent capabilities:

- Create custom subagents for repetitive tasks (content generation, code review, formatting)
- Develop reusable skills for common workflows (module scaffolding, chatbot testing)
- Document agent configurations for team knowledge sharing
- Maximize the 50 bonus points for demonstrating advanced Claude Code usage

**Rationale**: Demonstrates mastery of AI agent development and creates reusable assets for future projects.

## Technical Standards

### Technology Stack

**Frontend**:
- Docusaurus 3.x (React-based)
- TypeScript for type safety
- Tailwind CSS for styling (optional enhancement)

**Backend (RAG Chatbot)**:
- FastAPI (Python 3.10+)
- OpenAI Python SDK for LLM integration
- Neon Serverless Postgres for user data and chat history
- Qdrant Cloud (free tier) for vector embeddings

**Authentication** (bonus feature):
- better-auth library
- PostgreSQL session store
- JWT token management

**Deployment**:
- GitHub Pages or Vercel for frontend
- Railway/Fly.io/Render for FastAPI backend
- Environment variables for all secrets

### Data Management

- **Vector Embeddings**: Generate embeddings for all textbook content using OpenAI `text-embedding-3-small`
- **Database Schema**: Users, chat sessions, embeddings, personalization preferences
- **Backup Strategy**: Regular Postgres backups, version control for content
- **Privacy**: User data stored securely, GDPR-compliant data handling

### API Design

- RESTful endpoints for chatbot queries: `POST /api/chat`
- Streaming responses for real-time chat experience
- Rate limiting: 100 requests/hour per user
- Error handling with descriptive messages
- CORS configuration for cross-origin requests

## Development Workflow

### Specification-Driven Process

1. **Specification Phase** (`/sp.specify`):
   - Define feature requirements and acceptance criteria
   - Document user stories and edge cases
   - Review and approve before implementation

2. **Planning Phase** (`/sp.plan`):
   - Design technical architecture
   - Identify dependencies and risks
   - Create ADRs for significant decisions

3. **Task Breakdown** (`/sp.tasks`):
   - Generate dependency-ordered task list
   - Each task includes test cases and acceptance criteria
   - Tasks are atomic and independently verifiable

4. **Implementation Phase** (`/sp.implement`):
   - Execute tasks in order
   - Run tests continuously (TDD where applicable)
   - Document code with inline comments

5. **Review & Deploy**:
   - Code review checklist validation
   - Integration testing with full stack
   - Deployment to staging before production

### Git Workflow

- **Branching Strategy**: Feature branches from `main`
- **Commit Messages**: Conventional commits format (`feat:`, `fix:`, `docs:`)
- **Pull Requests**: Required for all changes, include demo video for UI changes
- **GitHub Actions**: Automated tests and deployment on merge to `main`

### Quality Gates

Before marking tasks complete:
- [ ] All tests pass
- [ ] No console errors or warnings
- [ ] Code reviewed against constitution principles
- [ ] Documentation updated
- [ ] Deployment successful

## Submission Requirements

### Hackathon Deliverables

1. **Public GitHub Repository**:
   - Clear README with project description
   - Setup instructions for local development
   - Architecture documentation
   - Link to deployed site

2. **Deployed Application**:
   - Live textbook accessible at public URL
   - RAG chatbot functional and responsive
   - All bonus features (if implemented) working

3. **Demo Video** (≤90 seconds):
   - Show textbook navigation and content quality
   - Demonstrate RAG chatbot answering complex questions
   - Highlight bonus features (auth, personalization, translation)
   - Use NotebookLM or screen recording tool

4. **Submission Form Data**:
   - GitHub repo URL
   - Deployed site URL
   - Demo video link (YouTube/Vimeo)
   - WhatsApp number for live presentation invitation

### Documentation Standards

- **README.md**: Project overview, setup, tech stack, team
- **ARCHITECTURE.md**: System design, data flow, API contracts
- **DEPLOYMENT.md**: Hosting configuration, environment variables, CI/CD
- **USER_GUIDE.md**: How to use the textbook and chatbot features

## Governance

### Amendment Process

1. **Proposal**: Document proposed changes with rationale
2. **Review**: Team discussion and impact analysis
3. **Approval**: Consensus required for principle changes
4. **Migration**: Update all dependent templates and documentation
5. **Version Bump**: Follow semantic versioning rules

### Versioning Policy

- **MAJOR**: Backward-incompatible changes (e.g., removing a principle, changing tech stack)
- **MINOR**: New principles or sections added (e.g., adding security principle)
- **PATCH**: Clarifications, wording improvements, typo fixes

### Compliance Review

- All pull requests MUST verify compliance with constitution principles
- Weekly constitution review during development sprints
- Document deviations with justification in ADR format

### Conflict Resolution

When principles conflict in practice:
1. Document the conflict scenario
2. Assess which principle has higher priority for project success
3. Create ADR documenting the decision and tradeoffs
4. Update constitution if conflict reveals ambiguity

**Version**: 1.0.0 | **Ratified**: 2025-12-26 | **Last Amended**: 2025-12-26
