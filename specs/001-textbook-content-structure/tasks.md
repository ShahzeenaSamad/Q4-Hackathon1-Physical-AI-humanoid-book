# Tasks: Physical AI Textbook Content Structure

**Input**: Design documents from `C:\Users\SheZziiii SaM\Hackathon 1\specs\001-textbook-content-structure\`
**Prerequisites**: plan.md (implementation plan), spec.md (user stories), constitution.md (project principles)

**Tests**: Tests are OPTIONAL for this project. This textbook project focuses on content delivery and user experience validation rather than TDD. Quality is ensured through content review checkl

ists and manual testing.

**Organization**: Tasks are grouped by user story (US1-US4) to enable independent implementation and testing. Base deliverable (100 points) = US1 + US4. Bonus features are separate phases.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (US1, US2, US3, US4)
- Include exact file paths in descriptions

## Path Conventions

- **Web app structure**: `content/docs/` (markdown chapters), `frontend/src/` (Docusaurus React), `backend/src/` (FastAPI Python)
- **Static assets**: `static/img/`, `static/code-examples/`
- **Config files**: `frontend/docusaurus.config.js`, `frontend/sidebars.js`

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure for textbook delivery

- [ ] T001 Create root project directory structure (content/, frontend/, backend/, static/, scripts/)
- [ ] T002 Initialize frontend/ as Docusaurus 3.x TypeScript project with `npx create-docusaurus@latest frontend classic --typescript`
- [ ] T003 [P] Initialize backend/ as FastAPI Python 3.10+ project with `python -m venv venv` and create requirements.txt
- [ ] T004 [P] Create .env.example file with placeholders for OPENAI_API_KEY, NEON_DATABASE_URL, QDRANT_API_KEY, QDRANT_CLUSTER_URL, JWT_SECRET
- [ ] T005 [P] Create .gitignore file excluding venv/, node_modules/, .env, __pycache__/, build/, .docusaurus/
- [ ] T006 Create README.md with project overview, setup instructions, and hackathon requirements (100pt base + 200pt bonus)
- [ ] T007 [P] Create ARCHITECTURE.md documenting system design (Docusaurus frontend + FastAPI backend + RAG chatbot architecture)

**Checkpoint**: Project structure initialized, dependencies ready to install

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before user stories (content and RAG chatbot foundation)

**⚠️ CRITICAL**: No content development or RAG implementation can begin until this phase is complete

- [ ] T008 Configure frontend/docusaurus.config.js with site metadata (title: "Physical AI & Humanoid Robotics", tagline, baseUrl, organizationName)
- [ ] T009 Configure frontend/sidebars.js with 4-module structure (module-1-ros2, module-2-simulation, module-3-isaac, module-4-vla)
- [ ] T010 [P] Create content/docs/ directory structure with subdirectories for each module (module-1-ros2/, module-2-simulation/, module-3-isaac/, module-4-vla/, appendices/)
- [ ] T011 [P] Create static/img/ directory structure with subdirectories per module (module-1/, module-2/, module-3/, module-4/)
- [ ] T012 [P] Create static/code-examples/ directory with subdirectories (ros2/, gazebo/, isaac/, vla/)
- [ ] T013 Install Docusaurus dependencies with `cd frontend && npm install`
- [ ] T014 [P] Install backend Python dependencies (fastapi, uvicorn, openai, qdrant-client, psycopg2-binary, python-jose, langchain) with `pip install -r backend/requirements.txt`
- [ ] T015 Create backend/src/core/config.py for environment variable management using pydantic BaseSettings
- [ ] T016 [P] Create backend/src/core/database.py for Neon Postgres connection with asyncpg
- [ ] T017 [P] Create backend/src/main.py FastAPI application entry point with CORS middleware configured for frontend origin
- [ ] T018 Create backend/src/api/health.py with GET /health endpoint returning {"status": "ok", "service": "textbook-rag-api"}
- [ ] T019 Test backend startup with `uvicorn src.main:app --reload` and verify http://localhost:8000/health responds
- [ ] T020 Test frontend startup with `npm start` in frontend/ and verify http://localhost:3000 loads Docusaurus default page

**Checkpoint**: Foundation ready - Docusaurus configured, FastAPI running, both services verified operational

---

## Phase 3: User Story 4 - Content Author Module Development (Priority: P1) 🎯 FOUNDATIONAL

**Goal**: Enable content authors (AI agents) to systematically generate chapters following specification, with clear templates and validation checklists

**Why First**: US4 must precede US1 because authors need templates/standards before writing student-facing content. This establishes the content creation workflow.

**Independent Test**: Provide chapter template to Claude Code agent, generate one chapter (e.g., Chapter 3: ROS 2 Nodes and Topics), validate against checklist (learning outcomes, code examples, exercises, assessments all present and properly formatted)

### Implementation for User Story 4

- [ ] T021 [P] [US4] Create content/docs/_templates/chapter-template.md with mandatory sections (Title, Prerequisites, Learning Outcomes, Introduction, Core Content, Code Examples, Hands-On Exercise, Summary, Assessment Questions, Estimated Time)
- [ ] T022 [P] [US4] Create content/docs/_templates/module-summary-template.md for Chapters 8, 13, 18, 24
- [ ] T023 [P] [US4] Create scripts/validate_content.py Python script to check chapter structure (validates presence of ## Learning Outcomes, ## Hands-On Exercise, ## Assessment Questions sections, counts code blocks ≥2, checks for estimated time)
- [ ] T024 [P] [US4] Create checklists/chapter-quality.md checklist with items: 3-5 learning outcomes, 2-5 code examples, 1-2 exercises, 5-10 assessments, 2+ visuals, troubleshooting tips
- [ ] T025 [US4] Document code formatting standards in content/docs/_templates/code-style-guide.md (Python examples use Black formatting, inline comments explain non-obvious logic, execution environment specified as "Ubuntu 22.04 + ROS 2 Humble")
- [ ] T026 [US4] Create example chapter content/docs/module-1-ros2/03-nodes-topics.md fully populated per template (serves as reference for AI agents generating other chapters)
- [ ] T027 [US4] Test scripts/validate_content.py on example chapter 03-nodes-topics.md and verify all checks pass
- [ ] T028 [US4] Create .claude/agents/chapter-generator-agent.md documenting Claude Code subagent configuration for systematic chapter generation (inputs: chapter number + title + topics, outputs: complete markdown file validated against checklist)

**Checkpoint**: Content authoring workflow established. Authors can now generate chapters 1-24 following templates. This unblocks US1 (student learning).

---

## Phase 4: User Story 1 - Student Learning Complete Module (Priority: P1) 🎯 MVP (BASE 100 POINTS)

**Goal**: Deliver complete textbook with 24 chapters across 4 modules, enabling students to learn Physical AI progressively from ROS 2 → Simulation → Isaac → VLA

**Independent Test**: Student accesses http://localhost:3000/docs/module-1-ros2/01-introduction-physical-ai, reads through Module 1 (Chapters 1-8) sequentially, completes hands-on exercises, verifies comprehension via assessment questions. Module should be self-contained with clear learning outcomes.

**Implementation Strategy for US1**: Generate 24 chapters in dependency order (Module 1 first, as Modules 2-4 reference ROS 2 concepts). Parallelize within modules where chapters are independent. For hackathon demo, prioritize 3-4 fully complete chapters per module over 24 partial chapters.

### Module 1: ROS 2 Fundamentals (Chapters 1-8)

- [ ] T029 [P] [US1] Write content/docs/module-1-ros2/01-introduction-physical-ai.md (Learning outcomes: Define physical AI, differentiate from digital AI, identify challenges | Topics: What is Physical AI, embodied agents, humanoid landscape | Hands-on: Install Ubuntu 22.04 WSL2/native | Assessments: 5 MCQs | Estimated time: 1.5 hours)
- [ ] T030 [P] [US1] Write content/docs/module-1-ros2/02-introduction-ros2.md (Learning outcomes: Explain ROS 2 architecture, install ROS 2 Humble, run basic commands | Topics: Why ROS 2, nodes/topics/messages, installation guide, colcon workspace | Hands-on: Create workspace + turtlesim demo | Assessments: 7 questions | Estimated time: 2.5 hours)
- [ ] T031 [P] [US1] Create static/img/module-1/ros2-architecture-diagram.png illustrating ROS 2 node graph with topics (use draw.io or similar, export PNG)
- [ ] T032 [P] [US1] Write content/docs/module-1-ros2/04-services-actions.md (Learning outcomes: Implement service calls, design action servers, choose communication patterns | Topics: Services (request-response), Actions (feedback), custom srv/action definitions | Hands-on: Robot arm positioning service + path following action | Assessments: 8 questions | Estimated time: 2 hours)
- [ ] T033 [P] [US1] Write content/docs/module-1-ros2/05-launch-files-parameters.md (Learning outcomes: Write launch files, configure parameters, organize complex systems | Topics: XML vs Python launch files, parameter overriding, namespaces/remapping | Hands-on: Multi-node robot system launch file | Assessments: Debug exercise | Estimated time: 1.5 hours)
- [ ] T034 [P] [US1] Create static/code-examples/ros2/launch_file_example.py demonstrating Python launch file with parameter configuration
- [ ] T035 [P] [US1] Write content/docs/module-1-ros2/06-understanding-urdf.md (Learning outcomes: Parse URDF files, define robot kinematics, visualize in RViz | Topics: URDF structure, Xacro, joint types, collision vs visual meshes | Hands-on: 3-DOF robotic arm URDF + RViz visualization | Assessments: 6 questions | Estimated time: 2 hours)
- [ ] T036 [P] [US1] Create static/code-examples/ros2/simple_arm.urdf demonstrating 3-DOF arm with revolute joints
- [ ] T037 [P] [US1] Write content/docs/module-1-ros2/07-best-practices.md (Learning outcomes: Apply coding standards, implement error handling/logging, write unit tests | Topics: Package organization, QoS policies, logging (RCLPY_INFO/ERROR), testing with unittest | Hands-on: Refactor poorly structured package | Assessments: Code review exercise | Estimated time: 2 hours)
- [ ] T038 [US1] Write content/docs/module-1-ros2/08-module-summary.md using module-summary-template.md (Recap Module 1 concepts, common pitfalls, debugging strategies, preview of Gazebo integration | Hands-on: Mini-capstone - simple robot controller using all ROS 2 concepts | Assessments: 10 cumulative questions | Estimated time: 3 hours)
- [ ] T039 Run scripts/validate_content.py on Module 1 chapters (01-08) and fix any validation errors

### Module 2: Gazebo & Unity Simulation (Chapters 9-13)

- [ ] T040 [P] [US1] Write content/docs/module-2-simulation/09-introduction-simulation.md (Learning outcomes: Explain simulation benefits, compare Gazebo/Unity/Isaac, setup Gazebo Garden | Topics: Why simulate, fidelity vs performance, Gazebo architecture, installation | Hands-on: Launch Gazebo with default worlds | Assessments: 5 MCQs | Estimated time: 1.5 hours)
- [ ] T041 [P] [US1] Write content/docs/module-2-simulation/10-sdf-world-building.md (Learning outcomes: Create custom Gazebo worlds using SDF, add environmental elements, design test scenarios | Topics: SDF syntax vs URDF, world files, model insertion, physics engine config | Hands-on: Warehouse environment with obstacles | Assessments: Custom world file task | Estimated time: 2.5 hours)
- [ ] T042 [P] [US1] Create static/code-examples/gazebo/warehouse_world.sdf demonstrating SDF world file with lighting, terrain, obstacles
- [ ] T043 [P] [US1] Write content/docs/module-2-simulation/11-sensor-simulation.md (Learning outcomes: Integrate simulated sensors (LiDAR/cameras/IMU), process sensor data in ROS 2, validate accuracy | Topics: LiDAR point clouds, RGB-D cameras, IMU data, Gazebo sensor plugins | Hands-on: Add sensors to URDF robot + visualize in RViz | Assessments: 8 questions | Estimated time: 2 hours)
- [ ] T044 [P] [US1] Write content/docs/module-2-simulation/12-unity-high-fidelity.md (Learning outcomes: Setup Unity with ROS 2 integration, render realistic environments, compare Gazebo vs Unity | Topics: Unity Robotics Hub, URDF Importer, ROS-TCP-Connector, visual vs physics simulation | Hands-on: Import Gazebo robot into Unity, control via ROS 2 | Assessments: 5 questions | Estimated time: 2 hours)
- [ ] T045 [US1] Write content/docs/module-2-simulation/13-module-summary.md (Recap Module 2, troubleshooting Gazebo issues, preview Isaac Sim advantages | Hands-on: Complete robot system (ROS 2 nodes + Gazebo environment) | Assessments: 8 cumulative questions | Estimated time: 2.5 hours)
- [ ] T046 Run scripts/validate_content.py on Module 2 chapters (09-13) and fix any validation errors

### Module 3: NVIDIA Isaac Platform (Chapters 14-18)

- [ ] T047 [P] [US1] Write content/docs/module-3-isaac/14-introduction-isaac.md (Learning outcomes: Explain Isaac Sim/Isaac ROS roles, setup on RTX systems or cloud, run first simulation | Topics: Isaac ecosystem, Omniverse/USD, system requirements, cloud alternatives (AWS, Omniverse Cloud) | Hands-on: Install Isaac Sim + launch Carter robot demo | Assessments: 5 MCQs | Estimated time: 2 hours)
- [ ] T048 [P] [US1] Write content/docs/module-3-isaac/15-isaac-sim-photorealistic.md (Learning outcomes: Create photorealistic environments, generate synthetic data, integrate ROS 2 with Isaac Sim | Topics: USD scene composition, PhysX physics, camera/sensor simulation, ROS 2 Bridge | Hands-on: Build warehouse scene + capture synthetic RGB-D data | Assessments: 7 questions | Estimated time: 3 hours)
- [ ] T049 [P] [US1] Create static/img/module-3/isaac-sim-warehouse-screenshot.png showing photorealistic Isaac Sim environment
- [ ] T050 [P] [US1] Write content/docs/module-3-isaac/16-isaac-ros-perception.md (Learning outcomes: Deploy Isaac ROS nodes on Jetson, implement VSLAM, optimize perception pipelines | Topics: Isaac ROS packages, VSLAM with visual odometry, stereo depth estimation, CUDA acceleration | Hands-on: Run VSLAM on RealSense/simulated data | Assessments: Coding challenge - configure Isaac ROS parameters | Estimated time: 2.5 hours)
- [ ] T051 [P] [US1] Write content/docs/module-3-isaac/17-navigation-nav2.md (Learning outcomes: Configure Nav2 for humanoid locomotion, implement path planning, handle dynamic obstacles | Topics: Nav2 architecture, costmaps/inflation, planner plugins (DWB, TEB), recovery behaviors | Hands-on: Configure Nav2 for bipedal humanoid in Isaac Sim | Assessments: 8 questions | Estimated time: 2.5 hours)
- [ ] T052 [US1] Write content/docs/module-3-isaac/18-module-summary.md (Recap Isaac Sim + Isaac ROS, performance optimization tips, preview cognitive planning Module 4 | Hands-on: Perception-to-navigation pipeline (mini-capstone) | Assessments: 10 cumulative questions | Estimated time: 3 hours)
- [ ] T053 Run scripts/validate_content.py on Module 3 chapters (14-18) and fix any validation errors

### Module 4: Vision-Language-Action (Chapters 19-24)

- [ ] T054 [P] [US1] Write content/docs/module-4-vla/19-introduction-vla.md (Learning outcomes: Define VLA models, explain LLM role in robotics, identify VLA use cases | Topics: What is VLA, LLMs as cognitive planners, multimodal AI (vision+language+action), industry examples (RT-2, PaLM-E) | Hands-on: Interact with GPT-4 to generate robot task plans | Assessments: 5 MCQs | Estimated time: 1.5 hours)
- [ ] T055 [P] [US1] Write content/docs/module-4-vla/20-voice-action-whisper.md (Learning outcomes: Integrate OpenAI Whisper for speech recognition, convert voice commands to ROS 2 actions, handle noisy audio | Topics: Speech recognition fundamentals, Whisper API setup, audio processing pipeline, mapping speech to actions | Hands-on: Voice-controlled robot executing "Move forward 2 meters" | Assessments: 6 questions | Estimated time: 2 hours)
- [ ] T056 [P] [US1] Create static/code-examples/vla/whisper_ros2_integration.py demonstrating Whisper audio transcription → ROS 2 action client
- [ ] T057 [P] [US1] Write content/docs/module-4-vla/21-cognitive-planning-llms.md (Learning outcomes: Use LLMs to decompose high-level tasks, implement feedback loops for error recovery, design prompts for robotics | Topics: Task decomposition ("Clean room" → pick/navigate/place), prompt engineering, LangChain agentic workflows, error handling/replanning | Hands-on: LLM-based planner generating ROS 2 action sequences | Assessments: Coding challenge - improve planner robustness | Estimated time: 2.5 hours)
- [ ] T058 [P] [US1] Write content/docs/module-4-vla/22-multimodal-interaction.md (Learning outcomes: Combine vision/language/action in unified system, implement object detection for task grounding, design HRI flows | Topics: Vision-Language Models (CLIP, LLaVA), grounding language in visual scenes, gesture recognition, human feedback | Hands-on: Robot identifies objects via camera + responds to voice commands | Assessments: 8 questions | Estimated time: 2.5 hours)
- [ ] T059 [US1] Write content/docs/module-4-vla/23-capstone-project.md (Learning outcomes: Integrate all 4 modules into complete system, demonstrate end-to-end autonomous behavior, reflect on learnings | Topics: Capstone specification, system integration checklist, testing/validation, future directions in Physical AI | Hands-on: FINAL PROJECT - Simulated humanoid receives voice command → plans path (Nav2) → navigates (Isaac Sim) → identifies object (Isaac ROS) → manipulates (ROS 2 control) → provides verbal feedback (TTS) | Assessments: Project rubric (20 points: functionality, integration, documentation, presentation) | Estimated time: 8 hours)
- [ ] T060 [US1] Write content/docs/module-4-vla/24-course-conclusion.md (Learning outcomes: Synthesize knowledge across all modules, identify next steps for advanced learning | Topics: Course recap (ROS 2 → Simulation → Isaac → VLA), career paths in Physical AI, advanced resources (papers, courses, open-source contributions) | Hands-on: None (reflection) | Assessments: 5 reflective questions | Estimated time: 1 hour)
- [ ] T061 Run scripts/validate_content.py on Module 4 chapters (19-24) and fix any validation errors

### Appendices

- [ ] T062 [P] [US1] Write content/docs/appendices/hardware-requirements.md documenting workstation specs (RTX 4070 Ti+, 64GB RAM), Jetson Orin Nano kits, RealSense cameras, cloud alternatives with cost estimates (references example.md hardware section)
- [ ] T063 [P] [US1] Write content/docs/appendices/software-installation.md with step-by-step guides for Ubuntu 22.04 setup (native/dual-boot/WSL2), ROS 2 Humble installation (apt + source build), Gazebo Garden, Isaac Sim local/cloud, Python venv/conda
- [ ] T064 [P] [US1] Write content/docs/appendices/glossary.md with alphabetical list of technical terms (URDF, SLAM, VLA, ROS 2, Isaac Sim, Nav2, etc.) and concise definitions
- [ ] T065 [P] [US1] Write content/docs/appendices/troubleshooting.md with common errors by module (ROS 2 node communication issues, Gazebo crashes, Isaac Sim GPU errors) and debug workflows
- [ ] T066 [P] [US1] Write content/docs/appendices/additional-resources.md with recommended reading (papers, books), online communities (ROS Discourse, NVIDIA forums), video tutorials (supplementary), GitHub repos with example code

### Final Validation

- [ ] T067 Run `npm run build` in frontend/ and verify Docusaurus builds without errors (validates all markdown syntax, internal links, image references)
- [ ] T068 Test complete student journey: Navigate from Chapter 1 → Chapter 24 sequentially, verify all internal links work, code examples display correctly, images load
- [ ] T069 Validate all 24 chapters against checklists/chapter-quality.md checklist (3-5 learning outcomes, 2-5 code examples, 1-2 exercises, 5-10 assessments, 2+ visuals, troubleshooting tips) - create checklist results report
- [ ] T070 Estimate reading + hands-on time for each chapter and add to chapter front matter (format: "Estimated Time: 2 hours (45 min reading + 75 min hands-on)")

**Checkpoint**: At this point, User Story 1 should be fully functional. Students can access complete 24-chapter textbook, read through modules sequentially, complete hands-on exercises, verify comprehension via assessments. This delivers the BASE 100 POINTS requirement.

---

## Phase 5: RAG Chatbot Integration (BASE 100 POINTS - Part 2)

**Goal**: Integrate RAG chatbot into Docusaurus site, enabling students to ask questions about textbook content with citations to relevant sections

**Independent Test**: Student opens any chapter, types question "What is ROS 2?" into chatbot widget, receives AI response with citations to Chapter 2 sections, clicks citation and is taken to relevant chapter content

### Backend RAG Service

- [ ] T071 [P] Create backend/src/models/chat_session.py SQLAlchemy model with fields: id (UUID), messages (JSONB), created_at, updated_at
- [ ] T072 [P] Create backend/src/models/textbook_content.py model with fields: id (UUID), module_id (int), chapter_id (int), section_title (str), content (text), embedding_id (str), content_type (enum), learning_outcomes (str array), created_at
- [ ] T073 Create backend/migrations/ directory and initialize Alembic with `alembic init migrations`
- [ ] T074 Create Alembic migration for chat_session and textbook_content tables in backend/migrations/versions/001_initial_schema.py
- [ ] T075 Run `alembic upgrade head` to apply migrations to Neon Postgres database
- [ ] T076 [P] Create backend/src/services/embedding_service.py with generate_embeddings(text: str) function using OpenAI text-embedding-3-small model
- [ ] T077 [P] Create backend/src/services/vector_store_service.py with QdrantClient integration (create_collection, upsert_embeddings, search_similar methods)
- [ ] T078 Create scripts/generate_embeddings.py to batch process all 24 chapters: read markdown → chunk into 500-token segments with 100-token overlap → generate OpenAI embeddings → store in Qdrant collection "textbook_embeddings" with metadata (module_id, chapter_id, section_title, learning_outcomes)
- [ ] T079 Run scripts/generate_embeddings.py and verify Qdrant collection populated with ~500-1000 embeddings (validate via Qdrant Cloud dashboard)
- [ ] T080 Create backend/src/services/rag_service.py with answer_question(question: str, context_chapter_id: Optional[int]) method: embed question → search Qdrant → retrieve top 5 chunks → construct prompt with context → call OpenAI GPT-4 → return response + source citations
- [ ] T081 Create backend/src/api/chat.py with POST /api/v1/chat endpoint (request: {session_id?: str, message: str, selected_text?: str, context_chapter_id?: int}, response: {session_id: str, response: str, sources: [{chapter_id: int, section_title: str, snippet: str}]})
- [ ] T082 [P] Create backend/src/api/chat.py with GET /api/v1/chat/sessions/{session_id} endpoint returning chat history from database
- [ ] T083 [P] Create backend/src/api/chat.py with DELETE /api/v1/chat/sessions/{session_id} endpoint to clear chat history
- [ ] T084 Test POST /api/v1/chat endpoint via Postman/curl with sample questions ("What is ROS 2?", "How do I create a URDF file?") and verify responses include relevant sources with chapter_id and section_title

### Frontend Chatbot Widget

- [ ] T085 [P] Create frontend/src/components/ChatbotWidget/index.tsx React component with chat window UI (message list, input field, send button, minimize/maximize toggle)
- [ ] T086 [P] Create frontend/src/components/ChatbotWidget/ChatWindow.tsx subcomponent handling message display (user messages left-aligned, assistant messages right-aligned with source citations as clickable links)
- [ ] T087 [P] Create frontend/src/components/ChatbotWidget/styles.module.css with responsive styles (fixed bottom-right position, mobile-friendly, dark mode support)
- [ ] T088 Implement chatbot API integration in frontend/src/components/ChatbotWidget/index.tsx: POST to http://localhost:8000/api/v1/chat on message send, display response in chat window, handle loading states and errors
- [ ] T089 Implement source citation click handling: when user clicks "[Chapter 5: Launch Files]" citation, navigate to /docs/module-1-ros2/05-launch-files-parameters and scroll to relevant section
- [ ] T090 Implement selected-text query feature: detect text selection on page, show "Ask AI about this" tooltip, send selected text as context to RAG endpoint
- [ ] T091 Add ChatbotWidget component to Docusaurus theme in frontend/src/theme/Root.tsx (renders on all /docs/** pages)
- [ ] T092 Test chatbot integration: Open http://localhost:3000/docs/module-1-ros2/02-introduction-ros2, type "What are ROS 2 nodes?", verify response cites Chapter 3, click citation and verify navigation to Chapter 3

**Checkpoint**: RAG chatbot fully functional. Students can ask questions and receive AI responses with chapter citations. This completes BASE 100 POINTS deliverable.

---

## Phase 6: User Story 2 - Instructor Course Planning (Priority: P2)

**Goal**: Enable instructors to easily plan 13-week course by providing clear table of contents, chapter summaries, and learning outcomes for syllabus creation

**Independent Test**: Instructor accesses textbook, views complete table of contents with all 24 chapters and appendices, reviews chapter summaries and learning outcomes, successfully maps Weeks 1-2 to Chapters 1-3, Weeks 3-5 to Chapters 4-8, etc., and designs assessments aligned with learning outcomes

### Implementation for User Story 2

- [ ] T093 [US2] Create content/docs/index.md as landing page with course overview: 13-week curriculum, 4 modules, target audience (students with Python basics), prerequisites, success criteria (SC-001 to SC-010)
- [ ] T094 [US2] Enhance frontend/sidebars.js to include module descriptions in sidebar (Module 1: Weeks 1-5 | ROS 2 Fundamentals, Module 2: Weeks 6-7 | Gazebo & Unity, etc.)
- [ ] T095 [US2] Create content/docs/course-structure.md page with detailed week-by-week mapping: Week 1 (Chapters 1-2), Week 2 (Chapter 2 continued), Week 3 (Chapters 3-4), ..., Week 13 (Chapters 23-24) with lecture topics and recommended readings
- [ ] T096 [US2] Extract learning outcomes from all 24 chapters into content/docs/learning-outcomes-matrix.md table format (Chapter | Learning Outcome 1 | Learning Outcome 2 | ... | Bloom's Level) for instructor assessment design
- [ ] T097 [US2] Create content/docs/instructor-guide/ directory with files: assessment-guidelines.md (how to design exams from learning outcomes), grading-rubrics.md (rubrics for hands-on exercises and capstone project), lab-setup-instructions.md (hardware/software setup for in-class labs)
- [ ] T098 [US2] Add "Instructor Resources" link to Docusaurus navbar in frontend/docusaurus.config.js pointing to /docs/instructor-guide/
- [ ] T099 [US2] Test instructor journey: Navigate to course-structure.md, verify week-to-chapter mapping is clear, check learning-outcomes-matrix.md, verify instructors can design assessments using provided rubrics

**Checkpoint**: Instructor course planning tools complete. Instructors can create 13-week syllabus within 2 hours using provided structure.

---

## Phase 7: User Story 3 - Self-Paced Learner Navigation (Priority: P3)

**Goal**: Enable self-paced learners with intermediate skills to skip foundational content and jump to advanced topics (Isaac, VLA) with clear prerequisites listed

**Independent Test**: Learner with ROS 2 experience directly accesses Chapter 15 (Isaac Sim), sees prerequisite knowledge clearly listed (ROS 2 nodes/topics, URDF basics), encounters unfamiliar concept (e.g., "sensor fusion"), clicks inline reference link back to Chapter 11 (Sensor Simulation)

### Implementation for User Story 3

- [ ] T100 [US3] Add "Prerequisites" section to every chapter front matter (Chapters 2-24) listing required prior knowledge (e.g., Chapter 15 prerequisites: "ROS 2 Humble installation, URDF syntax, Gazebo basics (Chapters 2, 6, 9)")
- [ ] T101 [US3] Add inline cross-reference links throughout chapters: when Chapter 15 mentions "ROS 2 nodes", insert hyperlink to Chapter 3 "ROS 2 Nodes and Topics" (use Docusaurus relative links: `[ROS 2 nodes](../module-1-ros2/03-nodes-topics.md#publishers-and-subscribers)`)
- [ ] T102 [US3] Create content/docs/learning-paths.md page with multiple learning tracks: "Full Course Path" (1→24 sequential), "Simulation-Focused Path" (1-3, 9-13, 23), "Isaac & Perception Path" (1-3, 14-18, 23), "VLA & AI Integration Path" (1-3, 19-24)
- [ ] T103 [US3] Add "Related Chapters" section at end of each chapter linking to prerequisite chapters (backward references) and dependent chapters (forward references)
- [ ] T104 [US3] Enhance content/docs/appendices/glossary.md to include chapter references for each term (e.g., "URDF - Unified Robot Description Format. See Chapter 6 for detailed explanation.")
- [ ] T105 [US3] Test self-paced learner journey: Start at Chapter 15 (skip Chapters 1-14), verify prerequisites clearly listed, follow inline reference link to Chapter 11, verify navigation works, check learning-paths.md and verify "Isaac & Perception Path" guidance is clear

**Checkpoint**: Self-paced learner navigation complete. Learners can skip to advanced topics with clear prerequisite guidance and cross-references.

---

## Phase 8: Bonus Feature - Authentication System (50 POINTS)

**Goal**: Implement better-auth signup/signin with background questions (software/hardware experience) to enable personalization and protected chat history

**Independent Test**: User visits textbook site, clicks "Sign Up", fills form with email/password + background questions (software experience: beginner/intermediate/advanced, hardware experience: none/hobbyist/professional), successfully signs in, chat history is saved and persists across sessions

### Implementation for Authentication

- [ ] T106 Install better-auth npm package in frontend/ with `npm install better-auth`
- [ ] T107 [P] Create frontend/src/components/AuthProvider/index.tsx React context providing authentication state (isAuthenticated, user, login, logout, signup methods)
- [ ] T108 [P] Create frontend/src/components/AuthProvider/SignupForm.tsx with fields: email, password, name, software_background (select: beginner/intermediate/advanced), hardware_background (select: none/hobbyist/professional)
- [ ] T109 [P] Create frontend/src/components/AuthProvider/SigninForm.tsx with fields: email, password
- [ ] T110 Create backend/src/models/user.py SQLAlchemy model with fields: id (UUID), email (str, unique), name (str), hashed_password (str), created_at
- [ ] T111 [P] Create backend/src/models/personalization_profile.py model with fields: id (UUID), user_id (UUID FK), software_background (enum), hardware_background (enum), learning_goals (str array), preferred_complexity (enum: simplified/standard/detailed), created_at, updated_at
- [ ] T112 Create Alembic migration for user and personalization_profile tables in backend/migrations/versions/002_auth_tables.py
- [ ] T113 Run `alembic upgrade head` to apply auth migrations
- [ ] T114 [P] Create backend/src/core/security.py with hash_password, verify_password functions using bcrypt, and create_access_token function using python-jose JWT
- [ ] T115 Create backend/src/api/auth.py with POST /api/v1/auth/signup endpoint (request: {email, password, name, software_background, hardware_background}, response: {user_id, access_token})
- [ ] T116 [P] Create backend/src/api/auth.py with POST /api/v1/auth/signin endpoint (request: {email, password}, response: {access_token, user})
- [ ] T117 [P] Create backend/src/api/auth.py with GET /api/v1/auth/profile endpoint (requires JWT authentication) returning {user: {id, email, name}, personalization: {software_background, hardware_background}}
- [ ] T118 Update backend/src/api/chat.py POST /api/v1/chat endpoint to accept optional Authorization header, save user_id to chat_session if authenticated
- [ ] T119 Update backend/src/api/chat.py GET /api/v1/chat/sessions/{session_id} to require authentication and verify user_id matches session owner
- [ ] T120 Test authentication flow: Sign up new user via frontend form, verify JWT token received, test protected endpoints require valid token, sign out and verify chat history is protected

**Checkpoint**: Authentication system complete (50 bonus points). Users can sign up with background questions, sign in, protected chat history persists.

---

## Phase 9: Bonus Feature - Content Personalization (50 POINTS)

**Goal**: Personalize chapter content based on user background (software/hardware experience) via "Personalize" button at chapter start, rewriting content to match user's complexity preference

**Independent Test**: Authenticated user with "beginner" software background opens Chapter 3 (ROS 2 Nodes), clicks "Personalize Content" button, content is rewritten with more explanatory text and simpler examples, cached for future visits

### Implementation for Personalization

- [ ] T121 [P] Create backend/src/models/personalized_content.py model with fields: id (UUID), user_id (UUID FK), chapter_id (int 1-24), personalized_markdown (text), generated_at, composite unique index (user_id, chapter_id)
- [ ] T122 Create Alembic migration for personalized_content table in backend/migrations/versions/003_personalization_table.py
- [ ] T123 Run `alembic upgrade head` to apply personalization migration
- [ ] T124 Create backend/src/services/personalization_service.py with personalize_chapter(chapter_id: int, user_profile: PersonalizationProfile) method: read original markdown → construct prompt based on user background ("Rewrite for beginner with no hardware experience: simplify technical jargon, add more explanations") → call OpenAI GPT-4 → return personalized markdown
- [ ] T125 Create backend/src/api/personalization.py with POST /api/v1/personalize/chapter endpoint (requires authentication, request: {chapter_id}, response: {chapter_id, personalized_content, cached: bool}) - checks cache first, generates if miss, stores in database
- [ ] T126 [P] Create backend/src/api/personalization.py with GET /api/v1/personalize/status/{chapter_id} endpoint (requires authentication) returning {available: bool, last_generated: timestamp}
- [ ] T127 [P] Create frontend/src/components/PersonalizationButton/index.tsx React component with "Personalize Content" button, loading state, and "Using Standard Content / Using Personalized Content" status indicator
- [ ] T128 Add PersonalizationButton component to Docusaurus MDX pages via frontMatter custom component (insert at top of each chapter after title and prerequisites)
- [ ] T129 Implement personalization logic in PersonalizationButton: on click → POST to /api/v1/personalize/chapter → replace page content with personalized markdown → show "Using Personalized Content" indicator with "Reset to Standard" option
- [ ] T130 Test personalization: Sign in as user with "beginner" software background, open Chapter 5 (Launch Files), click "Personalize Content", verify content is rewritten with simpler language, refresh page and verify personalized version persists (cached)

**Checkpoint**: Content personalization complete (50 bonus points). Users can personalize chapters based on background, cached for performance.

---

## Phase 10: Bonus Feature - Urdu Translation (50 POINTS)

**Goal**: Translate chapters to Urdu on demand via "Translate to Urdu" button, preserving technical terms (ROS 2, URDF, Isaac Sim) in English

**Independent Test**: User opens Chapter 2 (Introduction to ROS 2), clicks "Translate to Urdu" button, content is translated with technical terms preserved, cached for future users

### Implementation for Translation

- [ ] T131 [P] Create backend/src/models/translated_content.py model with fields: id (UUID), chapter_id (int 1-24), language_code (str 'ur'), translated_markdown (text), generated_at, composite unique index (chapter_id, language_code)
- [ ] T132 Create Alembic migration for translated_content table in backend/migrations/versions/004_translation_table.py
- [ ] T133 Run `alembic upgrade head` to apply translation migration
- [ ] T134 Create backend/src/services/translation_service.py with translate_chapter(chapter_id: int, target_language: str = 'ur') method: read original markdown → construct prompt ("Translate to Urdu. PRESERVE these terms in English: ROS 2, URDF, Gazebo, Isaac Sim, VSLAM, Nav2, LLM, VLA, Python. Translate surrounding text.") → call OpenAI GPT-4 → return translated markdown
- [ ] T135 Create backend/src/api/translation.py with POST /api/v1/translate/chapter endpoint (request: {chapter_id, target_language: 'ur'}, response: {chapter_id, translated_content, language: 'ur', cached: bool}) - checks cache first, generates if miss, stores in database
- [ ] T136 [P] Create backend/src/api/translation.py with GET /api/v1/translate/chapters endpoint returning {translations: [{chapter_id: int, languages: ['ur']}]} list of available translations
- [ ] T137 [P] Create frontend/src/components/TranslationButton/index.tsx React component with "Translate to Urdu / اردو میں ترجمہ کریں" button, loading state, and "Viewing in English / اردو میں دیکھ رہے ہیں" status indicator
- [ ] T138 Add TranslationButton component to Docusaurus MDX pages via frontMatter custom component (insert at top of each chapter next to PersonalizationButton)
- [ ] T139 Implement translation logic in TranslationButton: on click → POST to /api/v1/translate/chapter → replace page content with translated markdown → show "اردو میں دیکھ رہے ہیں" indicator with "Back to English" option
- [ ] T140 Test translation: Open Chapter 2, click "Translate to Urdu", verify content is translated with technical terms (ROS 2, URDF) preserved in English, refresh page and verify translation persists (cached)

**Checkpoint**: Urdu translation complete (50 bonus points). Users can translate chapters, technical terms preserved, cached for performance.

---

## Phase 11: Testing and Deployment (Final Validation)

**Goal**: Validate entire system, run tests, deploy to production, create demo video

**Independent Test**: Full end-to-end test: User visits deployed site at https://<username>.github.io/hackathon-1/, navigates through modules, uses chatbot, signs up/signs in, personalizes content, translates to Urdu, verifies all features work in production

### Testing

- [ ] T141 Create backend/tests/unit/test_embedding_service.py pytest tests for embedding generation (mock OpenAI API, verify embedding dimensions 1536)
- [ ] T142 [P] Create backend/tests/unit/test_rag_service.py pytest tests for RAG pipeline (mock Qdrant search, verify prompt construction, mock OpenAI completion)
- [ ] T143 [P] Create backend/tests/integration/test_chat_api.py pytest tests for POST /api/v1/chat endpoint (full integration with Qdrant + OpenAI, verify sources returned)
- [ ] T144 Run `pytest backend/tests/ -v --cov=backend/src --cov-report=term` and verify >85% code coverage
- [ ] T145 [P] Create frontend/tests/components/ChatbotWidget.test.tsx Jest tests for chatbot UI (render test, message send test, citation click test)
- [ ] T146 Run `npm test` in frontend/ and verify all React component tests pass
- [ ] T147 Create end-to-end test script tests/e2e/full_user_journey.spec.ts using Playwright: navigate to Chapter 1 → read through Module 1 → use chatbot → sign up → personalize Chapter 3 → translate Chapter 2 → verify all features work
- [ ] T148 Run Playwright e2e tests with `npx playwright test` and verify full user journey passes

### Deployment

- [ ] T149 Create .github/workflows/deploy-frontend.yml GitHub Actions workflow: on push to main → build Docusaurus (`npm run build`) → deploy to GitHub Pages (`gh-pages` branch)
- [ ] T150 [P] Create .github/workflows/deploy-backend.yml GitHub Actions workflow: on push to main → build Docker image → deploy to Railway/Fly.io using deployment API
- [ ] T151 [P] Create backend/Dockerfile for FastAPI app (Python 3.10 base image, copy requirements.txt, pip install, copy src/, expose port 8000, CMD uvicorn src.main:app --host 0.0.0.0)
- [ ] T152 Update frontend/docusaurus.config.js with production baseUrl (e.g., '/hackathon-1/') and deployed backend API URL (e.g., 'https://textbook-backend.railway.app')
- [ ] T153 Deploy backend to Railway: create new Railway project, connect GitHub repo, set environment variables (OPENAI_API_KEY, NEON_DATABASE_URL, QDRANT_API_KEY, JWT_SECRET), deploy from main branch
- [ ] T154 Deploy frontend to GitHub Pages: push to main branch, verify GitHub Actions workflow runs, check https://<username>.github.io/hackathon-1/ loads successfully
- [ ] T155 Update CORS middleware in backend/src/main.py to allow production frontend origin (https://<username>.github.io)
- [ ] T156 Test deployed site: Open production URL, test chatbot with 5 sample questions, verify responses, test authentication signup/signin, test personalization on 1 chapter, test Urdu translation on 1 chapter, verify all features work end-to-end

### Demo Video and Submission

- [ ] T157 Create demo video script covering: (1) Textbook navigation showing 4 modules and sample chapter content (15 sec), (2) RAG chatbot demo answering 2 questions with citations (30 sec), (3) Authentication signup with background questions (15 sec), (4) Content personalization demo (15 sec), (5) Urdu translation demo (15 sec) - Total: 90 seconds
- [ ] T158 Record demo video using screen recording tool (OBS Studio or NotebookLM), ensure <90 seconds duration, export as MP4
- [ ] T159 Upload demo video to YouTube/Vimeo and get shareable link
- [ ] T160 Update README.md with final deployment URLs (frontend, backend), demo video link, setup instructions, project overview, team info
- [ ] T161 Create DEPLOYMENT.md documenting hosting configuration (GitHub Pages for frontend, Railway for backend), environment variables required, CI/CD pipeline setup
- [ ] T162 Submit hackathon via form https://forms.gle/CQsSEGM3GeCrL43c8: public GitHub repo URL, deployed book URL, demo video link, WhatsApp number
- [ ] T163 Prepare live presentation (if invited): 5-minute demo covering all features (base textbook + RAG chatbot + auth + personalization + translation + Claude Code agent skills)

**Checkpoint**: Testing and deployment complete. System validated, deployed to production, demo video submitted. Hackathon submission ready.

---

## Phase 12: Bonus - Claude Code Agent Skills (50 POINTS)

**Goal**: Demonstrate advanced Claude Code usage by creating reusable subagents and skills for repetitive textbook tasks (content generation, code example formatting, markdown validation)

**Independent Test**: Run custom Claude Code skill `/chapter-gen "Chapter 25: Advanced Topics"` and verify it generates complete chapter following template, or run `/code-format static/code-examples/ros2/publisher.py` and verify Black formatting + inline comments added

### Implementation for Agent Skills

- [ ] T164 [P] Create .claude/agents/chapter-generator.md documenting subagent that takes chapter number + title + topics as input, generates complete markdown file following chapter-template.md, validates against chapter-quality.md checklist, outputs file path
- [ ] T165 [P] Create .claude/skills/format-code-example.md skill that takes code file path as input, applies Black formatting for Python, adds inline comments explaining non-obvious logic, adds execution environment comment at top ("# Environment: Ubuntu 22.04 + ROS 2 Humble + Python 3.10"), outputs formatted file
- [ ] T166 [P] Create .claude/skills/validate-markdown.md skill that takes chapter file path as input, runs scripts/validate_content.py, checks for broken internal links, validates code block syntax highlighting, outputs validation report with pass/fail + specific issues
- [ ] T167 Document agent configurations in .claude/README.md: how to invoke each subagent/skill, example commands, inputs/outputs, use cases for team reuse
- [ ] T168 Test chapter-generator subagent: Invoke to generate hypothetical "Chapter 25: Advanced Perception Techniques", verify output follows template, validate against checklist
- [ ] T169 Test format-code-example skill: Apply to static/code-examples/ros2/publisher.py, verify Black formatting applied, inline comments added, environment comment present
- [ ] T170 Test validate-markdown skill: Run on content/docs/module-1-ros2/03-nodes-topics.md, verify validation report generated with pass/fail status
- [ ] T171 Create demo video segment (15 sec) showcasing Claude Code agent skills: invoke `/chapter-gen` skill, show generated output, highlight reusability for 50 bonus points

**Checkpoint**: Claude Code agent skills complete (50 bonus points). Reusable intelligence demonstrated, documented for team use.

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion (T001-T007) - BLOCKS all subsequent phases
- **User Story 4 (Phase 3)**: Depends on Foundational (T008-T020) - Establishes content authoring workflow
- **User Story 1 (Phase 4)**: Depends on US4 completion (T021-T028) - Content authors need templates before writing chapters
- **RAG Chatbot (Phase 5)**: Depends on US1 partial completion (at least Module 1 chapters complete for embedding generation) - Can run in parallel with Module 2-4 content development
- **User Story 2 (Phase 6)**: Depends on US1 completion (all 24 chapters + appendices must exist for course structure page) - Can run in parallel with RAG Chatbot
- **User Story 3 (Phase 7)**: Depends on US1 completion (needs all chapters to add prerequisites and cross-references) - Can run in parallel with US2
- **Authentication (Phase 8)**: Depends on Foundational (backend/frontend running) - Independent of content phases, can run in parallel with US1-US3
- **Personalization (Phase 9)**: Depends on Authentication (Phase 8) + US1 completion (needs chapters to personalize)
- **Translation (Phase 10)**: Depends on US1 completion (needs chapters to translate) - Independent of Authentication, can run in parallel with Phases 8-9
- **Testing & Deployment (Phase 11)**: Depends on ALL desired features complete (base US1+US4+RAG for 100pts, or include bonus phases for 200pts)
- **Agent Skills (Phase 12)**: Independent - can run anytime, best done early to assist with content generation in Phase 4

### Critical Path (MVP - 100 Points)

1. **Phase 1: Setup** (T001-T007) → 2 hours
2. **Phase 2: Foundational** (T008-T020) → 4 hours
3. **Phase 3: US4 Content Authoring Workflow** (T021-T028) → 3 hours
4. **Phase 4: US1 Module 1 Content** (T029-T039) → 16 hours (8 chapters × 2 hours avg)
5. **Phase 4: US1 Modules 2-4 Content** (T040-T066) → 32 hours (16 chapters × 2 hours avg)
6. **Phase 5: RAG Chatbot** (T071-T092) → 12 hours
7. **Phase 11: Testing & Deployment (Base)** (T141-T163) → 8 hours

**Total MVP Time**: ~77 hours (Phases 1-5 + minimal testing/deployment)

### Parallel Execution Opportunities

**Within Phase 4 (US1 Content Development)**:
- Module 1 chapters can be written in parallel (T029-T037 all marked [P])
- Module 2 chapters in parallel (T040-T044 all marked [P])
- Module 3 chapters in parallel (T047-T051 all marked [P])
- Module 4 chapters in parallel (T054-T058 all marked [P])
- Appendices in parallel (T062-T066 all marked [P])

**Across Phases**:
- Once Module 1 complete, can start RAG Chatbot embedding generation (T078) while continuing Modules 2-4 content
- US2 (Instructor) and US3 (Self-Paced) can run fully in parallel after US1 complete
- Authentication (Phase 8) can run in parallel with US2/US3 (different code areas)
- Translation (Phase 10) independent of Authentication (Phase 8-9), can run in parallel

**Agent Skills (Phase 12)** should be created EARLY (after Phase 3) to assist with Phase 4 content generation, maximizing the 50 bonus points for demonstrating reusable intelligence.

---

## Parallel Example: Phase 4 Module 1

```bash
# Launch all Module 1 chapter writing tasks together (if team has multiple developers or using Claude Code agents):
Task T029: Write Chapter 01 (Introduction to Physical AI)
Task T030: Write Chapter 02 (Introduction to ROS 2)
Task T032: Write Chapter 04 (Services and Actions)  # Note: T031 creates diagram, can also run in parallel
Task T033: Write Chapter 05 (Launch Files)
Task T035: Write Chapter 06 (URDF)  # Note: T034 creates code example, can run in parallel
Task T037: Write Chapter 07 (Best Practices)

# Sequential dependency: Chapter 08 (Module Summary) depends on Chapters 01-07 being complete
Task T038: Write Chapter 08 (Module 1 Summary) - AFTER T029-T037 complete

# Validation: Run after all Module 1 chapters complete
Task T039: Validate Module 1 chapters with scripts/validate_content.py
```

---

## Implementation Strategy

### MVP First (Base 100 Points)

**Recommended Approach for Hackathon Timeline**:

1. **Phase 1: Setup** (2 hours) - Create project structure
2. **Phase 2: Foundational** (4 hours) - Configure Docusaurus + FastAPI, verify running
3. **Phase 3: US4 Content Workflow** (3 hours) - Create templates and validation tools
4. **Phase 4: US1 ABBREVIATED Content** (16 hours) - Write 3-4 COMPLETE chapters per module (Chapters 1, 2, 5, 9, 10, 14, 15, 19, 20) instead of all 24. This gives representative content for demo while meeting "textbook with 4 modules" requirement
5. **Phase 5: RAG Chatbot** (12 hours) - Implement on abbreviated content
6. **Phase 11: Deploy & Demo** (8 hours) - Deploy base system, create demo video

**Total MVP Time: ~45 hours** (achievable in hackathon timeline with focused execution)

**Then Add Bonus Features if Time Permits**:
- Authentication (50pts): 8 hours
- Personalization (50pts): 6 hours
- Translation (50pts): 6 hours
- Agent Skills (50pts): 4 hours

### Incremental Delivery Strategy

**Week 1** (Base Deliverable):
1. Complete Phases 1-3: Project setup + content authoring workflow → Can demonstrate systematic content generation
2. Write 1 complete module (Module 1: 8 chapters) → Demonstrates student learning flow
3. Implement basic RAG chatbot (no chat history persistence) → Demonstrates AI integration

**Week 2** (Enhance Base):
1. Complete remaining 3 modules (abbreviated: 3 chapters per module) → Full 4-module structure
2. Add chat history persistence to RAG → Complete base 100pts
3. Add US2 (Instructor) course structure page → Bonus points for usability
4. Deploy to GitHub Pages + Railway → Accessible demo

**Week 3** (Bonus Features):
1. Implement Authentication (50pts)
2. Implement Content Personalization (50pts) OR Urdu Translation (50pts) - choose one based on complexity preference
3. Create Claude Code agent skills (50pts) - showcase reusable intelligence
4. Polish, test, record demo video, submit

---

## Notes

- **[P] tasks**: Different files, can run in parallel with no dependencies
- **[Story] labels**: Map tasks to user stories (US1, US2, US3, US4) for traceability
- **File paths**: Exact paths included for LLM executability (e.g., `content/docs/module-1-ros2/01-introduction-physical-ai.md`)
- **Checkpoints**: Major milestones where functionality should be independently testable
- **MVP Strategy**: Prioritize quality over quantity - 3-4 complete chapters per module beats 24 partial chapters for demo
- **Time Estimates**: Tasks range 15-30 minutes (T001-T020), 1-3 hours for chapter writing (T029-T061), 2-4 hours for complex features (T071-T092 RAG, T106-T120 Auth)
- **Hackathon Focus**: Base 100 points (US1 content + RAG chatbot) should be completed first before bonus features (200 additional points)

**Avoid**:
- Writing all 24 chapters if timeline tight - prioritize 3-4 complete, high-quality chapters per module
- Implementing bonus features before base deliverable works - 100 points guaranteed > 50 bonus points at risk
- Parallelizing dependent tasks (e.g., Chapter 08 Module Summary needs Chapters 01-07 complete first)
- Skipping validation tasks (T067-T070, T141-T148) - these ensure quality for hackathon evaluation

**Success Criteria for Task Completion**:
Each task is considered complete when:
- Output file exists at specified path
- Content meets specification (learning outcomes, code examples, etc. for chapters)
- Validation passes (scripts/validate_content.py for chapters, tests pass for code)
- Functionality works when tested (can navigate to chapter, chatbot responds, auth flow works)
- Commit created with clear message referencing task ID (e.g., "T029: Write Chapter 01 - Introduction to Physical AI")
