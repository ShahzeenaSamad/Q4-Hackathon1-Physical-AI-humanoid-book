# Data Model: Physical AI Textbook

**Date**: 2025-12-29
**Feature**: 001-textbook-content-structure

## Entity Relationship Diagram

```
┌─────────────────┐       ┌─────────────────────────┐
│     User        │       │   PersonalizationProfile │
├─────────────────┤       ├─────────────────────────┤
│ id (PK, UUID)   │──────<│ id (PK, UUID)           │
│ email (unique)  │       │ user_id (FK)            │
│ name            │       │ software_background     │
│ hashed_password │       │ hardware_background     │
│ created_at      │       │ learning_goals          │
│ updated_at      │       │ preferred_complexity    │
└─────────────────┘       │ created_at              │
        │                 │ updated_at              │
        │                 └─────────────────────────┘
        │
        │         ┌─────────────────────────┐
        │         │     ChatSession         │
        └────────<├─────────────────────────┤
                  │ id (PK, UUID)           │
                  │ user_id (FK, nullable)  │
                  │ messages (JSONB)        │
                  │ created_at              │
                  │ updated_at              │
                  └─────────────────────────┘

┌─────────────────────────┐      ┌─────────────────────────┐
│   TextbookContent       │      │   PersonalizedContent   │
├─────────────────────────┤      ├─────────────────────────┤
│ id (PK, UUID)           │      │ id (PK, UUID)           │
│ module_id (int)         │      │ user_id (FK)            │
│ chapter_id (int)        │      │ chapter_id (int)        │
│ section_title (str)     │      │ personalized_markdown   │
│ content (text)          │      │ generated_at            │
│ embedding_id (str)      │      │ (unique: user,chapter)  │
│ content_type (enum)     │      └─────────────────────────┘
│ learning_outcomes (arr) │
│ created_at              │      ┌─────────────────────────┐
└─────────────────────────┘      │   TranslatedContent     │
                                 ├─────────────────────────┤
                                 │ id (PK, UUID)           │
                                 │ chapter_id (int)        │
                                 │ language_code (str)     │
                                 │ translated_markdown     │
                                 │ generated_at            │
                                 │ (unique: chapter,lang)  │
                                 └─────────────────────────┘
```

## Entity Definitions

### User (Bonus Feature)

Represents authenticated users who can save chat history and personalize content.

| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| id | UUID | PK, auto-gen | Unique identifier |
| email | VARCHAR(255) | UNIQUE, NOT NULL | User's email address |
| name | VARCHAR(100) | NOT NULL | Display name |
| hashed_password | VARCHAR(255) | NOT NULL | bcrypt hashed password |
| created_at | TIMESTAMP | DEFAULT NOW() | Account creation time |
| updated_at | TIMESTAMP | ON UPDATE NOW() | Last modification time |

**Indexes**: `idx_user_email` on email

### PersonalizationProfile (Bonus Feature)

Stores user preferences for content personalization based on background.

| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| id | UUID | PK, auto-gen | Unique identifier |
| user_id | UUID | FK(User), UNIQUE | One-to-one with User |
| software_background | ENUM | NOT NULL | 'beginner', 'intermediate', 'advanced' |
| hardware_background | ENUM | NOT NULL | 'none', 'hobbyist', 'professional' |
| learning_goals | TEXT[] | NULLABLE | Array of learning objectives |
| preferred_complexity | ENUM | DEFAULT 'standard' | 'simplified', 'standard', 'detailed' |
| created_at | TIMESTAMP | DEFAULT NOW() | Profile creation time |
| updated_at | TIMESTAMP | ON UPDATE NOW() | Last modification time |

**Enums**:
- `software_background_enum`: beginner, intermediate, advanced
- `hardware_background_enum`: none, hobbyist, professional
- `complexity_enum`: simplified, standard, detailed

### ChatSession

Stores RAG chatbot conversation history.

| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| id | UUID | PK, auto-gen | Session identifier |
| user_id | UUID | FK(User), NULLABLE | Optional user association |
| messages | JSONB | NOT NULL, DEFAULT '[]' | Array of message objects |
| created_at | TIMESTAMP | DEFAULT NOW() | Session start time |
| updated_at | TIMESTAMP | ON UPDATE NOW() | Last message time |

**Message JSONB Structure**:
```json
{
  "messages": [
    {
      "role": "user",
      "content": "What is ROS 2?",
      "timestamp": "2025-12-29T10:00:00Z",
      "context_chapter_id": 2,
      "selected_text": null
    },
    {
      "role": "assistant",
      "content": "ROS 2 (Robot Operating System 2) is...",
      "timestamp": "2025-12-29T10:00:02Z",
      "sources": [
        {"chapter_id": 2, "section_title": "What is ROS 2?", "snippet": "..."}
      ]
    }
  ]
}
```

**Indexes**: `idx_chat_session_user` on user_id

### TextbookContent

Stores processed chapter content for RAG embedding and retrieval.

| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| id | UUID | PK, auto-gen | Unique identifier |
| module_id | INTEGER | NOT NULL, 1-4 | Module number |
| chapter_id | INTEGER | NOT NULL, 1-24 | Chapter number |
| section_title | VARCHAR(255) | NOT NULL | Section heading |
| content | TEXT | NOT NULL | Chunk of markdown content |
| embedding_id | VARCHAR(100) | NOT NULL | Qdrant point ID reference |
| content_type | ENUM | NOT NULL | 'introduction', 'core', 'exercise', 'assessment' |
| learning_outcomes | TEXT[] | NULLABLE | Associated learning outcomes |
| created_at | TIMESTAMP | DEFAULT NOW() | Content ingestion time |

**Enums**:
- `content_type_enum`: introduction, core, exercise, assessment, summary

**Indexes**:
- `idx_content_chapter` on (module_id, chapter_id)
- `idx_content_embedding` on embedding_id

### PersonalizedContent (Bonus Feature)

Caches personalized chapter versions for users.

| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| id | UUID | PK, auto-gen | Unique identifier |
| user_id | UUID | FK(User), NOT NULL | User who requested personalization |
| chapter_id | INTEGER | NOT NULL, 1-24 | Chapter number |
| personalized_markdown | TEXT | NOT NULL | LLM-generated personalized content |
| generated_at | TIMESTAMP | DEFAULT NOW() | Generation timestamp |

**Constraints**: UNIQUE(user_id, chapter_id)

**Indexes**: `idx_personalized_user_chapter` on (user_id, chapter_id)

### TranslatedContent (Bonus Feature)

Caches Urdu translations for chapters.

| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| id | UUID | PK, auto-gen | Unique identifier |
| chapter_id | INTEGER | NOT NULL, 1-24 | Chapter number |
| language_code | VARCHAR(5) | NOT NULL, DEFAULT 'ur' | ISO language code |
| translated_markdown | TEXT | NOT NULL | LLM-translated content |
| generated_at | TIMESTAMP | DEFAULT NOW() | Translation timestamp |

**Constraints**: UNIQUE(chapter_id, language_code)

**Indexes**: `idx_translated_chapter_lang` on (chapter_id, language_code)

## Qdrant Vector Schema

Collection: `textbook_embeddings`

```json
{
  "name": "textbook_embeddings",
  "vectors": {
    "size": 1536,
    "distance": "Cosine"
  },
  "payload_schema": {
    "module_id": "integer",
    "chapter_id": "integer",
    "section_title": "keyword",
    "content_type": "keyword",
    "learning_outcomes": "keyword[]",
    "content_preview": "text"
  }
}
```

**Point Structure**:
```json
{
  "id": "uuid-string",
  "vector": [0.123, -0.456, ...],  // 1536 dimensions
  "payload": {
    "module_id": 1,
    "chapter_id": 3,
    "section_title": "Publishers and Subscribers",
    "content_type": "core",
    "learning_outcomes": ["Create custom ROS 2 nodes", "Publish and subscribe to topics"],
    "content_preview": "First 200 characters of chunk..."
  }
}
```

## Migration Strategy

### Initial Schema (Migration 001)
- Create enums
- Create chat_session table
- Create textbook_content table

### Authentication Tables (Migration 002 - Bonus)
- Create user table
- Create personalization_profile table
- Add user_id FK to chat_session

### Personalization Table (Migration 003 - Bonus)
- Create personalized_content table

### Translation Table (Migration 004 - Bonus)
- Create translated_content table

## Validation Rules

### User
- Email: Valid email format, max 255 chars
- Name: 2-100 characters, alphanumeric + spaces
- Password: Min 8 chars, hashed with bcrypt

### ChatSession
- Messages array max 100 messages per session
- Individual message content max 10,000 chars

### TextbookContent
- module_id: 1-4
- chapter_id: 1-24
- content: Max 5000 chars per chunk
- section_title: Max 255 chars

### PersonalizedContent
- personalized_markdown: Max 50,000 chars
- One personalization per user per chapter

### TranslatedContent
- language_code: 'ur' (Urdu) only for MVP
- One translation per chapter per language
