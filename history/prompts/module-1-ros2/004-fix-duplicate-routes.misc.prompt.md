---
id: 4
title: Fix duplicate routes and update docs paths
stage: misc
date: 2025-12-31
surface: agent
model: claude-sonnet-4-5-20250929
feature: module-1-ros2
branch: 001-textbook-content-structure
user: SheZziiii SaM
command: npm run build
labels: [fix, routing, documentation]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - frontend/docusaurus.config.ts
  - frontend/src/pages/index.tsx
  - frontend/docs/intro.md
  - frontend/docs/module-1-ros2/introduction-physical-ai.md
tests:
  - npm run build
---

## Prompt

fix the duplicate route warning at root

## Response snapshot

I've resolved the duplicate route conflict by moving the documentation to the `/docs/` base path in `docusaurus.config.ts`. I also updated all internal links in the landing page (`index.tsx`), footer (`docusaurus.config.ts`), and documentation files (`intro.md`, `introduction-physical-ai.md`) to point to the new `/docs/` paths. The build now completes successfully with zero warnings about duplicate routes or broken links.

## Outcome

- ✅ Impact: Clean build with no routing conflicts; improved site structure.
- 🧪 Tests: `npm run build` PASS (0 warnings).
- 📁 Files: Modified config, index page, and multiple markdown files.
- 🔁 Next prompts: None
- 🧠 Reflection: Moving docs to a subpath is the cleanest way to support a custom landing page in Docusaurus without collision.

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): none
- Next experiment (smallest change to try): none
