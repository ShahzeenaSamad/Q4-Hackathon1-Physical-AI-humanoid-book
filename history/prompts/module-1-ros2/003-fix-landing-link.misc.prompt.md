---
id: 3
title: Fix broken landing page link
stage: misc
date: 2025-12-31
surface: agent
model: claude-sonnet-4-5-20250929
feature: module-1-ros2
branch: 001-textbook-content-structure
user: SheZziiii SaM
command: npm run build
labels: [fix, landing-page, links]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - frontend/src/pages/index.tsx
tests:
  - npm run build
---

## Prompt

fix the broken link for docs intro

## Response snapshot

I've fixed the broken link on the landing page (`index.tsx`). It was pointing to `/docs/intro` (default Docusaurus boilerplate), but that file doesn't exist in your project. I've updated it to point to the actual first chapter: `/module-1-ros2/introduction-physical-ai` and updated the button text to "Start Learning Physical AI 🤖".

## Outcome

- ✅ Impact: Landing page now has a working call-to-action button.
- 🧪 Tests: `npm run build` PASS (No broken link warning for `/docs/intro`).
- 📁 Files: Modified `frontend/src/pages/index.tsx`.
- 🔁 Next prompts: None
- 🧠 Reflection: Boilerplate code often leaves "ghost" links that trigger build warnings. Auditing the landing page solved the issue.

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): none
- Next experiment (smallest change to try): none
