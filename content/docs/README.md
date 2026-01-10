# Physical AI Textbook - Content Authoring Guide

This directory contains the markdown source files for the Physical AI & Humanoid Robotics textbook.

## Directory Structure

```
content/docs/
├── _templates/
│   └── chapter-template.md        # Template for new chapters
├── module-1-ros2/                  # Module 1: ROS 2 Fundamentals (8 chapters)
├── module-2-simulation/            # Module 2: Gazebo & Unity (5 chapters)
├── module-3-isaac/                 # Module 3: NVIDIA Isaac (5 chapters)
├── module-4-vla/                   # Module 4: VLA Systems (6 chapters)
└── appendices/                     # Supporting materials (5 sections)
```

## Content Creation Workflow

### 1. Use the Chapter Template

Start with `_templates/chapter-template.md` which includes all mandatory sections per FR-003:

- **Learning Outcomes** (FR-006: 3-5 with Bloom's taxonomy verbs)
- **Introduction** (context and motivation)
- **Core Content** (subdivided sections with examples)
- **Hands-On Exercise** (FR-009: 1-2 exercises with step-by-step instructions)
- **Summary** (key concepts recap)
- **Assessment Questions** (FR-010: 5-10 questions with difficulty levels)

### 2. Follow Content Requirements

**Code Examples** (FR-007):
- Include 2-5 executable code examples per chapter
- Add inline comments explaining key concepts
- Provide expected output
- Test code before including

**Learning Outcomes** (FR-006):
- Use Bloom's taxonomy action verbs: define, understand, apply, analyze, evaluate, create
- Make outcomes measurable and specific
- Example: "Create a ROS 2 publisher-subscriber system" (not "Learn about ROS 2")

**Visual Aids** (FR-023):
- Minimum 2 diagrams/screenshots per chapter
- Store images in `../static/img/module-[n]-[topic]/`
- Caption all figures with Figure X.Y format

**Estimated Time** (FR-004):
- Specify reading time + hands-on exercise time
- Exclude assessment question time (self-checks)
- Example: "1.5 hours (Reading: 30 min, Hands-on: 60 min)"

### 3. Validate Your Chapter

Before committing, run the validation script:

```bash
python scripts/validate_chapter.py frontend/docs/module-1-ros2/your-chapter.md
```

**Expected Output**: [PASS] with stats showing:
- 3-5 learning outcomes
- 2-5 code examples
- 1-2 exercises
- 5-10 assessment questions

### 4. Build and Preview

After writing content, test it in Docusaurus:

```bash
cd frontend
npm start
# Navigate to http://localhost:3000/module-1-ros2/your-chapter
```

Verify:
- Chapter appears in sidebar navigation
- All links work (internal chapter references, external resources)
- Code blocks have proper syntax highlighting
- Images display correctly

## Quality Checklist

Before marking a chapter complete, ensure:

- [ ] All mandatory sections present (FR-003)
- [ ] 3-5 learning outcomes with Bloom's verbs (FR-006)
- [ ] 2-5 code examples tested and working (FR-007, FR-008)
- [ ] 1-2 hands-on exercises with clear instructions (FR-009)
- [ ] 5-10 assessment questions with difficulty labels (FR-010)
- [ ] Prerequisites clearly stated (FR-017)
- [ ] Minimum 2 visual aids with captions (FR-023)
- [ ] Estimated time specified (FR-004)
- [ ] Real-world use case included (FR-022)
- [ ] Troubleshooting section in exercises (FR-024)
- [ ] Connection to next chapter (FR-021)
- [ ] Validation script passes

## Example Chapters

See `frontend/docs/module-1-ros2/introduction-physical-ai.md` for a complete example chapter that passes all validation checks.

## Content Standards

### Code Style
- **Python**: Follow PEP 8, include docstrings
- **ROS 2**: Use rclpy (Python bindings), follow ROS 2 naming conventions
- **Comments**: Explain "why" not just "what"

### Writing Style
- **Tone**: Professional but approachable
- **Audience**: Intermediate programmers (know Python basics)
- **Clarity**: Define technical terms on first use
- **Examples**: Use relatable real-world scenarios

### Markdown Formatting
- **Headers**: Use `##` for major sections, `###` for subsections
- **Code Blocks**: Specify language for syntax highlighting
- **Links**: Use descriptive link text, not "click here"
- **Lists**: Use `-` for unordered, `1.` for ordered

## Chapter Numbering

Chapters are numbered sequentially across all modules (not restarting per module):
- Chapter 1-8: Module 1 (ROS 2 Fundamentals)
- Chapter 9-13: Module 2 (Gazebo & Unity)
- Chapter 14-18: Module 3 (NVIDIA Isaac)
- Chapter 19-24: Module 4 (VLA Systems)

## Getting Help

- **Spec Reference**: See `specs/001-textbook-content-structure/spec.md` for full requirements
- **Template Questions**: Review `content/docs/_templates/chapter-template.md` comments
- **Validation Issues**: Check error messages from `scripts/validate_chapter.py`

## AI-Assisted Content Generation

This textbook is developed using Claude Code with Spec-Kit Plus methodology:
1. Follow the specification (spec.md)
2. Use the template (chapter-template.md)
3. Validate with automation (validate_chapter.py)
4. Iterate based on feedback

For Claude Code agents generating content, use this prompt structure:
```
Write a chapter for Module [X], Chapter [Y]: [Title]
- Follow template at content/docs/_templates/chapter-template.md
- Meet all requirements from specs/001-textbook-content-structure/spec.md
- Include 3-5 learning outcomes, 2-5 code examples, 1 hands-on exercise
- Target audience: Intermediate programmers learning Physical AI
- Estimated time: [X] hours total
```

---

**Last Updated**: 2025-12-27
**Version**: 1.0.0
