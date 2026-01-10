#!/usr/bin/env python3
"""
Chapter Validation Script

Validates that a markdown chapter file meets the specification requirements:
- FR-003: Mandatory sections present
- FR-006: 3-5 learning outcomes with Bloom's taxonomy verbs
- FR-007: 2-5 code examples
- FR-009: 1-2 hands-on exercises
- FR-010: 5-10 assessment questions
- FR-017: Prerequisites section present
- FR-023: Minimum 2 visual aids referenced

Usage:
    python scripts/validate_chapter.py <chapter_file.md>
    python scripts/validate_chapter.py frontend/docs/module-1-ros2/*.md
"""

import re
import sys
from pathlib import Path
from typing import List, Dict, Tuple


class ChapterValidator:
    """Validates chapter markdown files against spec requirements"""

    # Mandatory sections from FR-003
    REQUIRED_SECTIONS = [
        "Learning Outcomes",
        "Introduction",
        "Core Content",
        "Hands-On Exercise",
        "Summary",
        "Assessment Questions"
    ]

    # Bloom's taxonomy action verbs from FR-006
    BLOOMS_VERBS = [
        "remember", "understand", "apply", "analyze", "evaluate", "create",
        "identify", "define", "describe", "explain", "demonstrate", "build",
        "implement", "design", "compare", "classify", "solve", "construct"
    ]

    def __init__(self, file_path: str):
        self.file_path = Path(file_path)
        self.content = ""
        self.errors: List[str] = []
        self.warnings: List[str] = []
        self.stats: Dict[str, int] = {}

    def validate(self) -> Tuple[bool, List[str], List[str], Dict[str, int]]:
        """Run all validation checks"""
        if not self.file_path.exists():
            self.errors.append(f"File not found: {self.file_path}")
            return False, self.errors, self.warnings, self.stats

        self.content = self.file_path.read_text(encoding='utf-8')

        # Run validation checks
        self._check_required_sections()
        self._check_learning_outcomes()
        self._check_code_examples()
        self._check_exercises()
        self._check_assessment_questions()
        self._check_prerequisites()
        self._check_visuals()
        self._check_estimated_time()

        return len(self.errors) == 0, self.errors, self.warnings, self.stats

    def _check_required_sections(self):
        """FR-003: Verify all mandatory sections present"""
        missing_sections = []
        for section in self.REQUIRED_SECTIONS:
            # Check for section headers (## or ###)
            pattern = rf'^##+ {re.escape(section)}'
            if not re.search(pattern, self.content, re.MULTILINE | re.IGNORECASE):
                missing_sections.append(section)

        if missing_sections:
            self.errors.append(
                f"Missing required sections: {', '.join(missing_sections)}"
            )

    def _check_learning_outcomes(self):
        """FR-006: Verify 3-5 learning outcomes with Bloom's verbs"""
        # Find learning outcomes section
        lo_pattern = r'## Learning Outcomes\s+(.*?)(?=\n##|\Z)'
        lo_match = re.search(lo_pattern, self.content, re.DOTALL | re.IGNORECASE)

        if not lo_match:
            self.errors.append("Learning Outcomes section not found")
            return

        lo_text = lo_match.group(1)

        # Count numbered outcomes (1., 2., 3., etc.)
        outcomes = re.findall(r'^\d+\.\s+(.+)$', lo_text, re.MULTILINE)
        self.stats['learning_outcomes'] = len(outcomes)

        if len(outcomes) < 3:
            self.errors.append(
                f"Insufficient learning outcomes: {len(outcomes)} (minimum 3 required)"
            )
        elif len(outcomes) > 5:
            self.warnings.append(
                f"Excessive learning outcomes: {len(outcomes)} (recommended maximum 5)"
            )

        # Check for Bloom's taxonomy verbs
        outcomes_with_verbs = 0
        for outcome in outcomes:
            outcome_lower = outcome.lower()
            if any(verb in outcome_lower for verb in self.BLOOMS_VERBS):
                outcomes_with_verbs += 1

        if outcomes_with_verbs < len(outcomes):
            self.warnings.append(
                f"Only {outcomes_with_verbs}/{len(outcomes)} learning outcomes use "
                "Bloom's taxonomy verbs"
            )

    def _check_code_examples(self):
        """FR-007: Verify 2-5 code examples present"""
        # Count code blocks (```...```)
        code_blocks = re.findall(r'```[\w]*\n.*?```', self.content, re.DOTALL)
        # Filter out output blocks (typically smaller or have "output" in language)
        code_examples = [
            block for block in code_blocks
            if len(block) > 50 and 'output' not in block[:20].lower()
        ]

        self.stats['code_examples'] = len(code_examples)

        if len(code_examples) < 2:
            self.errors.append(
                f"Insufficient code examples: {len(code_examples)} (minimum 2 required)"
            )
        elif len(code_examples) > 5:
            self.warnings.append(
                f"Many code examples: {len(code_examples)} (recommended maximum 5)"
            )

    def _check_exercises(self):
        """FR-009: Verify 1-2 hands-on exercises present"""
        # Look for "Exercise" headers or "Hands-On Exercise" section
        exercise_pattern = r'###? (Exercise \d+:|Hands-On Exercise)'
        exercises = re.findall(exercise_pattern, self.content, re.IGNORECASE)

        self.stats['exercises'] = len(exercises)

        if len(exercises) < 1:
            self.errors.append("No hands-on exercises found (minimum 1 required)")
        elif len(exercises) > 2:
            self.warnings.append(
                f"Many exercises: {len(exercises)} (recommended 1-2)"
            )

    def _check_assessment_questions(self):
        """FR-010: Verify 5-10 assessment questions present"""
        # Find assessment section
        assessment_pattern = r'## Assessment Questions\s+(.*?)(?=\n##|\Z)'
        assessment_match = re.search(
            assessment_pattern, self.content, re.DOTALL | re.IGNORECASE
        )

        if not assessment_match:
            self.errors.append("Assessment Questions section not found")
            return

        assessment_text = assessment_match.group(1)

        # Count questions (looking for "Question N" or "**Question N**")
        questions = re.findall(
            r'\*?\*?Question \d+\*?\*?', assessment_text, re.IGNORECASE
        )

        self.stats['assessment_questions'] = len(questions)

        if len(questions) < 5:
            self.errors.append(
                f"Insufficient assessment questions: {len(questions)} "
                "(minimum 5 required)"
            )
        elif len(questions) > 10:
            self.warnings.append(
                f"Many assessment questions: {len(questions)} "
                "(recommended maximum 10)"
            )

    def _check_prerequisites(self):
        """FR-017: Verify Prerequisites section present"""
        # Check for Prerequisites in frontmatter or early in document
        prereq_pattern = r'(?:Prerequisites?|Required Knowledge):\s*(.+?)(?=\n\n|\n#|$)'
        prereq_match = re.search(prereq_pattern, self.content, re.IGNORECASE | re.DOTALL)

        if not prereq_match:
            self.warnings.append(
                "Prerequisites section not clearly marked "
                "(recommended for learner guidance)"
            )

    def _check_visuals(self):
        """FR-023: Verify minimum 2 visual aids (images, diagrams)"""
        # Count image references ![...](...) or HTML <img>
        images = re.findall(r'!\[.*?\]\(.*?\)|<img.*?>', self.content)

        self.stats['visuals'] = len(images)

        if len(images) < 2:
            self.warnings.append(
                f"Few visual aids: {len(images)} (recommended minimum 2 for clarity)"
            )

    def _check_estimated_time(self):
        """FR-004: Verify estimated time is specified"""
        time_pattern = r'Estimated Time:\s*[\d\.]+'
        if not re.search(time_pattern, self.content, re.IGNORECASE):
            self.warnings.append(
                "Estimated time not specified (recommended for learner planning)"
            )


def main():
    """Validate one or more chapter files"""
    if len(sys.argv) < 2:
        print("Usage: python scripts/validate_chapter.py <chapter_file.md> [...]")
        print("\nExample:")
        print("  python scripts/validate_chapter.py frontend/docs/module-1-ros2/01-intro.md")
        sys.exit(1)

    files = sys.argv[1:]
    results = []

    print(f"\n{'='*70}")
    print(f"Chapter Validation Report")
    print(f"{'='*70}\n")

    for file_path in files:
        validator = ChapterValidator(file_path)
        is_valid, errors, warnings, stats = validator.validate()

        results.append((file_path, is_valid))

        # Print results for this file
        status = "PASS" if is_valid else "FAIL"
        status_symbol = "[PASS]" if is_valid else "[FAIL]"

        print(f"{status_symbol} {file_path}: {status}")

        if stats:
            print(f"  Stats:")
            for key, value in stats.items():
                print(f"    - {key.replace('_', ' ').title()}: {value}")

        if errors:
            print(f"  Errors:")
            for error in errors:
                print(f"    - {error}")

        if warnings:
            print(f"  Warnings:")
            for warning in warnings:
                print(f"    ! {warning}")

        print()

    # Summary
    print(f"{'='*70}")
    passed = sum(1 for _, valid in results if valid)
    total = len(results)

    print(f"Summary: {passed}/{total} chapters passed validation")

    if passed == total:
        print("All chapters meet specification requirements!")
        sys.exit(0)
    else:
        print("Some chapters need updates to meet requirements.")
        sys.exit(1)


if __name__ == "__main__":
    main()
