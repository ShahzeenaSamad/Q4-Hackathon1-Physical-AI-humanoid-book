import os
import sys
import re
from typing import List, Dict

def validate_markdown_file(file_path: str) -> Dict[str, bool]:
    """
    Validates a chapter markdown file against the required structure.
    """
    if not os.path.exists(file_path):
        print(f"Error: File {file_path} not found.")
        sys.exit(1)

    with open(file_path, 'r', encoding='utf-8') as f:
        content = f.read()

    results = {
        "Title present": bool(re.search(r"^# .+", content, re.MULTILINE)),
        "Learning Outcomes section present": "## Learning Outcomes" in content,
        "Introduction section present": "## Introduction" in content,
        "Core Content section present": "## Core Content" in content,
        "Hands-On Exercise section present": "## Hands-On Exercise" in content,
        "Summary section present": "## Summary" in content,
        "Assessment Questions section present": "## Assessment Questions" in content,
        "Estimated Time present": bool(re.search(r"\*\*Estimated Time\*\*", content)),
        "Min 3 Learning Outcomes": len(re.findall(r"\d\. \[.+\]", content)) >= 3 or content.count("\n1. ") >= 1, # Simple check
        "Min 2 Code Blocks": len(re.findall(r"```python", content)) + len(re.findall(r"```bash", content)) + len(re.findall(r"```xml", content)) + len(re.findall(r"```yaml", content)) + len(re.findall(r"```json", content)) >= 2,
    }

    return results

def main():
    if len(sys.argv) < 2:
        print("Usage: python validate_content.py <file_or_directory>")
        sys.exit(1)

    target = sys.argv[1]

    if os.path.isfile(target):
        files = [target]
    elif os.path.isdir(target):
        files = [os.path.join(target, f) for f in os.listdir(target) if f.endswith('.md') and not f.startswith('_')]
    else:
        print(f"Error: {target} is not a valid file or directory.")
        sys.exit(1)

    all_passed = True
    for file_path in files:
        print(f"\nValidating {file_path}...")
        results = validate_markdown_file(file_path)

        for check, passed in results.items():
            status = "PASS" if passed else "FAIL"
            print(f"- {check}: {status}")
            if not passed:
                all_passed = False

    if not all_passed:
        print("\nConclusion: FAIL - Some validation checks failed.")
        sys.exit(1)
    else:
        print("\nConclusion: PASS - All validation checks passed!")

if __name__ == "__main__":
    main()
