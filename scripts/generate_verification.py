# Copyright 2025 bburda
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Generate verification.rst from test files with @verifies tags."""

import os
from pathlib import Path
import re

# Determine workspace root relative to this script
# Script is in <workspace>/scripts/generate_verification.py
SCRIPT_DIR = Path(__file__).parent.resolve()
WORKSPACE_DIR = SCRIPT_DIR.parent
SRC_DIR = WORKSPACE_DIR / 'src'
OUTPUT_FILE = WORKSPACE_DIR / 'docs/requirements/verification.rst'
REQUIREMENTS_SPECS_DIR = WORKSPACE_DIR / 'docs/requirements/specs'
GITHUB_BASE_URL = 'https://github.com/selfpatch/ros2_medkit/blob/main'


def _extract_verifies_before(content, match_start, comment_prefix):
    """Extract @verifies tags from comment lines immediately before a position.

    Scans backwards from match_start through contiguous comment lines
    that contain @verifies (or Links to: / Verifies:) tags.
    """
    before_text = content[:match_start]
    before_lines = before_text.rstrip().split('\n')

    verifies_reqs = []
    for line in reversed(before_lines):
        stripped = line.strip()
        if not stripped:
            continue
        if not stripped.startswith(comment_prefix):
            break
        comment_content = stripped[len(comment_prefix):].strip()
        tag_match = re.match(
            r'(?:@verifies|Links to:|Verifies:)\s*(.*)', comment_content
        )
        if tag_match:
            reqs_text = tag_match.group(1)
            reqs = re.findall(r'(REQ_\w+)', reqs_text)
            verifies_reqs.extend(reqs)
        else:
            break
    return verifies_reqs


def parse_cpp_file(file_path):
    """Parse C++ test file for @verifies tags."""
    with open(file_path, 'r', encoding='utf-8') as f:
        content = f.read()

    # Regex to find TEST or TEST_F blocks
    # Matches: TEST_F(Suite, Name) { ... }
    test_pattern = re.compile(r'TEST(?:_F)?\s*\(\s*(\w+)\s*,\s*(\w+)\s*\)\s*\{')

    tests = []

    for match in test_pattern.finditer(content):
        suite_name = match.group(1)
        test_name = match.group(2)
        start_index = match.end()

        # Extract a chunk of text after the match to search for comments
        search_window = content[start_index:start_index + 2000]
        lines = search_window.split('\n')

        # Auto-generate ID and Title, including suite to avoid collisions
        test_id = f'TEST_{suite_name}_{test_name}'
        test_title = test_name
        line_number = content[:match.start()].count('\n') + 1

        verifies_reqs = []
        description = []

        # Check comments before the TEST macro
        verifies_reqs.extend(
            _extract_verifies_before(content, match.start(), '//')
        )

        for line in lines:
            line = line.strip()
            if not line:
                continue

            # Stop if we hit code (not a comment)
            if not line.startswith('//'):
                # If it's an opening brace on a new line, ignore
                if line == '{':
                    continue
                break

            comment_content = line[2:].strip()

            # Parse tags
            # Support '@verifies REQ_...' (list) or 'Links to: REQ_...'
            tag_match = re.match(
                r'(?:@verifies|Links to:|Verifies:)\s*(.*)', comment_content
            )

            if tag_match:
                reqs_text = tag_match.group(1)
                # Extract anything looking like REQ_\w+
                reqs = re.findall(r'(REQ_\w+)', reqs_text)
                verifies_reqs.extend(reqs)
            else:
                # Treat other comments as description
                description.append(comment_content)

        if verifies_reqs:
            tests.append(
                {
                    'id': test_id,
                    'title': test_title,
                    'verifies': list(set(verifies_reqs)),
                    'description': '\n   '.join(description),
                    'file': str(file_path.relative_to(WORKSPACE_DIR)),
                    'test_func': test_name,
                    'line': line_number,
                }
            )

    return tests


def parse_py_file(file_path):
    """Parse Python test file for @verifies tags."""
    with open(file_path, 'r', encoding='utf-8') as f:
        content = f.read()

    # Regex to find python test methods
    # Matches: def test_something(self):
    test_pattern = re.compile(r'def\s+(test_\w+)\s*\(self\):')

    # Build a list of (class_name, start_pos) so we can find which class
    # a test method belongs to.
    class_pattern = re.compile(r'^class\s+(\w+)\s*[\(:]', re.MULTILINE)
    class_ranges = []
    for cls_match in class_pattern.finditer(content):
        class_ranges.append((cls_match.group(1), cls_match.start()))

    def _find_class(pos):
        """Return the class name that contains *pos*, or None."""
        owner = None
        for cls_name, cls_start in class_ranges:
            if cls_start <= pos:
                owner = cls_name
            else:
                break
        return owner

    tests = []

    for match in test_pattern.finditer(content):
        test_name = match.group(1)
        start_index = match.end()

        # Extract a chunk of text after the match to search for docstrings/comments
        search_window = content[start_index:start_index + 2000]
        lines = search_window.split('\n')

        # Auto-generate ID and Title, including class name to avoid collisions
        cls_name = _find_class(match.start())
        if cls_name:
            test_id = f'TEST_{cls_name}_{test_name}'
        else:
            test_id = 'TEST_' + test_name
        test_title = test_name
        line_number = content[:match.start()].count('\n') + 1

        verifies_reqs = []
        description = []

        # Check comments before the def line
        verifies_reqs.extend(
            _extract_verifies_before(content, match.start(), '#')
        )

        in_docstring = False
        docstring_lines = []

        for line in lines:
            stripped = line.strip()
            if not stripped:
                continue

            # Check for docstring start/end
            if '"""' in stripped or "'''" in stripped:
                if in_docstring:
                    in_docstring = False
                    # End of docstring, parse collected lines
                    for dline in docstring_lines:
                        # Parse tags in docstring
                        tag_match = re.match(
                            r'(?:@verifies|Links to:|Verifies:)\s*(.*)', dline
                        )

                        if tag_match:
                            reqs_text = tag_match.group(1)
                            reqs = re.findall(r'(REQ_\w+)', reqs_text)
                            verifies_reqs.extend(reqs)
                        else:
                            description.append(dline)
                else:
                    in_docstring = True
                    # Handle one-line docstrings
                    if stripped.count('"""') == 2 or stripped.count("'''") == 2:
                        content_line = (
                            stripped.replace('"""', '').replace("'''", '').strip()
                        )
                        docstring_lines.append(content_line)
                        in_docstring = False
                        # Parse immediately
                        dline = content_line
                        tag_match = re.match(
                            r'(?:@verifies|Links to:|Verifies:)\s*(.*)', dline
                        )
                        if tag_match:
                            reqs_text = tag_match.group(1)
                            reqs = re.findall(r'(REQ_\w+)', reqs_text)
                            verifies_reqs.extend(reqs)
                        else:
                            description.append(dline)
                continue

            if in_docstring:
                docstring_lines.append(stripped)
                continue

            # Also check for comments #
            if stripped.startswith('#'):
                comment_content = stripped[1:].strip()
                tag_match = re.match(
                    r'(?:@verifies|Links to:|Verifies:)\s*(.*)', comment_content
                )

                if tag_match:
                    reqs_text = tag_match.group(1)
                    reqs = re.findall(r'(REQ_\w+)', reqs_text)
                    verifies_reqs.extend(reqs)
                continue

            # If we hit code that is not comment or docstring, stop
            if not in_docstring and not stripped.startswith('#'):
                break

        if verifies_reqs:
            tests.append(
                {
                    'id': test_id,
                    'title': test_title,
                    'verifies': list(set(verifies_reqs)),
                    'description': '\n   '.join(description),
                    'file': str(file_path.relative_to(WORKSPACE_DIR)),
                    'test_func': test_name,
                    'line': line_number,
                }
            )

    return tests


def generate_rst(tests):
    """Generate RST content from parsed tests."""
    lines = [
        'Verification',
        '============',
        '',
        'This section documents the test cases and their traceability to requirements.',
        'It is automatically generated from the source code.',
        '',
    ]

    for test in tests:
        lines.append('.. test:: ' + test['title'])
        lines.append('   :id: ' + test['id'])
        lines.append('   :status: verified')
        if test['verifies']:
            lines.append('   :verifies: ' + ', '.join(test['verifies']))
        lines.append('')
        if test['description']:
            lines.append('   ' + test['description'])
            lines.append('')

        github_url = f'{GITHUB_BASE_URL}/{test["file"]}#L{test["line"]}'
        lines.append(
            f"   **Implementation:** `{test['file']}#L{test['line']}"
            f' <{github_url}>`_'
            f" (Test: ``{test['test_func']}``)"
        )
        lines.append('')
        lines.append('')

    lines.append('.. needtable::')
    lines.append("   :filter: type == 'test'")
    lines.append('   :columns: id, title, status, verifies')
    lines.append('   :style: table')
    lines.append('')

    return '\n'.join(lines)


def update_requirement_status(verified_reqs):
    """Update requirement spec file statuses from open to verified."""
    if not REQUIREMENTS_SPECS_DIR.exists():
        print(f'Requirements specs directory {REQUIREMENTS_SPECS_DIR} does not exist.')
        return

    updated_files = []
    total_updated_reqs = 0

    for spec_file in REQUIREMENTS_SPECS_DIR.glob('*.rst'):
        if spec_file.name == 'index.rst':
            continue

        with open(spec_file, 'r', encoding='utf-8') as f:
            content = f.read()

        modified = False
        lines = content.split('\n')
        new_lines = []

        i = 0
        while i < len(lines):
            line = lines[i]

            # Check if this starts a new .. req:: block
            if re.match(r'\.\.\s+req::', line):
                # Collect all lines belonging to this req block (indented lines)
                req_block_lines = [line]
                i += 1

                # Collect indented lines that belong to this block
                while i < len(lines):
                    next_line = lines[i]
                    # Check if line is indented (part of directive) or empty
                    if (re.match(r'^\s+:', next_line)
                            or (next_line.strip() == ''
                                and i + 1 < len(lines)
                                and re.match(r'^\s+:', lines[i + 1]))):
                        req_block_lines.append(next_line)
                        i += 1
                    elif next_line.strip() == '':
                        req_block_lines.append(next_line)
                        i += 1
                    else:
                        break

                # Now analyze the req block to find :id: and :status:
                req_id = None
                status_line_idx = None
                current_status = None

                for j, block_line in enumerate(req_block_lines):
                    id_match = re.match(r'\s*:id:\s+(REQ_\w+)', block_line)
                    if id_match:
                        req_id = id_match.group(1)

                    status_match = re.match(r'(\s*):status:\s+(\w+)\s*$', block_line)
                    if status_match:
                        status_line_idx = j
                        current_status = status_match.group(2)

                # If we found a verified requirement with open status, update it
                if (req_id and req_id in verified_reqs
                        and status_line_idx is not None
                        and current_status == 'open'):
                    indent = re.match(r'(\s*)', req_block_lines[status_line_idx]).group(1)
                    req_block_lines[status_line_idx] = f'{indent}:status: verified'
                    modified = True
                    total_updated_reqs += 1

                new_lines.extend(req_block_lines)
            else:
                new_lines.append(line)
                i += 1

        if modified:
            with open(spec_file, 'w', encoding='utf-8') as f:
                f.write('\n'.join(new_lines))
            updated_files.append(spec_file.name)

    if updated_files:
        print(
            f'Updated {total_updated_reqs} requirement(s)'
            f" to 'verified' status in {len(updated_files)} file(s):"
        )
        for filename in updated_files:
            print(f'  - {filename}')
    else:
        print('No requirement status updates needed.')


def main():
    """Scan tests and generate verification.rst."""
    all_tests = []
    if not SRC_DIR.exists():
        print('Source directory ' + str(SRC_DIR) + ' does not exist.')
        return

    for root, dirs, files in os.walk(SRC_DIR):
        for file in files:
            file_path = Path(root) / file
            if file.endswith('.cpp'):
                all_tests.extend(parse_cpp_file(file_path))
            elif file.endswith('.py'):
                all_tests.extend(parse_py_file(file_path))

    # Sort by ID for consistency
    all_tests.sort(key=lambda x: x['id'])

    # Collect all verified requirement IDs
    verified_reqs = set()
    for test in all_tests:
        verified_reqs.update(test['verifies'])

    # Generate verification.rst
    rst_content = generate_rst(all_tests)

    with open(OUTPUT_FILE, 'w', encoding='utf-8') as f:
        f.write(rst_content)

    print('Generated ' + str(OUTPUT_FILE) + ' with ' + str(len(all_tests)) + ' tests.')
    print(f'Found {len(verified_reqs)} verified requirement(s): {sorted(verified_reqs)}')

    # Update requirement statuses
    update_requirement_status(verified_reqs)


if __name__ == '__main__':
    main()
