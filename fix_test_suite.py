import sys

file_path = "test_suite.py"
with open(file_path, "r") as f:
    content = f.read()

# I will fix the dictionary creation in run_tests to ensure the key is 'node_data'
# The previous replace might have failed because of the multi-line string.

import re

# Look for the results.append part in run_tests
old_pattern = r"results\.append\(\{\s*'label': label,\s*'occ': occ,\s*'cost': cost,\s*'snap': snap,\s*'data': node_snaps\s*\}\)"
new_text = "results.append({'label': label, 'occ': occ, 'cost': cost, 'snap': snap, 'node_data': node_snaps})"

content = re.sub(old_pattern, new_text, content)

with open(file_path, "w") as f:
    f.write(content)
