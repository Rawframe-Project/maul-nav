#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Sirac Ozmen
#
# The guide's C snippets are run by test/test_guide.c: every ```c block of
# docs/guide.md must appear there unchanged, each line indented by four
# spaces (blank lines stay blank), so a snippet that no longer builds or
# runs fails the tests instead of drifting from the API.
#
# usage: check_guide.py

import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def main():
    guide = open(os.path.join(ROOT, "docs", "guide.md"), encoding="utf-8").read()
    test = open(os.path.join(ROOT, "test", "test_guide.c"), encoding="utf-8").read()
    snippets = re.findall(r"```c\n(.*?)```", guide, re.S)
    missing = 0
    for number, snippet in enumerate(snippets, 1):
        lines = snippet.rstrip("\n").split("\n")
        indented = "\n".join("    " + line if line else "" for line in lines) + "\n"
        if indented not in test:
            print(f"docs/guide.md: C snippet {number} ({lines[0].strip()}) is not in "
                  "test/test_guide.c as written")
            missing += 1
    if missing:
        print(f"{missing} of {len(snippets)} guide snippets not run by the tests")
        return 1
    print(f"guide: all {len(snippets)} C snippets run by test/test_guide.c")
    return 0


if __name__ == "__main__":
    sys.exit(main())
