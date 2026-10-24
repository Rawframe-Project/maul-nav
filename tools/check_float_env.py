#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Sirac Ozmen
#
# The floating-point environment rule: every public function enters the
# library's floating-point environment, so a host's flush-to-zero,
# denormals-are-zero or rounding mode never changes a result, unless it
# computes no floats and tools/float-env-allowed.txt names it.
#
# The public functions are those the headers in include/<library>/
# declare with the <MACRO>_API marker and the <prefix> name prefix, both
# read from CMakeLists.txt (MAUL_LIBRARY, MAUL_MACRO_PREFIX,
# MAUL_API_PREFIX). Each must be defined once in src/ (or a directory
# tools/source-dirs.txt lists), and its body must call the guard: a
# function or macro whose name matches GUARD below. The allowed list
# gives names, never patterns, under the reason they need not: a reason
# is a line ending in a colon, its names indented below it:
#
#     reads a stored identity:
#         maniClipIdentity
#         maniSkeletonIdentity
#
# so a new public function fails until someone decides. A listed name
# that is no public function, one listed twice, and one that enters the
# environment anyway are reported too, so the list stays true.
#
# usage: check_float_env.py

import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ALLOWED = os.path.join(ROOT, "tools", "float-env-allowed.txt")


def cmake_setting(name):
    """A value the library's CMakeLists.txt sets with set(NAME value)."""
    with open(os.path.join(ROOT, "CMakeLists.txt"), encoding="utf-8") as f:
        match = re.search(r"^set\(" + name + r" \"?([^\")]+)\"?\)", f.read(), re.M)
    if not match:
        raise SystemExit("CMakeLists.txt does not set " + name)
    return match.group(1)


LIB = cmake_setting("MAUL_LIBRARY")
PREFIX = cmake_setting("MAUL_API_PREFIX")
MACRO = cmake_setting("MAUL_MACRO_PREFIX")

# What entering the environment looks like in a body: the family module's
# entry, or its run of a worker inside one, by name with the library's
# prefix.
GUARD = re.compile(r"\b" + PREFIX + r"FloatEnv(?:Enter|Run)\b")


def source_dirs():
    dirs = [os.path.join(ROOT, "src")]
    listed = os.path.join(ROOT, "tools", "source-dirs.txt")
    if os.path.exists(listed):
        with open(listed, encoding="utf-8") as f:
            for line in f:
                line = line.split("#", 1)[0].strip()
                if line:
                    dirs.append(os.path.join(ROOT, line, "src"))
    return dirs


def strip_comments(text):
    """The text with comments and string literals blanked, lines kept."""
    out = []
    i = 0
    n = len(text)
    while i < n:
        if text.startswith("//", i):
            end = text.find("\n", i)
            end = n if end < 0 else end
            out.append(" " * (end - i))
            i = end
        elif text.startswith("/*", i):
            end = text.find("*/", i + 2)
            end = n if end < 0 else end + 2
            out.append("".join("\n" if c == "\n" else " " for c in text[i:end]))
            i = end
        elif text[i] in "\"'":
            quote = text[i]
            j = i + 1
            while j < n and text[j] != quote:
                j += 2 if text[j] == "\\" else 1
            out.append(quote + " " * (min(j, n) - i - 1) + (quote if j < n else ""))
            i = j + 1
        else:
            out.append(text[i])
            i += 1
    return "".join(out)


def public_functions():
    """Every function the public headers declare with the API marker."""
    folder = os.path.join(ROOT, "include", LIB)
    decl = re.compile(r"\b" + MACRO + r"_API\b[^;{(]*?\b(" + PREFIX + r"[A-Za-z0-9_]*)\s*\(")
    names = set()
    for name in sorted(os.listdir(folder)):
        if name.endswith(".h"):
            with open(os.path.join(folder, name), encoding="utf-8") as f:
                names.update(decl.findall(strip_comments(f.read())))
    return names


def definitions(names):
    """Each public function's body, found where it is defined in a source
    file: its name at the start of a line's declarator, then its
    parameters and a brace. A list per name, to find one defined twice."""
    found = {name: [] for name in names}
    head = re.compile(r"^[A-Za-z_][\w \*]*?\b(" + PREFIX + r"\w*)\s*\(", re.M)
    for folder in source_dirs():
        for dirpath, _, files in os.walk(folder):
            for file in sorted(files):
                if not file.endswith((".c", ".m")):
                    continue
                path = os.path.join(dirpath, file)
                with open(path, encoding="utf-8") as f:
                    text = strip_comments(f.read())
                for match in head.finditer(text):
                    name = match.group(1)
                    if name not in found:
                        continue
                    # The parameters, then a body rather than a ';'.
                    depth = 0
                    i = match.end() - 1
                    while i < len(text):
                        depth += {"(": 1, ")": -1}.get(text[i], 0)
                        i += 1
                        if depth == 0:
                            break
                    rest = text[i:].lstrip()
                    if not rest.startswith("{"):
                        continue
                    start = i + text[i:].index("{")
                    depth = 0
                    j = start
                    while j < len(text):
                        depth += {"{": 1, "}": -1}.get(text[j], 0)
                        j += 1
                        if depth == 0:
                            break
                    line = text.count("\n", 0, match.start()) + 1
                    found[name].append((os.path.relpath(path, ROOT), line, text[start:j]))
    return found


def allowed():
    """The allowed list: {name: (reason, line)}; problems in its form."""
    names = {}
    problems = []
    if not os.path.exists(ALLOWED):
        return names, problems
    where = os.path.relpath(ALLOWED, ROOT)
    reason = None
    with open(ALLOWED, encoding="utf-8") as f:
        for number, line in enumerate(f, 1):
            text = line.split("#", 1)[0].rstrip()
            if not text.strip():
                continue
            if not text[0].isspace():
                if not text.endswith(":") or not text[:-1].strip():
                    problems.append(f"{where}:{number}: a reason is a line ending in a colon")
                reason = text[:-1].strip()
            elif reason is None:
                problems.append(f"{where}:{number}: a name before any reason")
            elif not re.fullmatch(PREFIX + r"\w+", text.strip()):
                problems.append(f"{where}:{number}: {text.strip()} is not a name of the API")
            elif text.strip() in names:
                problems.append(f"{where}:{number}: {text.strip()} is listed twice")
            else:
                names[text.strip()] = (reason, number)
    return names, problems


def main():
    names = public_functions()
    bodies = definitions(names)
    listed, problems = allowed()
    where = os.path.relpath(ALLOWED, ROOT)
    for name in sorted(names):
        places = bodies[name]
        if not places:
            problems.append(f"{name}: declared public, defined nowhere in the sources")
            continue
        if len(places) > 1:
            found = ", ".join(f"{p}:{l}" for p, l, _ in places)
            problems.append(f"{name}: defined more than once ({found})")
            continue
        path, line, body = places[0]
        guarded = GUARD.search(body) is not None
        if guarded and name in listed:
            problems.append(f"{path}:{line}: {name} enters the environment but {where}:"
                            f"{listed[name][1]} lists it")
        elif not guarded and name not in listed:
            problems.append(f"{path}:{line}: {name} does not enter the floating-point environment")
    for name, (_, number) in sorted(listed.items(), key=lambda item: item[1][1]):
        if name not in names:
            problems.append(f"{where}:{number}: {name} is no public function")
    for problem in problems:
        print(problem)
    print(f"float environment: {len(names)} public functions, {len(listed)} listed, "
          f"{len(problems)} problems")
    return 1 if problems else 0


if __name__ == "__main__":
    sys.exit(main())
