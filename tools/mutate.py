#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Sirac Ozmen
#
# Mutation testing, by hand-picked mutants: each one changes a single line
# of a source file, the library and the test suite are built and run, and
# the mutant is killed when a test fails or survived when none does. A
# survivor is a change no test notices: a test is missing, or the change
# is equivalent, which the person reading the result judges.
#
# The mutants file holds one mutant a line, four fields split by " | ":
#
#   name | src/file.c | the line as it is | the line as the mutant has it
#
# Lines are given whole, without their indentation, and must appear once
# in the file; blank lines and lines starting with # are skipped. Each
# file is restored from git's HEAD after its mutant, whatever happens, so
# run this on a clean tree only. Results go to standard output and, line
# by line as they come, to the results file, so an interrupted run keeps
# what it found. Not run by CI: a sweep takes hours.
#
# usage: mutate.py <mutants file> <build dir> [results file] [test timeout s]
#        mutate.py --check <mutants file>   (each mutant applies; nothing built)

import os
import re
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def run(command, cwd, timeout=None):
    return subprocess.run(command, cwd=cwd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                          text=True, timeout=timeout)


def read_mutants(path):
    mutants = []
    for number, line in enumerate(open(path, encoding="utf-8"), 1):
        line = line.rstrip("\n")
        if not line.strip() or line.lstrip().startswith("#"):
            continue
        fields = line.split(" | ")
        if len(fields) != 4:
            sys.exit(f"{path}:{number}: four fields split by ' | ' expected")
        mutants.append(fields)
    return mutants


def apply(file, old, new):
    lines = open(file, encoding="utf-8").read().split("\n")
    found = [i for i, line in enumerate(lines) if line.strip() == old.strip()]
    if len(found) != 1:
        return f"the line appears {len(found)} times"
    i = found[0]
    indent = lines[i][: len(lines[i]) - len(lines[i].lstrip())]
    lines[i] = indent + new.strip()
    open(file, "w", encoding="utf-8").write("\n".join(lines))
    return None


def test(build, timeout):
    built = run(["cmake", "--build", build], ROOT)
    if built.returncode != 0:
        return "build error"
    try:
        ran = run(["ctest", "-j", str(os.cpu_count() or 4), "--timeout", str(timeout)], build,
                  timeout=timeout * 4)
    except subprocess.TimeoutExpired:
        return "killed: the suite ran past its time"
    # ctest lists every failed test, whatever the kind (failed, a signal,
    # a timeout, an abort), after this line.
    listed = ran.stdout.split("The following tests FAILED:")
    failed = [line.strip() for line in listed[-1].split("\n")[1:]
              if re.match(r"\s*\d+ - ", line)] if len(listed) > 1 else []
    return f"killed: {', '.join(failed)}" if ran.returncode != 0 else "SURVIVED"


def check(path):
    bad = 0
    for name, file, old, new in read_mutants(path):
        lines = open(os.path.join(ROOT, file), encoding="utf-8").read().split("\n")
        count = sum(1 for line in lines if line.strip() == old.strip())
        if count != 1 or old.strip() == new.strip():
            print(f"{name}: the line appears {count} times" if count != 1 else
                  f"{name}: the mutant changes nothing")
            bad += 1
    print(f"{len(read_mutants(path))} mutants, {bad} that do not apply")
    return 1 if bad else 0


def main():
    if len(sys.argv) == 3 and sys.argv[1] == "--check":
        return check(sys.argv[2])
    if len(sys.argv) < 3:
        sys.exit("usage: mutate.py <mutants file> <build dir> [results file] [test timeout s]")
    mutants = read_mutants(sys.argv[1])
    build = os.path.abspath(sys.argv[2])
    results = sys.argv[3] if len(sys.argv) > 3 else None
    timeout = int(sys.argv[4]) if len(sys.argv) > 4 else 300
    if run(["git", "status", "--porcelain", "--", "src", "include"], ROOT).stdout.strip():
        sys.exit("src or include has changes; mutants need a clean tree")
    for name, file, old, new in mutants:
        path = os.path.join(ROOT, file)
        try:
            problem = apply(path, old, new)
            outcome = f"not applied: {problem}" if problem else test(build, timeout)
        finally:
            run(["git", "checkout", "-q", "HEAD", "--", file], ROOT)
        line = f"{name}: {outcome}"
        print(line, flush=True)
        if results:
            with open(results, "a", encoding="utf-8") as out:
                out.write(line + "\n")
    run(["cmake", "--build", build], ROOT)
    return 0


if __name__ == "__main__":
    sys.exit(main())
