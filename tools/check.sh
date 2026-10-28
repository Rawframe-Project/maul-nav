#!/usr/bin/env bash
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Sirac Ozmen
#
# The checks, in three tiers. Each tier includes the one before it.
#
#   tools/check.sh changed   the files changed against origin/main: the
#                            source rules, formatting, the documentation
#                            checks, the linux-clang-20-asan preset
#                            (assertions on), and the suites the changes
#                            reach. Under two minutes; run it before every
#                            push.
#   tools/check.sh pr        what the pull request check runs on this
#                            machine: every rule on every file, clang-tidy
#                            on a Release and a Debug configure,
#                            the linux-clang-20-asan and linux-gcc-14 presets
#                            (the Recast differential with it), and ten
#                            seconds of fuzzing per target.
#   tools/check.sh full      also every other Linux preset (ThreadSanitizer,
#                            Clang Release with the package test, shared),
#                            the large scenes and a minute per fuzz target.
#
# The builds are the CI cells' CMake presets (CMakePresets.json), in
# out/<preset>; the other trees go to out/check-*. Needs gcc-14, clang-20,
# clang-tidy-20, ninja and the clang-format version CI pins: the
# organization's CI image has them all. The org documentation check runs
# when the organization's .github checkout is at ORG_DEFAULTS (default:
# ../../.github from the repository).
set -euo pipefail

tier=${1:-changed}
case "$tier" in
changed | pr | full) ;;
*)
    echo "usage: tools/check.sh [changed|pr|full]" >&2
    exit 2
    ;;
esac

cd "$(dirname "$0")/.."
root=$PWD
out=$root/out
mkdir -p "$out"
org=${ORG_DEFAULTS:-$root/../../.github}
jobs=${JOBS:-8}

step() { printf '\n== %s\n' "$*"; }

# A CI cell: its configure, build and test presets. More ctest arguments
# (a label filter) follow the preset's name.
preset() {
    local name=$1
    shift
    step "preset $name"
    cmake --preset "$name" >"$out/$name.log"
    python3 tools/check_test_hygiene.py --ctest "out/$name"
    cmake --build --preset "$name" -j "$jobs" >>"$out/$name.log"
    ctest --preset "$name" -j "$jobs" "$@"
}

# The suites a changed path reaches: a suite's own file reaches it, the
# user documentation reaches the snippets' suite, and library code reaches
# every suite, since a module's behavior shows in suites named after others
# (the funnel in the corridor's, the commit in every query's).
labels_for() {
    local base
    case "$1" in
    *.md | LICENSE | .github/* | tools/check*.py | tools/build_docs.py | tools/gen_api.py | tools/*.sh) ;;
    docs/user/* | test/test_guide.c) echo guide ;;
    test/test_*.c)
        base=${1#test/test_}
        echo "${base%.c}"
        ;;
    test/fuzz_* | test/corpus/* | bench/* | samples/*) ;;
    *) echo all ;;
    esac
}

source_checks() {
    step "source, module, length, documentation and snippet rules"
    python3 tools/check_source.py
    python3 tools/check_modules.py
    python3 tools/check_float_env.py
    python3 tools/check_lengths.py
    python3 tools/check_docs.py
    python3 tools/check_guide.py
    python3 tools/check_test_hygiene.py --self-test
    python3 tools/check_test_hygiene.py
    if [ -f "$org/scripts/docs_check.py" ]; then
        python3 "$org/scripts/docs_check.py"
    else
        echo "skipped the organization's documentation check: no checkout at $org"
    fi
}

all_c_files() { git ls-files 'include/*.h' 'src/*.[ch]' 'test/*.[ch]' 'tools/*.[ch]' 'bench/*.[ch]' 'samples/*.[ch]'; }

if [ "$tier" = changed ]; then
    base=$(git merge-base HEAD origin/main 2>/dev/null || echo HEAD)
    mapfile -t changed < <({
        git diff --name-only "$base"
        git ls-files --others --exclude-standard
    } | sort -u)
    if [ ${#changed[@]} -eq 0 ]; then
        echo "nothing changed against origin/main"
        exit 0
    fi
    source_checks
    mapfile -t c_files < <(printf '%s\n' "${changed[@]}" | grep -E '\.[ch]$' || true)
    present=()
    for f in "${c_files[@]+"${c_files[@]}"}"; do [ -f "$f" ] && present+=("$f"); done
    if [ ${#present[@]} -gt 0 ]; then
        step "clang-format on ${#present[@]} changed files"
        clang-format --dry-run -Werror "${present[@]}"
    fi
    labels=""
    for f in "${changed[@]}"; do labels="$labels $(labels_for "$f")"; done
    if [ -z "${labels// /}" ]; then
        echo "no suite reached by the changes"
        exit 0
    fi
    if [[ " $labels " == *" all "* ]]; then
        preset linux-clang-20-asan
    else
        regex=$(printf '%s\n' $labels | sort -u | paste -sd '|')
        echo "suites labeled $regex"
        preset linux-clang-20-asan -L "^($regex)\$"
    fi
    exit 0
fi

source_checks

step "clang-format on every file"
all_c_files | xargs clang-format --dry-run -Werror

# As CI's checks job: on the Release preset, where the assertions compile
# out, and with them live. Each hides findings the other shows.
for build in Release Debug; do
    step "clang-tidy, $build"
    cmake --preset linux-clang-20 -B "$out/check-tidy-$build" -DCMAKE_BUILD_TYPE=$build \
        --no-warn-unused-cli -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DMAUL_NAV_BUILD_TESTS=OFF \
        -DMAUL_NAV_BUILD_BENCH=OFF -DMAUL_NAV_BUILD_SAMPLES=OFF >"$out/check-tidy-$build.log"
    for f in src/*.c; do
        clang-tidy-20 -p "$out/check-tidy-$build" --quiet --warnings-as-errors='*' "$f" 2>/dev/null
    done
done

preset linux-clang-20-asan
preset linux-gcc-14

step "the hand tiles are what their writer writes"
"$out/linux-gcc-14/test_hand_tiles" --check test/hand_tiles.h

fuzz_seconds=10
[ "$tier" = full ] && fuzz_seconds=60
step "fuzzing, $fuzz_seconds seconds per target"
tools/fuzz.sh build "$out/check-fuzz"
tools/fuzz.sh run "$out/check-fuzz" "$fuzz_seconds" "$((jobs / 2 > 0 ? jobs / 2 : 1))"

[ "$tier" = full ] || exit 0

for name in linux-clang-20-tsan linux-clang-20 linux-shared; do
    preset "$name"
done

step "the large scenes, and the dense forms under AddressSanitizer"
ctest --preset nightly
ctest --preset dense-asan -j "$jobs"

step "all checks passed"
