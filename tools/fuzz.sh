#!/usr/bin/env bash
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Sirac Ozmen
#
# Builds and runs the fuzz targets, the same way in CI and locally.
#
#   tools/fuzz.sh targets OUT                 the targets the build defines,
#                                             as a JSON array (configures OUT)
#   tools/fuzz.sh build OUT                   the targets, with Clang 20 and
#                                             the sanitizers, into OUT
#   tools/fuzz.sh seeds OUT                   every target's committed seeds,
#                                             into out/corpus-run/TARGET
#   tools/fuzz.sh run OUT SECONDS JOBS        every target for SECONDS from
#                                             its committed seeds, JOBS at once
#   tools/fuzz.sh night OUT TARGET SECONDS    one target for SECONDS from
#                                             out/corpus/TARGET and its seeds,
#                                             the corpus merged back there
#
# A target's seeds are test/corpus/TARGET.tar.gz, which must exist; fuzz_tile
# also gets test_tile --seed's tile. Every run goes through
# tools/fuzz_target.sh, bounded by -max_total_time: it keeps the log in
# out/fuzz-logs and a finding in out/fuzz-crashes, and reports a finding
# without its input. CI's fuzz job runs the targets as run does, step by
# step.
set -euo pipefail

cd "$(dirname "$0")/.."
mode=${1:?usage: tools/fuzz.sh targets|build|seeds|run|night OUT ...}
out=${2:?usage: tools/fuzz.sh targets|build|seeds|run|night OUT ...}
# The build's log goes beside OUT, which a fresh checkout does not have.
mkdir -p "$(dirname "$out")"

# The committed seeds of a target, unpacked into a directory. A target
# without seeds is an error, never an empty start.
seeds() { # target directory
    local tarball=test/corpus/$1.tar.gz
    [ -f "$tarball" ] || {
        echo "::error::$1 has no seeds: $tarball is missing"
        return 1
    }
    mkdir -p "$2"
    tar -xzf "$tarball" -C "$2" || return 1
    if [ "$1" = fuzz_tile ]; then
        "$out/test_tile" --seed "$2/level.tile" >/dev/null || return 1
    fi
}

# One target for some seconds over corpus directories, new inputs written
# to the first; a finding leaves its report in out/fuzz-reports.
fuzz() { # target seconds corpus...
    local target=$1 seconds=$2
    shift 2
    tools/fuzz_target.sh "$out/$target" -max_total_time="$seconds" -timeout=30 "$@"
}

# Every built target, or an error when none was built.
built() {
    targets=("$out"/fuzz_*)
    [ -x "${targets[0]}" ] || {
        echo "::error::no fuzz target was built"
        exit 1
    }
}

# Every built target's seeds, into out/corpus-run/TARGET.
seed_all() {
    local target name
    for target in "${targets[@]}"; do
        name=$(basename "$target")
        rm -rf "out/corpus-run/$name"
        seeds "$name" "out/corpus-run/$name"
    done
}

# Configures the fuzz build and lists the targets it defines, one a line, as
# CMake writes them to OUT/fuzz-targets.txt.
list_targets() {
    cmake -B "$out" -G Ninja -DCMAKE_C_COMPILER=clang-20 -DCMAKE_CXX_COMPILER=clang++-20 \
        -DMAUL_NAV_FUZZ=ON -DMAUL_NAV_WERROR=ON -DMAUL_NAV_BUILD_BENCH=OFF \
        -DMAUL_NAV_BUILD_SAMPLES=OFF >"$out.log"
    local targets
    targets=$(cat "$out/fuzz-targets.txt")
    [ -n "$targets" ] || {
        echo "::error::the build defines no fuzz target" >&2
        return 1
    }
    echo "$targets"
}

case "$mode" in
targets)
    targets=$(list_targets)
    # shellcheck disable=SC2086
    printf '[%s]\n' "$(printf '"%s",' $targets | sed 's/,$//')"
    ;;
build)
    targets=$(list_targets)
    # test_tile writes fuzz_tile's seed.
    # shellcheck disable=SC2086
    cmake --build "$out" --target $targets test_tile >>"$out.log"
    ;;
seeds)
    built
    seed_all
    ;;
run)
    seconds=${3:?seconds}
    jobs=${4:?jobs}
    built
    echo "${#targets[@]} targets, $seconds s each, $jobs at once"
    seed_all
    # The step pattern of tools/fuzz_target.sh: the reports cleared, the
    # targets' status kept, and every report printed once they finish.
    rm -rf out/fuzz-reports
    status=0
    export out seconds
    export -f fuzz
    # shellcheck disable=SC2016
    printf '%s\n' "${targets[@]}" |
        xargs -P "$jobs" -I {} bash -c 'fuzz "$(basename "$1")" "$seconds" "out/corpus-run/$(basename "$1")"' _ {} ||
        status=$?
    cat out/fuzz-reports/*.txt 2>/dev/null || true
    exit "$status"
    ;;
night)
    target=${3:?target}
    seconds=${4:?seconds}
    # fuzz-night gives each target 30 minutes. Setting up, starting the
    # image and building take about 40 s, and unpacking the seeds and the
    # merge after the run up to 120 s on the largest corpora (fuzz_bake,
    # fuzz_flight), a time that grows with the corpus. 1500 s leaves those
    # steps more than twice that, so the run is never killed before its
    # corpus is merged and saved. libFuzzer reads a negative
    # -max_total_time as no limit at all. At most four digits are matched
    # before the range test, since bash's integers overflow past 19.
    if ! [[ $seconds =~ ^[1-9][0-9]{0,3}$ ]] || ((seconds > 1500)); then
        echo "::error::fuzz seconds must be a whole number from 1 to 1500, not '$seconds'" >&2
        exit 2
    fi
    corpus=out/corpus/$target
    rm -rf "out/seeds/$target"
    mkdir -p "$corpus"
    seeds "$target" "out/seeds/$target"
    status=0
    rm -rf out/fuzz-reports
    # The seeds are a second corpus, so a night without a cached corpus
    # still starts from them; new inputs go to the first.
    fuzz "$target" "$seconds" "$corpus" "out/seeds/$target" || status=1
    cat out/fuzz-reports/*.txt 2>/dev/null || true
    # The merge keeps the inputs that reach new code, from the last nights'
    # corpus and the seeds, so the corpus never falls below the seeds.
    # The merge runs every input again; fuzz_target.sh keeps its log on
    # the machine and fails, with a report like a run's, when an input
    # crashed, so that corpus is not saved.
    rm -rf "out/merged/$target"
    if ! tools/fuzz_target.sh --merge "$out/$target" "out/merged/$target" "$corpus" "out/seeds/$target"; then
        cat "out/fuzz-reports/$target.txt" 2>/dev/null || true
        exit 1
    fi
    rm -rf "$corpus"
    mv "out/merged/$target" "$corpus"
    echo "$target: corpus of $(find "$corpus" -type f | wc -l) inputs"
    exit $status
    ;;
*)
    echo "usage: tools/fuzz.sh targets|build|seeds|run|night OUT ..." >&2
    exit 2
    ;;
esac
