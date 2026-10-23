#!/usr/bin/env bash
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Sirac Ozmen
#
# Builds and runs the fuzz targets, the same way in CI and locally.
#
#   tools/fuzz.sh build OUT                   the targets, with Clang 20 and
#                                             the sanitizers, into OUT
#   tools/fuzz.sh run OUT SECONDS JOBS        every target for SECONDS from
#                                             its committed seeds, JOBS at once
#   tools/fuzz.sh night OUT TARGET SECONDS    one target for SECONDS from
#                                             out/corpus/TARGET and its seeds,
#                                             the corpus merged back there
#
# A target's seeds are test/corpus/TARGET.tar.gz; fuzz_tile's first seed is
# written by test_tile --seed. Logs go to out/fuzz-logs, the inputs that
# failed to out/fuzz-crashes. Every run is bounded by -max_total_time.
set -euo pipefail

cd "$(dirname "$0")/.."
mode=${1:?usage: tools/fuzz.sh build|run|night OUT ...}
out=${2:?usage: tools/fuzz.sh build|run|night OUT ...}
mkdir -p out/fuzz-logs out/fuzz-crashes

# The committed seeds of a target, unpacked into a directory.
seeds() { # target directory
    mkdir -p "$2"
    if [ "$1" = fuzz_tile ]; then
        "$out/test_tile" --seed "$2/level.tile" >/dev/null
    else
        tar -xzf "test/corpus/$1.tar.gz" -C "$2"
    fi
}

# One target over a corpus directory for some seconds; its log is printed
# when it finds something.
fuzz() { # target corpus seconds
    local log=out/fuzz-logs/$1.log
    if "$out/$1" -max_total_time="$3" -timeout=30 -artifact_prefix=out/fuzz-crashes/ "$2" \
        >"$log" 2>&1; then
        echo "$1: no finding in $3 s"
    else
        tail -n 60 "$log"
        echo "::error::$1 found something"
        return 1
    fi
}

case "$mode" in
build)
    cmake -B "$out" -G Ninja -DCMAKE_C_COMPILER=clang-20 -DCMAKE_CXX_COMPILER=clang++-20 \
        -DMAUL_NAV_FUZZ=ON -DMAUL_NAV_WERROR=ON -DMAUL_NAV_BUILD_BENCH=OFF \
        -DMAUL_NAV_BUILD_SAMPLES=OFF >"$out.log"
    # test_tile writes fuzz_tile's seed.
    targets=$(ninja -C "$out" -t targets all | grep -oE '^fuzz_[a-z0-9]+:' | tr -d ':' | sort -u)
    [ -n "$targets" ] || {
        echo "::error::no fuzz target to build"
        exit 1
    }
    # shellcheck disable=SC2086
    cmake --build "$out" --target $targets test_tile >>"$out.log"
    ;;
run)
    seconds=${3:?seconds}
    jobs=${4:?jobs}
    targets=("$out"/fuzz_*)
    [ -x "${targets[0]}" ] || {
        echo "::error::no fuzz target was built"
        exit 1
    }
    echo "${#targets[@]} targets, $seconds s each, $jobs at once"
    run_one() {
        local name
        name=$(basename "$1")
        rm -rf "out/corpus-run/$name"
        seeds "$name" "out/corpus-run/$name"
        fuzz "$name" "out/corpus-run/$name" "$seconds"
    }
    export -f run_one seeds fuzz
    export out seconds
    printf '%s\n' "${targets[@]}" | xargs -P "$jobs" -I {} bash -c 'run_one "$1"' _ {}
    ;;
night)
    target=${3:?target}
    seconds=${4:?seconds}
    corpus=out/corpus/$target
    mkdir -p "$corpus" "out/seeds/$target"
    seeds "$target" "out/seeds/$target"
    status=0
    fuzz "$target" "$corpus" "$seconds" || status=1
    # The merge keeps the inputs that reach new code, from the last nights'
    # corpus and the seeds, so the corpus never falls below the seeds.
    rm -rf "out/merged/$target"
    mkdir -p "out/merged/$target"
    "$out/$target" -merge=1 "out/merged/$target" "$corpus" "out/seeds/$target" \
        >"out/fuzz-logs/$target.merge.log" 2>&1
    rm -rf "$corpus"
    mv "out/merged/$target" "$corpus"
    echo "$target: corpus of $(find "$corpus" -type f | wc -l) inputs"
    exit $status
    ;;
*)
    echo "usage: tools/fuzz.sh build|run|night OUT ..." >&2
    exit 2
    ;;
esac
