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
#   tools/fuzz.sh run OUT SECONDS JOBS        every target for SECONDS from
#                                             its committed seeds, JOBS at once
#   tools/fuzz.sh night OUT TARGET SECONDS    one target for SECONDS from
#                                             out/corpus/TARGET and its seeds,
#                                             the corpus merged back there
#
# A target's seeds are test/corpus/TARGET.tar.gz, which must exist; fuzz_tile
# also gets test_tile --seed's tile. Logs go to out/fuzz-logs, the inputs that
# failed to out/fuzz-crashes. Every run is bounded by -max_total_time.
set -euo pipefail

cd "$(dirname "$0")/.."
mode=${1:?usage: tools/fuzz.sh targets|build|run|night OUT ...}
out=${2:?usage: tools/fuzz.sh targets|build|run|night OUT ...}
mkdir -p out/fuzz-logs out/fuzz-crashes

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
# to the first; its log is printed when it finds something.
fuzz() { # target seconds corpus...
    local target=$1 seconds=$2 log=out/fuzz-logs/$1.log
    shift 2
    if "$out/$target" -max_total_time="$seconds" -timeout=30 -artifact_prefix=out/fuzz-crashes/ \
        "$@" >"$log" 2>&1; then
        echo "$target: no finding in $seconds s"
    else
        tail -n 60 "$log"
        echo "::error::$target found something"
        return 1
    fi
}

# Configures the fuzz build and lists the targets it defines, one a line.
list_targets() {
    cmake -B "$out" -G Ninja -DCMAKE_C_COMPILER=clang-20 -DCMAKE_CXX_COMPILER=clang++-20 \
        -DMAUL_NAV_FUZZ=ON -DMAUL_NAV_WERROR=ON -DMAUL_NAV_BUILD_BENCH=OFF \
        -DMAUL_NAV_BUILD_SAMPLES=OFF >"$out.log"
    local targets
    targets=$(ninja -C "$out" -t targets all | grep -oE '^fuzz_[a-z0-9]+:' | tr -d ':' | sort -u)
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
        seeds "$name" "out/corpus-run/$name" || return 1
        fuzz "$name" "$seconds" "out/corpus-run/$name"
    }
    export -f run_one seeds fuzz
    export out seconds
    printf '%s\n' "${targets[@]}" | xargs -P "$jobs" -I {} bash -c 'run_one "$1"' _ {}
    ;;
night)
    target=${3:?target}
    seconds=${4:?seconds}
    corpus=out/corpus/$target
    rm -rf "out/seeds/$target"
    mkdir -p "$corpus"
    seeds "$target" "out/seeds/$target"
    status=0
    # The seeds are a second corpus, so a night without a cached corpus
    # still starts from them; new inputs go to the first.
    fuzz "$target" "$seconds" "$corpus" "out/seeds/$target" || status=1
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
    echo "usage: tools/fuzz.sh targets|build|run|night OUT ..." >&2
    exit 2
    ;;
esac
