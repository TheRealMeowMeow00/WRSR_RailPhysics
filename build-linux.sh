#!/bin/bash
# build-linux.sh - build railphysics.dll on Linux with llvm-mingw (clang).
#
# The plugin is plain C++ + two GNU-as stubs; llvm-mingw compiles it as-is.
# Tested with llvm-mingw 20251118 (ucrt):
#   https://github.com/mstorsjo/llvm-mingw/releases
#
#   LLVM_MINGW=/path/to/llvm-mingw ./build-linux.sh
#
# LLVM_MINGW defaults to the newest llvm-mingw-* in ~/toolchains.
# Output: build/plugins/railphysics.dll + railphysics.ini, ready to drop
# into <game>/tesmioloader/build/plugins/.

set -e
cd "$(dirname "$0")"

if [ -z "${LLVM_MINGW:-}" ]; then
    LLVM_MINGW=$(ls -d "$HOME"/toolchains/llvm-mingw-* 2>/dev/null | sort | tail -1)
fi
if [ -z "$LLVM_MINGW" ] || [ ! -x "$LLVM_MINGW/bin/clang++" ]; then
    echo "[build] llvm-mingw not found - set LLVM_MINGW=/path/to/llvm-mingw" >&2
    exit 1
fi

CXX="$LLVM_MINGW/bin/clang++"
TARGET=x86_64-w64-mingw32
FLAGS="-O2 -Wall -static -shared"

mkdir -p build/plugins

echo "[build] plugins/railphysics.dll"
"$CXX" -target $TARGET $FLAGS -o build/plugins/railphysics.dll \
    plugins/railphysics/railphysics.cpp plugins/railphysics/stubs.S
cp -f plugins/railphysics/railphysics.ini build/plugins/railphysics.ini

echo "[build] ok -> build/plugins/"
echo "[build] install: copy both files into <game>/tesmioloader/build/plugins/"
