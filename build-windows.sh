#!/bin/sh
# build-windows.sh - build railphysics.dll on Windows (Git Bash) with
# llvm-mingw, then build the offline harness and, when GAME is set, run it
# against the real game executable (mapped as an image; no game code runs).
#
#   LLVM_MINGW=/c/path/to/llvm-mingw GAME=/c/path/to/SovietRepublic ./build-windows.sh
#
# Output: build/plugins/railphysics.dll + railphysics.ini, ready to drop into
# <game>/tesmioloader/build/plugins/ (close the game first - it holds the DLL).

set -e
cd "$(dirname "$0")"

CXX="${LLVM_MINGW:-}/bin/x86_64-w64-mingw32-clang++"
if [ -z "${LLVM_MINGW:-}" ] || [ ! -x "$CXX" ]; then
    echo "[build] llvm-mingw not found - set LLVM_MINGW=/path/to/llvm-mingw" >&2
    exit 1
fi

mkdir -p build/plugins

echo "[build] build/plugins/railphysics.dll"
"$CXX" -O2 -static -shared -o build/plugins/railphysics.dll \
    plugins/railphysics/railphysics.cpp plugins/railphysics/stubs.S
cp -f plugins/railphysics/railphysics.ini build/plugins/railphysics.ini

echo "[build] build/host_test14.exe (offline harness)"
"$CXX" -O2 -std=c++17 -w -static -o build/host_test14.exe \
    test/host_test14.cpp plugins/railphysics/stubs.S

if [ -n "${GAME:-}" ]; then
    echo "[test] harness against $GAME/SOVIET64.exe"
    build/host_test14.exe "$(cygpath -w "$GAME/SOVIET64.exe")" \
        "$(cygpath -w "$PWD/plugins/railphysics/railphysics.ini")"
else
    echo "[test] GAME not set - harness built, not run"
fi

echo "[build] ok -> build/plugins/"
