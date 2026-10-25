#!/bin/bash
# Build RenesasXHCIFix.kext with Command Line Tools + MacKernelSDK + Lilu SDK (no Xcode).
set -euo pipefail
cd "$(dirname "$0")"
SDK=../vendor/MacKernelSDK
LILU=../vendor/lilu-debug/Lilu.kext/Contents/Resources
OUT=build/RenesasXHCIFix.kext
CONF=${CONF:-RELEASE}

rm -rf build && mkdir -p build/obj "$OUT/Contents/MacOS"

CFLAGS=(-target x86_64-apple-macos10.13 -mkernel -fno-builtin -fno-common -nostdinc
	-DKERNEL -DKERNEL_PRIVATE -DDRIVER_PRIVATE -DAPPLE -DNeXT
	-DPRODUCT_NAME=RenesasXHCIFix -DMODULE_VERSION=1.0.0 -DLILU_KEXTPATCH_SUPPORT
	-isystem "$SDK/Headers" -I "$LILU" -I "$SDK/Headers/IOKit" -Wall -Wno-unused-function -O2)
[ "$CONF" = DEBUG ] && CFLAGS+=(-DDEBUG)
CXXFLAGS=("${CFLAGS[@]}" -std=c++17 -fapple-kext -fno-rtti -fno-exceptions -fno-asynchronous-unwind-tables)

clang "${CFLAGS[@]}" -c kmod_info.c -o build/obj/kmod_info.o
clang++ "${CXXFLAGS[@]}" -c kern_start.cpp -o build/obj/kern_start.o
clang++ "${CXXFLAGS[@]}" -c "$LILU/Library/plugin_start.cpp" -o build/obj/plugin_start.o

clang++ -target x86_64-apple-macos10.13 -nostdlib -Xlinker -kext -Xlinker -export_dynamic \
	build/obj/*.o "$SDK/Library/x86_64/libkmod.a" -o "$OUT/Contents/MacOS/RenesasXHCIFix"
cp Info.plist "$OUT/Contents/Info.plist"
echo "built $OUT"
