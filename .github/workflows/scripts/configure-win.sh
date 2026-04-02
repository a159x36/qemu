#!/usr/bin/env bash

set -euo pipefail

TARGET=${TARGET:-xtensa-softmmu}
VERSION=${VERSION:-dev}

echo DBG
./configure --help

./configure \
    --bindir=xtensa-softmmu \
    --datadir=share/qemu \
    --enable-gcrypt \
    --enable-iconv \
    --enable-gtk \
    --enable-pixman \
    --disable-sdl \
    --enable-strip \
    --enable-slirp \
    --enable-stack-protector \
    --extra-cflags=-Werror \
    --prefix=${PWD}/install/qemu \
    --target-list=${TARGET} \
    --with-pkgversion="${VERSION}" \
    --with-suffix="" \
    --enable-fdt=disabled \
    --without-default-features \
    --enable-download \
|| { cat meson-logs/meson-log.txt && false; }


# This fixes the issue that for some reason, meson is not able to determine correct
# paths for libiconv and libintl libraries from 'pkg-config --libs --static libgcrypt'.
# MSYS_BASE=$(cygpath -w / | sed 's/\\/\//g')
# sed -i "s|/mingw64/lib/libintl.dll.a|${MSYS_BASE}/mingw64/lib/libintl.dll.a|g; s|/mingw64/lib/libiconv.dll.a|${MSYS_BASE}/mingw64/lib/libiconv.dll.a|g" build/build.ninja
