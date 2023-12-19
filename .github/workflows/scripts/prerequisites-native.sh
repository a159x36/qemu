#!/usr/bin/env bash

set -euo pipefail

export DEBIAN_FRONTEND="noninteractive"

apt-get update -y -q \
&& apt-get install -y -q --no-install-recommends \
    build-essential \
    git \
    libgcrypt-dev \
    libglib2.0-dev \
    libpixman-1-dev \
    libsdl2-dev \
    libslirp-dev \
    ninja-build \
    libgtk-3-dev \
    python3-pip \
    libvte-2.91-dev \
    wget \
    zlib1g-dev \
&& :

# Even though ./configure installs meason, just specify a version
/usr/bin/pip3 install meson==1.2.3
