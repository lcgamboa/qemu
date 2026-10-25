#!/usr/bin/env bash

set -euo pipefail

export DEBIAN_FRONTEND="noninteractive"

apt-get update -y -q -o Acquire::Retries=5
apt-get install -y -q --no-install-recommends \
    build-essential \
    git \
    libgcrypt-dev \
    libglib2.0-dev \
    libpixman-1-dev \
    libsdl2-dev \
    libslirp-dev \
    ninja-build \
    python3-pip \
    wget \
    zlib1g-dev \
&& :

# Even though ./configure installs meson, just specify a version
/usr/bin/pip3 install meson==1.7.0 tomli==2.2.1
