# SPDX-License-Identifier: GPL-3.0-or-later
# Linux environment for building and running Zephyr native_sim tests of the
# TC32 workspace (Zephyr 4.4 needs Python 3.12 or newer).
FROM python:3.12-slim-bookworm
RUN apt-get update \
 && apt-get install -y --no-install-recommends \
      ninja-build gperf device-tree-compiler gcc g++ make file libc6-dev \
 && rm -rf /var/lib/apt/lists/*
COPY requirements-base.txt /tmp/requirements-base.txt
# Zephyr 4.4 needs CMake 3.28 or newer; bookworm has 3.25.
RUN pip install --no-cache-dir -r /tmp/requirements-base.txt 'cmake>=3.28'
