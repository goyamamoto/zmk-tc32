#!/bin/sh
# SPDX-License-Identifier: GPL-3.0-or-later
# Prints the tag of the Linux image for the native_sim tests (tc32/tests), and
# builds it the first time (network needed). $1: the Zephyr tree, whose
# scripts/requirements-base.txt goes into the image; the tag is a hash of the
# Dockerfile and that file.
set -eu
H=$(cd "$(dirname "$0")" && pwd)
cp "$1/scripts/requirements-base.txt" "$H/requirements-base.txt"
IMG=tlsr-usb-sim:$(cat "$H/Dockerfile" "$H/requirements-base.txt" | shasum -a 256 | cut -c1-12)
docker image inspect "$IMG" >/dev/null 2>&1 || docker build -t "$IMG" "$H" >&2
echo "$IMG"
