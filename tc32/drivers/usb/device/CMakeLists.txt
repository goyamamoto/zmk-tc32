# SPDX-License-Identifier: GPL-3.0-or-later

# The legacy usb_dc driver of the TLSR8278's USB device controller, in Zephyr's
# library of the legacy device controller drivers (drivers__usb__device), which
# zephyr_library_amend() names from this directory's path.
zephyr_library_amend()
zephyr_library_sources(usb_dc_b87.c)
