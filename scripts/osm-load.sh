#!/bin/sh
# SPDX-License-Identifier: GPL-2.0
#
# Load the vendor stack on the camera from a directory of .ko files and
# point it at the firmware by the vendor's own mechanisms (mhal
# drv_venc_wrapper.fw_load_path for chagall.bin, mhal isproot for
# iqfile0.bin). The files are opened by mhal kernel threads and by
# libcus3a; both see the main root (osm_compat opens absolute paths of
# kernel threads in the root of init). No chroot is needed: firmware only
# in $FW.
#
# usage: osm-load.sh [module dir]   (default: current dir)
# env:   FW = firmware dir (default /lib/firmware/sstar: chagall.bin,
#             iqfile0.bin)

D=${1:-.}
FW=${FW:-/lib/firmware/sstar}

die() {
	echo "osm-load: $*" >&2
	exit 1
}

load() {
	n=$1
	shift
	insmod "$D/$n.ko" "$@" || die "insmod $n failed"
}

load osm_compat
load osm_sstar mma_heap=mma_heap_name0,miu=0,sz=0x9E9C000
load mhal drv_venc_wrapper.fw_load_path="$FW/chagall.bin"
load mi_common
load mi_sys cmdQBufSize=768 logBufSize=256 bEnableMmu=0
# no mi_ai/mi_ao: replaced by ALSA, see README
# no mi_mipitx, mi_shadow, mi_divp: not used by the stream (tested); mi_rgn
# and mi_ldc are load dependencies of mi_vpe
for n in mi_rgn mi_ldc mi_sensor mi_vif mi_vpe; do
	load $n
done
load mi_venc

if [ ! -e /dev/mi_poll ]; then
	major=$(awk '$2 == "mi_poll" { print $1 }' /proc/devices)
	[ -n "$major" ] || die "mi_poll not in /proc/devices"
	mknod /dev/mi_poll c "$major" 0 || die "mknod /dev/mi_poll failed"
fi

echo isproot "$FW" >/dev/ispmid || die "cannot set isproot"
load sensor_imx415_mipi chmap=1
