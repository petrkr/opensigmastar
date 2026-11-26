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
#        SENSOR = vendor (default, sensor_imx415_mipi.ko) or mainline
#             (mainline imx415.ko + osm_snr_v4l2.ko proxy plug-in, needs the
#             in-tree media modules and imx415 in the module dir)
#        V4L2 = 1: also load the V4L2 capture device (osm_v4l2 + osm_pipe,
#             /dev/video0, NV12 1920x1080, the pipeline starts on STREAMON;
#             needs the in-tree media modules in the module dir and
#             $FW/imx415.bin, default 0). Do not run a userspace MI
#             application at the same time.
#        CMDQ = vendor (default) | shim | own: must match the CMDQ= the
#             modules were built with (osm-build, make CMDQ=); own loads
#             osm_cmdq before mhal, CMDQ_ARGS = its module parameters (shim
#             is part of mhal, parameters mhal.ntrace=N, mhal.dump)
#        ISPBIN = ISP tuning bin for V4L2=1, relative to /lib/firmware
#             (default sstar/imx415.bin, empty = none)

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

# load unless already there (media modules shared by SENSOR=mainline, V4L2=1)
loadonce() {
	m=$(echo "$1" | tr - _)
	grep -q "^$m " /proc/modules || load "$@"
}

load osm_compat
load osm_sstar mma_heap=mma_heap_name0,miu=0,sz=0x9E9C000
case ${CMDQ:-vendor} in
vendor | shim) ;;
own) load osm_cmdq $CMDQ_ARGS ;;
*) die "CMDQ must be vendor, shim or own" ;;
esac
load mhal drv_venc_wrapper.fw_load_path="$FW/chagall.bin" $MHAL_ARGS
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
case ${SENSOR:-vendor} in
vendor) load sensor_imx415_mipi chmap=1 ;;
mainline)
	for n in mc videodev v4l2-async v4l2-fwnode v4l2-cci; do
		loadonce $n
	done
	# the plug-in releases the sensor reset/PDN pins (VIF sensor
	# interface), so it must come before imx415, which probes the sensor
	load osm_snr_v4l2 chmap=1
	load imx415
	;;
*) die "SENSOR must be vendor or mainline" ;;
esac

if [ "${V4L2:-0}" = 1 ]; then
	for n in mc videodev videobuf2-common videobuf2-memops videobuf2-v4l2 \
		videobuf2-vmalloc; do
		loadonce $n
	done
	load osm_isp
	load osm_v4l2
	load osm_pipe isp_bin="${ISPBIN-sstar/imx415.bin}"
fi
