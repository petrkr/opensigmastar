#!/bin/sh
# SPDX-License-Identifier: GPL-2.0
#
# Build the kernel for the vendor modules: the mainline tree plus the local
# patches in patches/ (see docs/vendor-kernel-patches.md). The patches are
# applied only for the build and reverted afterwards, so the kernel tree
# stays clean; the result goes to its own build directory.
#
# usage: build-kernel.sh <kernel source> <config build dir> <output build dir> [make args]
set -eu

ksrc=$(realpath "$1")
cfgdir=$(realpath "$2")
out=$3
shift 3
patches=$(realpath "$(dirname "$0")/../patches")

applied=
revert() {
	for p in $applied; do
		patch -d "$ksrc" -p1 -R -s < "$p"
	done
}
trap revert EXIT

for p in "$patches"/*.patch; do
	patch -d "$ksrc" -p1 -N -s < "$p"
	applied="$p $applied"
done

mkdir -p "$out"
cp "$cfgdir/.config" "$out/.config"
cat "$(dirname "$0")/osm.config" >> "$out/.config"
make -C "$ksrc" O="$(realpath "$out")" olddefconfig
make -C "$ksrc" O="$(realpath "$out")" "$@" zImage dtbs modules_prepare
cp "$out/vmlinux.symvers" "$out/Module.symvers"
