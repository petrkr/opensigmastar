#!/bin/sh
# SPDX-License-Identifier: GPL-2.0
#
# Build the kernel for the vendor modules: the mainline tree plus the local
# patches in patches/ (see docs/vendor-kernel-patches.md). The patches are
# applied only for the build and reverted afterwards, so the kernel tree
# stays clean; the result goes to its own build directory.
#
# usage: build-kernel.sh <kernel source> <config build dir> <output build dir> [make args]
#        build-kernel.sh --patchset-id   (print the id used in the release)
set -eu
LC_ALL=C
export LC_ALL

patches=$(realpath "$(dirname "$0")/../patches")

# Hash ordered patch names and contents, without including their directory.
patch_records=
for p in "$patches"/*.patch; do
	patch_name=${p##*/}
	patch_sha=$(sha256sum "$p" | awk '{print $1}')
	patch_records="${patch_records}${patch_name} ${patch_sha}
"
done
patchset_sha=$(printf '%s' "$patch_records" | sha256sum | cut -c1-64)
patchset_id=$(printf '%s' "$patchset_sha" | cut -c1-12)
if [ "${1:-}" = --patchset-id ]; then
	echo "$patchset_id"
	exit 0
fi

ksrc=$(realpath "$1")
cfgdir=$(realpath "$2")
out=$3
shift 3

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
{
	printf 'aggregate_sha256 %s\n' "$patchset_sha"
	printf 'patchset_id %s\n' "$patchset_id"
	printf '%s' "$patch_records" | awk '{print "patch " $0}'
} > "$out/osm-patchset.manifest"
cp "$cfgdir/.config" "$out/.config"
cat "$(dirname "$0")/osm.config" >> "$out/.config"
make -C "$ksrc" O="$(realpath "$out")" LOCALVERSION="-p$patchset_id" olddefconfig
make -C "$ksrc" O="$(realpath "$out")" "$@" LOCALVERSION="-p$patchset_id" zImage dtbs modules_prepare
kernel_release=$(cat "$out/include/config/kernel.release")
case "$kernel_release" in
	*-osm-patches-p"$patchset_id") ;;
	*)
		printf 'unexpected kernel release: %s\n' "$kernel_release" >&2
		exit 1
		;;
esac
cp "$out/vmlinux.symvers" "$out/Module.symvers"
