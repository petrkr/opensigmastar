# SPDX-License-Identifier: GPL-2.0
# make VENDOR=<dir with the 4.9 vendor .ko> KDIR=<kernel build dir>
KDIR ?= ../linux/build-min
VENDOR ?= blobs
# vendor modules, relative to VENDOR, in load order
BLOBS := mhal mi_common mi_sys mi_rgn mi_ldc mi_ai mi_ao mi_mipitx mi_sensor \
	mi_shadow mi_divp mi_vif mi_vpe mi_venc sensor/sensor_imx415_mipi
BLOB_NAMES := $(notdir $(BLOBS))
PYTHON ?= python3
# mhal CMDQ: vendor (blob implementation), shim (osm_cmdq forwards and logs),
# own (osm_cmdq is the engine driver), see kernel/cmdq/osm_cmdq*.c
CMDQ ?= vendor

# Kernel source and build tree for the board DTS (dtc from the build tree)
KSRC ?= $(KDIR)/source
DTC ?= $(KDIR)/scripts/dtc/dtc
DTB := kernel/dts/ssc38m-v2-osm.dtb

all: modules

relink:
	for b in $(BLOBS); do \
		$(PYTHON) scripts/relink.py --map kernel/compat/redefine.map --symvers $(KDIR)/Module.symvers --extra-exports scripts/relink.exports --cmdq $(CMDQ) \
			$(VENDOR)/$$b.ko $$(basename $$b) kernel/blobs || exit 1; \
	done

modules: relink
	$(MAKE) -C $(KDIR) M=$(CURDIR)/kernel OSM_BLOBS="$(BLOB_NAMES)" OSM_CMDQ="$(CMDQ)" modules
	$(PYTHON) scripts/patch_mhal_riu.py kernel/blobs/mhal.ko

dtb: $(DTB)

$(DTB): kernel/dts/ssc38m-v2-osm.dts kernel/dts/sstar-video.dtsi kernel/dts/osm-compat.dtsi
	cpp -nostdinc -undef -D__DTS__ -x assembler-with-cpp \
		-I $(KSRC)/include -I $(KSRC)/arch/arm/boot/dts -I kernel/dts \
		$< | $(DTC) -q -I dts -O dtb -o $@ -

clean:
	$(MAKE) -C $(KDIR) M=$(CURDIR)/kernel clean
	rm -f kernel/blobs/*_blob.o_shipped kernel/blobs/*_wrap.c $(DTB)

.PHONY: all relink modules dtb clean
