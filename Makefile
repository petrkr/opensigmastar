# SPDX-License-Identifier: GPL-2.0
# make VENDOR=<dir with the 4.9 vendor .ko> KDIR=<kernel build dir>
KDIR ?= ../linux/build-min
VENDOR ?= blobs
# vendor modules, relative to VENDOR, in load order
BLOBS := mhal mi_common mi_sys mi_rgn mi_ldc mi_ai mi_ao mi_mipitx mi_sensor \
	mi_shadow mi_divp mi_vif mi_vpe mi_venc sensor/sensor_imx415_mipi
BLOB_NAMES := $(notdir $(BLOBS))
PYTHON ?= python3


all: modules

relink:
	for b in $(BLOBS); do \
		$(PYTHON) scripts/relink.py --map kernel/compat/redefine.map --symvers $(KDIR)/Module.symvers \
			$(VENDOR)/$$b.ko $$(basename $$b) kernel/blobs || exit 1; \
	done

modules: relink
	$(MAKE) -C $(KDIR) M=$(CURDIR)/kernel OSM_BLOBS="$(BLOB_NAMES)" modules


clean:
	$(MAKE) -C $(KDIR) M=$(CURDIR)/kernel clean
	rm -f kernel/blobs/*_blob.o_shipped kernel/blobs/*_wrap.c

.PHONY: all relink modules clean
