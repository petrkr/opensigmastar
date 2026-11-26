# opensigmastar

Out-of-tree kernel support to run the SigmaStar SSC338Q (Infinity6E) video
stack (sensor, VIF, ISP, VPE, VENC) on a current Linux kernel instead of
the vendor's unmaintained 4.9 BSP.

The peripheral drivers that can live in mainline (clocks, pinctrl, I2C,
Ethernet, SD, SPI NAND, USB, ...) are not here. They are in the Linux tree
(branch `sstar-ssc338q`). This repository has only what mainline will not
take: ports of the vendor support code and wrappers around the closed
vendor modules.

## Compatibility

| SoC     | Family     | Board      | Vendor SDK (mhal) | Status |
|---------|------------|------------|-------------------|--------|
| SSC338Q | Infinity6E | SSC_38M_2.0 | 22050121         | H.264 1080p20 RTSP, JPEG snapshot and auto exposure work |

Other SoCs are not supported yet.

## Known vendor bugs

- Division by zero at stream start: `_MI_VENC_OnInputPortFrcChanged`
  (mi_venc) divides by its cached previous frame rate, which is 0 on the
  first call, and `IQ_ISOIndexGet` (mhal) divides by zero before the first
  AE statistics. The vendor kernel patches `__aeabi_uidiv` to the hardware
  `udiv` instruction, which returns 0, so this was never visible there.
  `osm___aeabi_uidiv` uses `udiv` as well, with the same result.

## Firmware

The vendor modules open two firmware files by path:

| File          | Used by          | Content                                        |
|---------------|------------------|------------------------------------------------|
| `chagall.bin` | mhal (VENC)      | encoder firmware, must match the mhal version  |
| `iqfile0.bin` | mhal (ISP)       | ISP tuning of the sensor/board                 |

`scripts/osm-load.sh` expects them in `/lib/firmware/sstar/` (env `FW`
changes it) and passes that location to the vendor modules with their own
parameters. Without them the vendor modules fall back to their built-in
vendor paths (`/config/iqfile`, `/config/venc_fw/chagall.bin`,
`/data/ut/chagall.bin`). When loading the modules by hand, set:

- `mhal drv_venc_wrapper.fw_load_path=<dir>/chagall.bin` (the
  `mi_venc fw_path=` parameter is not used for loading)
- `echo isproot <dir> >/dev/ispmid` (after mi_venc, before the sensor
  module) for `iqfile0.bin`; without it mhal silently falls back to a
  built-in default IQ

The files are opened in the root of the process that starts the stream: a
streamer running in a chroot needs them at the same path inside the chroot.
The vendor 3A library (`libcus3a.so`) also reads `<isproot>/iqfile0.bin`
itself.

## Audio

The vendor audio modules `mi_ai` and `mi_ao` are built but not loaded by
`scripts/osm-load.sh`: audio (BACH) is handled by the mainline ALSA driver
(`sound/soc/sstar/`, Linux tree), which drives the same hardware block.
Use ALSA instead of the vendor MI_AI/MI_AO API.

## Layout

- `kernel/compat/`: compatibility layer that provides the Linux 4.9 kernel
  API expected by the vendor modules on top of the current kernel
- `kernel/sstar/`: vendor support drivers ported from source (OS wrapper,
  MIU, memory heap, system DMA, ...)
- `kernel/blobs/`: Kbuild wrappers that relink the closed vendor modules
  into modules for the current kernel
- `scripts/`: helpers, including fetching the vendor modules

The vendor modules are not part of this repository. `scripts/` extracts them
from an original firmware dump.

## Credits

- The approach follows OpenIPC's
  [openhisilicon](https://github.com/OpenIPC/openhisilicon), which does the
  same for HiSilicon and Goke SoCs: building the vendor support modules
  from source and relinking the closed vendor modules with thin wrappers
  for current kernels. No code is copied from it (its kernel modules are
  GPL-3.0, this repository is GPL-2.0).
- The vendor support code and the register information come from the
  SigmaStar Linux 4.9 BSP maintained by OpenIPC
  (https://github.com/OpenIPC/linux, branch `sigmastar-infinity6e`).
- The closed modules are extracted from an original firmware dump.

## License

GPL-2.0, except the vendor binaries, which keep their own terms and are not
distributed here.
