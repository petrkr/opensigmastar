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

## Vendor modules: status

All original vendor modules, with the size of the blob object that is
relinked. "Loaded as-is" means the closed code runs unmodified on top of
`osm_compat` (4.9 ABI layer). The `make` options (`CMDQ=`, `VIF=`, `SCL=`,
`JPE=`, `SENSOR=` for `scripts/osm-load.sh`) are build/load-time switches;
the default is always the vendor code. "Passes" means the 1080p20 RTSP
stream, JPEG snapshot, V4L2 capture and auto exposure were checked with
that option on the camera.

| Module (blob size) | Purpose | Status | Current use |
|---|---|---|---|
| `mhal` (1.69 MB) | All hardware drivers: CSI/VIF, ISP + IQ + AE/AWB, scaler, LDC, DIP, GOP, JPE and VENC host, CMDQ, display, audio, sensor registry | Loaded as-is by default; partly replaced via relink cuts (the code stays inside `mhal.ko`): CMDQ (`CMDQ=own`, passes), VIF + CSI (`VIF=own`, passes, 4-lane CSI only), SCL (`SCL=own`, partial: 1080p main port from 4K input passes, JPEG port 1 does not work), JPE (`JPE=shim`, logging shim only, no own implementation committed) | Every stage; ISP, AE/AWB, VENC and JPE stay vendor |
| `mi_common` (11 KB) | `MI_DEVICE_*` register helper, char device/proc glue | Loaded as-is | Dependency of all `mi_*` |
| `mi_sys` (479 KB) | Buffer pools, port binding, PTS/FRC, CMDQ service | Loaded as-is | Used by `osm_pipe` (`MI_SYS_*`) and by userspace via ioctl |
| `mi_sensor` (30 KB) | `MI_SNR_*` over the `MHal_SNR_*` sensor layer | Loaded as-is | Used by `osm_pipe` (`MI_SNR_*`); dependency of `mi_vif`/`mi_vpe` |
| `sensor_imx415_mipi` (31 KB) | IMX415 plug-in: init tables, gain/shutter, modes | Fully replaced by the mainline `imx415` driver through `osm_snr_v4l2` with `SENSOR=mainline` (3840x2160@20 linear only; passes). Vendor module is the default | Registers into the `mhal` sensor registry |
| `mi_vif` (76 KB) | `MI_VIF_*` over the VIF/CSI hardware | Loaded as-is; its `MHal_VIF_*` backend can be replaced inside `mhal` (`VIF=own`) | Used by `osm_pipe` (`MI_VIF_*`) |
| `mi_vpe` (215 KB) | ISP and scaler channel management, `MI_VPE_*IspApiData` | Loaded as-is; scaler backend partly replaceable (`SCL=own`) | Used by `osm_pipe` (`MI_VPE_*`); ISP entry point for the tuning bin |
| `mi_venc` (217 KB) | H.264/H.265/JPEG channels, rate control, stream ring | Loaded as-is; JPE backend only wrapped by a logging shim (`JPE=shim`) | Used by `osm_pipe` (`MI_VENC_*`, compressed V4L2 node); needs `chagall.bin` |
| `mi_rgn` (96 KB) | OSD / cover regions (GOP) | Loaded as-is | Not used by the pipeline; loaded only because `mi_vpe` depends on it |
| `mi_ldc` (620 KB) | Lens distortion correction, DIS, stitching | Loaded as-is | Not used by the pipeline; loaded only because `mi_vpe` depends on it |
| `mi_divp` (64 KB) | DIP scaler / format conversion | Not loaded: unused by the stream (stream tested without it) | Built only; load it before `mi_vif` if a DIVP feature is needed |
| `mi_mipitx` (24 KB) | MIPI TX (display output), likely | Not loaded: unused by the stream, no dependents | Built only |
| `mi_shadow` (48 KB) | Userspace-defined MI modules, likely | Not loaded: unused by the stream (tested) | Built only |
| `mi_ai` (225 KB) | Audio input | Not loaded: replaced by the mainline ALSA driver (see Audio) | Built only |
| `mi_ao` (94 KB) | Audio output | Not loaded: replaced by the mainline ALSA driver | Built only |

Our modules:

- `osm_compat`: Linux 4.9 kernel ABI emulation for the vendor modules.
- `osm_sstar`: vendor support code ported from source (CamOs wrapper, clocks, MIU, memory heap, msys, board GPIO map).
- `osm_isp`: loader that applies the ISP tuning bin (`imx415.bin`) through the vendor ISP API.
- `osm_snr_v4l2`: `mhal` sensor plug-in that drives the mainline `imx415` V4L2 sub-device (`SENSOR=mainline`).
- `osm_cmdq`: driver for the four CMDQ engines behind `MHAL_CMDQ_*` (`CMDQ=own`, loaded before `mhal`).
- `osm_pipe`: in-kernel sensor, VIF, VPE (ISP + scaler) and VENC pipeline built from the vendor `MI_*` implementation entry points.
- `osm_v4l2`: V4L2 capture device (NV12 node and H.264/MJPEG node) on top of `osm_pipe`.
- `mhal_vif_own.c`, `mhal_scl_own.c`: own VIF/CSI and SCL code linked into `mhal.ko` (`VIF=own`, `SCL=own`).
- `mhal_cmdq_shim.c`, `mhal_vif_shim.c`, `mhal_scl_shim.c`, `mhal_jpe_shim.c`: logging/diagnostic shims linked into `mhal.ko` that forward to the vendor code (`CMDQ=shim`, `VIF=shim`, `SCL=shim`, `JPE=shim`).

The firmware files (`chagall.bin`, `iqfile0.bin`, `imx415.bin`) are data, not
modules; they are separate from the status above and are still required.

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
