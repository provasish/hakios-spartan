# hakios-spartan: the realme GT Neo 3T (RMX3371) for HakiOS

A [HakiOS](https://github.com/oosexgang/hakios-xnu-kernel) device repository for the **realme GT Neo 3T** (`spartan`,
RMX3371), Qualcomm **Snapdragon 870** (SM8250-AC, "kona"): HakiOS's U-Boot for the phone, its device trees, the tools that
check them, and the scripts that test it. The first SM8250 phone of the project.

Everything runs **from RAM** (`fastboot boot`): nothing is flashed, nothing is formatted, and a reboot brings the phone back
to its own Android.

## What works

Seen on an RMX3371 (unlocked bootloader, A14 firmware 14.0.0.2100), 3 November 2026:

| Stage | What | Result |
|---|---|---|
| 1 | HakiOS's U-Boot, started by ABL from RAM | **works**: ABL takes the image; U-Boot's console on the panel (1080x2400 at `0x9C000000`), UFS (6 LUNs, HS gear 3), fastboot over USB (`18d1:4ee0`), the buttons registered; the A/B slots untouched |
| 2 | the HakiOS loader and the open XNU kernel, running `hello` | **works**: the loader knows the phone (`platform spartan`), the kernel boots (GICv3, md0 root) and prints "Hello from user space on Darwin!", then hands the phone back |
| 2, `cpus=8` | the same on every core | **works**: 8 cores registered, the boot core and the four Cortex-A77s online through PSCI ("5 of 8 processors are running"; the A55s 1-3 registered, not started), `hello` on cpu 5 |
| 3 | Darwin's own user space from RAM (launchd, SpringBoard) | not yet tried |

No driver of the phone's is ported yet: the screen is drawn in software; touch, GPU, sound, Wi-Fi and the rest are not
there.

## What is here

| Path | What |
|---|---|
| `u-boot/files/dts/upstream/src/arm64/qcom/sm8250-realme-spartan.dts` | U-Boot's device tree of the phone: the panel's framebuffer as ABL leaves it, USB as a device, UFS, the keys, and every memory region the firmware keeps (its head says where each number comes from) |
| `u-boot/files/configs/hakios_spartan_defconfig`, `u-boot/files/board/qualcomm/hakios-spartan.env` | U-Boot's configuration and environment, as the POCO X5 Pro's |
| `prebuilt/device-tree.dtb` | the phone's own device tree (`/sys/firmware/fdt`, msm-id 356 v2.1, board-id 0x10008) with `/chosen` emptied (no serial number, no boot arguments): what ABL is given, appended to U-Boot. ABL picks an appended tree by msm-id **and** board-id, so it has to be a merged tree like this one |
| `prebuilt/base-21733.dtb` | the base tree of OPLUS dtsi 21733 from the RMX3371 firmware's `boot.img`, for reference: same reserved memory; **not** for appending (board-id 0 0: ABL refuses it) |
| `prebuilt/mkbootimg.py` | AOSP's `mkbootimg.py` (Apache-2.0), as the other HakiOS devices carry it |
| `tools/check-memory.py` | checks that U-Boot's tree reserves every fixed carve-out of the phone's tree, and prints the ids and keys ABL and U-Boot need |
| `test/` | the tester's scripts (`test-spartan.ps1` for Windows, `test-spartan.sh` for Linux/macOS) and `HOW-TO-TEST.txt`: stages 1 and 2 with plain `fastboot`, logs, and the slot put back if ABL flips it |

## Building

In the HakiOS tree with this phone's changes, [provasish/hakios-xnu-kernel-spartan](https://github.com/provasish/hakios-xnu-kernel-spartan)
(HakiOS's `hakios-xnu-kernel` with its history, plus the loader's spartan platform and this repository in
`devices/manifest.conf`), with this repository at `devices/spartan` (`tools/hakios-device.py sync`) and HakiOS's UEFI
repository at `uefi` (branch `u-boot`), on Linux or WSL with `aarch64-linux-gnu-gcc` and `dtc`:

```sh
uefi/u-boot/apply.sh spartan          # U-Boot at uefi/u-boot/BASE, HakiOS's patch, this phone's files
uefi/u-boot/build.sh spartan          # u-boot/build-spartan/u-boot.bin and u-boot.dtb
python3 devices/spartan/tools/check-memory.py devices/spartan/prebuilt/device-tree.dtb u-boot/build-spartan/u-boot.dtb
python3 tools/mkubootimg.py -d spartan --no-ramdisk -o hakios-u-boot.img      # stage 1
```

The loader needs one change for this phone (its platform, known by the framebuffer at `0x9C000000`; and no log written to
realme's `logdump` partition): commit `1853d83` of hakios-xnu-kernel-spartan, not yet in oosexgang's repository.

A checkout made on Windows has CRLF line endings, which the build's shell scripts do not survive: build in WSL from a
checkout made there.

## Testing

`test/HOW-TO-TEST.txt`. Short version: an unlocked RMX3371 (`adb shell getprop ro.boot.prjname` = 21732), battery over 60 %,
fastboot mode, then `test-spartan.ps1 1` / `2` (Windows) or `test-spartan.sh 1` / `2`. Only `fastboot boot`: never
`fastboot flash` any of these images. On Windows the loader appears as a new USB device ("xnuloader") that needs Google's
Android Bootloader Interface driver picked for it once.

## Notes found on the way

- **OPLUS ABL and failed boots.** When ABL cannot start an image it was given with `fastboot boot`, it marks the active slot
  unbootable and makes the other one active. `fastboot set_active <slot>` puts it back; the test scripts do it themselves.
- **The memory map.** The firmware's carve-outs on this phone are the later kona layout (removed_region up to `0x8D800000`,
  the remote processors from `0x8DA00000`), not `sm8250.dtsi`'s, plus realme's own `kboot_uboot_logmem` at `0x9FE00000` and
  `ramoops` at `0xB0000000`. The DTS reserves all of them.
- **Volume down** is the PMIC's RESIN on this phone, not a GPIO.

## Nothing of Apple's, nothing of the phone's firmware

This repository carries no file of Apple's and no firmware of the phone. Darwin's user space (stage 3) is built by each person
from their own IPSW, and never shared.
