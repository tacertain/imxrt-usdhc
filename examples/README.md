# Examples

## rtic_sd_info

Initializes an SD card on the Teensy 4.1 and logs card info plus a root
directory listing over USB serial.

**Hardware required**: Teensy 4.1 with a FAT32-formatted micro-SD card inserted.

### Prerequisites

Install the ARM target and `cargo-binutils` if you haven't already:

```
rustup target add thumbv7em-none-eabihf
cargo install cargo-binutils
```

You'll also need a way to flash the resulting `.hex` file — either
[`teensy_loader_cli`](https://github.com/PaulStoffregen/teensy_loader_cli) or
the [Teensy Loader Application](https://www.pjrc.com/teensy/loader.html).

### Build

From the repository root:

```
cargo objcopy --example rtic_sd_info --release -- -O ihex rtic_sd_info.hex
```

Flash `rtic_sd_info.hex` to the Teensy 4.1 using your loader of choice.

### Output

Connect to the Teensy's USB serial port (e.g. with `screen` or PuTTY).
After a 2-second delay for USB enumeration, you should see:

```
[INFO rtic_sd_info::app]: SD card: Sdsc, 1910 MB (3911680 blocks)
[DEBUG embedded_sdmmc::volume_mgr]: Creating new embedded-sdmmc::VolumeManager
[TRACE embedded_sdmmc::volume_mgr]: Reading partition table
[TRACE embedded_sdmmc::fat::volume]: Reading BPB
[TRACE embedded_sdmmc::fat::volume]: Reading info block
[DEBUG embedded_sdmmc::volume_mgr]: Opening root on RawVolume(0x001388)
[DEBUG embedded_sdmmc::volume_mgr]: Opened root on RawVolume(0x001388), got RawDirectory(0x001389)
[INFO rtic_sd_info::app]: Root directory:
[TRACE embedded_sdmmc::fat::volume]: Reading FAT
[INFO rtic_sd_info::app]:   SYSTEM~1
[INFO rtic_sd_info::app]:   FOO.TXT
...
[INFO rtic_sd_info::app]: Done.
```

If no card is detected or initialization fails, the error is logged instead:

```
SD card not initialized (no card or init failed)
```
