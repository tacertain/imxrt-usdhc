# imxrt-usdhc

A USDHC SD card driver for i.MX RT MCUs with support for
[`embedded-sdmmc`](https://crates.io/crates/embedded-sdmmc).

Implements `embedded_sdmmc::BlockDevice` so you can read and write FAT16/FAT32
filesystems on SD cards connected to the i.MX RT USDHC peripheral.

## Features

- SDSC (≤ 2 GB) and SDHC (> 2 GB) card support
- 4-bit bus, ~25 MHz Default Speed mode
- Single-block PIO read/write (no DMA)
- Generic over USDHC instance number (USDHC1, USDHC2)
- Optional `defmt` support via the `defmt` feature flag

## Usage

Pin muxing and clock configuration must be done before constructing the driver.
On Teensy 4.1, the BSP configures the USDHC1 root clock automatically and
exposes `board::USDHC1_FREQUENCY`. Prepare each SD pin with
`hal::iomuxc::usdhc::prepare()`, then pass the peripheral and clock frequency
to `Usdhc::new()`:

```rust
use imxrt_usdhc::{Usdhc, embedded_sdmmc};

hal::iomuxc::usdhc::prepare(&mut pins.p45); // CMD
hal::iomuxc::usdhc::prepare(&mut pins.p44); // CLK
hal::iomuxc::usdhc::prepare(&mut pins.p43); // DATA0
hal::iomuxc::usdhc::prepare(&mut pins.p42); // DATA1
hal::iomuxc::usdhc::prepare(&mut pins.p47); // DATA2
hal::iomuxc::usdhc::prepare(&mut pins.p46); // DATA3

let sd = Usdhc::new(usdhc1, board::USDHC1_FREQUENCY)?;
let volume_mgr = embedded_sdmmc::VolumeManager::new(sd, time_source);
```

`BlockDevice` methods take `&self`, so the driver does not enforce mutual
exclusion internally. The caller must ensure only one context accesses the
driver at a time (e.g. via RTIC resource locking or NVIC masking).

## Development

To check the driver, enable an `imxrt-ral` chip feature:

```
cargo check --features=imxrt-ral/imxrt1062
```

To run unit tests (no hardware required):

```
cargo test
```

For a hardware example, see `examples/rtic_sd_info.rs`.

## License

Licensed under either of

- [Apache License, Version 2.0](LICENSE-APACHE)
- [MIT License](LICENSE-MIT)

at your option.
