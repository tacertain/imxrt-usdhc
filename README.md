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

## Development

To check the driver, enable an `imxrt-ral` chip feature:

```
cargo check --features=imxrt-ral/imxrt1062
```

To test on hardware, see the `rtic_sd_info` example in
[`teensy4-rs`](https://github.com/mciantyre/teensy4-rs).

## License

Licensed under either of

- [Apache License, Version 2.0](LICENSE-APACHE)
- [MIT License](LICENSE-MIT)

at your option.
