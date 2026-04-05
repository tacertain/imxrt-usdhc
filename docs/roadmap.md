# imxrt-usdhc Roadmap


## Performance

### Multi-block transfers

The `BlockDevice::read/write` methods currently loop CMD17/CMD24 (single-block)
over the slice. This works but has significant per-block overhead from command
setup and response polling.

Implement CMD18 (READ_MULTIPLE_BLOCK) and CMD25 (WRITE_MULTIPLE_BLOCK) with
CMD12 (STOP_TRANSMISSION) to transfer contiguous blocks in a single command.
This is the single highest-impact performance improvement.

### ADMA2 DMA support

The driver currently uses PIO (programmed I/O) -- the CPU reads/writes 32-bit
words from the DATA_BUFF_ACC_PORT register in a tight loop. This blocks the CPU
for the entire transfer (~160 us per block at 25 MHz).

The USDHC peripheral supports ADMA2 with scatter-gather descriptors. An ADMA2
implementation would need:

- Static descriptor buffer (similar to `imxrt-enet`'s `ReceiveBuffers<N>`)
- D-cache clean/invalidate around DMA transfers (the i.MX RT 1060 has D-cache)
- Possibly a const-generic buffer size parameter

## Correctness

### DAT3 pull resistor and ACMD42

The `imxrt-iomuxc` USDHC pin definitions configure GPIO_SD_B0_05 (DAT3) with
a 100k pull-down. This is correct during card detection (the card's internal
~50k pull-up on DAT3 pulls the line high when inserted), but wrong during
4-bit data transfer: the SD spec expects all data lines pulled high when idle.

After card initialization succeeds, the driver should:

1. Send ACMD42 (SET_CLR_CARD_DETECT) with argument 0 to disconnect the card's
   internal DAT3 pull-up (no longer needed after init).
2. Reconfigure the GPIO_SD_B0_05 pad from `Pulldown100k` to `Pullup100k` so
   DAT3 idles high like DAT0-2.

This requires the driver or BSP to have access to the IOMUXC pad control
register for DAT3 after initialization. Options:

- Accept a mutable reference to the DAT3 pad in `Usdhc::new()` and
  reconfigure it after `card_init()`.
- Add a callback or trait method that the BSP provides for pad reconfiguration.
- Have the BSP reconfigure the pad after `init_usdhc1()` returns, documented
  as a required post-init step.

If hot-plug support is added later, card removal would need to flip DAT3 back
to pull-down before the next insertion/init cycle.

## SD Protocol Features

### High Speed mode (50 MHz)

The driver currently runs at Default Speed (~25 MHz). High Speed mode doubles
the data clock to 50 MHz. This requires:

- CMD6 (SWITCH_FUNC) to negotiate High Speed with the card
- Adjusting SDCLKFS/DVS for the higher frequency

### UHS-I modes (SDR50, SDR104)

Ultra High Speed modes require 1.8V signaling and tuning. This is significantly
more complex:

- Voltage switch sequence (CMD11)
- Tuning procedure (CMD19/CMD21)
- Different clock divisor ranges

### Card hot-plug detection

The USDHC peripheral has card-detect signals. The driver currently only detects
the card at initialization time. Exposing card-detect would allow applications
to handle card insertion/removal at runtime.

### Write-protect signal

The USDHC peripheral can read a write-protect pin. The driver currently ignores
this. Exposing it would allow applications to check before attempting writes.

## Ecosystem

### Move to imxrt-rs organization

The crate should eventually live in the `imxrt-rs` GitHub org alongside
`imxrt-usbd` and `imxrt-enet`. This requires a discussion with the maintainer.

### imxrt-hal re-export

Once the crate is stable, `imxrt-hal` could re-export it (as it does for
`imxrt-enet`) so BSPs get it transitively. This is a convenience, not a
requirement.

### embedded-sdmmc version tracking

`embedded-sdmmc` is pinned to 0.9. When it releases 1.0, the crate will need
an update. This is the main reason the crate is released independently from
`imxrt-hal`.

## Testing

### Software-only testing (done)

The protocol layer is tested without hardware via `FakeSdHost` (in `src/fake.rs`,
`#[cfg(test)]` only). This mock implements the `SdHost` trait with a simulated
card state machine and in-memory block storage. Existing coverage:

- CSD parsing (v1 and v2) with known test vectors
- Full card initialization for SDHC (v2) and SDSC (v1) cards
- Clock divisor computation across multiple source/target combinations
- Block address translation (SDHC passthrough, SDSC byte offset)
- Error injection via `inject_cmd_error()` for timeout/CRC paths

### Additional test coverage

Possible additions to the existing test suite:

- Multi-block read/write correctness (once CMD18/CMD25 are implemented)
- Edge cases in `parse_csd()` (max capacity SDXC, unusual CSD v1 geometries)
- ACMD41 retry exhaustion (timeout after 1000 retries)
- Card re-initialization after `power_cycle()`

### CI

Set up CI with `cargo check --features imxrt-ral/imxrt1062` and `cargo clippy`.
Hardware testing remains manual on Teensy 4.1.
