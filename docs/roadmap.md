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

### Software-only testing strategy

The SD protocol state machine (card initialization, CSD parsing) contains
pure logic that could be unit-tested without hardware. Possible approaches:

- Extract CSD parsing into pure functions that take register values as input
- Create a mock register backend for the command state machine
- Test error recovery paths (timeout, CRC error) with injected faults

### CI

Set up CI with `cargo check --features imxrt-ral/imxrt1062` and `cargo clippy`.
Hardware testing remains manual on Teensy 4.1.
