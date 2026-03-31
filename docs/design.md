# imxrt-usdhc Design

## Overview

`imxrt-usdhc` is a standalone SD card driver for the i.MX RT USDHC peripheral.
It implements `embedded_sdmmc::BlockDevice`, allowing users to read and write
FAT16/FAT32 filesystems via the `embedded-sdmmc` crate.

The crate follows the pattern established by `imxrt-enet` (Ethernet) and
`imxrt-usbd` (USB) -- a standalone driver crate that implements an ecosystem
trait and is released independently from `imxrt-hal`.

## Crate Architecture

### Dependencies

The crate depends on `imxrt-ral` directly for register access (the "enet model"),
rather than defining its own register blocks (the "usbd model"). This avoids
duplicating the large USDHC register set. The tradeoff is that `imxrt-usdhc`
releases are tied to `imxrt-ral` releases, but those are infrequent.

The crate re-exports `embedded-sdmmc` for user convenience, matching how
`imxrt-enet` re-exports `smoltcp`.

### Module Layout

```
src/
  lib.rs    - Usdhc (public), UsdhcInner (private), BlockDevice impl,
              peripheral config, send_cmd, read/write_single_block
  card.rs   - Card initialization state machine (CMD0..CMD7), CSD parsing
  cmd.rs    - UsdCmdFlags enum, SD command constants, OCR flags
  error.rs  - CardType enum, SdError (Display + core::error::Error)
```

### Instance Generics

The constructor is generic over USDHC instance number:

```rust
pub fn new<const N: u8>(
    usdhc: ral::usdhc::Instance<N>,
    source_clock_hz: u32,
) -> Result<Self, SdError>
```

Internally, the instance number is erased via `into_any()` (same pattern as
`imxrt-enet`) so the `Usdhc` type doesn't carry a const generic.

### Interior Mutability

`embedded_sdmmc::BlockDevice` methods take `&self`, not `&mut self`. The driver
uses a hybrid approach:

- **`UsdhcInner`** (private, `Copy`, `Send`) holds a raw pointer to the MMIO
  register block plus card metadata (type, block count, RCA). It is `Copy` so
  it can be cheaply duplicated if needed.
- **`Usdhc`** (public, not `Copy`) wraps `UsdhcInner` and presents the safe API.
  It is automatically `Send` because `UsdhcInner` is.

Mutual exclusion is **the caller's responsibility**. This is consistent with
`imxrt-usbd` and `imxrt-enet`, and avoids both:
- The interrupt-latency cost of `critical_section::Mutex` (SD transactions take
  ~160 us per block at 25 MHz).
- The `!Sync` limitation of `RefCell`.

Callers typically use RTIC resource locking or NVIC masking to ensure exclusive
access.

### Pin Configuration

Following the `imxrt-enet` model, pin muxing and clock configuration are **not**
part of this crate. The BSP/HAL is responsible for:

1. Configuring the USDHC root clock (e.g. PLL2_PFD2 / 2 = 198 MHz on Teensy 4.1)
2. Enabling the USDHC clock gate
3. Calling `hal::iomuxc::usdhc::prepare()` on each SD pin
4. Passing the configured `ral::usdhc::Instance<N>` to `Usdhc::new()`

### Clock Configuration

The SD bus clock is derived from the USDHC root clock (configured by the BSP in
the CCM) via two internal dividers in the SYS_CTRL register:

- **SDCLKFS** (bits 15:8) -- prescaler, divides by `2 * SDCLKFS`. Must be a
  power of 2 from 1 to 128.
- **DVS** (bits 19:16) -- secondary divider, divides by `DVS + 1`. Range 0-15.

The resulting SD bus clock is: `source_clock_hz / (2 * SDCLKFS * (DVS + 1))`.

The driver computes the optimal SDCLKFS/DVS pair at runtime from the
`source_clock_hz` parameter to hit two target frequencies:

- **Identification mode** (≤ 400 kHz) -- used during CMD0-CMD7.
- **Data transfer mode** (≤ 25 MHz) -- used after card initialization.

For example, with a 198 MHz source (Teensy 4.1):

| Mode | SDCLKFS | DVS | Total divisor | Result |
|---|---|---|---|---|
| Identification | 128 | 1 | 512 | 386 kHz |
| Data transfer | 4 | 0 | 8 | 24.75 MHz |

### SD Card Initialization Sequence

The `card_init()` method in `card.rs` implements the standard SD initialization:

1. CMD0 -- Go Idle State
2. CMD8 -- Send Interface Condition (detect SD v2)
3. ACMD41 -- SD Send Op Condition (negotiate voltage, detect SDHC)
4. CMD2 -- All Send CID
5. CMD3 -- Send Relative Address
6. CMD9 -- Send CSD (read capacity)
7. CMD7 -- Select Card

After initialization, the driver switches to 4-bit bus width (ACMD6) and
~25 MHz clock.

### BlockDevice Implementation

The `BlockDevice` impl loops over the `blocks` slice, issuing one CMD17 (read)
or CMD24 (write) per block. This is correct but not optimal -- see the roadmap
for multi-block transfer plans.

### Error Handling

`SdError` implements `core::error::Error` (MSRV 1.81) and `core::fmt::Display`.
Variants:

- `Timeout` -- command or data timeout
- `CrcError` -- CRC check failed
- `NoCard` -- no card detected or card failed to initialize
- `CommandFailed(u32)` -- raw INT_STATUS error bits for diagnosis

On error, the driver resets the command and data lines (RSTC/RSTD) before
returning, leaving the peripheral in a usable state for retry.

## BSP Integration

The BSP (e.g. `teensy4-bsp`) provides a thin wrapper:

```rust
// teensy4-bsp/src/usdhc1.rs
pub fn init_usdhc1(usdhc1: ral::usdhc::USDHC1, sd_pins: SdPins) -> Result<Usdhc, SdError> {
    // Configure pins
    hal::iomuxc::usdhc::prepare(&mut sd_pins.cmd);
    // ... other pins ...

    // Delegate to the driver crate
    imxrt_usdhc::Usdhc::new(usdhc1, USDHC1_FREQUENCY)
}
```

The BSP also re-exports `embedded_sdmmc` from the driver crate so users don't
need a separate dependency for `VolumeManager`.

## Relationship to imxrt-hal

The crate is currently maintained separately. The plan is to eventually move it
into the `imxrt-rs` GitHub organization and optionally have `imxrt-hal` re-export
it (as it does for `imxrt-enet`), but this is not required for initial use.
