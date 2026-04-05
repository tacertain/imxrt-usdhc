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

### Feature Flags

- `defmt` -- derive `defmt::Format` on public types (`CardType`, `SdError`).

### Module Layout

```
src/
  lib.rs    - Usdhc (public), UsdhcInner (private), BlockDevice impl,
              peripheral config, clock computation, block address translation
  host.rs   - SdHost trait (hardware abstraction), CmdResponse, BusWidth enum;
              UsdhcInner implements SdHost with raw MMIO register access for
              send_cmd, read_block, write_block, set_clock, set_bus_width, etc.
  card.rs   - SdProtocol<H> (protocol layer), card initialization state machine
              (CMD0..CMD7), CSD parsing, bus width switching, single-block I/O
  cmd.rs    - UsdCmdFlags enum, SD command constants, OCR flags, error masks
  error.rs  - CardType enum, SdError (Display + core::error::Error)
  fake.rs   - (test-only) FakeSdHost mock with simulated card state machine,
              block storage, and error injection for host-free unit testing
```

### Layered Architecture

The driver is split into two layers connected by the `SdHost` trait:

1. **Protocol layer** (`SdProtocol<H: SdHost>` in `card.rs`) -- implements the
   SD card state machine, CSD parsing, and block I/O. This layer is generic
   over `H` and contains no hardware-specific code.

2. **Host layer** (`UsdhcInner` implementing `SdHost` in `host.rs` / `lib.rs`)
   -- performs raw MMIO register access: command submission, PIO data transfer,
   clock divisor programming, and bus width configuration.

This separation allows the protocol layer to be tested with `FakeSdHost` (a
mock host with a simulated card state machine and in-memory block storage)
without touching real hardware.

The `SdHost` trait:
```rust
pub(crate) trait SdHost {
    fn send_cmd(&self, cmd_index: u32, arg: u32, flags: UsdCmdFlags)
        -> Result<CmdResponse, SdError>;
    fn read_block(&self, cmd_index: u32, arg: u32, buf: &mut [u8; 512])
        -> Result<(), SdError>;
    fn write_block(&self, cmd_index: u32, arg: u32, buf: &[u8; 512])
        -> Result<(), SdError>;
    fn set_clock(&self, target_hz: u32);
    fn set_bus_width(&self, width: BusWidth);
    fn reset_and_configure(&self);
    fn send_initial_clocks(&self);
}
```

### Instance Generics

The constructor is generic over USDHC instance number:

```rust
pub fn new<const N: u8>(
    usdhc: ral::usdhc::Instance<N>,
    source_clock_hz: u32,
) -> Result<Self, SdError>
```

Internally, the instance number is erased by extracting the raw pointer to the
register block and constructing an `Instance<ANY_INSTANCE>` (same pattern as
`imxrt-enet`), so the `Usdhc` type doesn't carry a const generic.

### Interior Mutability

`embedded_sdmmc::BlockDevice` methods take `&self`, not `&mut self`. The driver
uses a layered approach:

- **`UsdhcInner`** (`pub(crate)`, `Clone`, `Copy`, `Send`) holds a raw pointer to
  the MMIO register block plus the `source_clock_hz`. It implements `SdHost`
  to perform raw register access. It is `Copy` so it can be cheaply duplicated.
- **`SdProtocol<H>`** (`pub(crate)`) wraps an `SdHost` implementation plus card
  metadata (card type, block count, RCA). It implements the SD protocol state
  machine and single-block I/O.
- **`Usdhc<H>`** (public, default `H = UsdhcInner`) wraps `SdProtocol<H>` and
  presents the safe public API. It is automatically `Send` because `UsdhcInner`
  is `Send`.

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

The `compute_clock_divisors()` function finds the smallest total divisor (i.e.
the highest clock frequency) that does not exceed the target. It iterates
SDCLKFS values {1, 2, 4, ..., 128} and computes the minimum DVS for each,
keeping the combination with the smallest total divisor. Falls back to
(128, 15) if no valid combination exists.

Two target frequencies are used during initialization:

- **Identification mode** (≤ 400 kHz) -- used during CMD0-CMD7.
- **Data transfer mode** (≤ 25 MHz) -- used after card initialization.

For example, with a 198 MHz source (Teensy 4.1):

| Mode | SDCLKFS | DVS | Total divisor | Result |
|---|---|---|---|---|
| Identification | 16 | 15 | 512 | 386 kHz |
| Data transfer | 1 | 3 | 8 | 24.75 MHz |

### SD Card Initialization Sequence

The `init_protocol()` method runs the full initialization:

1. `reset_and_configure()` -- software-reset the USDHC peripheral
2. `set_clock(400_000)` -- identification mode clock
3. `send_initial_clocks()` -- 80 clocks per SD spec
4. `card_init()` -- SD protocol state machine:
   1. CMD0 -- Go Idle State
   2. CMD8 -- Send Interface Condition (detect SD v2; timeout = v1 card)
   3. ACMD41 -- SD Send Op Condition (up to 1000 retries, negotiate voltage,
      detect SDHC via OCR_HCS bit)
   4. CMD2 -- All Send CID
   5. CMD3 -- Send Relative Address (extract RCA from R6)
   6. CMD9 -- Send CSD (parse capacity via `parse_csd()`)
   7. CMD7 -- Select Card (enter Transfer state)
5. `set_clock(25_000_000)` -- data transfer mode clock
6. `set_bus_width_4bit()` -- ACMD6 to switch to 4-bit bus
7. CMD16 -- SET_BLOCKLEN to 512 bytes

### Public API

| Method | Signature | Purpose |
|---|---|---|
| `new()` | `<const N: u8>(Instance<N>, u32) -> Result<Self, SdError>` | Initialize peripheral and detect card |
| `card_type()` | `&self -> CardType` | Return `Sdsc` or `Sdhc` |
| `block_count()` | `&self -> u32` | Total 512-byte blocks on the card |
| `capacity_mb()` | `&self -> u32` | `block_count / 2048` |
| `card_status()` | `&self -> Result<u32, SdError>` | CMD13 -- read R1 card status |

### BlockDevice Implementation

The `BlockDevice` impl loops over the `blocks` slice, issuing one CMD17 (read)
or CMD24 (write) per block via PIO (programmed I/O). The CPU reads/writes
128 × u32 words from the `DATA_BUFF_ACC_PORT` register in a tight loop,
blocking for the entire transfer (~160 us per block at 25 MHz).

This is correct but not optimal -- see the roadmap for multi-block transfer
and DMA plans.

### CSD Parsing

The `parse_csd()` function in `card.rs` decodes the 128-bit CSD register
(received via CMD9 R2 response, pre-normalized by the host layer):

- **CSD v1 (SDSC)**: extracts `READ_BL_LEN`, `C_SIZE`, and `C_SIZE_MULT` to
  compute capacity as `(C_SIZE + 1) * 2^(C_SIZE_MULT + 2) * 2^READ_BL_LEN`,
  then divides by 512 to get the block count.
- **CSD v2 (SDHC/SDXC)**: extracts the 22-bit `C_SIZE` field and computes
  `block_count = (C_SIZE + 1) * 1024`.

### Error Handling

`SdError` implements `core::error::Error` (MSRV 1.81) and `core::fmt::Display`.
Both `SdError` and `CardType` optionally derive `defmt::Format` via the `defmt`
feature flag.

Variants:

- `Timeout` -- command or data timeout
- `CrcError` -- CRC check failed
- `NoCard` -- no card detected or card failed to initialize
- `CommandFailed(u32)` -- raw INT_STATUS error bits for diagnosis

On error, the driver resets the command and data lines (RSTC/RSTD) before
returning, leaving the peripheral in a usable state for retry.

## Testing

The protocol layer is tested without hardware using `FakeSdHost`, a `SdHost`
implementation in `fake.rs` (compiled only under `#[cfg(test)]`). It simulates:

- A full SD card state machine (Idle → Ready → Identification → Standby →
  Transfer) with correct phase transitions for each command.
- In-memory block storage (`HashMap<u32, [u8; 512]>`) for read/write testing.
- CSD register generation for both v1 (SDSC) and v2 (SDHC) cards.
- Error injection via `inject_cmd_error()` for testing error recovery paths.
- Configurable ACMD41 retry delays for testing the power-up polling loop.

Unit tests cover:
- `compute_clock_divisors()` across multiple source/target combinations.
- `block_address()` translation for SDHC (passthrough) and SDSC (byte offset).
- `parse_csd()` for both CSD v1 and v2 with known test vectors.
- Full card initialization via `FakeSdHost` for SDHC and SDSC cards.

## BSP Integration

The BSP (e.g. `teensy4-bsp`) does **not** depend on or wrap `imxrt-usdhc`.
Instead, it provides the building blocks that the user combines:

1. **Raw peripheral** -- `board::Resources` exposes `usdhc1: ral::usdhc::USDHC1`.
2. **Clock setup** -- `board::t41()` configures the USDHC1 root clock
   (PLL2_PFD2 / 2 = 198 MHz) and enables the clock gate during board init.
3. **Frequency constant** -- `board::USDHC1_FREQUENCY` (198_000_000 Hz).
4. **Pin configuration** -- `hal::iomuxc::usdhc::prepare()` (from `imxrt-iomuxc`)
   sets alternate function, SION, and daisy registers for each SD pin.

The user adds `imxrt-usdhc` as a direct dependency and wires things together:

```rust
let board::Resources { usdhc1, mut pins, .. } = board::t41(peripherals);

hal::iomuxc::usdhc::prepare(&mut pins.p45); // CMD
hal::iomuxc::usdhc::prepare(&mut pins.p44); // CLK
hal::iomuxc::usdhc::prepare(&mut pins.p43); // DATA0
hal::iomuxc::usdhc::prepare(&mut pins.p42); // DATA1
hal::iomuxc::usdhc::prepare(&mut pins.p47); // DATA2
hal::iomuxc::usdhc::prepare(&mut pins.p46); // DATA3

let sd = imxrt_usdhc::Usdhc::new(usdhc1, board::USDHC1_FREQUENCY)?;
let volume_mgr = embedded_sdmmc::VolumeManager::new(sd, time_source);
```


