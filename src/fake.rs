//! Fake SD host for testing the SD protocol layer without real hardware.
//!
//! [`FakeSdHost`] implements [`SdHost`] with a state-machine-backed simulated
//! SD card. Tests configure the fake with a card species, drive the protocol
//! layer through real state transitions, and assert on outcomes (card type,
//! block count, bus width) — not on the exact command transcript.
//!
//! # State machine
//!
//! The fake tracks the card's phase through the SD init sequence:
//!
//! ```text
//!                   CMD0 (any state)
//!                      ↓
//!   [NoCard/Empty] → Idle → (ACMD41 busy=1) → Ready → (CMD2) →
//!   Identification → (CMD3) → Standby → (CMD7) → Transfer
//! ```

use std::cell::{Cell, RefCell};
use std::collections::{HashMap, VecDeque};

use crate::cmd::UsdCmdFlags;
use crate::error::{CardType, SdError};
use crate::host::{BusWidth, CmdResponse, SdHost};

// ---- SD version -------------------------------------------------------------------

/// SD Physical Layer Specification version.
///
/// Controls how the fake responds to CMD8 (SEND_IF_COND).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum SdVersion {
    /// Pre-v2 card. Times out on CMD8.
    V1,
    /// v2 card. Accepts CMD8 and echoes the voltage/check-pattern argument.
    V2,
}

// ---- Card configuration -----------------------------------------------------------

/// Configuration for a simulated SD card.
pub(crate) struct CardConfig {
    pub card_type: CardType,
    pub sd_version: SdVersion,
    /// Total 512-byte blocks.
    pub block_count: u32,
    /// Relative card address returned in the CMD3 response.
    pub rca: u16,
    /// Number of ACMD41 pairs to decline (return without OCR_BUSY) before
    /// reporting powered-up. 0 = ready on the very first ACMD41.
    pub acmd41_slow_count: u32,
    /// CID register value (128 bits), returned verbatim by CMD2.
    pub cid: u128,
}

impl CardConfig {
    pub(crate) fn sdhc(block_count: u32) -> Self {
        Self {
            card_type: CardType::Sdhc,
            sd_version: SdVersion::V2,
            block_count,
            rca: 0xBEEF,
            acmd41_slow_count: 0,
            cid: 0,
        }
    }

    pub(crate) fn sdsc(block_count: u32) -> Self {
        Self {
            card_type: CardType::Sdsc,
            sd_version: SdVersion::V1,
            block_count,
            rca: 0xBEEF,
            acmd41_slow_count: 0,
            cid: 0,
        }
    }
}

// ---- Card state machine -----------------------------------------------------------

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Phase {
    Idle,
    Ready,
    Identification,
    Standby,
    Transfer,
}

struct CardSim {
    config: CardConfig,
    phase: Phase,
    /// True after CMD55; the next command is dispatched as an ACMD.
    app_cmd_pending: bool,
    /// Number of ACMD41 attempts issued so far in this power cycle.
    acmd41_attempts: u32,
    /// Block storage. Absent blocks read as 0x00.
    blocks: HashMap<u32, [u8; 512]>,
}

impl CardSim {
    fn new(config: CardConfig) -> Self {
        Self {
            config,
            phase: Phase::Idle,
            app_cmd_pending: false,
            acmd41_attempts: 0,
            blocks: HashMap::new(),
        }
    }

    fn reset_to_idle(&mut self) {
        self.phase = Phase::Idle;
        self.app_cmd_pending = false;
        self.acmd41_attempts = 0;
        // blocks intentionally NOT cleared — flash is non-volatile
    }

    fn csd_response(&self) -> CmdResponse {
        let csd = match self.config.card_type {
            CardType::Sdhc => csd_v2(self.config.block_count),
            CardType::Sdsc => csd_v1(self.config.block_count),
        };
        as_r2(csd)
    }

    fn dispatch(&mut self, cmd_index: u32, arg: u32) -> Result<CmdResponse, SdError> {
        if self.app_cmd_pending {
            self.app_cmd_pending = false;
            return self.dispatch_acmd(cmd_index, arg);
        }
        self.dispatch_cmd(cmd_index, arg)
    }

    fn dispatch_cmd(&mut self, cmd_index: u32, arg: u32) -> Result<CmdResponse, SdError> {
        use crate::cmd::*;

        match cmd_index {
            CMD0_GO_IDLE_STATE => {
                self.reset_to_idle();
                Ok(empty())
            }

            CMD8_SEND_IF_COND => match (self.config.sd_version, self.phase) {
                (SdVersion::V1, _) => Err(SdError::Timeout),
                (SdVersion::V2, Phase::Idle) => Ok(rsp0(arg & 0xFFF)),
                _ => Err(wrong_state()),
            },

            CMD55_APP_CMD => match self.phase {
                Phase::Idle | Phase::Standby | Phase::Transfer => {
                    self.app_cmd_pending = true;
                    Ok(empty())
                }
                _ => Err(wrong_state()),
            },

            CMD2_ALL_SEND_CID => {
                if self.phase != Phase::Ready {
                    return Err(wrong_state());
                }
                self.phase = Phase::Identification;
                Ok(as_r2(self.config.cid))
            }

            CMD3_SEND_RELATIVE_ADDR => {
                if self.phase != Phase::Identification {
                    return Err(wrong_state());
                }
                self.phase = Phase::Standby;
                Ok(rsp0((self.config.rca as u32) << 16))
            }

            CMD9_SEND_CSD => {
                if self.phase != Phase::Standby {
                    return Err(wrong_state());
                }
                Ok(self.csd_response())
            }

            CMD7_SELECT_CARD => {
                let cmd_rca = (arg >> 16) as u16;
                if cmd_rca == self.config.rca {
                    match self.phase {
                        Phase::Standby => {
                            self.phase = Phase::Transfer;
                            Ok(empty())
                        }
                        Phase::Transfer => Ok(empty()), // re-select is a no-op
                        _ => Err(wrong_state()),
                    }
                } else {
                    // Deselect
                    if self.phase == Phase::Transfer {
                        self.phase = Phase::Standby;
                    }
                    Ok(empty())
                }
            }

            CMD13_SEND_STATUS => {
                if self.phase != Phase::Transfer {
                    return Err(wrong_state());
                }
                // CURRENT_STATE = 4 (Transfer) in R1 bits [12:9]
                Ok(rsp0(4 << 9))
            }

            CMD16_SET_BLOCKLEN => {
                if self.phase != Phase::Transfer {
                    return Err(wrong_state());
                }
                Ok(empty())
            }

            _ => Err(wrong_state()),
        }
    }

    fn dispatch_acmd(&mut self, cmd_index: u32, _arg: u32) -> Result<CmdResponse, SdError> {
        use crate::cmd::*;

        match cmd_index {
            ACMD41_SD_SEND_OP_COND => {
                if self.phase != Phase::Idle {
                    return Err(wrong_state());
                }
                let hcs = if self.config.card_type == CardType::Sdhc {
                    OCR_HCS
                } else {
                    0
                };
                if self.acmd41_attempts < self.config.acmd41_slow_count {
                    // Card still powering up — return without OCR_BUSY
                    self.acmd41_attempts += 1;
                    Ok(rsp0(hcs | 0x00FF_8000))
                } else {
                    // Powered up
                    self.phase = Phase::Ready;
                    Ok(rsp0(OCR_BUSY | hcs | 0x00FF_8000))
                }
            }

            ACMD6_SET_BUS_WIDTH => {
                if self.phase != Phase::Transfer {
                    return Err(wrong_state());
                }
                Ok(empty())
            }

            _ => Err(wrong_state()),
        }
    }
}

// ---- FakeSdHost -------------------------------------------------------------------

/// Fake SD host backed by a state-machine card simulator.
///
/// Construct with [`FakeSdHost::empty`], [`FakeSdHost::with_sdhc_card`],
/// [`FakeSdHost::with_sdsc_card`], or [`FakeSdHost::with_card`].
/// Inspect [`FakeSdHost::clock_hz`] and [`FakeSdHost::bus_width`] after
/// running protocol code to assert on host-side side effects.
pub(crate) struct FakeSdHost {
    card: RefCell<Option<CardSim>>,
    /// Last frequency passed to [`SdHost::set_clock`].
    pub clock_hz: Cell<u32>,
    /// Last width passed to [`SdHost::set_bus_width`].
    pub bus_width: Cell<BusWidth>,
    /// Queued command errors: each entry is `(cmd_index, error)`. When
    /// `send_cmd` is called, if the front entry matches the incoming
    /// `cmd_index` it is popped and returned instead of dispatching to
    /// the card state machine.
    cmd_errors: RefCell<VecDeque<(u32, SdError)>>,
}

impl FakeSdHost {
    /// Empty slot — every command returns `Err(SdError::Timeout)`.
    pub(crate) fn empty() -> Self {
        Self {
            card: RefCell::new(None),
            clock_hz: Cell::new(0),
            bus_width: Cell::new(BusWidth::One),
            cmd_errors: RefCell::new(VecDeque::new()),
        }
    }

    /// SDHC card with the given block count (`SdVersion::V2`).
    pub(crate) fn with_sdhc_card(block_count: u32) -> Self {
        Self::with_card(CardConfig::sdhc(block_count))
    }

    /// SDSC card with the given block count (`SdVersion::V1`).
    pub(crate) fn with_sdsc_card(block_count: u32) -> Self {
        Self::with_card(CardConfig::sdsc(block_count))
    }

    /// Full control over card parameters.
    pub(crate) fn with_card(config: CardConfig) -> Self {
        Self {
            card: RefCell::new(Some(CardSim::new(config))),
            clock_hz: Cell::new(0),
            bus_width: Cell::new(BusWidth::One),
            cmd_errors: RefCell::new(VecDeque::new()),
        }
    }

    /// Queue an error to be returned when `send_cmd` is called with
    /// `cmd_index`. The next call with that command index will return
    /// `Err(error)` rather than dispatching to the card state machine.
    /// Multiple errors for the same command can be queued; they fire
    /// in FIFO order.
    pub(crate) fn inject_cmd_error(&self, cmd_index: u32, error: SdError) {
        self.cmd_errors.borrow_mut().push_back((cmd_index, error));
    }

    /// Simulate a card power cycle.
    ///
    /// Resets the card's state machine to `Idle` and clears volatile state
    /// (ACMD41 counter, `app_cmd_pending`). Block storage is preserved —
    /// flash is non-volatile.
    ///
    /// Does not affect [`clock_hz`](Self::clock_hz) — the USDHC peripheral's
    /// clock setting is not affected by a card power cycle.
    pub(crate) fn power_cycle(&self) {
        self.bus_width.set(BusWidth::One);
        if let Some(sim) = self.card.borrow_mut().as_mut() {
            sim.reset_to_idle();
        }
    }
}

impl SdHost for FakeSdHost {
    fn send_cmd(
        &self,
        cmd_index: u32,
        arg: u32,
        _flags: UsdCmdFlags,
    ) -> Result<CmdResponse, SdError> {
        // Check the error injection queue before dispatching to the state machine.
        let injected = {
            let mut q = self.cmd_errors.borrow_mut();
            if q.front().map(|(c, _)| *c) == Some(cmd_index) {
                q.pop_front().map(|(_, e)| e)
            } else {
                None
            }
        };
        if let Some(err) = injected {
            return Err(err);
        }
        match self.card.borrow_mut().as_mut() {
            None => Err(SdError::Timeout),
            Some(sim) => sim.dispatch(cmd_index, arg),
        }
    }

    fn read_block(
        &self,
        _cmd_index: u32,
        arg: u32,
        buf: &mut [u8; 512],
    ) -> Result<(), SdError> {
        match self.card.borrow_mut().as_mut() {
            None => Err(SdError::Timeout),
            Some(sim) => {
                if sim.phase != Phase::Transfer {
                    return Err(wrong_state());
                }
                let key = block_key(sim.config.card_type, arg);
                match sim.blocks.get(&key) {
                    Some(data) => *buf = *data,
                    None => buf.fill(0x00),
                }
                Ok(())
            }
        }
    }

    fn write_block(
        &self,
        _cmd_index: u32,
        arg: u32,
        buf: &[u8; 512],
    ) -> Result<(), SdError> {
        match self.card.borrow_mut().as_mut() {
            None => Err(SdError::Timeout),
            Some(sim) => {
                if sim.phase != Phase::Transfer {
                    return Err(wrong_state());
                }
                let key = block_key(sim.config.card_type, arg);
                sim.blocks.insert(key, *buf);
                Ok(())
            }
        }
    }

    fn set_clock(&self, hz: u32) {
        self.clock_hz.set(hz);
    }

    fn set_bus_width(&self, width: BusWidth) {
        self.bus_width.set(width);
    }

    fn reset_and_configure(&self) {
        // No card state change — matches real hardware: a USDHC peripheral
        // reset does not power-cycle the card. The card resets only via CMD0.
        self.clock_hz.set(0);
        self.bus_width.set(BusWidth::One);
    }

    fn send_initial_clocks(&self) {
        // No-op.
    }
}

// ---- CSD generation ---------------------------------------------------------------

/// CSD v2 (SDHC/SDXC): CSD_STRUCTURE=1 at [127:126], C_SIZE at [69:48].
///
/// `block_count = (C_SIZE + 1) × 1024`
fn csd_v2(block_count: u32) -> u128 {
    let c_size = (block_count / 1024).saturating_sub(1);
    (1u128 << 126) | ((c_size as u128) << 48)
}

/// CSD v1 (SDSC): CSD_STRUCTURE=0 at [127:126], READ_BL_LEN=9 at [83:80].
///
/// Solves for the largest MULT (best granularity) that fits `block_count`
/// within the 12-bit C_SIZE limit. Rounds down to the nearest representable
/// value. Panics if `block_count` exceeds the CSD v1 maximum (~2 097 152).
fn csd_v1(block_count: u32) -> u128 {
    const READ_BL_LEN: u32 = 9;
    for csm in (0u32..=7).rev() {
        let mult = 1u32 << (csm + 2);
        if block_count >= mult {
            let c_size = block_count / mult - 1;
            if c_size <= 4095 {
                return ((READ_BL_LEN as u128) << 80)
                    | ((c_size as u128) << 62)
                    | ((csm as u128) << 47);
            }
        }
    }
    panic!(
        "block_count={block_count} cannot be encoded in CSD v1 (maximum ~2_097_152)"
    );
}

// ---- Helpers ----------------------------------------------------------------------

/// Translate the hardware `arg` back to a block index for HashMap storage.
///
/// The protocol layer applies address translation before calling the host:
/// - SDHC: `arg` = LBA              (block address)
/// - SDSC: `arg` = LBA × 512        (byte address)
fn block_key(card_type: CardType, arg: u32) -> u32 {
    match card_type {
        CardType::Sdhc => arg,
        CardType::Sdsc => arg / 512,
    }
}

fn empty() -> CmdResponse {
    CmdResponse { rsp0: 0, rsp1: 0, rsp2: 0, rsp3: 0 }
}

fn rsp0(rsp0: u32) -> CmdResponse {
    CmdResponse { rsp0, rsp1: 0, rsp2: 0, rsp3: 0 }
}

fn as_r2(bits: u128) -> CmdResponse {
    CmdResponse {
        rsp0: bits as u32,
        rsp1: (bits >> 32) as u32,
        rsp2: (bits >> 64) as u32,
        rsp3: (bits >> 96) as u32,
    }
}

fn wrong_state() -> SdError {
    SdError::CommandFailed(0x0001_0000)
}
