//! USDHC SD card driver for NXP's i.MX RT MCUs.
//!
//! This crate provides an SD card driver for the i.MX RT USDHC peripheral,
//! implementing [`embedded_sdmmc::BlockDevice`] for use with the
//! embedded-sdmmc FAT filesystem stack.
//!
//! # Usage
//!
//! Pin muxing and clock configuration must be done by the caller (BSP/HAL)
//! before constructing the driver. On the Teensy 4.1, use
//! `hal::iomuxc::usdhc::prepare()` on each SD pin and configure the USDHC1
//! root clock to 198 MHz before calling [`Usdhc::new`].
//!
//! ```no_run
//! # // pseudo-code — actual types come from the BSP
//! use imxrt_usdhc::{Usdhc, embedded_sdmmc};
//!
//! let driver = Usdhc::new(usdhc1_peripheral, 198_000_000).unwrap();
//! let mut volume_mgr = embedded_sdmmc::VolumeManager::new(driver, time_source);
//! ```
//!
//! # Mutual exclusion
//!
//! [`BlockDevice`](embedded_sdmmc::BlockDevice) methods take `&self`. The
//! driver does not enforce mutual exclusion internally. The caller must ensure
//! only one context accesses the driver at a time (e.g. via RTIC resource
//! locking or NVIC masking).
//!
//! # Feature flags
//!
//! - `defmt` — derive [`defmt::Format`] on public types.

#![cfg_attr(all(target_arch = "arm", target_os = "none"), no_std)]

mod card;
mod cmd;
mod error;
pub(crate) mod host;
#[cfg(test)]
mod fake;

pub use embedded_sdmmc;
pub use error::{CardType, SdError};

pub(crate) use imxrt_ral as ral;

use card::SdProtocol;
use cmd::{CMD13_SEND_STATUS, CMD16_SET_BLOCKLEN, USDHC_INT_ERROR_MASK, UsdCmdFlags};
use host::{BusWidth, CmdResponse, SdHost};

const ANY_INSTANCE: u8 = u8::MAX;
type AnyUsdhcInstance = ral::usdhc::Instance<{ ANY_INSTANCE }>;

/// Discard the compile-time instance number, keeping only the MMIO pointer.
fn into_any<const N: u8>(inst: ral::usdhc::Instance<N>) -> AnyUsdhcInstance {
    // Safety: a properly-constructed instance points to static MMIO and is
    // assumed to own that MMIO space. Assume static lifetime and take ownership.
    unsafe {
        let rb: *const ral::usdhc::RegisterBlock = &*inst;
        AnyUsdhcInstance::new(rb)
    }
}

/// Internal raw hardware handle.
///
/// Pure hardware interface — no protocol state. Protocol state lives
/// in [`SdProtocol`].
#[derive(Clone, Copy)]
struct UsdhcInner {
    base: *const ral::usdhc::RegisterBlock,
    source_clock_hz: u32,
}

// Safety: USDHC is a hardware singleton at a fixed MMIO address. The caller
// must ensure exclusive access (e.g. via RTIC resource locking or NVIC masking).
unsafe impl Send for UsdhcInner {}

/// USDHC SD card driver.
///
/// Implements [`embedded_sdmmc::BlockDevice`] for use with the embedded-sdmmc
/// FAT filesystem stack. See [crate-level docs](crate) for usage notes.
// UsdhcInner is pub(crate); the default type param is intentionally not satisfiable outside this crate.
#[allow(private_interfaces)]
pub struct Usdhc<H = UsdhcInner>(SdProtocol<H>);

impl Usdhc<UsdhcInner> {
    /// Initialize the USDHC peripheral and detect the inserted SD card.
    ///
    /// Resets the USDHC peripheral, runs the SD initialization sequence
    /// (CMD0 → CMD8 → ACMD41 → CMD2 → CMD3 → CMD9 → CMD7), then switches to
    /// 4-bit bus width and ~25 MHz clock.
    ///
    /// # Clock
    ///
    /// The `source_clock_hz` parameter is the USDHC root clock frequency
    /// in Hz, as configured by the BSP/HAL in the CCM. On the Teensy 4.1
    /// this is 198 MHz (PLL2_PFD2 / 2). The driver computes SDCLKFS/DVS
    /// divisors from this value to produce ~400 kHz for card identification
    /// and ~25 MHz for data transfer.
    ///
    /// # Panics
    ///
    /// Does not panic. Returns `Err` if no card is detected or initialization
    /// fails.
    pub fn new<const N: u8>(
        usdhc: ral::usdhc::Instance<N>,
        source_clock_hz: u32,
    ) -> Result<Self, SdError> {
        let usdhc = into_any(usdhc);
        let base = &*usdhc as *const ral::usdhc::RegisterBlock;
        let inner = UsdhcInner {
            base,
            source_clock_hz,
        };

        let proto = SdProtocol {
            host: inner,
            card_type: CardType::Sdhc,
            block_count: 0,
            rca: 0,
        };

        let driver = Self::init_protocol(proto)?;

        // Release the RAL instance — we use the raw pointer from here on.
        core::mem::forget(usdhc);

        Ok(driver)
    }
}

// SdHost is pub(crate); the bound is intentionally not satisfiable outside this crate.
#[allow(private_bounds)]
impl<H: SdHost> Usdhc<H> {
    /// Run the full SD initialization sequence on a fresh protocol instance.
    ///
    /// Shared by [`Usdhc::new`] and the `#[cfg(test)]` helper.
    pub(crate) fn init_protocol(mut proto: SdProtocol<H>) -> Result<Self, SdError> {
        proto.host.reset_and_configure();
        proto.host.set_clock(400_000);
        proto.host.send_initial_clocks();
        proto.card_init()?;
        proto.host.set_clock(25_000_000);
        proto.set_bus_width_4bit()?;
        proto.send_cmd(CMD16_SET_BLOCKLEN, 512, UsdCmdFlags::R1)?;
        Ok(Usdhc(proto))
    }

    /// Return the card type (SDSC or SDHC).
    pub fn card_type(&self) -> CardType {
        self.0.card_type
    }

    /// Return the total number of 512-byte blocks on the card.
    pub fn block_count(&self) -> u32 {
        self.0.block_count
    }

    /// Return the total capacity in megabytes.
    pub fn capacity_mb(&self) -> u32 {
        self.0.block_count / 2048
    }

    /// Query the card's current status via CMD13 (SEND_STATUS).
    pub fn card_status(&self) -> Result<u32, SdError> {
        Ok(self
            .0
            .send_cmd(CMD13_SEND_STATUS, (self.0.rca as u32) << 16, UsdCmdFlags::R1)?
            .rsp0)
    }

    /// Construct a driver from a host directly and run the full init sequence.
    #[cfg(test)]
    pub(crate) fn test_init(host: H) -> Result<Self, SdError> {
        let proto = SdProtocol { host, card_type: CardType::Sdhc, block_count: 0, rca: 0 };
        Self::init_protocol(proto)
    }

    /// Return a reference to the underlying host (for test assertions).
    #[cfg(test)]
    pub(crate) fn host(&self) -> &H {
        &self.0.host
    }

    /// Consume the driver and return the underlying host (for reuse across reinit tests).
    #[cfg(test)]
    pub(crate) fn into_host(self) -> H {
        self.0.host
    }
}

impl<H: SdHost> embedded_sdmmc::BlockDevice for Usdhc<H> {
    type Error = SdError;

    fn read(
        &self,
        blocks: &mut [embedded_sdmmc::Block],
        start_block_idx: embedded_sdmmc::BlockIdx,
    ) -> Result<(), Self::Error> {
        for (i, block) in blocks.iter_mut().enumerate() {
            self.0
                .read_single_block(start_block_idx.0 + i as u32, &mut block.contents)?;
        }
        Ok(())
    }

    fn write(
        &self,
        blocks: &[embedded_sdmmc::Block],
        start_block_idx: embedded_sdmmc::BlockIdx,
    ) -> Result<(), Self::Error> {
        for (i, block) in blocks.iter().enumerate() {
            self.0
                .write_single_block(start_block_idx.0 + i as u32, &block.contents)?;
        }
        Ok(())
    }

    fn num_blocks(&self) -> Result<embedded_sdmmc::BlockCount, Self::Error> {
        Ok(embedded_sdmmc::BlockCount(self.0.block_count))
    }
}

// ---- SdHost implementation for real hardware ----

impl SdHost for UsdhcInner {
    fn send_cmd(
        &self,
        cmd_index: u32,
        arg: u32,
        flags: UsdCmdFlags,
    ) -> Result<CmdResponse, SdError> {
        // SAFETY: self.base was initialized from a valid RAL instance; caller
        // ensures exclusive access (e.g. via RTIC resource locking).
        let usdhc = unsafe { &*self.base };

        let mut timeout = 100_000u32;
        while ral::read_reg!(ral::usdhc, usdhc, PRES_STATE, CIHB == 1) {
            timeout = timeout.checked_sub(1).ok_or(SdError::Timeout)?;
        }

        ral::write_reg!(ral::usdhc, usdhc, INT_STATUS, 0xFFFF_FFFF);
        ral::write_reg!(ral::usdhc, usdhc, CMD_ARG, arg);

        flags.write_cmd_xfr_typ(usdhc, cmd_index);

        loop {
            let status = ral::read_reg!(ral::usdhc, usdhc, INT_STATUS);
            if status & USDHC_INT_ERROR_MASK != 0 {
                ral::write_reg!(ral::usdhc, usdhc, INT_STATUS, status);
                ral::modify_reg!(ral::usdhc, usdhc, SYS_CTRL, RSTC: 1, RSTD: 1);
                while ral::read_reg!(ral::usdhc, usdhc, SYS_CTRL, RSTC == 1) {}
                while ral::read_reg!(ral::usdhc, usdhc, SYS_CTRL, RSTD == 1) {}
                if ral::read_reg!(ral::usdhc, usdhc, INT_STATUS, CTOE) != 0 {
                    return Err(SdError::Timeout);
                }
                if ral::read_reg!(ral::usdhc, usdhc, INT_STATUS, CCE) != 0
                    || ral::read_reg!(ral::usdhc, usdhc, INT_STATUS, DCE) != 0
                {
                    return Err(SdError::CrcError);
                }
                return Err(SdError::CommandFailed(status));
            }
            if ral::read_reg!(ral::usdhc, usdhc, INT_STATUS, CC == 1) {
                ral::write_reg!(ral::usdhc, usdhc, INT_STATUS, CC: 1);
                break;
            }
        }

        let rsp0 = ral::read_reg!(ral::usdhc, usdhc, CMD_RSP0);

        if matches!(flags, UsdCmdFlags::R2) {
            // R2 normalization: the hardware stores bits [127:8] of the
            // 136-bit response across RSP3..0 with an 8-bit right-shift.
            // Undo this so CSD/CID bit N maps to bit N of the combined
            // 128-bit value.
            let rsp1 = ral::read_reg!(ral::usdhc, usdhc, CMD_RSP1);
            let rsp2 = ral::read_reg!(ral::usdhc, usdhc, CMD_RSP2);
            let rsp3 = ral::read_reg!(ral::usdhc, usdhc, CMD_RSP3);
            Ok(CmdResponse {
                rsp0: rsp0 << 8,
                rsp1: (rsp1 << 8) | (rsp0 >> 24),
                rsp2: (rsp2 << 8) | (rsp1 >> 24),
                rsp3: (rsp3 << 8) | (rsp2 >> 24),
            })
        } else {
            Ok(CmdResponse {
                rsp0,
                rsp1: 0,
                rsp2: 0,
                rsp3: 0,
            })
        }
    }

    fn read_block(
        &self,
        cmd_index: u32,
        arg: u32,
        buf: &mut [u8; 512],
    ) -> Result<(), SdError> {
        // SAFETY: self.base was initialized from a valid RAL instance; caller
        // ensures exclusive access (e.g. via RTIC resource locking).
        let usdhc = unsafe { &*self.base };

        let mut timeout = 100_000u32;
        while ral::read_reg!(ral::usdhc, usdhc, PRES_STATE, CIHB == 1)
            || ral::read_reg!(ral::usdhc, usdhc, PRES_STATE, CDIHB == 1)
        {
            timeout = timeout.checked_sub(1).ok_or(SdError::Timeout)?;
        }

        ral::write_reg!(ral::usdhc, usdhc, INT_STATUS, 0xFFFF_FFFF);
        ral::write_reg!(ral::usdhc, usdhc, BLK_ATT, BLKSIZE: 512, BLKCNT: 1);
        ral::write_reg!(ral::usdhc, usdhc, MIX_CTRL,
            DTDSEL: 1, MSBSEL: 0, BCEN: 0, DMAEN: 0, AC12EN: 0);
        ral::write_reg!(ral::usdhc, usdhc, CMD_ARG, arg);
        ral::write_reg!(ral::usdhc, usdhc, CMD_XFR_TYP,
            CMDINX: cmd_index, DPSEL: 1, CICEN: 1, CCCEN: 1, RSPTYP: 2);

        // Phase 1: Command Complete
        let mut timeout = 100_000u32;
        loop {
            let status = ral::read_reg!(ral::usdhc, usdhc, INT_STATUS);
            if status & USDHC_INT_ERROR_MASK != 0 {
                ral::write_reg!(ral::usdhc, usdhc, INT_STATUS, status);
                ral::modify_reg!(ral::usdhc, usdhc, SYS_CTRL, RSTC: 1, RSTD: 1);
                while ral::read_reg!(ral::usdhc, usdhc, SYS_CTRL, RSTC == 1) {}
                while ral::read_reg!(ral::usdhc, usdhc, SYS_CTRL, RSTD == 1) {}
                if ral::read_reg!(ral::usdhc, usdhc, INT_STATUS, CTOE) != 0 {
                    return Err(SdError::Timeout);
                }
                if ral::read_reg!(ral::usdhc, usdhc, INT_STATUS, CCE) != 0 {
                    return Err(SdError::CrcError);
                }
                return Err(SdError::CommandFailed(status));
            }
            if ral::read_reg!(ral::usdhc, usdhc, INT_STATUS, CC == 1) {
                ral::write_reg!(ral::usdhc, usdhc, INT_STATUS, CC: 1);
                break;
            }
            timeout = timeout.checked_sub(1).ok_or(SdError::Timeout)?;
        }

        // Phase 2: Buffer Read Ready
        let mut timeout = 150_000_000u32;
        loop {
            let status = ral::read_reg!(ral::usdhc, usdhc, INT_STATUS);
            if status & USDHC_INT_ERROR_MASK != 0 {
                ral::write_reg!(ral::usdhc, usdhc, INT_STATUS, status);
                ral::modify_reg!(ral::usdhc, usdhc, SYS_CTRL, RSTC: 1, RSTD: 1);
                while ral::read_reg!(ral::usdhc, usdhc, SYS_CTRL, RSTC == 1) {}
                while ral::read_reg!(ral::usdhc, usdhc, SYS_CTRL, RSTD == 1) {}
                if ral::read_reg!(ral::usdhc, usdhc, INT_STATUS, DTOE) != 0 {
                    return Err(SdError::Timeout);
                }
                if ral::read_reg!(ral::usdhc, usdhc, INT_STATUS, DCE) != 0 {
                    return Err(SdError::CrcError);
                }
                return Err(SdError::CommandFailed(status));
            }
            if ral::read_reg!(ral::usdhc, usdhc, INT_STATUS, BRR == 1) {
                break;
            }
            timeout = timeout.checked_sub(1).ok_or(SdError::Timeout)?;
        }

        // SAFETY: data_port is a valid volatile MMIO register; buf_u32 points
        // into a 512-byte buffer so all 128 word offsets are in bounds;
        // write_unaligned is used because [u8; 512] has alignment 1, not 4.
        unsafe {
            let data_port = core::ptr::addr_of!(usdhc.DATA_BUFF_ACC_PORT) as *const u32;
            let buf_u32 = buf.as_mut_ptr() as *mut u32;
            for i in 0..128 {
                let word = core::ptr::read_volatile(data_port);
                core::ptr::write_unaligned(buf_u32.add(i), word);
            }
        }

        ral::write_reg!(ral::usdhc, usdhc, INT_STATUS, BRR: 1);

        // Phase 3: Transfer Complete
        let mut timeout = 10_000_000u32;
        loop {
            if ral::read_reg!(ral::usdhc, usdhc, INT_STATUS, TC == 1) {
                ral::write_reg!(ral::usdhc, usdhc, INT_STATUS, TC: 1);
                break;
            }
            let status = ral::read_reg!(ral::usdhc, usdhc, INT_STATUS);
            if status & USDHC_INT_ERROR_MASK != 0 {
                ral::write_reg!(ral::usdhc, usdhc, INT_STATUS, status);
                return Err(SdError::CommandFailed(status));
            }
            timeout = timeout.checked_sub(1).ok_or(SdError::Timeout)?;
        }

        Ok(())
    }

    fn write_block(
        &self,
        cmd_index: u32,
        arg: u32,
        buf: &[u8; 512],
    ) -> Result<(), SdError> {
        // SAFETY: self.base was initialized from a valid RAL instance; caller
        // ensures exclusive access (e.g. via RTIC resource locking).
        let usdhc = unsafe { &*self.base };

        let mut timeout = 100_000u32;
        while ral::read_reg!(ral::usdhc, usdhc, PRES_STATE, CIHB == 1)
            || ral::read_reg!(ral::usdhc, usdhc, PRES_STATE, CDIHB == 1)
        {
            timeout = timeout.checked_sub(1).ok_or(SdError::Timeout)?;
        }

        ral::write_reg!(ral::usdhc, usdhc, INT_STATUS, 0xFFFF_FFFF);
        ral::write_reg!(ral::usdhc, usdhc, BLK_ATT, BLKSIZE: 512, BLKCNT: 1);
        ral::write_reg!(ral::usdhc, usdhc, MIX_CTRL,
            DTDSEL: 0, MSBSEL: 0, BCEN: 0, DMAEN: 0, AC12EN: 0);
        ral::write_reg!(ral::usdhc, usdhc, CMD_ARG, arg);
        ral::write_reg!(ral::usdhc, usdhc, CMD_XFR_TYP,
            CMDINX: cmd_index, DPSEL: 1, CICEN: 1, CCCEN: 1, RSPTYP: 2);

        // Phase 1: Command Complete
        let mut timeout = 100_000u32;
        loop {
            let status = ral::read_reg!(ral::usdhc, usdhc, INT_STATUS);
            if status & USDHC_INT_ERROR_MASK != 0 {
                ral::write_reg!(ral::usdhc, usdhc, INT_STATUS, status);
                ral::modify_reg!(ral::usdhc, usdhc, SYS_CTRL, RSTC: 1, RSTD: 1);
                while ral::read_reg!(ral::usdhc, usdhc, SYS_CTRL, RSTC == 1) {}
                while ral::read_reg!(ral::usdhc, usdhc, SYS_CTRL, RSTD == 1) {}
                if ral::read_reg!(ral::usdhc, usdhc, INT_STATUS, CTOE) != 0 {
                    return Err(SdError::Timeout);
                }
                if ral::read_reg!(ral::usdhc, usdhc, INT_STATUS, CCE) != 0 {
                    return Err(SdError::CrcError);
                }
                return Err(SdError::CommandFailed(status));
            }
            if ral::read_reg!(ral::usdhc, usdhc, INT_STATUS, CC == 1) {
                ral::write_reg!(ral::usdhc, usdhc, INT_STATUS, CC: 1);
                break;
            }
            timeout = timeout.checked_sub(1).ok_or(SdError::Timeout)?;
        }

        // Phase 2: Buffer Write Ready
        let mut timeout = 150_000_000u32;
        loop {
            let status = ral::read_reg!(ral::usdhc, usdhc, INT_STATUS);
            if status & USDHC_INT_ERROR_MASK != 0 {
                ral::write_reg!(ral::usdhc, usdhc, INT_STATUS, status);
                ral::modify_reg!(ral::usdhc, usdhc, SYS_CTRL, RSTC: 1, RSTD: 1);
                while ral::read_reg!(ral::usdhc, usdhc, SYS_CTRL, RSTC == 1) {}
                while ral::read_reg!(ral::usdhc, usdhc, SYS_CTRL, RSTD == 1) {}
                if ral::read_reg!(ral::usdhc, usdhc, INT_STATUS, DTOE) != 0 {
                    return Err(SdError::Timeout);
                }
                if ral::read_reg!(ral::usdhc, usdhc, INT_STATUS, DCE) != 0 {
                    return Err(SdError::CrcError);
                }
                return Err(SdError::CommandFailed(status));
            }
            if ral::read_reg!(ral::usdhc, usdhc, INT_STATUS, BWR == 1) {
                break;
            }
            timeout = timeout.checked_sub(1).ok_or(SdError::Timeout)?;
        }

        // SAFETY: data_port is a valid volatile MMIO register; buf_u32 points
        // into a 512-byte buffer so all 128 word offsets are in bounds;
        // read_unaligned is used because [u8; 512] has alignment 1, not 4.
        unsafe {
            let data_port = core::ptr::addr_of!(usdhc.DATA_BUFF_ACC_PORT) as *mut u32;
            let buf_u32 = buf.as_ptr() as *const u32;
            for i in 0..128 {
                let word = core::ptr::read_unaligned(buf_u32.add(i));
                core::ptr::write_volatile(data_port, word);
            }
        }

        ral::write_reg!(ral::usdhc, usdhc, INT_STATUS, BWR: 1);

        // Phase 3: Transfer Complete
        let mut timeout = 150_000_000u32;
        loop {
            if ral::read_reg!(ral::usdhc, usdhc, INT_STATUS, TC == 1) {
                ral::write_reg!(ral::usdhc, usdhc, INT_STATUS, TC: 1);
                break;
            }
            let status = ral::read_reg!(ral::usdhc, usdhc, INT_STATUS);
            if status & USDHC_INT_ERROR_MASK != 0 {
                ral::write_reg!(ral::usdhc, usdhc, INT_STATUS, status);
                return Err(SdError::CommandFailed(status));
            }
            timeout = timeout.checked_sub(1).ok_or(SdError::Timeout)?;
        }

        Ok(())
    }

    fn set_clock(&self, target_hz: u32) {
        let (best_sdclkfs, best_dvs) =
            compute_clock_divisors(self.source_clock_hz, target_hz);
        // SAFETY: self.base was initialized from a valid RAL instance; caller
        // ensures exclusive access (e.g. via RTIC resource locking).
        let usdhc = unsafe { &*self.base };
        // Gate clock during divisor change for clean transition
        ral::modify_reg!(ral::usdhc, usdhc, VEND_SPEC, FRC_SDCLK_ON: 0);
        ral::modify_reg!(ral::usdhc, usdhc, SYS_CTRL,
            SDCLKFS: best_sdclkfs, DVS: best_dvs);
        while ral::read_reg!(ral::usdhc, usdhc, PRES_STATE, SDSTB == 0) {}
        ral::modify_reg!(ral::usdhc, usdhc, VEND_SPEC, FRC_SDCLK_ON: 1);
    }

    fn set_bus_width(&self, width: BusWidth) {
        // SAFETY: self.base was initialized from a valid RAL instance; caller
        // ensures exclusive access (e.g. via RTIC resource locking).
        let usdhc = unsafe { &*self.base };
        let dtw = match width {
            BusWidth::One => 0,
            BusWidth::Four => 1,
        };
        ral::modify_reg!(ral::usdhc, usdhc, PROT_CTRL, DTW: dtw);
    }

    fn reset_and_configure(&self) {
        // SAFETY: self.base was initialized from a valid RAL instance; caller
        // ensures exclusive access (e.g. via RTIC resource locking).
        let usdhc = unsafe { &*self.base };

        // Software reset
        ral::modify_reg!(ral::usdhc, usdhc, SYS_CTRL, RSTA: 1);
        while ral::read_reg!(ral::usdhc, usdhc, SYS_CTRL, RSTA == 1) {}

        // Data timeout
        ral::modify_reg!(ral::usdhc, usdhc, SYS_CTRL, DTOCV: 0xE);

        // 1-bit mode, little-endian
        ral::write_reg!(ral::usdhc, usdhc, PROT_CTRL, DTW: 0, EMODE: 2);

        // Watermark: 128 words (512 bytes = full block)
        ral::write_reg!(ral::usdhc, usdhc, WTMK_LVL, RD_WML: 128, WR_WML: 128);

        // Enable status flags for polling
        ral::write_reg!(
            ral::usdhc, usdhc, INT_STATUS_EN,
            CCSEN: 1, TCSEN: 1, BWRSEN: 1, BRRSEN: 1,
            CTOESEN: 1, CCESEN: 1, CEBESEN: 1, CIESEN: 1,
            DTOESEN: 1, DCESEN: 1, DEBESEN: 1
        );

        // Polling mode — no interrupt signals
        ral::write_reg!(ral::usdhc, usdhc, INT_SIGNAL_EN, 0);

        // Force clock active for card identification
        ral::modify_reg!(ral::usdhc, usdhc, VEND_SPEC, FRC_SDCLK_ON: 1);
    }

    fn send_initial_clocks(&self) {
        // SAFETY: self.base was initialized from a valid RAL instance; caller
        // ensures exclusive access (e.g. via RTIC resource locking).
        let usdhc = unsafe { &*self.base };
        ral::modify_reg!(ral::usdhc, usdhc, SYS_CTRL, INITA: 1);
        while ral::read_reg!(ral::usdhc, usdhc, SYS_CTRL, INITA == 1) {}
    }
}

// ---- Pure functions (hardware-free) ----

/// Compute SDCLKFS and DVS divisors so that the SD bus clock is at most
/// `target_hz`.
///
/// The SD bus clock = `source_hz` / (2 × SDCLKFS × (DVS + 1)).
/// SDCLKFS is a power of 2 in 1..=128; DVS is in 0..=15.
///
/// The algorithm finds the smallest total divisor (i.e. the clock closest
/// to `target_hz` without exceeding it).  When no valid combination can
/// reach `target_hz` the fall-through defaults (SDCLKFS=128, DVS=15) are
/// returned, giving the slowest possible clock.
pub(crate) fn compute_clock_divisors(source_hz: u32, target_hz: u32) -> (u32, u32) {
    let mut best_sdclkfs = 128u32;
    let mut best_dvs = 15u32;

    for shift in 0..=7u32 {
        let sdclkfs = 1u32 << shift; // 1, 2, 4, 8, 16, 32, 64, 128
        let prescaled = source_hz / (2 * sdclkfs);
        let dvs_plus_1 = (prescaled + target_hz - 1) / target_hz;
        if dvs_plus_1 == 0 {
            continue;
        }
        let dvs = dvs_plus_1 - 1;
        if dvs > 15 {
            continue;
        }
        let total = 2 * sdclkfs * (dvs + 1);
        let best_total = 2 * best_sdclkfs * (best_dvs + 1);
        if total < best_total {
            best_sdclkfs = sdclkfs;
            best_dvs = dvs;
        }
    }

    (best_sdclkfs, best_dvs)
}

/// Translate a logical block address to the hardware command argument.
///
/// SDHC/SDXC cards are block-addressed (argument = LBA).
/// SDSC cards are byte-addressed (argument = LBA × 512).
/// Returns `Err(SdError::Timeout)` on overflow for SDSC cards.
pub(crate) fn block_address(card_type: CardType, lba: u32) -> Result<u32, SdError> {
    match card_type {
        CardType::Sdhc => Ok(lba),
        CardType::Sdsc => lba.checked_mul(512).ok_or(SdError::Timeout),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // ---- compute_clock_divisors ----

    fn actual_clock(source_hz: u32, sdclkfs: u32, dvs: u32) -> u32 {
        source_hz / (2 * sdclkfs * (dvs + 1))
    }

    #[test]
    fn clock_teensy_slow() {
        let (sdclkfs, dvs) = compute_clock_divisors(198_000_000, 400_000);
        assert!(
            actual_clock(198_000_000, sdclkfs, dvs) <= 400_000,
            "clock exceeds 400 kHz"
        );
        // First valid combination found: sdclkfs=16, dvs=15, total divisor=512
        assert_eq!((sdclkfs, dvs), (16, 15));
        assert_eq!(actual_clock(198_000_000, sdclkfs, dvs), 386_718);
    }

    #[test]
    fn clock_teensy_fast() {
        let (sdclkfs, dvs) = compute_clock_divisors(198_000_000, 25_000_000);
        assert!(
            actual_clock(198_000_000, sdclkfs, dvs) <= 25_000_000,
            "clock exceeds 25 MHz"
        );
        // sdclkfs=1, dvs=3: 198 MHz / (2*1*4) = 24.75 MHz
        assert_eq!((sdclkfs, dvs), (1, 3));
        assert_eq!(actual_clock(198_000_000, sdclkfs, dvs), 24_750_000);
    }

    #[test]
    fn clock_exact_divider() {
        let (sdclkfs, dvs) = compute_clock_divisors(50_000_000, 25_000_000);
        // sdclkfs=1, dvs=0: 50 MHz / 2 = 25 MHz exactly
        assert_eq!((sdclkfs, dvs), (1, 0));
        assert_eq!(actual_clock(50_000_000, sdclkfs, dvs), 25_000_000);
    }

    #[test]
    fn clock_evk_slow() {
        let (sdclkfs, dvs) = compute_clock_divisors(400_000_000, 400_000);
        assert!(
            actual_clock(400_000_000, sdclkfs, dvs) <= 400_000,
            "clock exceeds 400 kHz"
        );
        // First valid: sdclkfs=32, dvs=15, total=1024, clock=390625
        assert_eq!((sdclkfs, dvs), (32, 15));
        assert_eq!(actual_clock(400_000_000, sdclkfs, dvs), 390_625);
    }

    #[test]
    fn clock_no_valid_combination() {
        // target=1 Hz: no SDCLKFS/DVS combination can reach it;
        // fall-through defaults are returned.
        let (sdclkfs, dvs) = compute_clock_divisors(198_000_000, 1);
        assert_eq!((sdclkfs, dvs), (128, 15));
    }

    // ---- block_address ----

    #[test]
    fn block_addr_sdhc_passthrough() {
        assert_eq!(block_address(CardType::Sdhc, 0), Ok(0));
        assert_eq!(block_address(CardType::Sdhc, 1000), Ok(1000));
        assert_eq!(block_address(CardType::Sdhc, u32::MAX), Ok(u32::MAX));
    }

    #[test]
    fn block_addr_sdsc_byte_offset() {
        assert_eq!(block_address(CardType::Sdsc, 0), Ok(0));
        assert_eq!(block_address(CardType::Sdsc, 1000), Ok(512_000));
    }

    #[test]
    fn block_addr_sdsc_overflow() {
        // 0x0080_0000 × 512 = 0x1_0000_0000, which overflows u32
        assert_eq!(
            block_address(CardType::Sdsc, 0x0080_0000),
            Err(SdError::Timeout)
        );
    }

    // ---- end-to-end tests via Usdhc<FakeSdHost> ------------------------------------

    use crate::fake::FakeSdHost;
    use crate::host::BusWidth;
    use embedded_sdmmc::{Block, BlockDevice, BlockIdx};

    fn sdhc_driver() -> Usdhc<FakeSdHost> {
        Usdhc::test_init(FakeSdHost::with_sdhc_card(8_388_608)).unwrap()
    }

    #[test]
    fn e2e_sdhc_init_and_properties() {
        let driver = sdhc_driver();
        assert_eq!(driver.card_type(), CardType::Sdhc);
        assert_eq!(driver.block_count(), 8_388_608);
        assert_eq!(driver.capacity_mb(), 4096);
        // Full init sequence: clock should have been raised to 25 MHz, bus switched to 4-bit.
        assert_eq!(driver.host().clock_hz.get(), 25_000_000);
        assert_eq!(driver.host().bus_width.get(), BusWidth::Four);
    }

    #[test]
    fn e2e_sdsc_init_and_properties() {
        let driver = Usdhc::test_init(FakeSdHost::with_sdsc_card(262_144)).unwrap();
        assert_eq!(driver.card_type(), CardType::Sdsc);
        assert_eq!(driver.block_count(), 262_144);
        assert_eq!(driver.host().clock_hz.get(), 25_000_000);
        assert_eq!(driver.host().bus_width.get(), BusWidth::Four);
    }

    #[test]
    fn e2e_no_card_returns_error() {
        assert!(Usdhc::test_init(FakeSdHost::empty()).is_err());
    }

    #[test]
    fn e2e_card_status() {
        let driver = sdhc_driver();
        // Fake returns CURRENT_STATE=4 (Transfer) in R1 bits [12:9]
        assert_eq!(driver.card_status().unwrap(), 4 << 9);
    }

    #[test]
    fn e2e_num_blocks() {
        let driver = sdhc_driver();
        assert_eq!(driver.num_blocks().unwrap().0, 8_388_608);
    }

    #[test]
    fn e2e_block_device_write_read_single() {
        let driver = sdhc_driver();
        let mut block = Block::new();
        block.contents.fill(0xAB);
        driver.write(&[block.clone()], BlockIdx(100)).unwrap();

        let mut read_blocks = [Block::new()];
        driver.read(&mut read_blocks, BlockIdx(100)).unwrap();
        assert_eq!(read_blocks[0].contents, block.contents);
    }

    #[test]
    fn e2e_block_device_write_read_multi() {
        let driver = sdhc_driver();
        let mut blocks: [Block; 4] = core::array::from_fn(|_| Block::new());
        for (i, b) in blocks.iter_mut().enumerate() {
            b.contents.fill(0x10 + i as u8);
        }
        driver.write(&blocks, BlockIdx(200)).unwrap();

        let mut read_blocks: [Block; 4] = core::array::from_fn(|_| Block::new());
        driver.read(&mut read_blocks, BlockIdx(200)).unwrap();
        for (i, (written, read)) in blocks.iter().zip(read_blocks.iter()).enumerate() {
            assert_eq!(read.contents, written.contents, "block {i} mismatch");
        }
    }

    #[test]
    fn e2e_unwritten_block_reads_zeros() {
        let driver = sdhc_driver();
        let mut blocks = [Block::new()];
        blocks[0].contents.fill(0xFF);
        driver.read(&mut blocks, BlockIdx(0)).unwrap();
        assert_eq!(blocks[0].contents, [0u8; 512]);
    }

    #[test]
    fn e2e_sdsc_write_read_via_block_device() {
        // Verify that byte-address translation (LBA × 512) goes through
        // correctly when using the BlockDevice trait on an SDSC card.
        let driver = Usdhc::test_init(FakeSdHost::with_sdsc_card(262_144)).unwrap();
        let mut block = Block::new();
        block.contents.fill(0xCD);
        driver.write(&[block.clone()], BlockIdx(42)).unwrap();

        let mut read_blocks = [Block::new()];
        driver.read(&mut read_blocks, BlockIdx(42)).unwrap();
        assert_eq!(read_blocks[0].contents, block.contents);
    }

    #[test]
    fn e2e_power_cycle_without_reinit_returns_error() {
        // After a card power cycle the card state machine drops back to Idle.
        // Any I/O attempt — read, write, or card_status — must fail because
        // the driver has not re-run the init sequence.
        let driver = sdhc_driver();

        driver.host().power_cycle();

        assert!(
            driver.read(&mut [Block::new()], BlockIdx(0)).is_err(),
            "read after power cycle should return an error"
        );
        assert!(
            driver.write(&[Block::new()], BlockIdx(0)).is_err(),
            "write after power cycle should return an error"
        );
        assert!(
            driver.card_status().is_err(),
            "card_status after power cycle should return an error"
        );
    }

    #[test]
    fn e2e_power_cycle_reinit_retains_data() {
        // Write blocks, power cycle, reinitialize with the same host (same
        // underlying flash storage), and confirm the data survives.
        let driver = sdhc_driver();

        let mut block_a = Block::new();
        let mut block_b = Block::new();
        block_a.contents.fill(0x11);
        block_b.contents.fill(0x22);
        driver.write(&[block_a.clone()], BlockIdx(10)).unwrap();
        driver.write(&[block_b.clone()], BlockIdx(11)).unwrap();

        // Verify data is readable before the power cycle.
        let mut readback: [Block; 2] = core::array::from_fn(|_| Block::new());
        driver.read(&mut readback, BlockIdx(10)).unwrap();
        assert_eq!(readback[0].contents, block_a.contents);
        assert_eq!(readback[1].contents, block_b.contents);

        // Simulate card power cycle: card resets to Idle, block storage preserved.
        let host = driver.into_host();
        host.power_cycle();

        // Reinitialize the driver with the same host.
        let driver2 = Usdhc::test_init(host).unwrap();

        // Data must still be readable after reinit.
        let mut readback2: [Block; 2] = core::array::from_fn(|_| Block::new());
        driver2.read(&mut readback2, BlockIdx(10)).unwrap();
        assert_eq!(readback2[0].contents, block_a.contents);
        assert_eq!(readback2[1].contents, block_b.contents);
    }
}
