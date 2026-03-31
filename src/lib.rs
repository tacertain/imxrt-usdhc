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

pub use embedded_sdmmc;
pub use error::{CardType, SdError};

pub(crate) use imxrt_ral as ral;

use cmd::{
    CMD13_SEND_STATUS, CMD16_SET_BLOCKLEN, CMD17_READ_SINGLE_BLOCK, CMD24_WRITE_SINGLE_BLOCK,
    CMD55_APP_CMD, USDHC_INT_ERROR_MASK,
};
use cmd::UsdCmdFlags;

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
/// `Copy` so it can be duplicated across contexts (e.g. USB MSC and
/// VolumeManager) without moves, with mutual exclusion left to the caller.
#[derive(Clone, Copy)]
struct UsdhcInner {
    base: *const ral::usdhc::RegisterBlock,
    source_clock_hz: u32,
    card_type: CardType,
    block_count: u32,
    rca: u16,
}

// Safety: USDHC is a hardware singleton at a fixed MMIO address. The caller
// must ensure exclusive access (e.g. via RTIC resource locking or NVIC masking).
unsafe impl Send for UsdhcInner {}

/// USDHC SD card driver.
///
/// Implements [`embedded_sdmmc::BlockDevice`] for use with the embedded-sdmmc
/// FAT filesystem stack. See [crate-level docs](crate) for usage notes.
pub struct Usdhc(UsdhcInner);

impl Usdhc {
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
        let mut inner = UsdhcInner {
            base,
            source_clock_hz,
            card_type: CardType::Sdhc,
            block_count: 0,
            rca: 0,
        };

        inner.software_reset();
        inner.configure_peripheral();
        inner.set_clock_slow();
        inner.send_initial_clocks();
        inner.card_init()?;
        inner.set_clock_fast();
        inner.set_bus_width_4bit()?;
        inner.send_cmd(CMD16_SET_BLOCKLEN, 512, UsdCmdFlags::R1)?;

        // Release the RAL instance — we use the raw pointer from here on.
        core::mem::forget(usdhc);

        Ok(Usdhc(inner))
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
        self.0
            .send_cmd(CMD13_SEND_STATUS, (self.0.rca as u32) << 16, UsdCmdFlags::R1)
    }
}

impl embedded_sdmmc::BlockDevice for Usdhc {
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

// ---- UsdhcInner implementation ----

impl UsdhcInner {
    fn software_reset(&self) {
        unsafe {
            let usdhc = &*self.base;
            ral::modify_reg!(ral::usdhc, usdhc, SYS_CTRL, RSTA: 1);
            while ral::read_reg!(ral::usdhc, usdhc, SYS_CTRL, RSTA == 1) {}
        }
    }

    fn configure_peripheral(&self) {
        unsafe {
            let usdhc = &*self.base;

            ral::modify_reg!(ral::usdhc, usdhc, SYS_CTRL, DTOCV: 0xE);

            ral::write_reg!(ral::usdhc, usdhc, PROT_CTRL, DTW: 0, EMODE: 2);

            // Watermark: 128 words (512 bytes = full block)
            ral::write_reg!(ral::usdhc, usdhc, WTMK_LVL, RD_WML: 128, WR_WML: 128);

            ral::write_reg!(
                ral::usdhc, usdhc, INT_STATUS_EN,
                CCSEN: 1, TCSEN: 1, BWRSEN: 1, BRRSEN: 1,
                CTOESEN: 1, CCESEN: 1, CEBESEN: 1, CIESEN: 1,
                DTOESEN: 1, DCESEN: 1, DEBESEN: 1
            );

            // Polling mode — no interrupt signals
            ral::write_reg!(ral::usdhc, usdhc, INT_SIGNAL_EN, 0);

            ral::modify_reg!(ral::usdhc, usdhc, VEND_SPEC, FRC_SDCLK_ON: 1);
        }
    }

    /// Set the SD clock to at most `target_hz`.
    ///
    /// The SD bus clock = source_clock_hz / (2 * SDCLKFS * (DVS + 1)).
    /// SDCLKFS must be a power of 2 in 1..=128. DVS is 0..=15.
    fn set_clock(&self, target_hz: u32) {
        // Find the smallest total divisor that brings the clock at or below target.
        // Try each SDCLKFS (powers of 2), pick the smallest DVS that works.
        let mut best_sdclkfs = 128u32;
        let mut best_dvs = 15u32;

        for shift in 0..=7u32 {
            let sdclkfs = 1u32 << shift; // 1, 2, 4, 8, 16, 32, 64, 128
            let prescaled = self.source_clock_hz / (2 * sdclkfs);
            // We need prescaled / (dvs + 1) <= target_hz
            // So dvs + 1 >= prescaled / target_hz
            // dvs >= (prescaled / target_hz) - 1, but use ceiling division
            let dvs_plus_1 = (prescaled + target_hz - 1) / target_hz;
            if dvs_plus_1 == 0 {
                continue;
            }
            let dvs = dvs_plus_1 - 1;
            if dvs > 15 {
                continue;
            }
            // This is a valid combination. Pick the one with the smallest
            // total divisor (closest to target without exceeding it).
            let total = 2 * sdclkfs * (dvs + 1);
            let best_total = 2 * best_sdclkfs * (best_dvs + 1);
            if total < best_total {
                best_sdclkfs = sdclkfs;
                best_dvs = dvs;
            }
        }

        unsafe {
            let usdhc = &*self.base;
            ral::modify_reg!(ral::usdhc, usdhc, SYS_CTRL,
                SDCLKFS: best_sdclkfs, DVS: best_dvs);
            while ral::read_reg!(ral::usdhc, usdhc, PRES_STATE, SDSTB == 0) {}
        }
    }

    fn set_clock_slow(&self) {
        // SD spec: identification clock must be ≤ 400 kHz
        self.set_clock(400_000);
    }

    fn set_clock_fast(&self) {
        // Default Speed mode: ≤ 25 MHz
        unsafe {
            let usdhc = &*self.base;
            ral::modify_reg!(ral::usdhc, usdhc, VEND_SPEC, FRC_SDCLK_ON: 0);
        }
        self.set_clock(25_000_000);
        unsafe {
            let usdhc = &*self.base;
            ral::modify_reg!(ral::usdhc, usdhc, VEND_SPEC, FRC_SDCLK_ON: 1);
        }
    }

    fn send_initial_clocks(&self) {
        unsafe {
            let usdhc = &*self.base;
            ral::modify_reg!(ral::usdhc, usdhc, SYS_CTRL, INITA: 1);
            while ral::read_reg!(ral::usdhc, usdhc, SYS_CTRL, INITA == 1) {}
        }
    }

    fn send_cmd(&self, cmd_index: u32, arg: u32, flags: UsdCmdFlags) -> Result<u32, SdError> {
        unsafe {
            let usdhc = &*self.base;

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

            Ok(ral::read_reg!(ral::usdhc, usdhc, CMD_RSP0))
        }
    }

    fn send_acmd(&self, cmd_index: u32, arg: u32, flags: UsdCmdFlags) -> Result<u32, SdError> {
        self.send_cmd(CMD55_APP_CMD, (self.rca as u32) << 16, UsdCmdFlags::R1)?;
        self.send_cmd(cmd_index, arg, flags)
    }

    fn read_single_block(&self, lba: u32, buf: &mut [u8; 512]) -> Result<(), SdError> {
        let addr = match self.card_type {
            CardType::Sdhc => lba,
            CardType::Sdsc => lba.checked_mul(512).ok_or(SdError::Timeout)?,
        };

        unsafe {
            let usdhc = &*self.base;

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
            ral::write_reg!(ral::usdhc, usdhc, CMD_ARG, addr);
            ral::write_reg!(ral::usdhc, usdhc, CMD_XFR_TYP,
                CMDINX: CMD17_READ_SINGLE_BLOCK, DPSEL: 1, CICEN: 1, CCCEN: 1, RSPTYP: 2);

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

            // Read 512 bytes via 128 × 32-bit reads from DATA_BUFF_ACC_PORT
            let data_port = core::ptr::addr_of!(usdhc.DATA_BUFF_ACC_PORT) as *const u32;
            let buf_u32 = buf.as_mut_ptr() as *mut u32;
            for i in 0..128 {
                let word = core::ptr::read_volatile(data_port);
                core::ptr::write_unaligned(buf_u32.add(i), word);
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
        }

        Ok(())
    }

    fn write_single_block(&self, lba: u32, buf: &[u8; 512]) -> Result<(), SdError> {
        let addr = match self.card_type {
            CardType::Sdhc => lba,
            CardType::Sdsc => lba.checked_mul(512).ok_or(SdError::Timeout)?,
        };

        unsafe {
            let usdhc = &*self.base;

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
            ral::write_reg!(ral::usdhc, usdhc, CMD_ARG, addr);
            ral::write_reg!(ral::usdhc, usdhc, CMD_XFR_TYP,
                CMDINX: CMD24_WRITE_SINGLE_BLOCK, DPSEL: 1, CICEN: 1, CCCEN: 1, RSPTYP: 2);

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

            // Write 512 bytes via 128 × 32-bit writes to DATA_BUFF_ACC_PORT
            let data_port = core::ptr::addr_of!(usdhc.DATA_BUFF_ACC_PORT) as *mut u32;
            let buf_u32 = buf.as_ptr() as *const u32;
            for i in 0..128 {
                let word = core::ptr::read_unaligned(buf_u32.add(i));
                core::ptr::write_volatile(data_port, word);
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
        }

        Ok(())
    }
}
