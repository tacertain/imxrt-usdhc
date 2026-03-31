//! Card initialization state machine and CSD register parsing.

use crate::cmd::{
    UsdCmdFlags, ACMD6_SET_BUS_WIDTH, ACMD41_SD_SEND_OP_COND, CMD0_GO_IDLE_STATE,
    CMD2_ALL_SEND_CID, CMD3_SEND_RELATIVE_ADDR, CMD7_SELECT_CARD, CMD8_SEND_IF_COND,
    CMD9_SEND_CSD, OCR_BUSY, OCR_HCS, USDHC_ACMD41_RETRIES, USDHC_INT_ERROR_MASK,
};
use crate::error::{CardType, SdError};
use crate::ral;
use crate::UsdhcInner;

impl UsdhcInner {
    /// Run the SD card initialization sequence.
    ///
    /// CMD0 → CMD8 → ACMD41 → CMD2 → CMD3 → CMD9 → CMD7
    pub(crate) fn card_init(&mut self) -> Result<(), SdError> {
        self.send_cmd(CMD0_GO_IDLE_STATE, 0, UsdCmdFlags::None)?;

        let r7 = self.send_cmd(CMD8_SEND_IF_COND, 0x1AA, UsdCmdFlags::R7);
        let is_v2 = match r7 {
            Ok(resp) => {
                if resp & 0x1FF != 0x1AA {
                    return Err(SdError::NoCard);
                }
                true
            }
            Err(SdError::Timeout) => false,
            Err(e) => return Err(e),
        };

        let hcs = if is_v2 { OCR_HCS } else { 0 };
        let mut ocr = 0u32;
        for _ in 0..USDHC_ACMD41_RETRIES {
            let resp =
                self.send_acmd(ACMD41_SD_SEND_OP_COND, hcs | 0x00FF_8000, UsdCmdFlags::R3)?;
            if resp & OCR_BUSY != 0 {
                ocr = resp;
                break;
            }
            cortex_m::asm::delay(600_000); // ~1ms at 600 MHz
        }
        if ocr & OCR_BUSY == 0 {
            return Err(SdError::NoCard);
        }

        self.card_type = if ocr & OCR_HCS != 0 {
            CardType::Sdhc
        } else {
            CardType::Sdsc
        };

        self.send_cmd(CMD2_ALL_SEND_CID, 0, UsdCmdFlags::R2)?;

        let r6 = self.send_cmd(CMD3_SEND_RELATIVE_ADDR, 0, UsdCmdFlags::R6)?;
        self.rca = (r6 >> 16) as u16;

        self.read_csd()?;

        self.send_cmd(CMD7_SELECT_CARD, (self.rca as u32) << 16, UsdCmdFlags::R1B)?;

        Ok(())
    }

    /// Read the CSD register (CMD9) and compute block count.
    pub(crate) fn read_csd(&mut self) -> Result<(), SdError> {
        unsafe {
            let usdhc = &*self.base;

            let mut timeout = 100_000u32;
            while ral::read_reg!(ral::usdhc, usdhc, PRES_STATE, CIHB == 1) {
                timeout = timeout.checked_sub(1).ok_or(SdError::Timeout)?;
            }

            ral::write_reg!(ral::usdhc, usdhc, INT_STATUS, 0xFFFF_FFFF);
            ral::write_reg!(ral::usdhc, usdhc, CMD_ARG, (self.rca as u32) << 16);

            // CMD9 uses R2 (136-bit), sent directly to avoid double CIHB wait
            ral::write_reg!(ral::usdhc, usdhc, CMD_XFR_TYP,
                CMDINX: CMD9_SEND_CSD, RSPTYP: 1);

            loop {
                let status = ral::read_reg!(ral::usdhc, usdhc, INT_STATUS);
                if status & USDHC_INT_ERROR_MASK != 0 {
                    ral::write_reg!(ral::usdhc, usdhc, INT_STATUS, status);
                    ral::modify_reg!(ral::usdhc, usdhc, SYS_CTRL, RSTC: 1);
                    while ral::read_reg!(ral::usdhc, usdhc, SYS_CTRL, RSTC == 1) {}
                    return Err(SdError::CommandFailed(status));
                }
                if ral::read_reg!(ral::usdhc, usdhc, INT_STATUS, CC == 1) {
                    ral::write_reg!(ral::usdhc, usdhc, INT_STATUS, CC: 1);
                    break;
                }
            }

            let _rsp0 = ral::read_reg!(ral::usdhc, usdhc, CMD_RSP0);
            let rsp1 = ral::read_reg!(ral::usdhc, usdhc, CMD_RSP1);
            let rsp2 = ral::read_reg!(ral::usdhc, usdhc, CMD_RSP2);
            let rsp3 = ral::read_reg!(ral::usdhc, usdhc, CMD_RSP3);

            let csd_structure = (rsp3 >> 22) & 0x3;
            self.block_count = match csd_structure {
                // CSD v1 (SDSC)
                0 => {
                    let read_bl_len = (rsp2 >> 8) & 0xF;
                    let c_size = ((rsp2 & 0x3) << 10) | ((rsp1 >> 22) & 0x3FF);
                    let c_size_mult = (rsp1 >> 7) & 0x7;
                    let block_len = 1u32 << read_bl_len;
                    let mult = 1u32 << (c_size_mult + 2);
                    let capacity_bytes =
                        (c_size as u64 + 1) * mult as u64 * block_len as u64;
                    (capacity_bytes / 512) as u32
                }
                // CSD v2 (SDHC/SDXC)
                1 => {
                    let c_size = (rsp1 >> 8) & 0x3F_FFFF;
                    (c_size + 1) * 1024
                }
                _ => 0,
            };
        }
        Ok(())
    }

    /// Switch to 4-bit bus width via ACMD6.
    pub(crate) fn set_bus_width_4bit(&self) -> Result<(), SdError> {
        self.send_acmd(ACMD6_SET_BUS_WIDTH, 2, UsdCmdFlags::R1)?;
        unsafe {
            let usdhc = &*self.base;
            ral::modify_reg!(ral::usdhc, usdhc, PROT_CTRL, DTW: 1);
        }
        Ok(())
    }
}
