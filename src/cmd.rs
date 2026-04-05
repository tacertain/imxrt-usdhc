use crate::ral;

// SD command indices
pub(crate) const CMD0_GO_IDLE_STATE: u32 = 0;
pub(crate) const CMD2_ALL_SEND_CID: u32 = 2;
pub(crate) const CMD3_SEND_RELATIVE_ADDR: u32 = 3;
pub(crate) const CMD7_SELECT_CARD: u32 = 7;
pub(crate) const CMD8_SEND_IF_COND: u32 = 8;
pub(crate) const CMD9_SEND_CSD: u32 = 9;
pub(crate) const CMD13_SEND_STATUS: u32 = 13;
pub(crate) const CMD16_SET_BLOCKLEN: u32 = 16;
pub(crate) const CMD17_READ_SINGLE_BLOCK: u32 = 17;
pub(crate) const CMD24_WRITE_SINGLE_BLOCK: u32 = 24;
pub(crate) const CMD55_APP_CMD: u32 = 55;
pub(crate) const ACMD6_SET_BUS_WIDTH: u32 = 6;
pub(crate) const ACMD41_SD_SEND_OP_COND: u32 = 41;

// OCR flags
pub(crate) const OCR_HCS: u32 = 1 << 30;
pub(crate) const OCR_BUSY: u32 = 1 << 31;

/// INT_STATUS error mask (bits 16–28).
pub(crate) const USDHC_INT_ERROR_MASK: u32 = 0x1FF_0000;

/// Number of ACMD41 retries during card initialization.
pub(crate) const USDHC_ACMD41_RETRIES: u32 = 1000;

/// Command configuration flags controlling response type, CRC, and data presence.
#[derive(Clone, Copy)]
pub(crate) enum UsdCmdFlags {
    /// No response (CMD0).
    None,
    /// 48-bit response with CRC and index check (most commands).
    R1,
    /// 48-bit response with busy signal (CMD7, CMD12, etc.).
    R1B,
    /// 136-bit response, no CRC/index check (CMD2, CMD9).
    R2,
    /// 48-bit OCR response, no CRC/index check (ACMD41).
    R3,
    /// 48-bit response with CRC and index check (CMD3).
    R6,
    /// 48-bit response with CRC and index check (CMD8).
    R7,
}

impl UsdCmdFlags {
    /// Write CMD_XFR_TYP using named fields for this response type.
    pub(crate) fn write_cmd_xfr_typ(
        self,
        usdhc: &ral::usdhc::RegisterBlock,
        cmd_index: u32,
    ) {
        match self {
            UsdCmdFlags::None => {
                ral::write_reg!(ral::usdhc, usdhc, CMD_XFR_TYP, CMDINX: cmd_index);
            }
            UsdCmdFlags::R1 | UsdCmdFlags::R6 | UsdCmdFlags::R7 => {
                ral::write_reg!(ral::usdhc, usdhc, CMD_XFR_TYP,
                    CMDINX: cmd_index, RSPTYP: 2, CCCEN: 1, CICEN: 1);
            }
            UsdCmdFlags::R1B => {
                ral::write_reg!(ral::usdhc, usdhc, CMD_XFR_TYP,
                    CMDINX: cmd_index, RSPTYP: 3, CCCEN: 1, CICEN: 1);
            }
            UsdCmdFlags::R2 => {
                ral::write_reg!(ral::usdhc, usdhc, CMD_XFR_TYP,
                    CMDINX: cmd_index, RSPTYP: 1);
            }
            UsdCmdFlags::R3 => {
                ral::write_reg!(ral::usdhc, usdhc, CMD_XFR_TYP,
                    CMDINX: cmd_index, RSPTYP: 2);
            }
        }
    }
}
