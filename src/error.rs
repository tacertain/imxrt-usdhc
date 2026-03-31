/// SD card type determined during initialization.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum CardType {
    /// Standard Capacity (≤ 2 GB), byte-addressed.
    Sdsc,
    /// High Capacity (> 2 GB), block-addressed.
    Sdhc,
}

/// Errors that can occur during SD card operations.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum SdError {
    /// Command or data timeout.
    Timeout,
    /// CRC check failed.
    CrcError,
    /// No card detected or card failed to initialize.
    NoCard,
    /// Command returned error status bits in R1 response.
    CommandFailed(u32),
}

impl core::fmt::Display for SdError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            SdError::Timeout => write!(f, "SD timeout"),
            SdError::CrcError => write!(f, "SD CRC error"),
            SdError::NoCard => write!(f, "No SD card"),
            SdError::CommandFailed(status) => write!(f, "SD command failed: {:#010x}", status),
        }
    }
}

impl core::error::Error for SdError {}
