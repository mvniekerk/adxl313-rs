//! AXL313 register addresses
#![allow(non_camel_case_types)]

#[allow(dead_code)]
#[repr(u8)]
pub enum Register {
    DEVID_0         = 0x00,
    DEVID_1         = 0x01,
    PARTID          = 0x02,
    REVID           = 0x03,
    XID             = 0x04,
    SOFT_RESET      = 0x18,
    OFSX            = 0x1E,
    OFSY            = 0x1F,
    OFSZ            = 0x20,
    THRESH_ACT      = 0x24,
    THRESH_INACT    = 0x25,
    TIME_INACT      = 0x26,
    ACT_INACT_CTL   = 0x27,
    BW_RATE         = 0x2C,
    POWER_CTL       = 0x2D,
    INT_ENABLE      = 0x2E,
    INT_MAP         = 0x2F,
    INT_SOURCE      = 0x30,
    DATA_FORMAT     = 0x31,
    DATA_X0         = 0x32,
    DATA_X1         = 0x33,
    DATA_Y0         = 0x34,
    DATA_Y1         = 0x35,
    DATA_Z0         = 0x36,
    DATA_Z1         = 0x37,
    FIFO_CTL        = 0x38,
    FIFO_STATUS     = 0x39
}

impl Register {
    /// Get register address
    pub fn addr(self) -> u8 {
        self as u8
    }
}