#![allow(non_camel_case_types)]

#[derive(Copy, Clone, Debug)]
pub enum Range {
    _0d5G = 0b00,
    _1d0G = 0b01,
    _2d0G = 0b10,
    _4d0G = 0b11,
}

impl Range {
    pub fn val(self) -> u8 {
        self as u8
    }
}

impl From<Range> for f32 {
    fn from(range: Range) -> f32 {
        match range {
            Range::_0d5G => 0.512,
            Range::_1d0G => 1.024,
            Range::_2d0G => 2.048,
            Range::_4d0G => 4.096,
        }
    }
}

impl Default for Range {
    fn default() -> Self {
        Range::_0d5G
    }
}

#[derive(Copy, Clone, Debug)]
/// Output data rate (odr) and Low pass filter corner frequency (lpf)
pub enum OutputDataRate {
    /// odr = 3200 Hz and lpf = 1600 Hz
    ODR_3200_HZ = 0b1111,
    /// odr = 1600 Hz and lpf = 800 Hz
    ODR_1600_HZ = 0b1110,
    /// odr = 800 Hz and lpf = 400 Hz
    ODR_800_HZ = 0b1101,
    /// odr = 400 Hz and lpf = 200 Hz
    ODR_400_HZ = 0b1100,
    /// odr = 200 Hz and lpf = 100 Hz
    ODR_200_HZ = 0b1011,
    /// odr = 100 Hz and lpf = 50 Hz
    ODR_100_HZ = 0b1010,
    /// odr = 50 Hz and lpf = 25 Hz
    ODR_50_HZ = 0b1001,
    /// odr = 25 Hz and lpf = 12.5 Hz
    ODR_25_HZ = 0b1000,
    /// odr = 12.5 Hz and lpf = 6.25 Hz
    ODR_12d5_HZ = 0b0111,
    /// odr = 6.25 Hz and lpf = 3.125 Hz
    ODR_6d25_HZ = 0b0110,
}

impl OutputDataRate {
    pub fn val(self) -> u8 {
        self as u8
    }
}

impl From<OutputDataRate> for f32 {
    fn from(rate: OutputDataRate) -> f32 {
        match rate {
            OutputDataRate::ODR_3200_HZ => 3200.00,
            OutputDataRate::ODR_1600_HZ => 1600.00,
            OutputDataRate::ODR_800_HZ => 800.00,
            OutputDataRate::ODR_400_HZ => 400.00,
            OutputDataRate::ODR_200_HZ => 200.00,
            OutputDataRate::ODR_100_HZ => 100.00,
            OutputDataRate::ODR_50_HZ => 50.00,
            OutputDataRate::ODR_25_HZ => 25.00,
            OutputDataRate::ODR_12d5_HZ => 12.50,
            OutputDataRate::ODR_6d25_HZ => 6.25,
        }
    }
}

impl Default for OutputDataRate {
    fn default() -> Self {
        OutputDataRate::ODR_100_HZ
    }
}

#[derive(Copy, Clone, Debug)]
pub enum SleepModeFrequencyReadings {
    _8_HZ = 0b00,
    _4_HZ = 0b01,
    _2_HZ = 0b10,
    _1_HZ = 0b11
}

impl SleepModeFrequencyReadings {
    pub fn val(self) -> u8 {
        self as u8
    }
}

pub struct InterruptSource {
    pub value: u8
}

impl InterruptSource {
    pub fn new(
        data_ready: bool,
        activity: bool,
        inactivity: bool,
        watermark: bool,
        overrun: bool
    ) -> InterruptSource {
        let value =
            ((data_ready as u8) << 7) +
            ((activity as u8) << 4) +
            ((inactivity as u8) << 3) +
            ((watermark as u8) << 1) +
            (overrun as u8);
        InterruptSource { value }
    }

    pub fn data_ready(&self) -> bool {
        self.value & 0b1000_0000 != 0
    }

    pub fn activity(&self) -> bool {
        self.value & 0b0001_0000 != 0
    }

    pub fn inactivity(&self) -> bool {
        self.value & 0b0000_1000 != 0
    }

    pub fn watermark(&self) -> bool {
        self.value & 0b0000_0010 != 0
    }

    pub fn overrun(&self) -> bool {
        self.value & 0b0000_0001 != 0
    }
}

#[derive(Copy, Clone, Debug)]
pub enum FifoMode {
    Bypass = 0b00,
    Fifo = 0b01,
    Stream = 0b10,
    Trigger = 0b11
}

impl FifoMode {
    pub fn val(self) -> u8 {
        self as u8
    }
}

pub struct FifoStatus {
    pub triggered: bool,
    pub samples: u8
}

impl FifoStatus {
    pub fn new(val: u8) -> FifoStatus {
        FifoStatus {
            triggered: val & 0b1000_0000 != 0,
            samples: (val & 0b0001_1111)
        }
    }
}

#[derive(Copy, Clone, Debug)]
pub enum SpiMode {
    _4_WIRE = 0,
    _3_WIRE = 1
}

impl SpiMode {
    pub fn val(self) -> u8 {
        self as u8
    }
}

impl Default for SpiMode {
    fn default() -> Self {
        SpiMode::_4_WIRE
    }
}

#[derive(Copy, Clone, Debug)]
pub enum IrqMode {
    ACTIVE_HIGH = 0,
    ACTIVE_LOW = 1
}

impl IrqMode {
    pub fn val(self) -> u8 {
        self as u8
    }
}

#[derive(Copy, Clone, Debug)]
pub enum AcDcCoupling {
    DcCoupling = 0,
    AcCoupling = 1
}

impl AcDcCoupling {
    pub fn val(self) -> u8 {
        self as u8
    }
}

impl Default for IrqMode {
    fn default() -> Self {
        IrqMode::ACTIVE_HIGH
    }
}

#[derive(Copy, Clone, Debug)]
pub enum FullResolutionMode {
    DISABLED = 0,
    ENABLED = 1
}

impl Default for FullResolutionMode {
    fn default() -> Self {
        FullResolutionMode::DISABLED
    }
}

#[derive(Copy, Clone, Debug)]
pub enum SelfTestMode {
    DISABLED = 0,
    ENABLED = 1
}

impl Default for SelfTestMode {
    fn default() -> Self {
        SelfTestMode::DISABLED
    }
}

// pub struct DataFormatRegister {
//     pub(crate) self_test_mode: SelfTestMode,
//     pub(crate) spi_mode: SpiMode,
//     pub(crate)
// }

pub struct Config {
    pub(crate) range: Option<Range>,
    pub(crate) odr: Option<OutputDataRate>,
    pub(crate) spi_mode: Option<SpiMode>,
    pub(crate) irq_mode: Option<IrqMode>,
    pub(crate) full_resolution_mode: Option<FullResolutionMode>
}

/// ADXL313 configuration struct
impl Config {
    // Creates a new configuration object with default values
    pub fn new() -> Self {
        Config {
            range: None,
            odr: None,
            spi_mode: None,
            irq_mode: None,
            full_resolution_mode: None
        }
    }

    /// Sets the range configuration
    /// Default is 2G
    pub fn range(&mut self, range: Range) -> &mut Self {
        self.range = Some(range);
        self
    }

    /// Sets the output data rate and low pass filter settings.
    /// Default data rate is `100 Hz`
    /// The low pass filter is fixed as 1/4 of the output data rate (fs)
    pub fn odr(&mut self, odr: OutputDataRate) -> &mut Self {
        self.odr = Some(odr);
        self
    }
}