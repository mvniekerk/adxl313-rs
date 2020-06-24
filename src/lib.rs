#![no_std]

mod conf;
mod register;

use core::fmt::Debug;

use embedded_hal as hal;

use hal::blocking::spi;
use hal::digital::v2::OutputPin;

pub use accelerometer::{Accelerometer, RawAccelerometer, error, Error, vector::{I32x3, F32x3}};

pub use conf::*;
use register::Register;

const SPI_READ: u8 = 0x01;
const SPI_WRITE: u8 = 0x00;

const EXPECTED_DEVID_0: u8 = 0xad;
const EXPECTED_DEVID_1: u8 = 0x1d;
const EXPECTED_PART_ID: u8 = 0b1100_1011;
const EXPECTED_DEVICE_ID: u32 =
    ((EXPECTED_DEVID_0 as u32) << 16) |
    ((EXPECTED_DEVID_1 as u32) << 8) |
    (EXPECTED_PART_ID as u32);

const ACCEL_MAX_I20: u32 = 524_287; // = 2^(20-1)-1

/// ADXL313 driver
pub struct Adxl313<SPI, CS> {
    spi: SPI,
    cs: CS
}

impl<SPI, CS, E, PinError> Adxl313<SPI, CS>
    where
        SPI: spi::Transfer<u8, Error=E> + spi::Write<u8, Error=E>,
        CS: OutputPin<Error = PinError>
{
    /// Takes a config object to initialize the adxl313 driver
    pub fn new(spi:SPI, cs:CS) -> Result<Self, E> {
        let mut adxl313 = Adxl313 {
            spi,
            cs
        };

        let id = adxl313.get_device_id();

        if id != EXPECTED_DEVICE_ID {
            // error

        }
        Ok(adxl313)
    }

    pub fn xid(&mut self) -> u8 {
        let mut buffer = [0u8; 1];
        self.read_reg(Register::XID.addr(), &mut buffer);
        buffer[0]
    }

    pub fn offsets(&mut self, offset_x: u8, offset_y: u8, offset_z: u8) {
        self.write_reg(Register::OFSX.addr(), offset_x);
        self.write_reg(Register::OFSY.addr(), offset_y);
        self.write_reg(Register::OFSZ.addr(), offset_z);
    }

    pub fn activity_threshold(&mut self, threshold: u8) {
        self.write_reg(Register::THRESH_ACT.addr(), threshold);
    }

    pub fn inactivity_threshold(&mut self, threshold: u8) {
        self.write_reg(Register::THRESH_INACT.addr(), threshold);
    }

    pub fn inactivity_time(&mut self, time: u8) {
        self.write_reg(Register::TIME_INACT.addr(), time);
    }

    pub fn activity_inactivity_control(
        &mut self, activity_coupling: AcDcCoupling, inactivity_coupling: AcDcCoupling,
        enable_x_activity: bool, enable_x_inactivity: bool,
        enable_y_activity: bool, enable_y_inactivity: bool,
        enable_z_activity: bool, enable_z_inactivity: bool
    ) {
        let val =
            (activity_coupling.val() << 7) +
            (enable_x_activity as u8) << 6 +
            (enable_y_activity as u8) << 5 +
            (enable_z_activity as u8) << 4 +
            (inactivity_coupling.val() << 3) +
            (enable_x_inactivity as u8) << 2 +
            (enable_y_inactivity as u8) << 1 +
            (enable_z_inactivity as u8);
        self.write_reg(Register::ACT_INACT_CTL.addr(), val);
    }

    pub fn output_data_rate_and_low_power(&mut self, rate: OutputDataRate, low_power_enabled: bool) {
        let val =
            ((low_power_enabled as u8) << 4) +
            rate.val();
        self.write_reg(Register::BW_RATE.addr(), val);
    }

    pub fn power_control(
        &mut self,
        i2c_disabled: bool,
        concurrent_activity_inactivity: bool,
        auto_sleep: bool,
        measuring: bool,
        sleep_enabled: bool,
        sleep_mode_frequency_reading: SleepModeFrequencyReadings
    ) {
        let val =
            ((i2c_disabled as u8) << 6) +
            ((concurrent_activity_inactivity as u8) << 5) +
            ((auto_sleep as u8) << 4) +
            ((measuring as u8) << 3) +
            ((sleep_enabled as u8) << 2) +
            sleep_mode_frequency_reading.val();
        self.write_reg(Register::POWER_CTL.addr(), val);
    }

    pub fn interrupt_enable(&mut self, conf: InterruptSource) {
        self.write_reg(Register::INT_ENABLE.addr(), conf.value);
    }

    pub fn interrupt_pin_mapping(&mut self, conf: InterruptSource) {
        self.write_reg(Register::INT_MAP.addr(), conf.value);
    }

    pub fn interrupt_source(&mut self) -> InterruptSource {
        let mut buffer = [0u8; 1];
        self.read_reg(Register::INT_SOURCE.addr(), &buffer);
        InterruptSource {
            value: buffer[0]
        }
    }

    pub fn data_format(&mut self, self_test: bool, spi_mode: SpiMode, irq_mode: IrqMode, full_resolution: bool, justify: bool, range: Range) {
        let val: u8 =
            ((self_test as u8) << 7) +
            (spi_mode.val() << 6) +
            (irq_mode.val() << 5) +
            ((full_resolution as u8) << 3) +
            ((justify as u8) << 2) + range.val();
        self.write_reg(Register::DATA_FORMAT.addr(), val);
    }

    pub fn soft_reset(&mut self) {
        let val = 0x52;
        self.write_reg(Register::SOFT_RESET.addr(), val);
    }

    /// Puts the device in `Measurement mode`. The default after power up is `Standby mode`.
    pub fn start(&mut self) {
        self.write_reg(Register::POWER_CTL.addr(), 0);
    }

    /// Get the device ID
    pub fn get_device_id(&mut self) -> u32 {
        let mut output = [0u8; 3];
        // Read 3 bytes
        self.read_reg(Register::DEVID_0.addr(), &mut output);
        ((output[0] as u32) << 16)
            | ((output[1] as u32) << 8)
            | (output[0] as u32)
    }

    fn write_reg(&mut self, reg: u8, value: u8) {
        let mut bytes = [(reg << 1)  | SPI_WRITE, value];
        self.cs.set_low().ok();
        self.spi.write(&mut bytes).ok();
        self.cs.set_high().ok();
    }

    fn read_reg(&mut self, reg: u8, buffer: &mut [u8]) {
        let mut bytes = [(reg << 1)  | SPI_READ, 0];
        self.cs.set_low().ok();
        self.spi.transfer(&mut bytes).ok();
        self.cs.set_high().ok();
        buffer[0] = bytes[1];
    }

    fn read(&mut self, bytes: &mut [u8]) {
        self.cs.set_low().ok();
        self.spi.transfer(bytes).ok();
        self.cs.set_high().ok();
    }
}

impl<SPI, CS, E, EO> RawAccelerometer<I32x3> for Adxl313<SPI, CS>
    where
        SPI: spi::Transfer<u8, Error=E> + spi::Write<u8, Error=E>,
        CS: OutputPin<Error = EO>,
        E: Debug
{
    type Error = E;

    /// Gets acceleration vector reading from the accelerometer
    /// Returns a 3D vector with x,y,z, fields in a Result
    fn accel_raw(&mut self) -> Result<I32x3, Error<E>> {
        let mut bytes = [0u8; 9+1];
        bytes[0] = (Register::XDATA3.addr() << 1)  | SPI_READ;
        self.read(&mut bytes);

        // combine 3 bytes into one i32 value
        // right-shift with sign-extend to 20-bit
        let x = ((((bytes[1] as i32) << 24) | ((bytes[2] as i32) << 16) | ((bytes[3] & 0xF0) as i32) << 8)) >> 12;
        let y = ((((bytes[4] as i32) << 24) | ((bytes[5] as i32) << 16) | ((bytes[6] & 0xF0) as i32) << 8)) >> 12;
        let z = ((((bytes[7] as i32) << 24) | ((bytes[8] as i32) << 16) | ((bytes[9] & 0xF0) as i32) << 8)) >> 12;

        Ok(I32x3::new(x, y, z))
    }

}

impl<SPI, CS, E, PinError> Accelerometer for Adxl313<SPI, CS>
    where
        SPI: spi::Transfer<u8, Error=E> + spi::Write<u8, Error=E>,
        CS: OutputPin<Error = PinError>,
        E: Debug
{
    type Error = E;

    fn accel_norm(&mut self) -> Result<F32x3, Error<Self::Error>> {
        let raw_data: I32x3 = self.accel_raw()?;
        let range: f32 = self.range.into(); // range in [g], so 0.5, 1, 2, 4 or 8

        let x = (raw_data.x as f32 / ACCEL_MAX_I20 as f32) * range;
        let y = (raw_data.y as f32 / ACCEL_MAX_I20 as f32) * range;
        let z = (raw_data.z as f32 / ACCEL_MAX_I20 as f32) * range;

        Ok(F32x3::new(x, y, z))
    }

    fn sample_rate(&mut self) -> Result<f32, Error<Self::Error>> {
        Ok(self.odr.into())
    }
}