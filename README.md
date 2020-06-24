# ADXL313 embedded-hal SPI driver crate

Rust HAL implementation (using SPI drivers) for Analog Device's ADXL313 3-axis accelerometer.
Based off the [adxl355 crate](https://github.com/JitterCompany/adxl355-rs)

## Usage

Include [library](https://crates.io/crates/adxl313) as a dependency in your Cargo.toml

```
[dependencies.adxl313]
version = "<version>"
```

Use embedded-hal implementation to get SPI and a GPIO OutputPin for the chip select, then create the accelerometer handle

```rust

use adxl313::{Adxl313, Accelerometer};

// to create sensor with default configuration:
let mut accelerometer = Adxl313::new(spi, cs)?;

// start measurements
accelerometer.start_measuring();

// to get 3d accerlation data:
let accel = accelerometer.acceleration()?;
println!("{:?}", accel);

```
