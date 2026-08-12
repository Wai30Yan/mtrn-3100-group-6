use core::f32;

use stm32g4xx_hal::hal::i2c::I2c;

use crate::{I2cDev, print};

pub struct Imu<'a> {
    i2c_dev: I2cDev<'a>,

    ax: f32,
    ay: f32,
    gz: f32,
}

const ADDRESS: u8 = 0x68;
const ACCEL_FACTOR: f32 = 8.0 * 9.81 / 65535.0;
const GYRO_FACTOR: f32 = 1000.0 / 360.0 * 2.0 * f32::consts::PI / 65535.0;

const PWR_MGMT_1_REG: u8 = 0x6b;
const SMPLRT_DIV_REG: u8 = 0x19;
const CONFIG_REG: u8 = 0x26;
const GYRO_CONFIG_REG: u8 = 0x1B;
const ACCEL_CONFIG_REG: u8 = 0x1C;

impl<'a> Imu<'a> {
    pub fn new(mut i2c_dev: I2cDev<'a>) -> Self {
        i2c_dev.write(ADDRESS, &[PWR_MGMT_1_REG, 0x01]).unwrap();
        i2c_dev.write(ADDRESS, &[SMPLRT_DIV_REG, 0x00]).unwrap();
        i2c_dev.write(ADDRESS, &[CONFIG_REG, 0x00]).unwrap();
        i2c_dev.write(ADDRESS, &[GYRO_CONFIG_REG, 0x08]).unwrap();
        i2c_dev.write(ADDRESS, &[ACCEL_CONFIG_REG, 0x08]).unwrap();

        Self {
            i2c_dev,
            ax: 0.0,
            ay: 0.0,
            gz: 0.0,
        }
    }

    /// Read and buffer measurements from the IMU
    pub fn update(&mut self) {
        let mut buf = [0u8; 14];
        if let Err(e) = self.i2c_dev.write_read(ADDRESS, &[0x3b], &mut buf) {
            print!("{:?}\r\n", e);
        }

        self.ax = i16::from_be_bytes(buf[0..2].try_into().unwrap()) as f32 * ACCEL_FACTOR;
        self.ay = i16::from_be_bytes(buf[2..4].try_into().unwrap()) as f32 * ACCEL_FACTOR;
        self.gz = i16::from_be_bytes(buf[12..14].try_into().unwrap()) as f32 * GYRO_FACTOR;
    }

    /// Linear acceleration in the x direction in m/s^2
    /// fwd = +ve, rev = -ve
    pub fn ax(&self) -> f32 {
        self.ax
    }

    /// Linear acceleration in the y direction in m/s^2
    /// left = +ve, right = -ve
    pub fn ay(&self) -> f32 {
        self.ay
    }

    /// Angular velocity about the z axis in rad/s
    /// ccw = +ve, cw = -ve
    pub fn gz(&self) -> f32 {
        self.gz
    }
}
