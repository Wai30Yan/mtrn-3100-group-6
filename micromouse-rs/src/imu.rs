use crate::I2cBus;

pub struct Imu<'a> {
    i2c_bus: &'a I2cBus<'a>,

    ax: f32,
    ay: f32,
    gz: f32,
}

const ADDRESS: u8 = 0x68;
const ACCELEROMETER_FACTOR: f32 = 9.81 / 8192.0;
const GYROSCOPE_FACTOR: f32 = 1.0 / 1.143191;

impl<'a> Imu<'a> {
    pub fn new(i2c_bus: &'a I2cBus<'a>) -> Self {
        // PwrMgmt1
        i2c_bus.borrow_mut().write(ADDRESS, &[0x6b, 0x01]).unwrap();
        // SmplrtDiv
        i2c_bus.borrow_mut().write(ADDRESS, &[0x19, 0x00]).unwrap();
        // ConfigReg
        i2c_bus.borrow_mut().write(ADDRESS, &[0x26, 0x00]).unwrap();
        // GyroConfig
        i2c_bus.borrow_mut().write(ADDRESS, &[0x27, 0x08]).unwrap();
        // AccelConfigReg
        i2c_bus.borrow_mut().write(ADDRESS, &[0x28, 0x08]).unwrap();

        Self {
            i2c_bus,
            ax: 0.0,
            ay: 0.0,
            gz: 0.0,
        }
    }

    /// Read and buffer measurements from the IMU
    pub fn update(&mut self) {
        let mut buf = [0u8; 14];
        self.i2c_bus
            .borrow_mut()
            .write_read(ADDRESS, &[0x3b], &mut buf)
            .unwrap();

        self.ax = i16::from_le_bytes(*buf[0..1].as_array().unwrap()) as f32 * ACCELEROMETER_FACTOR;
        self.ay = i16::from_le_bytes(*buf[2..3].as_array().unwrap()) as f32 * ACCELEROMETER_FACTOR;
        self.gz = i16::from_le_bytes(*buf[6..7].as_array().unwrap()) as f32 * GYROSCOPE_FACTOR;
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
