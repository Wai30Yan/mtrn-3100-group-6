use cortex_m::asm::delay;
use nb::Error::WouldBlock;
use stm32g4xx_hal::gpio::{AnyPin, OpenDrain, Output};

use crate::I2cBus;

const IDENTIFICATION_MODEL_ID: u16 = 0x000;
const IDENTIFICATION_MODEL_REV_MAJOR: u16 = 0x001;
const IDENTIFICATION_MODEL_REV_MINOR: u16 = 0x002;
const IDENTIFICATION_MODULE_REV_MAJOR: u16 = 0x003;
const IDENTIFICATION_MODULE_REV_MINOR: u16 = 0x004;
const IDENTIFICATION_DATE_HI: u16 = 0x006;
const IDENTIFICATION_DATE_LO: u16 = 0x007;
const IDENTIFICATION_TIME: u16 = 0x008; // 16-bit

const SYSTEM_MODE_GPIO0: u16 = 0x010;
const SYSTEM_MODE_GPIO1: u16 = 0x011;
const SYSTEM_HISTORY_CTRL: u16 = 0x012;
const SYSTEM_INTERRUPT_CONFIG_GPIO: u16 = 0x014;
const SYSTEM_INTERRUPT_CLEAR: u16 = 0x015;
const SYSTEM_FRESH_OUT_OF_RESET: u16 = 0x016;
const SYSTEM_GROUPED_PARAMETER_HOLD: u16 = 0x017;

const SYSRANGE_START: u16 = 0x018;
const SYSRANGE_THRESH_HIGH: u16 = 0x019;
const SYSRANGE_THRESH_LOW: u16 = 0x01A;
const SYSRANGE_INTERMEASUREMENT_PERIOD: u16 = 0x01B;
const SYSRANGE_MAX_CONVERGENCE_TIME: u16 = 0x01C;
const SYSRANGE_CROSSTALK_COMPENSATION_RATE: u16 = 0x01E; // 16-bit
const SYSRANGE_CROSSTALK_VALID_HEIGHT: u16 = 0x021;
const SYSRANGE_EARLY_CONVERGENCE_ESTIMATE: u16 = 0x022; // 16-bit
const SYSRANGE_PART_TO_PART_RANGE_OFFSET: u16 = 0x024;
const SYSRANGE_RANGE_IGNORE_VALID_HEIGHT: u16 = 0x025;
const SYSRANGE_RANGE_IGNORE_THRESHOLD: u16 = 0x026; // 16-bit
const SYSRANGE_MAX_AMBIENT_LEVEL_MULT: u16 = 0x02C;
const SYSRANGE_RANGE_CHECK_ENABLES: u16 = 0x02D;
const SYSRANGE_VHV_RECALIBRATE: u16 = 0x02E;
const SYSRANGE_VHV_REPEAT_RATE: u16 = 0x031;

const SYSALS_START: u16 = 0x038;
const SYSALS_THRESH_HIGH: u16 = 0x03A;
const SYSALS_THRESH_LOW: u16 = 0x03C;
const SYSALS_INTERMEASUREMENT_PERIOD: u16 = 0x03E;
const SYSALS_ANALOGUE_GAIN: u16 = 0x03F;
const SYSALS_INTEGRATION_PERIOD: u16 = 0x040;

const RESULT_RANGE_STATUS: u16 = 0x04D;
const RESULT_ALS_STATUS: u16 = 0x04E;
const RESULT_INTERRUPT_STATUS_GPIO: u16 = 0x04F;
const RESULT_ALS_VAL: u16 = 0x050; // 16-bit
const RESULT_HISTORY_BUFFER_0: u16 = 0x052; // 16-bit
const RESULT_HISTORY_BUFFER_1: u16 = 0x054; // 16-bit
const RESULT_HISTORY_BUFFER_2: u16 = 0x056; // 16-bit
const RESULT_HISTORY_BUFFER_3: u16 = 0x058; // 16-bit
const RESULT_HISTORY_BUFFER_4: u16 = 0x05A; // 16-bit
const RESULT_HISTORY_BUFFER_5: u16 = 0x05C; // 16-bit
const RESULT_HISTORY_BUFFER_6: u16 = 0x05E; // 16-bit
const RESULT_HISTORY_BUFFER_7: u16 = 0x060; // 16-bit
const RESULT_RANGE_VAL: u16 = 0x062;
const RESULT_RANGE_RAW: u16 = 0x064;
const RESULT_RANGE_RETURN_RATE: u16 = 0x066; // 16-bit
const RESULT_RANGE_REFERENCE_RATE: u16 = 0x068; // 16-bit
const RESULT_RANGE_RETURN_SIGNAL_COUNT: u16 = 0x06C; // 32-bit
const RESULT_RANGE_REFERENCE_SIGNAL_COUNT: u16 = 0x070; // 32-bit
const RESULT_RANGE_RETURN_AMB_COUNT: u16 = 0x074; // 32-bit
const RESULT_RANGE_REFERENCE_AMB_COUNT: u16 = 0x078; // 32-bit
const RESULT_RANGE_RETURN_CONV_TIME: u16 = 0x07C; // 32-bit
const RESULT_RANGE_REFERENCE_CONV_TIME: u16 = 0x080; // 32-bit

const RANGE_SCALER: u16 = 0x096; // 16-bit - see STSW-IMG003 core/inc/vl6180x_def.h

const READOUT_AVERAGING_SAMPLE_PERIOD: u16 = 0x10A;
const FIRMWARE_BOOTUP: u16 = 0x119;
const FIRMWARE_RESULT_SCALER: u16 = 0x120;
const I2C_SLAVE_DEVICE_ADDRESS: u16 = 0x212;
const INTERLEAVED_MODE_ENABLE: u16 = 0x2A3;

const DEFAULT_LIDAR_ADDR: u8 = 0x29;

pub fn concat<T: Copy + Default, const A: usize, const B: usize>(
    a: &[T; A],
    b: &[T; B],
) -> [T; A + B] {
    let mut whole: [T; A + B] = [Default::default(); A + B];
    let (one, two) = whole.split_at_mut(A);
    one.copy_from_slice(a);
    two.copy_from_slice(b);
    whole
}

pub struct Lidar<'a> {
    i2c_bus: &'a I2cBus<'a>,
    address: u8,
    distance: f32,
}

impl<'a> Lidar<'a> {
    pub fn new(
        i2c_bus: &'a I2cBus<'a>,
        mut enable_pin: AnyPin<Output<OpenDrain>>,
        address: u8,
    ) -> Self {
        enable_pin.set_high();
        // Arbitary wait
        delay(24 * 1024 * 1024);

        let mut lidar = Self {
            i2c_bus,
            address: DEFAULT_LIDAR_ADDR,
            distance: 0.0,
        };

        // magic code to initialize the lidar, taken from the datasheet
        lidar.write_reg(0x0208, 0x01);
        lidar.write_reg(0x0207, 0x01);
        lidar.write_reg(0x0096, 0x00);
        lidar.write_reg(0x0097, 0xFD); // RANGE_SCALER = 253
        lidar.write_reg(0x00E3, 0x01);
        lidar.write_reg(0x00E4, 0x03);
        lidar.write_reg(0x00E5, 0x02);
        lidar.write_reg(0x00E6, 0x01);
        lidar.write_reg(0x00E7, 0x03);
        lidar.write_reg(0x00F5, 0x02);
        lidar.write_reg(0x00D9, 0x05);
        lidar.write_reg(0x00DB, 0xCE);
        lidar.write_reg(0x00DC, 0x03);
        lidar.write_reg(0x00DD, 0xF8);
        lidar.write_reg(0x009F, 0x00);
        lidar.write_reg(0x00A3, 0x3C);
        lidar.write_reg(0x00B7, 0x00);
        lidar.write_reg(0x00BB, 0x3C);
        lidar.write_reg(0x00B2, 0x09);
        lidar.write_reg(0x00CA, 0x09);
        lidar.write_reg(0x0198, 0x01);
        lidar.write_reg(0x01B0, 0x17);
        lidar.write_reg(0x01AD, 0x00);
        lidar.write_reg(0x00FF, 0x05);
        lidar.write_reg(0x0100, 0x05);
        lidar.write_reg(0x0199, 0x05);
        lidar.write_reg(0x01A6, 0x1B);
        lidar.write_reg(0x01AC, 0x3E);
        lidar.write_reg(0x01A7, 0x1F);
        lidar.write_reg(0x0030, 0x00);

        // update slave address
        lidar.write_reg(I2C_SLAVE_DEVICE_ADDRESS, address);
        lidar.address = address;

        lidar
    }

    fn write_reg(&mut self, reg: u16, val: u8) {
        self.i2c_bus
            .borrow_mut()
            .write(self.address, &concat(&reg.to_be_bytes(), &[val]))
            .unwrap();
    }

    fn read_reg(&mut self, reg: u16) -> u8 {
        let mut buf = [0u8; 1];
        self.i2c_bus
            .borrow_mut()
            .write_read(self.address, &reg.to_be_bytes(), &mut buf)
            .unwrap();

        buf[0]
    }

    pub fn update(&mut self) -> nb::Result<(), !> {
        // TODO: convert from raw value to m
        self.distance = self.read_reg(RESULT_RANGE_VAL) as f32;

        // TODO: return nb::Result::Err(WouldBlock) if the value is not ready
        nb::Result::Ok(())
    }

    pub fn distance(&self) -> f32 {
        self.distance
    }
}
