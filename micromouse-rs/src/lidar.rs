use crate::I2cBus;

// const IDENTIFICATION_MODEL_ID: u16 = 0x000;
// const IDENTIFICATION_MODEL_REV_MAJOR: u16 = 0x001;
// const IDENTIFICATION_MODEL_REV_MINOR: u16 = 0x002;
// const IDENTIFICATION_MODULE_REV_MAJOR: u16 = 0x003;
// const IDENTIFICATION_MODULE_REV_MINOR: u16 = 0x004;
// const IDENTIFICATION_DATE_HI: u16 = 0x006;
// const IDENTIFICATION_DATE_LO: u16 = 0x007;
// const IDENTIFICATION_TIME: u16 = 0x008; // 16-bit

// const SYSTEM_MODE_GPIO0: u16 = 0x010;
// const SYSTEM_MODE_GPIO1: u16 = 0x011;
// const SYSTEM_HISTORY_CTRL: u16 = 0x012;
// const SYSTEM_INTERRUPT_CONFIG_GPIO: u16 = 0x014;
// const SYSTEM_INTERRUPT_CLEAR: u16 = 0x015;
// const SYSTEM_FRESH_OUT_OF_RESET: u16 = 0x016;
// const SYSTEM_GROUPED_PARAMETER_HOLD: u16 = 0x017;

// const SYSRANGE_START: u16 = 0x018;
// const SYSRANGE_THRESH_HIGH: u16 = 0x019;
// const SYSRANGE_THRESH_LOW: u16 = 0x01A;
// const SYSRANGE_INTERMEASUREMENT_PERIOD: u16 = 0x01B;
// const SYSRANGE_MAX_CONVERGENCE_TIME: u16 = 0x01C;
// const SYSRANGE_CROSSTALK_COMPENSATION_RATE: u16 = 0x01E; // 16-bit
// const SYSRANGE_CROSSTALK_VALID_HEIGHT: u16 = 0x021;
// const SYSRANGE_EARLY_CONVERGENCE_ESTIMATE: u16 = 0x022; // 16-bit
// const SYSRANGE_PART_TO_PART_RANGE_OFFSET: u16 = 0x024;
// const SYSRANGE_RANGE_IGNORE_VALID_HEIGHT: u16 = 0x025;
// const SYSRANGE_RANGE_IGNORE_THRESHOLD: u16 = 0x026; // 16-bit
// const SYSRANGE_MAX_AMBIENT_LEVEL_MULT: u16 = 0x02C;
// const SYSRANGE_RANGE_CHECK_ENABLES: u16 = 0x02D;
// const SYSRANGE_VHV_RECALIBRATE: u16 = 0x02E;
// const SYSRANGE_VHV_REPEAT_RATE: u16 = 0x031;

// const SYSALS_START: u16 = 0x038;
// const SYSALS_THRESH_HIGH: u16 = 0x03A;
// const SYSALS_THRESH_LOW: u16 = 0x03C;
// const SYSALS_INTERMEASUREMENT_PERIOD: u16 = 0x03E;
// const SYSALS_ANALOGUE_GAIN: u16 = 0x03F;
// const SYSALS_INTEGRATION_PERIOD: u16 = 0x040;

// const RESULT_RANGE_STATUS: u16 = 0x04D;
// const RESULT_ALS_STATUS: u16 = 0x04E;
// const RESULT_INTERRUPT_STATUS_GPIO: u16 = 0x04F;
// const RESULT_ALS_VAL: u16 = 0x050; // 16-bit
// const RESULT_HISTORY_BUFFER_0: u16 = 0x052; // 16-bit
// const RESULT_HISTORY_BUFFER_1: u16 = 0x054; // 16-bit
// const RESULT_HISTORY_BUFFER_2: u16 = 0x056; // 16-bit
// const RESULT_HISTORY_BUFFER_3: u16 = 0x058; // 16-bit
// const RESULT_HISTORY_BUFFER_4: u16 = 0x05A; // 16-bit
// const RESULT_HISTORY_BUFFER_5: u16 = 0x05C; // 16-bit
// const RESULT_HISTORY_BUFFER_6: u16 = 0x05E; // 16-bit
// const RESULT_HISTORY_BUFFER_7: u16 = 0x060; // 16-bit
// const RESULT_RANGE_VAL: u16 = 0x062;
// const RESULT_RANGE_RAW: u16 = 0x064;
// const RESULT_RANGE_RETURN_RATE: u16 = 0x066; // 16-bit
// const RESULT_RANGE_REFERENCE_RATE: u16 = 0x068; // 16-bit
// const RESULT_RANGE_RETURN_SIGNAL_COUNT: u16 = 0x06C; // 32-bit
// const RESULT_RANGE_REFERENCE_SIGNAL_COUNT: u16 = 0x070; // 32-bit
// const RESULT_RANGE_RETURN_AMB_COUNT: u16 = 0x074; // 32-bit
// const RESULT_RANGE_REFERENCE_AMB_COUNT: u16 = 0x078; // 32-bit
// const RESULT_RANGE_RETURN_CONV_TIME: u16 = 0x07C; // 32-bit
// const RESULT_RANGE_REFERENCE_CONV_TIME: u16 = 0x080; // 32-bit

// const RANGE_SCALER: u16 = 0x096; // 16-bit - see STSW-IMG003 core/inc/vl6180x_def.h

// const READOUT_AVERAGING_SAMPLE_PERIOD: u16 = 0x10A;
// const FIRMWARE_BOOTUP: u16 = 0x119;
// const FIRMWARE_RESULT_SCALER: u16 = 0x120;
// const I2C_SLAVE_DEVICE_ADDRESS: u16 = 0x212;
// const INTERLEAVED_MODE_ENABLE: u16 = 0x2A3;

const DEFAULT_LIDAR_ADDR: u8 = 0x29;
pub struct Lidar <'a> {
    i2c_bus: &'a I2cBus<'a>,
    address: u8,
    distance: f32,
}

impl<'a> Lidar<'a> {
    pub fn new(i2c_bus: &'a I2cBus <'a>, address: u8) -> Self {
        // magic code to initialize the lidar, taken from the datasheet
        // writeReg(0x207, 0x01);
        i2c_bus.borrow_mut().write(DEFAULT_LIDAR_ADDR, &[0x02, 0x07, 0x01]).unwrap();
        // writeReg(0x208, 0x01);
        i2c_bus.borrow_mut().write(DEFAULT_LIDAR_ADDR, &[0x02, 0x08, 0x01]).unwrap();
        // writeReg(0x096, 0x00);
        i2c_bus.borrow_mut().write(DEFAULT_LIDAR_ADDR, &[0x00, 0x96, 0x00]).unwrap();
        // writeReg(0x097, 0xFD); // RANGE_SCALER = 253        
        i2c_bus.borrow_mut().write(DEFAULT_LIDAR_ADDR, &[0x00, 0x97, 0xFD]).unwrap();
        // writeReg(0x0E3, 0x01);
        i2c_bus.borrow_mut().write(DEFAULT_LIDAR_ADDR, &[0x00, 0xE3, 0x01]).unwrap();
        // writeReg(0x0E4, 0x03);
        i2c_bus.borrow_mut().write(DEFAULT_LIDAR_ADDR, &[0x00, 0xE4, 0x03]).unwrap();
        // writeReg(0x0E5, 0x02);
        i2c_bus.borrow_mut().write(DEFAULT_LIDAR_ADDR, &[0x00, 0xE5, 0x02]).unwrap();
        // writeReg(0x0E6, 0x01);
        i2c_bus.borrow_mut().write(DEFAULT_LIDAR_ADDR, &[0x00, 0xE6, 0x01]).unwrap();
        // writeReg(0x0E7, 0x03);
        i2c_bus.borrow_mut().write(DEFAULT_LIDAR_ADDR, &[0x00, 0xE7, 0x03]).unwrap();
        // writeReg(0x0F5, 0x02);
        i2c_bus.borrow_mut().write(DEFAULT_LIDAR_ADDR, &[0x00, 0xF5, 0x02]).unwrap();
        // writeReg(0x0D9, 0x05);
        i2c_bus.borrow_mut().write(DEFAULT_LIDAR_ADDR, &[0x00, 0xD9, 0x05]).unwrap();
        // writeReg(0x0DB, 0xCE);
        i2c_bus.borrow_mut().write(DEFAULT_LIDAR_ADDR, &[0x00, 0xDB, 0xCE]).unwrap();
        // writeReg(0x0DC, 0x03);
        i2c_bus.borrow_mut().write(DEFAULT_LIDAR_ADDR, &[0x00, 0xDC, 0x03]).unwrap();
        // writeReg(0x0DD, 0xF8);
        i2c_bus.borrow_mut().write(DEFAULT_LIDAR_ADDR, &[0x00, 0xDD, 0xF8]).unwrap();
        // writeReg(0x09F, 0x00);
        i2c_bus.borrow_mut().write(DEFAULT_LIDAR_ADDR, &[0x00, 0x9F, 0x00]).unwrap();
        // writeReg(0x0A3, 0x3C);
        i2c_bus.borrow_mut().write(DEFAULT_LIDAR_ADDR, &[0x00, 0xA3, 0x3C]).unwrap();
        // writeReg(0x0B7, 0x00);
        i2c_bus.borrow_mut().write(DEFAULT_LIDAR_ADDR, &[0x00, 0xB7, 0x00]).unwrap();
        // writeReg(0x0BB, 0x3C);
        i2c_bus.borrow_mut().write(DEFAULT_LIDAR_ADDR, &[0x00, 0xBB, 0x3C]).unwrap();
        // writeReg(0x0B2, 0x09);
        i2c_bus.borrow_mut().write(DEFAULT_LIDAR_ADDR, &[0x00, 0xB2, 0x09]).unwrap();
        // writeReg(0x0CA, 0x09);
        i2c_bus.borrow_mut().write(DEFAULT_LIDAR_ADDR, &[0x00, 0xCA, 0x09]).unwrap();
        // writeReg(0x198, 0x01);
        i2c_bus.borrow_mut().write(DEFAULT_LIDAR_ADDR, &[0x01, 0x98, 0x01]).unwrap();
        // writeReg(0x1B0, 0x17);
        i2c_bus.borrow_mut().write(DEFAULT_LIDAR_ADDR, &[0x01, 0xB0, 0x17]).unwrap();
        // writeReg(0x1AD, 0x00);
        i2c_bus.borrow_mut().write(DEFAULT_LIDAR_ADDR, &[0x01, 0xAD, 0x00]).unwrap();
        // writeReg(0x0FF, 0x05);
        i2c_bus.borrow_mut().write(DEFAULT_LIDAR_ADDR, &[0x00, 0xFF, 0x05]).unwrap();
        // writeReg(0x100, 0x05);
        i2c_bus.borrow_mut().write(DEFAULT_LIDAR_ADDR, &[0x01, 0x00, 0x05]).unwrap();
        // writeReg(0x199, 0x05);
        i2c_bus.borrow_mut().write(DEFAULT_LIDAR_ADDR, &[0x01, 0x99, 0x05]).unwrap();
        // writeReg(0x1A6, 0x1B);
        i2c_bus.borrow_mut().write(DEFAULT_LIDAR_ADDR, &[0x01, 0xA6, 0x1B]).unwrap();
        // writeReg(0x1AC, 0x3E);
        i2c_bus.borrow_mut().write(DEFAULT_LIDAR_ADDR, &[0x01, 0xAC, 0x3E]).unwrap();
        // writeReg(0x1A7, 0x1F);
        i2c_bus.borrow_mut().write(DEFAULT_LIDAR_ADDR, &[0x01, 0xA7, 0x1F]).unwrap();
        // writeReg(0x030, 0x00);
        i2c_bus.borrow_mut().write(DEFAULT_LIDAR_ADDR, &[0x00, 0x30, 0x00]).unwrap();


        // update slave address
        i2c_bus.borrow_mut().write(DEFAULT_LIDAR_ADDR, &[0x2, 0x12, address]).unwrap();

        Self {
            i2c_bus,
            address,
            distance: 0.0,
        }
    }

    pub fn update(&mut self) {
        let mut buf = [0u8; 2];
        self.i2c_bus
            .borrow_mut()
            .write_read(self.address, &[0x1E], &mut buf)
            .unwrap();
        self.distance = u16::from_be_bytes(buf) as f32;
    }

    pub fn get_distance(&self) -> f32 {
        self.distance
    }
} 