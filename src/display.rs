use embedded_graphics::{pixelcolor::BinaryColor, prelude::*};
use embedded_hal_bus::i2c::RefCellDevice;
use ssd1306::{
    I2CDisplayInterface, Ssd1306, mode::DisplayConfig, rotation::DisplayRotation,
    size::DisplaySize128x64,
};

use crate::I2cDev;

pub struct Display<'a> {
    display: Ssd1306<
        ssd1306::prelude::I2CInterface<
            RefCellDevice<
                'a,
                stm32g4xx_hal::i2c::I2c<
                    stm32g4::Periph<stm32g4::stm32g431::i2c1::RegisterBlock, 1073764352>,
                    stm32g4xx_hal::gpio::Pin<
                        'A',
                        8,
                        stm32g4xx_hal::gpio::Alternate<4, stm32g4xx_hal::gpio::OpenDrain>,
                    >,
                    stm32g4xx_hal::gpio::Pin<
                        'A',
                        9,
                        stm32g4xx_hal::gpio::Alternate<4, stm32g4xx_hal::gpio::OpenDrain>,
                    >,
                >,
            >,
        >,
        DisplaySize128x64,
        ssd1306::mode::BufferedGraphicsMode<DisplaySize128x64>,
    >,
}

impl<'a> Display<'a> {
    pub fn new(i2c_dev: I2cDev<'a>) -> Self {
        let mut display = Ssd1306::new(
            I2CDisplayInterface::new(i2c_dev),
            DisplaySize128x64,
            DisplayRotation::Rotate0,
        )
        .into_buffered_graphics_mode();
        display.init().unwrap();
        display.clear(BinaryColor::Off).unwrap();
        Self { display }
    }

    pub fn flush(&mut self) {
        self.display.flush().unwrap();
    }

    // pub fn print(&mut self, text: &str) {
    //     for c in text.chars() {
    //         self.display.write_char(c).unwrap();
    //     }
    // }

    pub fn clear(&mut self) {
        self.display.clear(BinaryColor::Off).unwrap();
    }
}

impl<'a> DrawTarget for Display<'a> {
    type Color = BinaryColor;
    type Error = core::convert::Infallible;

    fn draw_iter<I>(&mut self, pixels: I) -> Result<(), Self::Error>
    where
        I: IntoIterator<Item = Pixel<Self::Color>>,
    {
        let _ = self.display.draw_iter(pixels);
        Ok(())
    }
}

impl<'a> OriginDimensions for Display<'a> {
    fn size(&self) -> Size {
        Size::new(128, 64)
    }
}
