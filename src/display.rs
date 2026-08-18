use embedded_graphics::{
    Drawable,
    geometry::Point,
    mono_font::{MonoTextStyle, ascii::FONT_6X10},
    pixelcolor::BinaryColor,
    text::Text,
};
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
        Self { display }
    }

    pub fn draw<const X: usize, const Y: usize>(
        &mut self,
        tx: usize,
        ty: usize,
        img: [[bool; Y]; X],
        msg: &str,
    ) {
        let style = MonoTextStyle::new(&FONT_6X10, BinaryColor::On);
        self.display.clear_buffer();
        Text::new(msg, Point::new(72, 16), style)
            .draw(&mut self.display)
            .unwrap();

        for y in 0..Y {
            for x in 0..X {
                self.display
                    .set_pixel((tx + x) as u32, (ty + y) as u32, img[x][y]);
            }
        }
        self.display.flush().unwrap();
    }
}
