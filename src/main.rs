#![no_std]
#![no_main]

// pick a panicking behavior
use panic_halt as _; // you can put a breakpoint on `rust_begin_unwind` to catch panics
                     // use panic_abort as _; // requires nightly
                     // use panic_itm as _; // logs messages over ITM; requires ITM support
                     // use panic_semihosting as _; // logs messages to the host stderr; requires a debugger

use cortex_m::asm;
use cortex_m_rt::entry;


use wio_terminal::*;
use wio_terminal::hal::prelude::*;
use wio_terminal::hal::delay::Delay;
use wio_terminal::pac::{CorePeripherals, Peripherals};
use wio_terminal::hal::{
    clock::GenericClockController,
    time::KiloHertz,
};

use wio_terminal::{spi_master_lcd, spi_master_rtl8720};

use ili9341::{Ili9341, Orientation};
use embedded_graphics::{
    fonts::{Font12x16, Text},
    pixelcolor::{Rgb565, Rgb888},
    prelude::*,
    primitives::Circle,
    style::{PrimitiveStyle, TextStyle},
};

#[entry]
fn main() -> ! {
    let mut peripherals = Peripherals::take().unwrap();
    let core = CorePeripherals::take().unwrap();
    let mut clocks = GenericClockController::with_external_32kosc(
        peripherals.GCLK,
        &mut peripherals.MCLK,
        &mut peripherals.OSC32KCTRL,
        &mut peripherals.OSCCTRL,
        &mut peripherals.NVMCTRL,
    );
    clocks.configure_gclk_divider_and_source(hal::clock::ClockGenId::GCLK0, 1, hal::clock::ClockSource::DPLL0, false);

    let mut pins = Pins::new(peripherals.PORT);
    
    let mut led = pins.user_led.into_push_pull_output(&mut pins.port);
    
    let mut lcd_cs = pins.lcd_cs.into_push_pull_output(&mut pins.port);
    let mut lcd_dc = pins.lcd_dc.into_push_pull_output(&mut pins.port);
    let mut lcd_reset = pins.lcd_reset.into_push_pull_output(&mut pins.port);
    let mut lcd_backlight_ctr = pins.lcd_backlight_ctr.into_push_pull_output(&mut pins.port);
    let mut spi_lcd = spi_master_lcd(&mut clocks, KiloHertz(60000u32), peripherals.SERCOM7, &mut peripherals.MCLK, pins.lcd_sck, pins.lcd_mosi, pins.lcd_miso, &mut pins.port);
    
    let mut rtl_cs = pins.rtl8720d_hspi_cs.into_push_pull_output(&mut pins.port);
    let mut spi_rtl = spi_master_rtl8720(&mut clocks, KiloHertz(60000u32), peripherals.SERCOM0, &mut peripherals.MCLK, pins.rtl8720d_hspi_clk, pins.rtl8720d_hspi_mosi, pins.rtl8720d_hspi_miso, &mut pins.port);
    
    
    let mut delay = Delay::new(core.SYST, &mut clocks);

    let mut lcd = Ili9341::new_spi(spi_lcd, lcd_cs, lcd_dc, lcd_reset, &mut delay).unwrap();
    lcd.set_orientation(Orientation::LandscapeFlipped);

    lcd_backlight_ctr.set_high();

    let c = Circle::new(Point::new(160, 120), 20).into_styled(PrimitiveStyle::with_fill((Rgb565::BLUE)));
    let t = Text::new("Hello, Rust!", Point::new(0, 0))
        .into_styled(TextStyle::new(Font12x16, Rgb565::BLUE));
    c.draw(&mut lcd);
    t.draw(&mut lcd);
    loop {
        delay.delay_ms(1000u16);
        led.toggle();
    }
}
