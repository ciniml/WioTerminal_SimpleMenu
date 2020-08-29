#![no_std]
#![no_main]


// pick a panicking behavior
use panic_halt as _; // you can put a breakpoint on `rust_begin_unwind` to catch panics
                     // use panic_abort as _; // requires nightly
                     // use panic_itm as _; // logs messages over ITM; requires ITM support
                     // use panic_semihosting as _; // logs messages to the host stderr; requires a debugger

use cortex_m_rt::entry;
use heapless::{consts, spsc::Queue};

use wio_terminal::hal::delay::Delay;
use wio_terminal::hal::timer::TimerCounter;
use wio_terminal::hal::prelude::*;
use wio_terminal::hal::{clock::GenericClockController, time::KiloHertz, time::Nanoseconds};
use wio_terminal::pac::{CorePeripherals, Peripherals};
use wio_terminal::*;

use wio_terminal::{spi_master_lcd, spi_master_rtl8720};

use embedded_hal::timer::CountDown;

use embedded_graphics::{
    fonts::{Font12x16, Text},
    pixelcolor::{Rgb565},
    prelude::*,
    primitives::Circle,
    style::{PrimitiveStyle, TextStyle},
};
use ili9341::{Ili9341, Orientation};

mod esp_at;
mod atsamd_dmac;
mod async_operation;

use atsamd_dmac::{Dmac};
use async_operation::{AsyncOperation, OperationStatus, AsyncOperationError};
use esp_at::WioTerminalSPITransport;

use embedded_dma::*;
use core::cell::RefCell;

use atat::{prelude::*, atat_derive::*, ClientBuilder, ComQueue, Queues, ResQueue, UrcQueue, NoopUrcMatcher};



#[derive(Clone, AtatResp)]
pub struct NoResponse;

#[derive(Clone, AtatCmd)]
#[at_cmd("AT", NoResponse, timeout_ms = 1000)]
pub struct AT;

struct RTLTimer<Timer, TimeConverter> {
    timer: Timer,
    converter: TimeConverter,
}
impl<Timer: CountDown, TimeConverter: Fn(u32) -> Timer::Time> RTLTimer<Timer, TimeConverter> {
    pub fn new(timer: Timer, converter: TimeConverter) -> Self {
        Self {
            timer: timer,
            converter: converter,
        }
    }
}

impl<Timer: CountDown, TimeConverter: Fn(u32) -> Timer::Time> CountDown for RTLTimer<Timer, TimeConverter> {
    type Time = u32;
    fn start<T: Into<Self::Time>>(&mut self, count: T) {
        self.timer.start((self.converter)(count.into()))
    }
    fn wait(&mut self) -> nb::Result<(), void::Void> {
        self.timer.wait()
    }
}


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
    clocks.configure_gclk_divider_and_source(
        hal::clock::ClockGenId::GCLK0,
        1,
        hal::clock::ClockSource::DPLL0,
        false,
    );
    let gclk0 = clocks.gclk0();
    let gclk5 = clocks.get_gclk(pac::gclk::pchctrl::GEN_A::GCLK5).unwrap();
    

    let mut pins = Pins::new(peripherals.PORT);
    let mut delay = Delay::new(core.SYST, &mut clocks);

    let mut led = pins.user_led.into_push_pull_output(&mut pins.port);

    let lcd_cs = pins.lcd_cs.into_push_pull_output(&mut pins.port);
    let lcd_dc = pins.lcd_dc.into_push_pull_output(&mut pins.port);
    let lcd_reset = pins.lcd_reset.into_push_pull_output(&mut pins.port);
    let mut lcd_backlight_ctr = pins.lcd_backlight_ctr.into_push_pull_output(&mut pins.port);
    let spi_lcd = spi_master_lcd(
        &mut clocks,
        KiloHertz(60000u32),
        peripherals.SERCOM7,
        &mut peripherals.MCLK,
        pins.lcd_sck,
        pins.lcd_mosi,
        pins.lcd_miso,
        &mut pins.port,
    );

    let rtl_cs = pins.rtl8720d_hspi_cs.into_push_pull_output(&mut pins.port);
    let mut rtl_chip_pu = pins.rtl8720d_chip_pu.into_push_pull_output(&mut pins.port);
    let rtl_irq0 = pins.irq0.into_floating_input(&mut pins.port);
    let rtl_sync = pins.sync.into_floating_input(&mut pins.port);
    let spi_rtl = spi_master_rtl8720(
        &mut clocks,
        KiloHertz(60000u32),
        peripherals.SERCOM0,
        &mut peripherals.MCLK,
        pins.rtl8720d_hspi_clk,
        pins.rtl8720d_hspi_mosi,
        pins.rtl8720d_hspi_miso,
        &mut pins.port,
    );
    let tc2_tc3 = clocks.tc2_tc3(&gclk5).unwrap();
    let rtl_timer = TimerCounter::tc2_(&tc2_tc3, peripherals.TC2, &mut peripherals.MCLK);
    let mut rtl_timer = RTLTimer::new(rtl_timer, |v| Nanoseconds(v * 1000000));
    rtl_timer.start(1000u32);
    let rtl_config: atat::Config = atat::Config::default();

    rtl_chip_pu.set_low();
    delay.delay_ms(100u16);
    rtl_chip_pu.set_high();
    let mut rtl_transport = WioTerminalSPITransport::new(spi_rtl, rtl_cs, rtl_irq0, rtl_sync);

    static mut RES_QUEUE: ResQueue<consts::U256, consts::U5> = Queue(heapless::i::Queue::u8());
    static mut URC_QUEUE: UrcQueue<consts::U256, consts::U10> = Queue(heapless::i::Queue::u8());
    static mut COM_QUEUE: ComQueue<consts::U3> = Queue(heapless::i::Queue::u8());
    let queues = Queues {
        res_queue: unsafe { RES_QUEUE.split() },
        urc_queue: unsafe { URC_QUEUE.split() },
        com_queue: unsafe { COM_QUEUE.split() },
    };
    let (mut tx, mut rx) = rtl_transport.split();
    let (mut client, ingress) =
        ClientBuilder::new(tx, rtl_timer, atat::Config::new(atat::Mode::Timeout))
            .with_custom_urc_matcher(NoopUrcMatcher{})
            .build(queues);
    
    let send_success = match client.send(&AT) {
        Ok(_response) => true,
        Err(_e) => false,
    };

    // let dmac = Dmac::new(peripherals.DMAC, &mut peripherals.MCLK);
    // let channel = dmac.allocate_channel(0);
    // let source = RefCell::new([0xaau8; 1024]);
    // let mut destination = RefCell::new([0u8; 1024]);

    // let transfer = channel.transfer_mem(source.borrow(), destination.borrow_mut(), 1024).unwrap();
    // let channel = loop {
    //     match transfer.poll() {
    //         OperationStatus::Running => {},
    //         OperationStatus::Completed => break transfer.get_result(),
    //     }
    // };

    let mut lcd = Ili9341::new_spi(spi_lcd, lcd_cs, lcd_dc, lcd_reset, &mut delay).unwrap();
    lcd.set_orientation(Orientation::LandscapeFlipped);

    lcd_backlight_ctr.set_high();

    let c = Circle::new(Point::new(160, 120), 20)
        .into_styled(PrimitiveStyle::with_fill(Rgb565::BLUE));
    let t = Text::new("Hello, Rust!", Point::new(0, 0))
        .into_styled(TextStyle::new(Font12x16, Rgb565::BLUE));
    c.draw(&mut lcd);
    if send_success {
        t.draw(&mut lcd);
    }
    loop {
        delay.delay_ms(1000u16);
        led.toggle();
    }
}
