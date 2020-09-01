#![no_std]
#![no_main]


// pick a panicking behavior
use panic_halt as _; // you can put a breakpoint on `rust_begin_unwind` to catch panics
                     // use panic_abort as _; // requires nightly
                     // use panic_itm as _; // logs messages over ITM; requires ITM support
                     // use panic_semihosting as _; // logs messages to the host stderr; requires a debugger

use heapless::{consts::*, spsc::Queue};

use wio_terminal::hal::delay::Delay;
use wio_terminal::hal::timer::TimerCounter;
use wio_terminal::hal::prelude::*;
use wio_terminal::hal::{clock::GenericClockController, time::KiloHertz, time::Nanoseconds, time::Miliseconds};
use wio_terminal::{pac, gpio, hal, sercom, spi_master_lcd, spi_master_rtl8720, Pins};

use embedded_hal::timer::CountDown;


use embedded_graphics::{
    fonts::{Font12x16, Text},
    pixelcolor::{Rgb565},
    prelude::*,
    primitives::Circle,
    style::{PrimitiveStyle, TextStyle},
};
use ili9341::{Ili9341, Orientation, spi::SpiInterface};

mod esp_at;
mod atsamd_dmac;
mod async_operation;
use esp_at::WioTerminalSPITransport;


use atat::{prelude::*, atat_derive::*, ClientBuilder, ComQueue, Queues, ResQueue, UrcQueue, NoopUrcMatcher};

use rtic::app;

#[derive(Clone, AtatResp)]
pub struct NoResponse;

#[derive(Clone, AtatCmd)]
#[at_cmd("AT", NoResponse, timeout_ms = 1000)]
pub struct AT;

pub struct RTLTimer<Timer> {
    timer: Timer,
}
impl<Timer: CountDown<Time=Nanoseconds>> RTLTimer<Timer> {
    pub fn new(timer: Timer) -> Self {
        Self {
            timer: timer,
        }
    }
}

impl<Timer: CountDown<Time=Nanoseconds>> CountDown for RTLTimer<Timer> {
    type Time = u32;
    fn start<T: Into<Self::Time>>(&mut self, count: T) {
        self.timer.start(Nanoseconds(count.into()))
    }
    fn wait(&mut self) -> nb::Result<(), void::Void> {
        self.timer.wait()
    }
}

type SpiLcd = atsamd_hal::sercom::SPIMaster7<
    sercom::Sercom7Pad2<gpio::Pb18<gpio::PfD>>,
    sercom::Sercom7Pad3<gpio::Pb19<gpio::PfD>>,
    sercom::Sercom7Pad1<gpio::Pb20<gpio::PfD>>,
>;

type RtlSpiMaster = sercom::SPIMaster0<
    sercom::Sercom0Pad2<gpio::Pc24<gpio::PfC>>, 
    sercom::Sercom0Pad0<gpio::Pb24<gpio::PfC>>, 
    sercom::Sercom0Pad1<gpio::Pb25<gpio::PfC>>,
>;
type RtlSpiTransport = esp_at::WioTerminalSPITransport<
    'static,
    RtlSpiMaster, 
    gpio::Pc25<gpio::Output<gpio::PushPull>>, 
    gpio::Pc20<gpio::Input<gpio::Floating>>, 
    gpio::Pa19<gpio::Input<gpio::Floating>>
>;

type LcdSpi = SpiInterface<SpiLcd, 
    gpio::Pb21<gpio::Output<gpio::PushPull>>, 
    gpio::Pc6<gpio::Output<gpio::PushPull>>,
>;
type Lcd = Ili9341<
    LcdSpi, 
    gpio::Pc7<gpio::Output<gpio::PushPull>>
>;

type RtlAtClient = atat::Client<
    esp_at::WioTerminalSPITransportTx<'static, U256>, 
    RTLTimer<TimerCounter<atsamd51p19a::TC3>>,
>;

#[app(device = atsamd51p19a, peripherals = true)]
const APP: () = {
    struct Resources {
        lcd: Lcd,
        led: gpio::Pa15<gpio::Output<gpio::PushPull>>,
        rtl_transport: RtlSpiTransport,
        delay: Delay,
        client: RtlAtClient,
        ingress: atat::IngressManager<U256, atat::NoopUrcMatcher>,
    }
    #[init(spawn = [comm_loop])]
    fn init(context: init::Context) -> init::LateResources {
        let core = context.core;
        let mut peripherals = context.device;

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
        let mut pins = Pins::new(peripherals.PORT);
        let mut delay = Delay::new(core.SYST, &mut clocks);

        let gclk5 = clocks.get_gclk(pac::gclk::pchctrl::GEN_A::GCLK5).unwrap();
        
        // Timer2 for software tasks.
        let tc2_tc3 = clocks.tc2_tc3(&gclk5).unwrap();
        let mut swtask_timer = TimerCounter::tc2_(&tc2_tc3, peripherals.TC2, &mut peripherals.MCLK);
        swtask_timer.enable_interrupt();
        swtask_timer.start(Miliseconds(1000u32));

        // Initialize LED pin
        let led = pins.user_led.into_push_pull_output(&mut pins.port);
        
        // Initialize RTL8720D transport
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
        
        // Reset RTL8720D
        rtl_chip_pu.set_low();
        delay.delay_ms(100u16);
        rtl_chip_pu.set_high();

        static mut RTL_READ_QUEUE: esp_at::ReadQueue = Queue(heapless::i::Queue::u8());
        static mut RTL_WRITE_QUEUE: esp_at::WriteQueue = Queue(heapless::i::Queue::u8());
        let (read_producer, read_consumer) = unsafe {RTL_READ_QUEUE.split()};
        let (write_producer, write_consumer) = unsafe{RTL_WRITE_QUEUE.split()};
        let mut rtl_transport = WioTerminalSPITransport::new(spi_rtl, rtl_cs, rtl_irq0, rtl_sync, read_producer, write_consumer);
        
        // Initialize ATAT client for RTL8720D
        let rtl_timer = TimerCounter::tc3_(&tc2_tc3, peripherals.TC3, &mut peripherals.MCLK);
        let mut rtl_timer = RTLTimer::new(rtl_timer);
        rtl_timer.start(1000u32);
        
        let rtl_config: atat::Config = atat::Config::default();
            
        static mut RES_QUEUE: ResQueue<U256, U5> = Queue(heapless::i::Queue::u8());
        static mut URC_QUEUE: UrcQueue<U256, U10> = Queue(heapless::i::Queue::u8());
        static mut COM_QUEUE: ComQueue<U3> = Queue(heapless::i::Queue::u8());
        let queues = Queues {
            res_queue: unsafe { RES_QUEUE.split() },
            urc_queue: unsafe { URC_QUEUE.split() },
            com_queue: unsafe { COM_QUEUE.split() },
        };
        let rx = esp_at::WioTerminalSPITransportRx::new(read_consumer);
        let tx = esp_at::WioTerminalSPITransportTx::new(write_producer);
        let (mut client, ingress) =
            ClientBuilder::new(tx, rtl_timer, atat::Config::new(atat::Mode::Timeout))
                .with_custom_urc_matcher(NoopUrcMatcher{})
                .build(queues);
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
        
        context.spawn.comm_loop().unwrap();
        
        // Initialize LCD
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

        let mut lcd = Ili9341::new_spi(spi_lcd, lcd_cs, lcd_dc, lcd_reset, &mut delay).unwrap();
        lcd.set_orientation(Orientation::LandscapeFlipped).unwrap();
        lcd_backlight_ctr.set_high().unwrap();

        init::LateResources {
            lcd: lcd,
            led: led,
            rtl_transport: rtl_transport,
            delay: delay,
            client: client,
            ingress: ingress,
        }
    }

    #[task(spawn = [comm_loop], resources = [rtl_transport])]
    fn comm_loop(context: comm_loop::Context) {
        context.resources.rtl_transport.process();

        context.spawn.comm_loop().unwrap();
    }

    #[task(spawn = [ui_loop], resources = [lcd, led])]
    fn ui_loop(context: ui_loop::Context) {
        static mut ui_interval: u32 = 0;
        static mut led_interval: u32 = 0;

        if *ui_interval == 0 {
            let c = Circle::new(Point::new(160, 120), 20)
            .into_styled(PrimitiveStyle::with_fill(Rgb565::BLUE));
            let t = Text::new("Hello, Rust!", Point::new(0, 0))
                .into_styled(TextStyle::new(Font12x16, Rgb565::BLUE));
            c.draw(context.resources.lcd).unwrap();
            t.draw(context.resources.lcd).unwrap();
        }
        *ui_interval = if *ui_interval < 33u32 { *ui_interval + 1u32 } else { 0 };

        
        if *led_interval == 0 {
            context.resources.led.toggle();
        }
        *led_interval = if *led_interval < 500u32 { *led_interval + 1u32 } else { 0 };
        
        context.spawn.ui_loop().unwrap();
    }

    extern "C" {
        fn TC0();
    }
};
