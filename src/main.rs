#![deny(unsafe_code)]
#![no_std]
#![no_main]

use panic_rtt_target as _;

#[rtic::app(device = stm32f1xx_hal::pac)]
mod app {
    use core::fmt::Write;
    use cortex_m::prelude::_embedded_hal_serial_Read;
    use embedded_hal::digital::v2::OutputPin;

    use rtt_target::{rprint, rprintln, rtt_init_print};
    use stm32f1xx_hal::afio::AfioExt;
    use stm32f1xx_hal::flash::FlashExt;
    use stm32f1xx_hal::gpio::{CRH, GpioExt, Output, Pin, PushPull};
    use stm32f1xx_hal::pac::{Interrupt, USART2};
    use stm32f1xx_hal::rcc::RccExt;
    use stm32f1xx_hal::serial::{Config, Rx, Serial, StopBits};
    use stm32f1xx_hal::time::{Hertz, U32Ext};
    use heapless::spsc::{Consumer, Producer, Queue};

    #[shared]
    struct Shared {
        p: Producer<'static, u32, 5>,
        c: Consumer<'static, u32, 5>,
        rx1: Option<Rx<USART2>>,
        led: Option<Pin<Output<PushPull>, CRH, 'C', 13>>
    }

    #[local]
    struct Local {}

    #[init(local = [q: Queue<u32, 5> = Queue::new()])]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        rtt_init_print!();
        // Cortex-M peripherals
        let _core: cortex_m::Peripherals = cx.core;

        // Device specific peripherals
        let _device: stm32f1xx_hal::pac::Peripherals = cx.device;

        let mut flash = _device.FLASH.constrain();
        let rcc = _device.RCC.constrain();

        let mut afio = _device.AFIO.constrain();
        let mut gpioc = _device.GPIOC.split();
        let mut gpioa = _device.GPIOA.split();

        let tx = gpioa.pa2.into_alternate_push_pull(&mut gpioa.crl);
        let rx = gpioa.pa3;

        let mut led = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);
        led.set_high();

        let clocks = rcc.cfgr.sysclk(Hertz::from_raw( 16 * 1000 * 1000)).freeze(&mut flash.acr);

        let serial = Serial::usart2(
            _device.USART2,
            (tx, rx),
            &mut afio.mapr,
            Config::default()
                .baudrate(115200.bps())
                .stopbits(StopBits::STOP1)
                .parity_none(),
            clocks,
        );
        let (mut tx1, mut rx1) = serial.split();

        tx1.listen();
        rx1.listen();
        rx1.listen_idle();

        tx1.write_str("AT+CIPMUX=1\r\n").unwrap();
        tx1.write_str("AT+CIPSERVER=1\r\n").unwrap();

        let (p, c) = cx.local.q.split();

        rtic::pend(Interrupt::USART2);

        rprintln!("init");

        (Shared { p, c, rx1: (Option::Some(rx1)), led: (Option::Some(led)) }, Local {}, init::Monotonics())
    }

    #[idle(shared = [c])]
    fn idle(_: idle::Context) -> ! {
        loop {
            cortex_m::asm::nop();
        }
    }

    #[task(binds = USART2, shared = [p, rx1, led])]
    fn usart2_interrupt(mut cx: usart2_interrupt::Context) {
        cx.shared.rx1.lock(|rx: _| {
            if let Some(rx) = rx.as_mut() {
                if rx.is_rx_not_empty() {
                    if let Ok(w) = nb::block!(rx.read()) {
                        let byte = w as char;
                        rprint!("{}", byte);
                        if byte == '&' {
                            rprintln!("LED ON");
                            cx.shared.led.lock(|led1: _| {
                                if let Some(l) = led1.as_mut() {
                                    l.set_low();
                                }
                            })
                        }
                        if byte == '#' {
                            rprintln!("LED OFF");
                            cx.shared.led.lock(|led1: _| {
                                if let Some(l) = led1.as_mut() {
                                    l.set_high();
                                }
                            })
                        }
                    }
                }
            }
        })

    }

}