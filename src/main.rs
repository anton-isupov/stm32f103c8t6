#![deny(unsafe_code)]
#![no_std]
#![no_main]

use panic_rtt_target as _;

#[rtic::app(device = stm32f1xx_hal::pac, dispatchers = [WWDG])]
mod app {
    use core::fmt::Write;
    use cortex_m::delay::Delay;
    use cortex_m::prelude::_embedded_hal_serial_Read;
    use embedded_hal::digital::ToggleableOutputPin;
    use embedded_hal::digital::v2::OutputPin;

    use rtt_target::{rprint, rprintln, rtt_init_print};
    use stm32f1xx_hal::afio::AfioExt;
    use stm32f1xx_hal::flash::FlashExt;
    use stm32f1xx_hal::gpio::{CRH, GpioExt, Output, Pin, PushPull};
    use stm32f1xx_hal::pac::{Interrupt, USART2};
    use stm32f1xx_hal::rcc::RccExt;
    use stm32f1xx_hal::serial::{Config, Rx, Serial, StopBits, Tx};
    use stm32f1xx_hal::time::{Hertz, U32Ext};
    use heapless::spsc::{Consumer, Producer, Queue};
    use stm32f1xx_hal::prelude::_fugit_ExtU32;

    use systick_monotonic::*;
    use systick_monotonic::fugit::ExtU32; // Implements the `Monotonic` trait

    // A monotonic timer to enable scheduling in RTIC
    #[monotonic(binds = SysTick, default = true)]
    type MonotonicTimer = Systick<1000>; // 1000 Hz / 1 ms granularity

    #[shared]
    struct Shared {
        producer: Producer<'static, u8, 1024>,
        consumer: Consumer<'static, u8, 1024>,
        rx1: Option<Rx<USART2>>,
        tx1: Option<Tx<USART2>>,
        led: Option<Pin<Output<PushPull>, CRH, 'C', 13>>,
    }

    #[local]
    struct Local {}

    #[init(local = [q: Queue < u8, 1024 > = Queue::new()])]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        rtt_init_print!();
        rprintln!("Start initializing...");
        // Cortex-M peripherals
        let _core: cortex_m::Peripherals = cx.core;

        // Device specific peripherals
        let _device: stm32f1xx_hal::pac::Peripherals = cx.device;

        // Initialize the monotonic (SysTick rate in QEMU is 72 MHz)
        let mono = Systick::new(_core.SYST, 72_000_000);

        let mut flash = _device.FLASH.constrain();
        let rcc = _device.RCC.constrain();

        let mut afio = _device.AFIO.constrain();
        let mut gpioc = _device.GPIOC.split();
        let mut gpioa = _device.GPIOA.split();

        let tx = gpioa.pa2.into_alternate_push_pull(&mut gpioa.crl);
        let rx = gpioa.pa3;

        let mut led = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);
        led.set_high();

        let clocks = rcc.cfgr.sysclk(Hertz::from_raw(16 * 1000 * 1000)).freeze(&mut flash.acr);

        let (producer, consumer) = cx.local.q.split();

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

        command_processor::spawn_after(systick_monotonic::ExtU64::millis(1)).unwrap();

        tx1.listen();
        rx1.listen();
        rx1.listen_idle();

        tx1.write_str("AT+CIPMUX=1\r\n").unwrap();
        tx1.write_str("AT+CIPSERVER=1\r\n").unwrap();
        tx1.write_str("AT+CIPSTATUS\r\n").unwrap();

        rtic::pend(Interrupt::USART2);

        rprintln!("Initialized complete");

        (Shared { producer, consumer, rx1: (Option::Some(rx1)), tx1: (Option::Some(tx1)), led: (Option::Some(led)) }, Local {}, init::Monotonics(mono))
    }

    #[task(local = [buffer: [char; 128] = ['.'; 128], widx: usize = 0, cmd: bool = false], shared = [consumer])]
    fn command_processor(mut cx: command_processor::Context) {
        const LED_TOGGLE_CMD: [char; 4] = ['T', 'G', 'G', 'L'];
        const SEND_CMD: [char; 4] = ['S', 'E', 'N', 'D'];
        let mut buffer = &mut cx.local.buffer;
        let mut widx = cx.local.widx;
        let mut cmd = cx.local.cmd;
        if *cmd && *widx == 4 {
            if buffer[0..*widx] == LED_TOGGLE_CMD {
                led_toggle::spawn().unwrap();
            }
            *widx = 0;
        }
        if let Some(byte) = cx.shared.consumer.lock(|c| c.dequeue()) {
            if *widx == 128 {
                *widx = 0;
            }
            rprint!("{}", byte as char);
            buffer[*widx] = byte as char;
            *widx += 1;
            if byte as char == '!' {
                *cmd = true;
                *widx = 0;
            } else if byte as char == '%' {
                *cmd = false;
                *widx = 0;
            }
        } else {
            cortex_m::asm::nop();
        }
        command_processor::spawn_after(systick_monotonic::ExtU64::millis(1)).unwrap();
    }

    #[task(shared = [led])]
    fn led_toggle(mut cx: led_toggle::Context) {
        cx.shared.led.lock(|led1: _| {
            if let Some(l) = led1.as_mut() {
                l.toggle();
            }
        })
    }

    #[task(binds = USART2, shared = [producer, rx1])]
    fn usart2_receiver(mut cx: usart2_receiver::Context) {
        cx.shared.rx1.lock(|rx: _| {
            if let Some(rx) = rx.as_mut() {
                if rx.is_rx_not_empty() {
                    if let Ok(w) = nb::block!(rx.read()) {
                        rprint!("{}", w as char);
                        cx.shared.producer.lock(|p| p.enqueue(w).unwrap());
                    }
                }
            }
        })
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            cortex_m::asm::nop();
        }
    }
}