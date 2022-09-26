#![no_std]
#![no_main]

use core::fmt::Write;
use cortex_m::delay::Delay;
use cortex_m::singleton;
use panic_rtt_target as _;

use stm32f1xx_hal::{prelude::*, pac};
use cortex_m_rt::entry;
use dht11::Dht11;
use embedded_hal::blocking::delay;
use nb::block;
use stm32f1xx_hal::dma::Half;
use stm32f1xx_hal::gpio::{CRH, Output, Pin, PushPull};
use stm32f1xx_hal::pac::USART2;
use stm32f1xx_hal::serial::{Config, Event, Rx, Rx2, Serial, StopBits, Tx};
use stm32f1xx_hal::time::{Hertz};
use stm32f1xx_hal::pac::interrupt;
use rtt_target::{rtt_init_print, rprintln, rprint};

const CLOCK_SPEED: u32 = 16 * 1000 * 1000;

static mut RX: Option<Rx<USART2>> = None;
static mut TX: Option<Tx<USART2>> = None;
static mut LED: Option<Pin<Output<PushPull>, CRH, 'C', 13>> = None;

// Определяем входную функцию.
#[entry]
fn main() -> ! {
    rtt_init_print!();
    // Получаем управление над аппаратными средствами
    let cp = cortex_m::Peripherals::take().unwrap();
    let dp = pac::Peripherals::take().unwrap();
    let mut flash = dp.FLASH.constrain();
    let mut rcc = dp.RCC.constrain();

    let mut afio = dp.AFIO.constrain();
    let mut gpioc = dp.GPIOC.split();
    let mut gpioa = dp.GPIOA.split();

    let tx = gpioa.pa2.into_alternate_push_pull(&mut gpioa.crl);
    let rx = gpioa.pa3;

    let mut led = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);
    led.set_high();

    let clocks = rcc.cfgr.sysclk(Hertz::from_raw(CLOCK_SPEED)).freeze(&mut flash.acr);
    let mut delay = Delay::new(cp.SYST, CLOCK_SPEED);
    let mut serial = Serial::usart2(
        dp.USART2,
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

    cortex_m::interrupt::free(|_| unsafe {
        TX.replace(tx1);
        RX.replace(rx1);
        LED.replace(led);
    });

    unsafe {
        cortex_m::peripheral::NVIC::unmask(pac::Interrupt::USART2);
    }

    unsafe {
        write("AT+CIPMUX=1\r\n".as_bytes());
        write("AT+CIPSERVER=1\r\n".as_bytes())
    }

    loop {
        cortex_m::asm::wfi()
    }
}

const BUFFER_LEN: usize = 4096;
static mut BUFFER: &mut [u8; BUFFER_LEN] = &mut [0; BUFFER_LEN];
static mut WIDX: usize = 0;

unsafe fn write(buf: &[u8]) {
    if let Some(tx) = TX.as_mut() {
        buf.iter()
            .for_each(|w| if let Err(_err) = nb::block!(tx.write(*w)) {})
    }
}

#[interrupt]
unsafe fn USART2() {
    cortex_m::interrupt::free(|_| {
        if let Some(led) = LED.as_mut() {
            if let Some(rx) = RX.as_mut() {
                if rx.is_rx_not_empty() {
                    if let Ok(w) = nb::block!(rx.read()) {
                        BUFFER[WIDX] = w;
                        WIDX += 1;
                    }
                } else {
                    for (i, b) in (&mut BUFFER[0..WIDX]).iter().enumerate() {
                        let byte = *b as char;
                        rprint!("{}", byte);
                        if byte == '&' {
                            rprintln!("LED ON");
                            led.set_low();
                        }
                        if byte == '#' {
                            rprintln!("LED OFF");
                            led.set_high();
                        }
                    }

                    WIDX = 0;
                }
            }
        }
    })
}