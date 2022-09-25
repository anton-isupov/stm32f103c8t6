#![deny(unsafe_code)]
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
use rtt_target::{rprint, rprintln, rtt_init_print};
use stm32f1xx_hal::dma::Half;
use stm32f1xx_hal::serial::{Config, Event, Rx, Rx2, Serial, StopBits};
use stm32f1xx_hal::time::{Hertz};

const CLOCK_SPEED: u32 = 16 * 1000 * 1000;

// Определяем входную функцию.
#[entry]
fn main() -> ! {
    rtt_init_print!();
    rprintln!("Initializing...");

    // Получаем управление над аппаратными средствами
    let cp = cortex_m::Peripherals::take().unwrap();
    let dp = pac::Peripherals::take().unwrap();
    let mut flash = dp.FLASH.constrain();
    let mut rcc = dp.RCC.constrain();

    let mut afio = dp.AFIO.constrain();
    let mut gpiob = dp.GPIOB.split();
    let mut gpioc = dp.GPIOC.split();
    let mut gpioa = dp.GPIOA.split();

    let channels = dp.DMA1.split();

    let tx = gpioa.pa2.into_alternate_push_pull(&mut gpioa.crl);
    let rx = gpioa.pa3;

    let pin = gpiob.pb0.into_open_drain_output(&mut gpiob.crl);
    let led = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);

    rprintln!("Initialized pins");

    let clocks = rcc.cfgr.sysclk(Hertz::from_raw(CLOCK_SPEED)).freeze(&mut flash.acr);
    let mut delay = Delay::new(cp.SYST, CLOCK_SPEED);
    let mut dht11 = Dht11::new(pin);

    rprintln!("Initialized clocks and DHT11");

    let mut serial = Serial::usart2(
        dp.USART2,
        (tx, rx),
        &mut afio.mapr,
        Config::default()
            .baudrate(115200.bps())
            .stopbits(StopBits::STOP1)
            .parity_none(),
        clocks
    );

    let (mut tx1, mut rx1) = serial.split();

    let res = tx1.write_str("AT+CIFSR\r\n").unwrap();
    rprintln!("Written AT+CIFSR command");
    rx1 = read_ok_msg(rx1);

    let res = tx1.write_str("AT+GMR\r\n").unwrap();
    rprintln!("Written AT+GMR command");
    rx1 = read_ok_msg(rx1);

    // Uncomment for first usage. Start TCP server TODO
/*    let res = tx1.write_str(" AT+CIPMUX=1\r\n").unwrap();
    rprintln!("Written AT+CIPMUX=1 command");
    rx1 = read_ok_msg(rx1);

    let res = tx1.write_str("AT+CIPSERVER=1\r\n").unwrap();
    rprintln!("Written AT+CIPSERVER=1 command");
    rx1 = read_ok_msg(rx1);*/

    let res = tx1.write_str("AT+CWSAP?\r\n").unwrap();
    rprintln!("Written AT+CWSAP? command");
    rx1 = read_ok_msg(rx1);

    let res = tx1.write_str("AT+CIPSTATUS\r\n").unwrap();
    rprintln!("Written AT+CIPSTATUS command");
    rx1 = read_ok_msg(rx1);

    rprintln!("Start infinite loop");
    loop {
        let byte_result = block!(rx1.read());
        if byte_result.is_err() {
            rprintln!("error"); //TODO Overrun error
        } else {
            let byte = byte_result.unwrap();
            rprint!("{}", byte as char);
            if byte as char == '!' {
                rprintln!();
                let res = tx1.write_str("AT+CIPSEND=0,3\r\n").unwrap();
                rprintln!("Written AT+CIPSEND=0,3 command");
                rx1 = read_ok_msg(rx1);
                let res = tx1.write_str("Hi!").unwrap();
                rx1 = read_ok_msg(rx1);
                break;
            }
        }
    }

    // The DHT11 datasheet suggests 1 second
    rprintln!("Waiting on the sensor...");
    delay.delay_ms(1000_u32);

    loop {
        match dht11.perform_measurement(&mut delay) {
            Ok(meas) => {
                rprintln!("Measurement {}, {}", meas.temperature, meas.humidity);
            }
            Err(e) => {
                rprintln!("Error {:?}", e);
            }
        };
        // Delay of at least 500ms before polling the sensor again, 1 second or more advised
        delay.delay_ms(2000_u32);
    }
}

fn read_ok_msg(mut rx1: Rx2) -> Rx2 {
    let mut is_o = false;
    let mut is_k = false;

    loop {
        let byte_result = block!(rx1.read());
        if byte_result.is_err() {
            rprintln!("error");  //TODO Overrun error
        } else {
            let byte = byte_result.unwrap();
            rprint!("{}", byte as char);
            if byte as char == 'O' {
                is_o = true;
            } else if byte as char == 'K' {
                is_k = true;
            }
            if is_o && is_k {
                break;
            }
        }
    }
    rprintln!();
    return rx1;
}