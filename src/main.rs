#![deny(unsafe_code)]
#![no_std]
#![no_main]

use cortex_m::delay::Delay;
use panic_rtt_target as _;

use nb::block;

use stm32f1xx_hal::{prelude::*, pac, timer::Timer, stm32};
use cortex_m_rt::entry;
use cortex_m_semihosting::{debug, hprintln};
use dht11::Dht11;
use embedded_hal::digital::v2::{InputPin, OutputPin};
use rtt_target::{rprintln, rtt_init_print};
use stm32f1xx_hal::adc::Adc;
use stm32f1xx_hal::delay;
use stm32f1xx_hal::time::MegaHertz;

const CLOCK_SPEED: u32 = 16;

// Определяем входную функцию.
#[entry]
fn main() -> ! {
    rtt_init_print!();
    rprintln!("Initialized 1");
    // Получаем управление над аппаратными средствами
    let cp = cortex_m::Peripherals::take().unwrap();
    let dp = pac::Peripherals::take().unwrap();
    let mut flash = dp.FLASH.constrain();
    let mut rcc = dp.RCC.constrain();


    let mut gpiob = dp.GPIOB.split(&mut rcc.apb2);
    let mut gpioc = dp.GPIOC.split(&mut rcc.apb2);

    let mut pin = gpiob.pb0.into_open_drain_output(&mut gpiob.crl);
    let mut led = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);

    rprintln!("Initialized 2");

    let mut clocks = rcc.cfgr.sysclk(CLOCK_SPEED.mhz()).freeze(&mut flash.acr);
    let mut delay = delay::Delay::new(cp.SYST, clocks);
    let mut dht11 = Dht11::new(pin);

    // The DHT11 datasheet suggests 1 second
    rprintln!("Waiting on the sensor...");
    delay.delay_ms(1000_u16);

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
        delay.delay_ms(2000_u16);
    }
}