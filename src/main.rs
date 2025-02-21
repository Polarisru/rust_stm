#![deny(unsafe_code)]
#![no_std]
#![no_main]

use panic_halt as _;


use stm32g0xx_hal as hal;
use hal::prelude::*;
use hal::rcc::Config;
use hal::stm32;

use cortex_m_rt::entry;

#[entry]
fn main() -> ! {
    let dp = stm32::Peripherals::take().expect("cannot take peripherals");
    let mut rcc = dp.RCC.freeze(Config::lsi());
    let mut delay = dp.TIM16.delay(&mut rcc);

    let gpioa = dp.GPIOA.split(&mut rcc);
    let mut led = gpioa.pa4.into_push_pull_output();

    loop {
        led.toggle().unwrap();
        delay.delay(500.ms());
    }
}
