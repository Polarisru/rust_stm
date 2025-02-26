#![allow(unsafe_code)]
#![no_std]
#![no_main]

use panic_halt as _;


use stm32g0xx_hal as hal;
use hal::prelude::*;
use hal::rcc::Config;
use hal::stm32;

use cortex_m_rt::entry;
use cortex_m::Peripherals;
use cortex_m::peripheral::{syst::SystClkSource, SYST};

/// Delay abstraction using SysTick
pub struct Delay {
    systick: SYST,
    clock_freq: u32,
}

impl Delay {
    /// Create new delay instance using the SysTick peripheral
    /// 
    /// # Parameters
    /// * `systick` - SysTick peripheral instance
    /// * `clock_freq` - System clock frequency in Hz (default is 16MHz for STM32G030)
    pub fn new(mut systick: SYST, clock_freq: u32) -> Self {
        // Configure SysTick to use processor clock
        systick.set_clock_source(SystClkSource::Core);
        
        // Disable SysTick counter and interrupt
        systick.disable_counter();
        systick.disable_interrupt();
        
        Self { systick, clock_freq }
    }
    
    /// Delay for the given number of milliseconds
    pub fn delay_ms(&mut self, ms: u32) {
        // Calculate the number of ticks needed
        let ticks = self.clock_freq / 1000 * ms;
        self.delay_ticks(ticks);
    }
    
    /// Delay for the given number of microseconds
    pub fn delay_us(&mut self, us: u32) {
        // Calculate the number of ticks needed
        let ticks = self.clock_freq / 1_000_000 * us;
        if ticks > 0 {
            self.delay_ticks(ticks);
        } else {
            // For very short delays, we need at least 1 tick
            self.delay_ticks(1);
        }
    }
    
    /// Delay for the specified number of CPU ticks
    fn delay_ticks(&mut self, ticks: u32) {
        // The SysTick timer is 24-bit, so we might need multiple iterations
        const MAX_TICKS: u32 = 0x00FF_FFFF;
        
        let mut remaining = ticks;
        
        while remaining > 0 {
            // Load however many ticks we need, up to the maximum
            let current_ticks = if remaining > MAX_TICKS {
                MAX_TICKS
            } else {
                remaining
            };
            
            // Set reload value (note: value is 1 less than ticks needed)
            self.systick.set_reload(current_ticks - 1);
            
            // Clear current value
            self.systick.clear_current();
            
            // Enable counter
            self.systick.enable_counter();
            
            // Wait for the counter to reach zero
            while !self.systick.has_wrapped() {}
            
            // Disable counter after use
            self.systick.disable_counter();
            
            // Subtract from remaining ticks
            remaining -= current_ticks;
        }
    }
}

#[entry]
fn main() -> ! {
    // Get access to the device peripherals
    let cp = Peripherals::take().unwrap();
    let dp = stm32::Peripherals::take().unwrap();

    // Create delay instance (16MHz default clock)
    let mut delay = Delay::new(cp.SYST, 16_000_000);

    // Get RCC and GPIOA peripherals
    let rcc = &dp.RCC;
    let gpioa = &dp.GPIOA;

    // Enable GPIOA clock
    rcc.iopenr.modify(|_, w| w.iopaen().set_bit());

    // Configure PA4 as output (01 in MODER register)
    unsafe {
        gpioa.moder.modify(|_, w| w.moder4().bits(0b01));
    }

    loop {
        // Toggle the LED
        if gpioa.odr.read().odr4().bit_is_set() {
            // Turn off the LED
            gpioa.bsrr.write(|w| w.br4().set_bit());
        } else {
            // Turn on the LED
            gpioa.bsrr.write(|w| w.bs4().set_bit());
        }

        // Delay for 500ms
        delay.delay_ms(500);
    }
}
