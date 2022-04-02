#![no_std]
#![no_main]

// pick a panicking behavior
use panic_halt as _; // you can put a breakpoint on `rust_begin_unwind` to catch panics
// use panic_abort as _; // requires nightly
// use panic_itm as _; // logs messages over ITM; requires ITM support
// use panic_semihosting as _; // logs messages to the host stderr; requires a debugger

use cortex_m_rt::entry;
use cortex_m_semihosting::hprintln;

mod g4test;
mod led;

#[entry]
fn main() -> ! {
    use led::Led;
    use stm32g4::stm32g431;

    // hprintln!("Hello, STM32G4!").unwrap();
    // stm32f401モジュールより、ペリフェラルの入り口となるオブジェクトを取得する。
    let perip = stm32g431::Peripherals::take().unwrap();
    let led0 = g4test::Led0::new(&perip);
    let led1 = g4test::Led1::new(&perip);

    led0.init();
    led1.init();
    loop {
        loop {
            // hprintln!("Set Led High").unwrap();
            for _ in 0..20_000 {
                led0.on();
                led1.on();
            }
            // hprintln!("Set Led Low").unwrap();
            for _ in 0..20_000 {
                led0.off();
                led1.off();
            }
        }
    }
}