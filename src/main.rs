#![no_std]
#![no_main]

// pick a panicking behavior
use panic_halt as _; // you can put a breakpoint on `rust_begin_unwind` to catch panics
                     // use panic_abort as _; // requires nightly
                     // use panic_itm as _; // logs messages over ITM; requires ITM support
                     // use panic_semihosting as _; // logs messages to the host stderr; requires a debugger

use cortex_m_rt::entry;
use cortex_m_semihosting::hprintln;
use core::fmt::Write;

mod g4test;
mod led;
mod potensio;


mod app {
    use crate::led::Led;

    pub struct App<'a> {
        led0: &'a dyn Led,
        led1: &'a dyn Led,
    }

    impl<'a> App<'a> {
        pub fn new(led0: &'a dyn Led, led1: &'a dyn Led) -> Self {
            Self { led0, led1 }
        }
        pub fn periodic_task(&self) {
            self.led0.toggle();
        }
    }
}

#[entry]
fn main() -> ! {
    use stm32g4::stm32g431;

    // hprintln!("Hello, STM32G4!").unwrap();
    // stm32f401モジュールより、ペリフェラルの入り口となるオブジェクトを取得する。
    let perip = stm32g431::Peripherals::take().unwrap();
    let led0 = g4test::Led0::new(&perip);
    let led1 = g4test::Led1::new(&perip);

    let app = app::App::new(&led0, &led1);
    g4test::clock_init(&perip);
    let mut uart = g4test::Uart0::new(&perip);

    let potensio0 = g4test::Potensio0::new(&perip);

    let mut t = perip.TIM3.cnt.read().cnt().bits();
    let mut prev = t;
    // hprintln!("t: {}", t).unwrap();
    let mut cnt = 0;
    loop {
        t = perip.TIM3.cnt.read().cnt().bits();
        if t.wrapping_sub(prev) > 10000 {
            cnt += 1;
            // hprintln!("t: {}", t).unwrap();
            if cnt > 0 {
                app.periodic_task();

                uart.write_str("hello");
                write!(uart, "{} + {} = {}\r\n", 2, 3, 2+3);
                write!(uart, "{} \r\n", potensio0.sigle_conversion());
                cnt = 0;
            }
            prev = t;
            // hprintln!("next: {}", last_t.wrapping_add(1000)).unwrap();
        }
    }

}
