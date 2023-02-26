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
mod indicator;
mod potensio;
mod fsr;


mod app {
    use crate::indicator::Indicator;

    pub struct App<'a> {
        led0: &'a dyn Indicator,
        led1: &'a dyn Indicator,
    }

    impl<'a> App<'a> {
        pub fn new(led0: &'a dyn Indicator, led1: &'a dyn Indicator) -> Self {
            Self { led0, led1 }
        }
        pub fn periodic_task(&self) {
            self.led0.toggle();
        }
    }
}

static adc_data:[u16; 4] = [7; 4];

#[entry]
fn main() -> ! {
    use stm32g4::stm32g431;

    // hprintln!("Hello, STM32G4!").unwrap();
    // stm32f401モジュールより、ペリフェラルの入り口となるオブジェクトを取得する。
    let perip = stm32g431::Peripherals::take().unwrap();
    let mut core_perip = stm32g431::CorePeripherals::take().unwrap();
    let led0 = g4test::Led0::new(&perip);
    let led1 = g4test::Led1::new(&perip);

    let app = app::App::new(&led0, &led1);
    g4test::clock_init(&perip);
    g4test::adc2_init(&perip);
    g4test::dma_init(&perip, &mut core_perip, &adc_data);
    let mut uart = g4test::Uart0::new(&perip);
    g4test::dma_adc2_start(&perip);

    // let potensio0 = g4test::Potensio0::new(&perip);

    let mut t = perip.TIM3.cnt.read().cnt().bits();
    let mut prev = t;
    // hprintln!("t: {}", t).unwrap();
    let mut cnt = 0;
    let mut index = 0;
    loop {
        t = perip.TIM3.cnt.read().cnt().bits();
        if t.wrapping_sub(prev) > 1000 {
            cnt += 1;
            // hprintln!("t: {}", t).unwrap();
            if cnt > 0 {
                app.periodic_task();

                uart.write_str("hello ");
                write!(uart, "{} + {} = {}\r\n", 2, 4, 2+4);
                unsafe{
                    write!(uart, "{}, {}, {}, {}\r\n", adc_data[0], adc_data[1], adc_data[2], adc_data[3]);
                }
                // write!(uart, "{} \r\n", potensio0.sigle_conversion());
                cnt = 0;

                let adc = &perip.ADC2;
                adc.cr.modify(|_, w| w.adstart().start());   // ADC start
                // while adc.isr.read().eoc().is_not_complete() {
                //     // Wait for ADC complete
                // }
                // while adc.isr.read().eos().is_not_complete() {
                //     // Wait for ADC complete
                // }
                // adc.isr.modify(|_, w| w.eoc().clear());   // clear eoc flag
                // adc.isr.modify(|_, w| w.eos().clear());   // clear eoc flag
                write!(uart, "{}: {}\r\n", index%4, adc.dr.read().rdata().bits());
                index += 1;
            }
            prev = t;
            // hprintln!("next: {}", last_t.wrapping_add(1000)).unwrap();
        }
    }

}
