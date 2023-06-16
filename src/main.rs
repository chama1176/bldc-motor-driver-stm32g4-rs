#![no_std]
#![no_main]

// pick a panicking behavior
use panic_halt as _; // you can put a breakpoint on `rust_begin_unwind` to catch panics
                     // use panic_abort as _; // requires nightly
                     // use panic_itm as _; // logs messages over ITM; requires ITM support
                     // use panic_semihosting as _; // logs messages to the host stderr; requires a debugger

use core::fmt::Write;
use cortex_m_rt::entry;
use cortex_m_semihosting::hprintln;

mod fsr;
mod imu_fsr_stm32g4;
mod indicator;
mod potensio;

mod app {
    use crate::indicator::Indicator;

    pub struct App<'a> {
        led0: &'a dyn Indicator,
        led1: &'a dyn Indicator,
        led2: &'a dyn Indicator,
    }

    impl<'a> App<'a> {
        pub fn new(
            led0: &'a dyn Indicator,
            led1: &'a dyn Indicator,
            led2: &'a dyn Indicator,
        ) -> Self {
            Self { led0, led1, led2 }
        }
        pub fn periodic_task(&self) {
            self.led0.toggle();
            self.led1.toggle();
            self.led2.toggle();
        }
    }
}

// static adc_data:[u16; 4] = [7; 4];

#[entry]
fn main() -> ! {
    use stm32g4::stm32g431;

    // hprintln!("Hello, STM32G4!").unwrap();
    // stm32f401モジュールより、ペリフェラルの入り口となるオブジェクトを取得する。
    let perip = stm32g431::Peripherals::take().unwrap();
    let mut core_perip = stm32g431::CorePeripherals::take().unwrap();
    let led0 = imu_fsr_stm32g4::Led0::new(&perip);
    let led1 = imu_fsr_stm32g4::Led1::new(&perip);
    let led2 = imu_fsr_stm32g4::Led2::new(&perip);

    let app = app::App::new(&led0, &led1, &led2);
    imu_fsr_stm32g4::clock_init(&perip);
    imu_fsr_stm32g4::adc2_init(&perip);

    let adc_data: [u16; 4] = [7; 4];
    let dma_buf_addr: u32 = adc_data.as_ptr() as u32;
    imu_fsr_stm32g4::dma_init(&perip, &mut core_perip, dma_buf_addr);
    let mut uart = imu_fsr_stm32g4::Uart3::new(&perip);
    imu_fsr_stm32g4::dma_adc2_start(&perip);

    // let potensio0 = imu_fsr_stm32g4::Potensio0::new(&perip);

    let mut t = perip.TIM3.cnt.read().cnt().bits();
    let mut prev = t;
    // hprintln!("t: {}", t).unwrap();
    let mut cnt = 0;
    let mut adc_data_fir: [u16; 4] = [0; 4];
    loop {
        t = perip.TIM3.cnt.read().cnt().bits();
        if t.wrapping_sub(prev) > 50 {
            for i in 0..4 {
                adc_data_fir[i] = (adc_data_fir[i] as f32 * 0.9 + adc_data[i] as f32 * 0.1) as u16;
            }
            cnt += 1;
            // hprintln!("t: {}", t).unwrap();
            if cnt > 100 {
                app.periodic_task();

                // uart.write_str("hello ");
                // write!(uart, "{} + {} = {}\r\n", 2, 4, 2+4);
                unsafe {
                    write!(
                        uart,
                        "{{\"FSR\":[{}, {}, {}, {}]}}\r\n",
                        adc_data_fir[3], adc_data_fir[0], adc_data_fir[1], adc_data_fir[2]
                    );
                }
                // write!(uart, "{} \r\n", potensio0.sigle_conversion());
                cnt = 0;
            }
            prev = t;
            // hprintln!("next: {}", last_t.wrapping_add(1000)).unwrap();
        }
    }
}
