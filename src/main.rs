#![no_std]
#![no_main]

// pick a panicking behavior
use core::cell::RefCell;
use core::fmt::Write;
use panic_halt as _;

use cortex_m::interrupt::{free, Mutex};
use cortex_m_rt::entry;
use cortex_m_semihosting::hprintln;

use stm32g4::stm32g431;
use stm32g4::stm32g431::interrupt;
use stm32g4::stm32g431::Interrupt::TIM3; // you can put a breakpoint on `rust_begin_unwind` to catch panics
                                         // use panic_abort as _; // requires nightly
                                         // use panic_itm as _; // logs messages over ITM; requires ITM support
                                         // use panic_semihosting as _; // logs messages to the host stderr; requires a debugger

mod bldc_motor_driver_stspin32g4;
mod indicator;

use crate::indicator::Indicator;

mod app {
    use crate::indicator::Indicator;
    // use crate::bldc_motor_driver_stspin32g4::BldcPwm;   // 👺

    pub struct App<'a> {
        dt: &'a f32,
        param: f32,
        led0: &'a dyn Indicator,
        // led1: &'a dyn Indicator,
        // led2: &'a dyn Indicator,
        // bldc: &'a BldcPwm<'a>,
    }

    impl<'a> App<'a> {
        pub fn new(
            dt: &'a f32,
            led0: &'a dyn Indicator,
            // led1: &'a dyn Indicator,
            // led2: &'a dyn Indicator,
            // bldc: &'a BldcPwm<'a>,
        ) -> Self {
            let param = 0.0;
            Self {
                dt, param, led0, /*led1, led2, bldc*/
            }
        }
        pub fn periodic_task(&self, p2: &f32) {
            self.led0.toggle();
        }
    }
}

// static adc_data:[u16; 4] = [7; 4];

#[interrupt]
fn TIM3() {
    let led0: bldc_motor_driver_stspin32g4::Led0 = bldc_motor_driver_stspin32g4::Led0::new();
    let dt : f32= 0.0;
    let app = app::App::new(&dt, &led0);
    static  mut data: f32 = 0.0;

    free(|cs| {
        match bldc_motor_driver_stspin32g4::G_PERIPHERAL
            .borrow(cs)
            .borrow()
            .as_ref()
        {
            None => (),
            Some(perip) => {
                perip.TIM3.sr.modify(|_, w| w.uif().clear_bit());
            }
        }
        unsafe{
            app.periodic_task(&data);
        }
    });
    hprintln!("test!").unwrap();
}

#[entry]
fn main() -> ! {
    use stm32g4::stm32g431;

    // hprintln!("Hello, STM32G4!").unwrap();
    // stm32f401モジュールより、ペリフェラルの入り口となるオブジェクトを取得する。
    let perip = stm32g431::Peripherals::take().unwrap();
    let mut core_perip = stm32g431::CorePeripherals::take().unwrap();
    // let led1 = bldc_motor_driver_stspin32g4::Led1::new(&perip);
    // let led2 = bldc_motor_driver_stspin32g4::Led2::new(&perip);

    bldc_motor_driver_stspin32g4::clock_init(&perip, &mut core_perip);

    // let mut uart = bldc_motor_driver_stspin32g4::Uart1::new(&perip);
    // let pwm = bldc_motor_driver_stspin32g4::BldcPwm::new(&perip);
    // pwm.set_u_pwm(600);
    // pwm.set_v_pwm(600);
    // pwm.set_w_pwm(600);
    bldc_motor_driver_stspin32g4::adc2_init(&perip);
    let adc_data: [u16; 7] = [77; 7];
    let dma_buf_addr: u32 = adc_data.as_ptr() as u32;
    bldc_motor_driver_stspin32g4::dma_init(&perip, &mut core_perip, dma_buf_addr);
    bldc_motor_driver_stspin32g4::dma_adc2_start(&perip);

    bldc_motor_driver_stspin32g4::init_g_peripheral(perip);

    let dt = 0.0;
    let led0 = bldc_motor_driver_stspin32g4::Led0::new();
    led0.init();
    let app = app::App::new(&dt, &led0 /*&led1, &led2 , &pwm*/);

    loop {}

    // let mut t = perip.TIM3.cnt.read().cnt().bits();
    // let mut prev = t;

    // let mut cnt = 0;
    // loop {
    //     t = perip.TIM3.cnt.read().cnt().bits(); // 0.1ms
    //     // if t.wrapping_sub(prev) > 50 {
    //     if (t+10000-prev)%10000 >= 5000 {
    //         cnt += 1;
    //         // hprintln!("t: {}", t).unwrap();
    //         app.periodic_task();
    //         if cnt > 100 {
    //             // app.periodic_task();

    //             // uart.write_str("hello ");
    //             // write!(uart, "{} + {} = {}\r\n", 2, 4, 2+4);
    //             // unsafe {
    //             //     write!(
    //             //         uart,
    //             //         "{{\"ADC\":[{:4}, {:4}, {:4}, {:4}, {:4}, {:4}, {:4}]}}\r\n",
    //             //         adc_data[0], adc_data[1], adc_data[2], adc_data[3], adc_data[4], adc_data[5], adc_data[6]
    //             //     );
    //             // }
    //             // write!(uart, "{} \r\n", potensio0.sigle_conversion());
    //             cnt = 0;
    //         }
    //         prev = t;
    //         // hprintln!("next: {}", last_t.wrapping_add(1000)).unwrap();
    //     }
    // }
}
