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
    use crate::bldc_motor_driver_stspin32g4::BldcPwm;
    use crate::indicator::Indicator; // 👺

    pub struct App<T0, T1, T2>
    where
        T0: Indicator,
        T1: Indicator,
        T2: Indicator,
    {
        led0: T0,
        led1: T1,
        led2: T2,
        // bldc: &'a BldcPwm<'a>,
    }

    impl<T0, T1, T2> App<T0, T1, T2>
    where
        T0: Indicator,
        T1: Indicator,
        T2: Indicator,
    {
        pub fn new(led0: T0, led1: T1, led2: T2 /*, bldc: &'a BldcPwm<'a> */) -> Self {
            Self {
                led0,
                led1,
                led2,
                // bldc,
            }
        }
        pub fn periodic_task(&self, p2: &f32) {
            self.led0.toggle();
            self.led1.toggle();
            self.led2.toggle();
        }
    }
}

// static adc_data:[u16; 4] = [7; 4];
pub static G_APP: Mutex<
    RefCell<
        Option<
            app::App<
                bldc_motor_driver_stspin32g4::Led0,
                bldc_motor_driver_stspin32g4::Led1,
                bldc_motor_driver_stspin32g4::Led2,
            >,
        >,
    >,
> = Mutex::new(RefCell::new(None));

#[interrupt]
fn TIM3() {
    // let led0: bldc_motor_driver_stspin32g4::Led0 = bldc_motor_driver_stspin32g4::Led0::new();
    // let led1: bldc_motor_driver_stspin32g4::Led1 = bldc_motor_driver_stspin32g4::Led1::new();
    // let led2: bldc_motor_driver_stspin32g4::Led2 = bldc_motor_driver_stspin32g4::Led2::new();
    // let app = app::App::new(led0, led1, led2);
    static mut data: f32 = 0.0;

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
        match G_APP.borrow(cs).borrow().as_ref() {
            None => (),
            Some(app) => {
                app.periodic_task(&data);
            }
        }
    });
    // hprintln!("test!").unwrap();
}

#[entry]
fn main() -> ! {
    use stm32g4::stm32g431;

    // hprintln!("Hello, STM32G4!").unwrap();
    // stm32f401モジュールより、ペリフェラルの入り口となるオブジェクトを取得する。
    let perip = stm32g431::Peripherals::take().unwrap();
    let mut core_perip = stm32g431::CorePeripherals::take().unwrap();

    bldc_motor_driver_stspin32g4::clock_init(&perip, &mut core_perip);
    bldc_motor_driver_stspin32g4::adc2_init(&perip);
    let adc_data: [u16; 7] = [77; 7];
    let dma_buf_addr: u32 = adc_data.as_ptr() as u32;
    bldc_motor_driver_stspin32g4::dma_init(&perip, &mut core_perip, dma_buf_addr);
    bldc_motor_driver_stspin32g4::dma_adc2_start(&perip);

    bldc_motor_driver_stspin32g4::init_g_peripheral(perip);

    let mut uart = bldc_motor_driver_stspin32g4::Uart1::new();
    uart.init();
    let pwm = bldc_motor_driver_stspin32g4::BldcPwm::new();
    pwm.init();
    pwm.set_u_pwm(600);
    pwm.set_v_pwm(600);
    pwm.set_w_pwm(600);

    let led0 = bldc_motor_driver_stspin32g4::Led0::new();
    led0.init();
    let led1 = bldc_motor_driver_stspin32g4::Led1::new();
    led1.init();
    let led2 = bldc_motor_driver_stspin32g4::Led2::new();
    led2.init();
    let app = app::App::new(led0, led1, led2 /* , &pwm*/);
    free(|cs| G_APP.borrow(cs).replace(Some(app)));

    let mut t = 0;
    free(|cs| {
        match bldc_motor_driver_stspin32g4::G_PERIPHERAL
            .borrow(cs)
            .borrow()
            .as_ref()
        {
            None => (),
            Some(perip) => {
                t = perip.TIM3.cnt.read().cnt().bits();
            }
        }
    });
    let mut prev = t;
    let mut cnt = 0;

    loop{}

    loop {
        free(|cs| {
            match bldc_motor_driver_stspin32g4::G_PERIPHERAL
                .borrow(cs)
                .borrow()
                .as_ref()
            {
                None => (),
                Some(perip) => {
                    t = perip.TIM3.cnt.read().cnt().bits();
                }
            }
        });
        if (t + 10000 - prev) % 10000 >= 1000 { // 0.1ms
            cnt += 1;
            if cnt > 5000 {
                hprintln!("t: {}", t).unwrap();
                // app.periodic_task();
                // uart.write_str("hello ");
                // write!(uart, "{} + {} = {}\r\n", 2, 4, 2+4);
                // unsafe {
                //     write!(
                //         uart,
                //         "{{\"ADC\":[{:4}, {:4}, {:4}, {:4}, {:4}, {:4}, {:4}]}}\r\n",
                //         adc_data[0], adc_data[1], adc_data[2], adc_data[3], adc_data[4], adc_data[5], adc_data[6]
                //     );
                // }
                // write!(uart, "{} \r\n", potensio0.sigle_conversion());
                cnt = 0;
            }
            prev = t;
            // hprintln!("next: {}", last_t.wrapping_add(1000)).unwrap();
        }
    }
}
