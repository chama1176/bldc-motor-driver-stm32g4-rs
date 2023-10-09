#![no_std]
#![no_main]

// pick a panicking behavior
use core::cell::{Cell, RefCell};
use core::fmt::Write;
use core::ops::DerefMut;
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

mod app;
mod bldc_motor_driver_stspin32g4;
mod indicator;

use motml::encoder::Encoder;
use motml::motor_driver;
use motml::utils::Deg;

static G_APP: Mutex<
    RefCell<
        Option<
            app::App<
                bldc_motor_driver_stspin32g4::Led0,
                bldc_motor_driver_stspin32g4::Led1,
                bldc_motor_driver_stspin32g4::Led2,
                bldc_motor_driver_stspin32g4::BldcPwm,
                bldc_motor_driver_stspin32g4::Spi3,
            >,
        >,
    >,
> = Mutex::new(RefCell::new(None));

#[interrupt]
fn TIM3() {
    static mut COUNT: u64 = 0;
    // `COUNT` has type `&mut u32` and it's safe to use
    *COUNT += 1;

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
        match G_APP.borrow(cs).borrow_mut().deref_mut() {
            None => (),
            Some(app) => {
                app.set_count(COUNT.clone());
                app.periodic_task();
            }
        }
    });
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

    let spi = bldc_motor_driver_stspin32g4::Spi3::new();
    spi.init();
    spi.reset_error();
    let led0 = bldc_motor_driver_stspin32g4::Led0::new();
    led0.init();
    let led1 = bldc_motor_driver_stspin32g4::Led1::new();
    led1.init();
    let led2 = bldc_motor_driver_stspin32g4::Led2::new();
    led2.init();
    let app = app::App::new(led0, led1, led2, pwm, spi);
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
        if (t + 10000 - prev) % 10000 >= 1000 {
            // 0.1ms
            cnt += 1;
            if cnt > 5000 {
                // hprintln!("t: {}", t).unwrap();
                // app.periodic_task();
                // uart.write_str("hello ");
                // write!(uart, "{} + {} = {}\r\n", 2, 4, 2+4);
                write!(
                    uart,
                    "{{\"ADC\":[{:4}, {:4}, {:4}, {:4}, {:4}, {:4}, {:4}]}}\r\n",
                    adc_data[0],
                    adc_data[1],
                    adc_data[2],
                    adc_data[3],
                    adc_data[4],
                    adc_data[5],
                    adc_data[6]
                )
                .unwrap();
                // adc_data[4] 2000 ~ 6000 c: 4000
                cnt = 0;
                let mut tv = (adc_data[4] as f32 - 2000.0f32) / 1000.0f32;
                if tv > 1.0 {
                    tv = 1.0;
                }
                let mut rad = 0.;
                let mut calib_count = 7;
                free(|cs| match G_APP.borrow(cs).borrow_mut().deref_mut() {
                    None => (),
                    Some(app) => {
                        app.set_target_velocity(tv);
                        rad = app.read_encoder_data();
                        calib_count = app.calib_count();
                        if tv > 0.1 {
                            // app.set_sate(app::State::OperatingForcedCommutation2);
                            // app.set_sate(app::State::Operating120DegreeDrive);
                            app.set_sate(app::State::OperatingQPhase);
                            
                        } else if tv < -0.5 {
                            app.set_sate(app::State::Calibrating);
                        } else {
                            app.set_sate(app::State::Waiting);
                        }
                    }
                });
                let deg = rad.rad2deg();
                // hprintln!("deg: {:}, rad: {:}", deg, rad).unwrap();
                write!(uart, "{}, {}, {}\r\n", calib_count, deg, rad).unwrap();

                // write!(uart, "\"tv\": {:4}\r\n", tv,).unwrap();
            }
            prev = t;
        }
    }
}
