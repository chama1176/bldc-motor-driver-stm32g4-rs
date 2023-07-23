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

mod bldc_motor_driver_stspin32g4;
mod indicator;
mod three_phase_motor_driver;

use crate::indicator::Indicator;
use crate::three_phase_motor_driver::ThreePhaseMotorDriver;

static COUNTER: Mutex<Cell<u64>> = Mutex::new(Cell::new(0));

mod app {
    use crate::indicator::Indicator;
    use crate::three_phase_motor_driver::ThreePhase;
    use crate::three_phase_motor_driver::ThreePhaseMotorDriver;

    pub struct Data {
        tv: f32,
        count: u64,
    }
    impl Data {
        pub fn new() -> Self {
            Self { tv: 0.0, count: 0 }
        }
    }

    pub struct App<T0, T1, T2, M>
    where
        T0: Indicator,
        T1: Indicator,
        T2: Indicator,
        M: ThreePhaseMotorDriver,
    {
        data: Data,
        led0: T0,
        led1: T1,
        led2: T2,
        bldc: M,
    }

    impl<T0, T1, T2, M> App<T0, T1, T2, M>
    where
        T0: Indicator,
        T1: Indicator,
        T2: Indicator,
        M: ThreePhaseMotorDriver,
    {
        pub fn new(led0: T0, led1: T1, led2: T2, bldc: M) -> Self {
            Self {
                data: Data::new(),
                led0,
                led1,
                led2,
                bldc,
            }
        }
        #[rustfmt::skip]
        pub fn periodic_task(&mut self) {
            self.led0.toggle();
            self.led1.toggle();
            self.led2.toggle();

            let mut tp: ThreePhase<u32> = ThreePhase { u: 0, v: 0, w: 0 };

            match ((self.data.count as f32/(10.0*1.0)) as u64)%6 {
                0 => tp = ThreePhase{u: 200, v: 0, w: 0},
                1 => tp = ThreePhase{u: 200, v: 200, w: 0},
                2 => tp = ThreePhase{u: 0, v: 200, w: 0},
                3 => tp = ThreePhase{u: 0, v: 200, w: 200},
                4 => tp = ThreePhase{u: 0, v: 0, w: 200},
                5 => tp = ThreePhase{u: 200, v: 0, w: 200},
                6_u64..=u64::MAX => (),
            }
            if self.data.tv < 0.1 {
                tp = ThreePhase{ u: 0, v: 0, w: 0 };
            }
            self.bldc.set_u_pwm(tp.u);
            self.bldc.set_v_pwm(tp.v);
            self.bldc.set_w_pwm(tp.w);
        }
        pub fn set_target_velocity(&mut self, tv: f32) {
            self.data.tv = tv;
        }
        pub fn set_count(&mut self, c: u64) {
            self.data.count = c;
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
                bldc_motor_driver_stspin32g4::BldcPwm,
            >,
        >,
    >,
> = Mutex::new(RefCell::new(None));

#[interrupt]
fn TIM3() {
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
                app.set_count(COUNTER.borrow(cs).get());
                app.periodic_task();
            }
        }
    });
    free(|cs| COUNTER.borrow(cs).set(COUNTER.borrow(cs).get() + 1));
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
    pwm.set_u_pwm(0);
    pwm.set_v_pwm(0);
    pwm.set_w_pwm(0);

    let led0 = bldc_motor_driver_stspin32g4::Led0::new();
    led0.init();
    let led1 = bldc_motor_driver_stspin32g4::Led1::new();
    led1.init();
    let led2 = bldc_motor_driver_stspin32g4::Led2::new();
    led2.init();
    let app = app::App::new(led0, led1, led2, pwm);
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
                let mut tv = (adc_data[4] as f32 - 4000.0f32) / 2000.0f32;
                if tv < 0.0 {
                    tv = 0.0;
                }
                if tv > 1.0 {
                    tv = 1.0;
                }
                free(|cs| match G_APP.borrow(cs).borrow_mut().deref_mut() {
                    None => (),
                    Some(app) => {
                        app.set_target_velocity(tv);
                    }
                });
                write!(uart, "\"tv\": {:4}\r\n", tv,).unwrap();
            }
            prev = t;
        }
    }
}
