#![no_std]
#![no_main]

// pick a panicking behavior
use core::cell::RefCell;
use core::clone;
use core::fmt::Write;
use core::ops::DerefMut;
use defmt_rtt as _;
use panic_halt as _;

use cortex_m::interrupt::{free, Mutex};
use cortex_m_rt::entry;

use stm32g4::stm32g431;
use stm32g4::stm32g431::interrupt;
use stm32g4::stm32g431::Interrupt::{TIM3, DMA1_CH1}; // you can put a breakpoint on `rust_begin_unwind` to catch panics
                                         // use panic_abort as _; // requires nightly
                                         // use panic_itm as _; // logs messages over ITM; requires ITM support
                                         // use panic_semihosting as _; // logs messages to the host stderr; requires a debugger

mod app;
mod bldc_motor_driver_stm32g4;
mod indicator;

use crate::indicator::Indicator;

use motml::encoder::Encoder;
use motml::motor_driver::{self, ThreePhaseCurrent};
use motml::utils::Deg;
use motml::motor::ThreePhaseMotor;


static G_APP: Mutex<
    RefCell<
        Option<
            app::App<
                bldc_motor_driver_stm32g4::Led0,
                bldc_motor_driver_stm32g4::Led1,
                bldc_motor_driver_stm32g4::BldcPwm,
                bldc_motor_driver_stm32g4::Spi1,
            >,
        >,
    >,
> = Mutex::new(RefCell::new(None));



#[interrupt]
fn DMA1_CH1(){
    static mut TIM3_COUNT: u32 = 0;
    // `TIM3_COUNT` has type `&mut u32` and it's safe to use
    *TIM3_COUNT += 1;
    let mut tim_count = 0;
    free(|cs| {
        match bldc_motor_driver_stm32g4::G_PERIPHERAL
            .borrow(cs)
            .borrow()
            .as_ref()
        {
            None => (),
            Some(perip) => {
                if perip.DMA1.isr.read().tcif1().bit_is_set() {
                    perip.DMA1.ifcr.write(|w| w.tcif1().set_bit());
                }else{
                    // 想定と違う割り込み要因
                    defmt::error!("Something went wrong!");
                    perip.DMA1.ifcr.write(|w| w.gif1().set_bit());
                    return;
                }
                tim_count = perip.TIM3.cnt.read().cnt().bits();
            }
        }
        match G_APP.borrow(cs).borrow_mut().deref_mut() {
            None => (),
            Some(app) => {
                app.set_count(TIM3_COUNT.clone());
                app.periodic_task();
                app.diff_count = tim_count as u32 - app.last_tim_count;
                app.last_tim_count = tim_count as u32;
            }
        }
    });

}

#[interrupt]
fn TIM3() {

    // mainで初期化済み
    let mut uart = bldc_motor_driver_stm32g4::Uart1::new();

    free(|cs| {
        match bldc_motor_driver_stm32g4::G_PERIPHERAL
            .borrow(cs)
            .borrow()
            .as_ref()
        {
            None => (),
            Some(perip) => {
                perip.TIM3.sr.modify(|_, w| w.uif().clear_bit());
            }
        }
    });
    free(|cs| match G_APP.borrow(cs).borrow_mut().deref_mut() {
        None => (),
        Some(app) => {
            let adcd =  free(|cs| bldc_motor_driver_stm32g4::G_ADC_DATA.borrow(cs).borrow().clone());

            let mut tv = (adcd[1] as f32 - 2000.0f32) / 1000.0f32;
            if tv > 1.0 {
                tv = 1.0;
            }

            app.led_tick_task();
            app.set_target_velocity(tv);
            if tv > 0.1 {
                // app.set_control_mode(app::ControlMode::OperatingForcedCommutation2);
                // app.set_control_mode(app::ControlMode::Operating120DegreeDrive);
                app.set_control_mode(app::ControlMode::OperatingQPhase);
            } else if tv < -0.5 {
                app.set_control_mode(app::ControlMode::Calibrating);
            } else {
                app.set_control_mode(app::ControlMode::Waiting);
            }
        }
    });


    // write!(
    //     uart,
    //     "{{\"ADC\":[{:4}, {:4}, {:4}, {:4}, {:4}, {:4}, {:4}]}}\r\n",
    //     adcd[0],
    //     adcd[1],
    //     adcd[2],
    //     adcd[3],
    //     adcd[4],
    //     adcd[5],
    //     adcd[6]
    // )
    // .unwrap();

    // // I = V / R
    // // 60V/V, 0.003
    // let current = ThreePhaseCurrent::<f32> {
    //     i_u: (((adcd[2] as f32) / adcd[6] as f32 * 1.5) - 1.5) / 60.0 / 0.003 - 0.0,
    //     i_v: (((adcd[3] as f32) / adcd[6] as f32 * 1.5) - 1.5) / 60.0 / 0.003 - 0.0,
    //     i_w: 0.0,
    // };
    // let dq = current.to_dq(app.last_electrical_angle);

    // defmt::info!("diff: {}", app.diff_count);

    // // floatのまま送るとFLASHをバカほど食うのでcastする
    // write!(
    //     uart,
    //     "{{\"ea\":{:4}}}\r\n",
    //     (app.last_electrical_angle * 1000.0) as i32,
    // )
    // .unwrap();
    
    // // floatのまま送るとFLASHをバカほど食うのでcastする
    // write!(
    //     uart,
    //     "{{\"ma\":{:4}}}\r\n",
    //     (app.last_mechanical_angle * 1000.0) as i32,
    // )
    // .unwrap();
    
    // // floatのまま送るとFLASHをバカほど食うのでcastする
    // write!(
    //     uart,
    //     "{{\"iu\":{:4}}}\r\n",
    //     (current.i_u * 1000.0) as i32,
    // )
    // .unwrap();
    // // floatのまま送るとFLASHをバカほど食うのでcastする
    // write!(
    //     uart,
    //     "{{\"iv\":{:4}}}\r\n",
    //     (current.i_v * 1000.0) as i32,
    // )
    // .unwrap();
    
    // // floatのまま送るとFLASHをバカほど食うのでcastする
    // write!(
    //     uart,
    //     "{{\"d\":{:4}}}\r\n",
    //     (dq.i_d * 1000.0) as i32,
    // )
    // .unwrap();
    // write!(
    //     uart,
    //     "{{\"q\":{:4}}}\r\n",
    //     (dq.i_q * 1000.0) as i32,
    // )
    // .unwrap();





}

defmt::timestamp!("{=u32:us}", {
    // NOTE(interrupt-safe) single instruction volatile read operation
    free(|cs| {
        match bldc_motor_driver_stm32g4::G_PERIPHERAL
            .borrow(cs)
            .borrow()
            .as_ref()
        {
            None => 0,
            Some(perip) => {
                perip.TIM3.cnt.read().cnt().bits() as u32
            }
        }
    })
});

#[entry]
fn main() -> ! {
    use stm32g4::stm32g431;

    defmt::info!("Hello, from STM32G4!");
    // stm32f401モジュールより、ペリフェラルの入り口となるオブジェクトを取得する。
    let perip = stm32g431::Peripherals::take().unwrap();
    let mut core_perip = stm32g431::CorePeripherals::take().unwrap();

    bldc_motor_driver_stm32g4::clock_init(&perip, &mut core_perip);
    bldc_motor_driver_stm32g4::adc2_init(&perip);

    bldc_motor_driver_stm32g4::dma_init(&perip, &mut core_perip);
    bldc_motor_driver_stm32g4::dma_adc2_start(&perip);

    bldc_motor_driver_stm32g4::init_g_peripheral(perip);

    let mut uart = bldc_motor_driver_stm32g4::Uart1::new();
    uart.init();
    let pwm = bldc_motor_driver_stm32g4::BldcPwm::new();
    pwm.init();

    let spi = bldc_motor_driver_stm32g4::Spi3::new();
    spi.init();
    let led0 = bldc_motor_driver_stm32g4::Led0::new();
    led0.init();
    let led1 = bldc_motor_driver_stm32g4::Led1::new();
    led1.init();
    let spi_enc = bldc_motor_driver_stm32g4::Spi1::new();
    spi_enc.init();
    spi_enc.reset_error();


    // let flash = bldc_motor_driver_stm32g4::FrashStorage::new();
    // flash.write();

    // Setup motor driver
    // Change Buck Convetor Frequency
    // spi.txrx(0x917000);
    // spi.txrx(0x110000);

    // Set Dead Time
    spi.txrx(0x800000 | 0x1B_0000);
    spi.txrx(0x000000 | 0x1B_0000);

    let app = app::App::new(led0, led1, pwm, spi_enc);
    free(|cs| G_APP.borrow(cs).replace(Some(app)));

    let mut t = 0;
    free(|cs| {
        match bldc_motor_driver_stm32g4::G_PERIPHERAL
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
            match bldc_motor_driver_stm32g4::G_PERIPHERAL
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
            cnt += 1;
            if cnt > 500 {
                // defmt::info!("hello from defmt");

                cnt = 0;
                let mut rad = 0.;
                let mut calib_count = 7;
                free(|cs| match G_APP.borrow(cs).borrow_mut().deref_mut() {
                    None => (),
                    Some(app) => {
                        rad = app.read_encoder_data();
                        calib_count = app.calib_count();
                    }
                });
                let deg = rad.rad2deg();
                defmt::info!("deg: {}, rad: {}", deg, rad);
                // write!(uart, "{}, {:4}, {:4}", calib_count, deg, rad).unwrap();
                // write!(uart, "\"tv\": {:4}\r\n", tv,).unwrap();
            }
            prev = t;
        }
    }
}
