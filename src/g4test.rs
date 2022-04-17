use crate::led::Led;
use crate::potensio::Potensio;

use stm32g4::stm32g431::Peripherals;
use cortex_m_semihosting::hprintln;
use core::fmt::{self, Write};


pub struct Uart0<'a> {
    perip: &'a Peripherals,
}

impl<'a> Write for Uart0<'a> {
    fn write_str(&mut self, s: &str) -> fmt::Result {
        for c in s.bytes() {
            self.putc(c);
        }
        Ok(())
    }
}

impl<'a> Uart0<'a> {
    pub fn new(perip: &'a Peripherals) -> Self {
        
        // GPIOポートの電源投入(クロックの有効化)
        perip.RCC.ahb2enr.modify(|_, w| w.gpioben().set_bit());

        perip.RCC.apb2enr.modify(|_, w| w.usart1en().enabled());

        // gpioモード変更
        let gpiob = &perip.GPIOB;
        gpiob.moder.modify(|_, w| w.moder6().alternate());
        gpiob.moder.modify(|_, w| w.moder7().alternate());
        gpiob.afrl.modify(|_, w| w.afrl6().af7());
        gpiob.afrl.modify(|_, w| w.afrl7().af7());
        
        let uart = &perip.USART1;
        // Set over sampling mode
        uart.cr1.modify(|_, w| w.over8().clear_bit());
        // Set parity mode
        uart.cr1.modify(|_, w| w.pce().clear_bit());
        // Set word length
        uart.cr1.modify(|_, w| w.m0().clear_bit());
        uart.cr1.modify(|_, w| w.m1().clear_bit());
        // FIFO?
        // Set baud rate
        uart.brr.modify(|_, w| unsafe{ w.bits(0x4BF) });    // 140MHz / 115200

        // Set stop bit
        uart.cr2.modify(|_, w| unsafe{ w.stop().bits(0b00) });

        // Set uart enable
        uart.cr1.modify(|_, w| w.ue().set_bit());

        // Set uart recieve enable
        uart.cr1.modify(|_, w| w.re().set_bit());
        // Set uart transmitter enable
        uart.cr1.modify(|_, w| w.te().set_bit());


        Self { perip }
    }
    fn putc(&self, c: u8) {
        let uart = &self.perip.USART1;
        uart.tdr.modify(|_, w| unsafe{ w.tdr().bits(c.into()) });
        // while uart.isr.read().tc().bit_is_set() {}
        while uart.isr.read().txe().bit_is_clear() {}
    }

}

    
pub fn clock_init(perip: &Peripherals) {

    perip.RCC.cr.modify(|_, w| w.hsebyp().bypassed());
    perip.RCC.cr.modify(|_, w| w.hseon().on());
    while perip.RCC.cr.read().hserdy().is_not_ready() {}

    // Disable the PLL
    perip.RCC.cr.modify(|_, w| w.pllon().off());
    // Wait until PLL is fully stopped
    while perip.RCC.cr.read().pllrdy().is_ready() {}
    perip.RCC.pllcfgr.modify(|_, w| w.pllsrc().hse());
    perip.RCC.pllcfgr.modify(|_, w| w.pllm().div12());
    // perip.RCC.pllcfgr.modify(|_, w| w.plln().div85());
    perip.RCC.pllcfgr.modify(|_, w| w.plln().div70());
    perip.RCC.pllcfgr.modify(|_, w| w.pllr().div2());
    
    perip.RCC.cr.modify(|_, w| w.pllon().on());
    while perip.RCC.cr.read().pllrdy().is_not_ready() {}
    perip.RCC.pllcfgr.modify(|_, w| w.pllren().set_bit());

    perip.FLASH.acr.modify(|_,w| unsafe { w.latency().bits(4) });
    while perip.FLASH.acr.read().latency().bits() != 4 {
        hprintln!("latency bit: {}", perip.FLASH.acr.read().latency().bits()).unwrap();
    }

    perip.RCC.cfgr.modify(|_, w| w.sw().pll());
    // perip.RCC.cfgr.modify(|_, w| w.sw().hse());
    // hprintln!("sw bit: {}", perip.RCC.cfgr.read().sw().bits()).unwrap();
    while !perip.RCC.cfgr.read().sw().is_pll() {}
    while !perip.RCC.cfgr.read().sws().is_pll() {
        // hprintln!("sw bit: {}", perip.RCC.cfgr.read().sw().bits()).unwrap();
        // hprintln!("sws bit: {}", perip.RCC.cfgr.read().sws().bits()).unwrap();
    }
    // while !perip.RCC.cfgr.read().sws().is_hse() {}

    perip.RCC.apb1enr1.modify(|_,w| w.tim3en().enabled());
    
    let tim3 = &perip.TIM3;
    // tim3.psc.modify(|_, w| unsafe { w.bits(170 - 1) });
    tim3.psc.modify(|_, w| unsafe { w.bits(15_000 - 1) });
    // tim3.arr.modify(|_, w| unsafe { w.bits(1000 - 1) });    // 1kHz
    tim3.dier.modify(|_, w| w.uie().set_bit());
    tim3.cr1.modify(|_, w| w.cen().set_bit());

}

pub struct Potensio0<'a> {
    perip: &'a Peripherals,
}

impl<'a> Potensio for Potensio0<'a> {
    fn get_angle(&self) -> f32 {
        0.0
    }
}

impl<'a> Potensio0<'a> {
    pub fn new(perip: &'a Peripherals) -> Self {
        // GPIOポートの電源投入(クロックの有効化)
        perip.RCC.ahb2enr.modify(|_, w| w.gpiocen().set_bit());

        // gpioモード変更
        let gpioc = &perip.GPIOC;
        gpioc.moder.modify(|_, w| w.moder13().output());

        Self { perip }
    }
}


pub struct Led0<'a> {
    perip: &'a Peripherals,
}

impl<'a> Led for Led0<'a> {
    fn on(&self) {
        let gpioc = &self.perip.GPIOC;
        gpioc.bsrr.write(|w| w.bs13().set());
    }
    fn off(&self) {
        let gpioc = &self.perip.GPIOC;
        gpioc.bsrr.write(|w| w.br13().reset());
    }
    fn toggle(&self) {
        let gpioc = &self.perip.GPIOC;
        if gpioc.odr.read().odr13().is_low() {
            gpioc.bsrr.write(|w| w.bs13().set());
        } else {
            gpioc.bsrr.write(|w| w.br13().reset());
        }
    }
}

impl<'a> Led0<'a> {
    pub fn new(perip: &'a Peripherals) -> Self {
        // GPIOポートの電源投入(クロックの有効化)
        perip.RCC.ahb2enr.modify(|_, w| w.gpiocen().set_bit());

        // gpioモード変更
        let gpioc = &perip.GPIOC;
        gpioc.moder.modify(|_, w| w.moder13().output());

        Self { perip }
    }
}

pub struct Led1<'a> {
    perip: &'a Peripherals,
}

impl<'a> Led for Led1<'a> {
    fn on(&self) {
        let gpioc = &self.perip.GPIOC;
        gpioc.bsrr.write(|w| w.bs14().set());
    }
    fn off(&self) {
        let gpioc = &self.perip.GPIOC;
        gpioc.bsrr.write(|w| w.br14().reset());
    }
    fn toggle(&self) {
        let gpioc = &self.perip.GPIOC;
        if gpioc.odr.read().odr14().is_low() {
            gpioc.bsrr.write(|w| w.bs14().set());
        } else {
            gpioc.bsrr.write(|w| w.br14().reset());
        }
    }
}

impl<'a> Led1<'a> {
    pub fn new(perip: &'a Peripherals) -> Self {
        // GPIOポートの電源投入(クロックの有効化)
        perip.RCC.ahb2enr.modify(|_, w| w.gpiocen().set_bit());

        // gpioモード変更
        let gpioc = &perip.GPIOC;
        gpioc.moder.modify(|_, w| w.moder14().output());

        Self { perip }
    }
}

