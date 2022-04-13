use crate::led::Led;
use stm32g4::stm32g431::Peripherals;
use cortex_m_semihosting::hprintln;
use crate::xprintln::Xprintln;

pub struct UART0<'a> {
    perip: &'a Peripherals,
}

impl<'a> Xprintln for UART0<'a> {
    fn xprintln(&self) {
        let gpioc = &self.perip.GPIOC;
        gpioc.bsrr.write(|w| w.bs13().set());
    }
}

impl<'a> UART0<'a> {
    pub fn new(perip: &'a Peripherals, baudrate: u32) -> Self {

        // baudrate設定とか

        Self { perip }
    }
    fn putc() {

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
        // GPIOCポートの電源投入(クロックの有効化)
        perip.RCC.ahb2enr.modify(|_, w| w.gpiocen().set_bit());

        // gpio初期化(PC13を出力に指定)
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
        // GPIOCポートの電源投入(クロックの有効化)
        perip.RCC.ahb2enr.modify(|_, w| w.gpiocen().set_bit());

        // gpio初期化(PC14を出力に指定)
        let gpioc = &perip.GPIOC;
        gpioc.moder.modify(|_, w| w.moder14().output());

        Self { perip }
    }
}

