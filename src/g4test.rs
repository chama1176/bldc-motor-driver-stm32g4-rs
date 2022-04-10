use crate::led::Led;
use crate::periodic::PeriodicTask;
use stm32g4::stm32g431::Peripherals;
use stm32g4::stm32g431::interrupt;
use cortex_m_semihosting::hprintln;


pub struct Interrupt0<'a> {
    perip: &'a Peripherals,
    interrupt0: &'a dyn PeriodicTask,
}

impl<'a> Interrupt0<'a> {
    
    pub fn new(perip: &'a Peripherals, interrupt0: &'a dyn PeriodicTask) -> Self {

        unsafe{
            stm32g4::stm32g431::NVIC::unmask(interrupt::TIM3);
        }
        perip.RCC.cr.modify(|_, w| w.hseon().on());

        perip.RCC.pllcfgr.modify(|_, w| w.pllsrc().hse());
        perip.RCC.pllcfgr.modify(|_, w| w.pllm().div12());
        perip.RCC.pllcfgr.modify(|_, w| w.plln().div85());
        perip.RCC.pllcfgr.modify(|_, w| w.pllr().div2());
        
        perip.RCC.cr.modify(|_, w| w.pllon().on());
        while perip.RCC.cr.read().pllrdy().is_not_ready() {}

        perip.FLASH.acr.modify(|_,w| unsafe { w.latency().bits(4) });

        perip.RCC.cfgr.modify(|_, w| w.sw().pll());
        while !perip.RCC.cfgr.read().sw().is_pll() {}
        // while !perip.RCC.cfgr.read().sws().is_pll() {}

        perip.RCC.apb1enr1.modify(|_,w| w.tim3en().enabled());
        
        let tim3 = &perip.TIM3;
        tim3.psc.modify(|_, w| unsafe { w.bits(170 - 1) });
        tim3.arr.modify(|_, w| unsafe { w.bits(1000 - 1) });    // 1kHz
        tim3.dier.modify(|_, w| w.uie().set_bit());
        tim3.cr1.modify(|_, w| w.cen().set_bit());

        Self { perip, interrupt0 }
    }

    // #[interrupt]
    // pub unsafe fn TIM3(&self) -> ! {
    //     unsafe {
    //         self.perip.sr.modify(|_, w| w.uif().clear());
    //         // self.interrupt0.run();
    //     }
    // }

}

#[interrupt]
fn TIM3() {
    static mut COUNT: u32 = 0;
    unsafe {
        let dp = Peripherals::steal();   // (23)Peripheralsの取得
        dp.TIM3.sr.modify(|_, w| w.uif().clear_bit());  // (24)更新フラグのクリア
    }
    // `COUNT` has type `&mut u32` and it's safe to use
    *COUNT += 1;
    hprintln!("count").unwrap();
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
    fn toggle(&self) {}
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
    fn toggle(&self) {}
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

