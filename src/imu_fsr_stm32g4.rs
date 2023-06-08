use crate::fsr::Fsr;
use crate::indicator::Indicator;
use crate::potensio::Potensio;

use stm32g4::stm32g431::CorePeripherals;
use stm32g4::stm32g431::Interrupt;
use stm32g4::stm32g431::Peripherals;
use stm32g4::stm32g431::NVIC;

use core::fmt::{self, Write};
#[allow(unused_imports)]
use cortex_m_semihosting::hprintln;

pub struct Uart1<'a> {
    perip: &'a Peripherals,
}

impl<'a> Write for Uart1<'a> {
    fn write_str(&mut self, s: &str) -> fmt::Result {
        for c in s.bytes() {
            self.putc(c);
        }
        Ok(())
    }
}

impl<'a> Uart1<'a> {
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
        uart.brr.modify(|_, w| unsafe { w.bits(0x4BF) }); // 140MHz / 115200

        // Set stop bit
        uart.cr2.modify(|_, w| unsafe { w.stop().bits(0b00) });

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
        uart.tdr.modify(|_, w| unsafe { w.tdr().bits(c.into()) });
        // while uart.isr.read().tc().bit_is_set() {}
        while uart.isr.read().txe().bit_is_clear() {}
    }
}

pub struct Uart3<'a> {
    perip: &'a Peripherals,
}
impl<'a> Write for Uart3<'a> {
    fn write_str(&mut self, s: &str) -> fmt::Result {
        for c in s.bytes() {
            self.putc(c);
        }
        Ok(())
    }
}

impl<'a> Uart3<'a> {
    pub fn new(perip: &'a Peripherals) -> Self {
        // GPIOポートの電源投入(クロックの有効化)
        perip.RCC.ahb2enr.modify(|_, w| w.gpioben().set_bit());

        perip.RCC.apb1enr1.modify(|_, w| w.usart3en().enabled());

        // gpioモード変更
        let gpiob = &perip.GPIOB;
        gpiob.moder.modify(|_, w| w.moder8().alternate());
        gpiob.moder.modify(|_, w| w.moder9().alternate());
        gpiob.afrh.modify(|_, w| w.afrh8().af7());
        gpiob.afrh.modify(|_, w| w.afrh9().af7());
        // ここまでみた
        let uart = &perip.USART3;
        // Set over sampling mode
        uart.cr1.modify(|_, w| w.over8().clear_bit());
        // Set parity mode
        uart.cr1.modify(|_, w| w.pce().clear_bit());
        // Set word length
        uart.cr1.modify(|_, w| w.m0().clear_bit());
        uart.cr1.modify(|_, w| w.m1().clear_bit());
        // FIFO?
        // Set baud rate
        uart.brr.modify(|_, w| unsafe { w.bits(0x4BF) }); // 140MHz / 115200

        // Set stop bit
        uart.cr2.modify(|_, w| unsafe { w.stop().bits(0b00) });
        // Set swap
        uart.cr2.modify(|_, w| w.swap().set_bit());

        // Set uart enable
        uart.cr1.modify(|_, w| w.ue().set_bit());

        // Set uart recieve enable
        uart.cr1.modify(|_, w| w.re().set_bit());
        // Set uart transmitter enable
        uart.cr1.modify(|_, w| w.te().set_bit());

        Self { perip }
    }
    fn putc(&self, c: u8) {
        let uart = &self.perip.USART3;
        uart.tdr.modify(|_, w| unsafe { w.tdr().bits(c.into()) });
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

    perip
        .FLASH
        .acr
        .modify(|_, w| unsafe { w.latency().bits(4) });
    while perip.FLASH.acr.read().latency().bits() != 4 {
        hprintln!("latency bit: {}", perip.FLASH.acr.read().latency().bits()).unwrap();
    }
    // Switch boot configuration. Use nBOOT0 for boot configuration.
    perip.FLASH.optr.modify(|_, w| w.n_swboot0().clear_bit());

    perip.RCC.cfgr.modify(|_, w| w.sw().pll());
    // perip.RCC.cfgr.modify(|_, w| w.sw().hse());
    // hprintln!("sw bit: {}", perip.RCC.cfgr.read().sw().bits()).unwrap();
    while !perip.RCC.cfgr.read().sw().is_pll() {}
    while !perip.RCC.cfgr.read().sws().is_pll() {
        // hprintln!("sw bit: {}", perip.RCC.cfgr.read().sw().bits()).unwrap();
        // hprintln!("sws bit: {}", perip.RCC.cfgr.read().sws().bits()).unwrap();
    }
    // while !perip.RCC.cfgr.read().sws().is_hse() {}

    perip.RCC.apb1enr1.modify(|_, w| w.tim3en().enabled());
    perip.RCC.apb1enr1.modify(|_, w| w.tim6en().enabled());

    let tim3 = &perip.TIM3;
    // tim3.psc.modify(|_, w| unsafe { w.bits(170 - 1) });
    tim3.psc.modify(|_, w| unsafe { w.bits(15_000 - 1) });
    // tim3.arr.modify(|_, w| unsafe { w.bits(1000 - 1) });    // 1kHz
    tim3.dier.modify(|_, w| w.uie().set_bit());
    tim3.cr1.modify(|_, w| w.cen().set_bit());

    let tim6 = &perip.TIM6;
    tim6.psc.modify(|_, w| unsafe { w.bits(15_000 - 1) });
    tim6.arr.modify(|_, w| unsafe { w.bits(1000 - 1) }); // 1kHz
    tim6.dier.modify(|_, w| w.uie().set_bit());
    tim6.cr2.modify(|_, w| unsafe { w.mms().bits(0b010) });
}

pub fn dma_init(perip: &Peripherals, core_perip: &mut CorePeripherals, address: u32) {
    // DMAの電源投入(クロックの有効化)
    // perip.RCC.ahb1rstr.modify(|_, w| w.dmamux1rst().reset());
    // perip.RCC.ahb1rstr.modify(|_, w| w.dma1rst().reset());
    perip.RCC.ahb1rstr.modify(|_, w| w.dmamux1rst().set_bit());
    perip.RCC.ahb1rstr.modify(|_, w| w.dma1rst().set_bit());
    perip.RCC.ahb1rstr.modify(|_, w| w.dmamux1rst().clear_bit());
    perip.RCC.ahb1rstr.modify(|_, w| w.dma1rst().clear_bit());
    perip.RCC.ahb1enr.modify(|_, w| w.dmamuxen().set_bit());
    perip.RCC.ahb1enr.modify(|_, w| w.dma1en().set_bit());

    perip.DMA1.ccr1.modify(|_, w| unsafe { w.pl().bits(0b10) }); // priority level 2
    perip
        .DMA1
        .ccr1
        .modify(|_, w| unsafe { w.msize().bits(0b01) }); // 16bit
    perip
        .DMA1
        .ccr1
        .modify(|_, w| unsafe { w.psize().bits(0b01) }); // 16bit
    perip.DMA1.ccr1.modify(|_, w| w.circ().set_bit()); // circular mode
    perip.DMA1.ccr1.modify(|_, w| w.minc().set_bit()); // increment memory ptr
    perip.DMA1.ccr1.modify(|_, w| w.pinc().clear_bit()); // not increment periph  ptr
    perip.DMA1.ccr1.modify(|_, w| w.mem2mem().clear_bit()); // memory-to-memory mode
    perip.DMA1.ccr1.modify(|_, w| w.dir().clear_bit()); // read from peripheral
    perip.DMA1.ccr1.modify(|_, w| w.teie().clear_bit()); // transfer error interrupt enable
    perip.DMA1.ccr1.modify(|_, w| w.htie().clear_bit()); // half transfer interrupt enable
    perip.DMA1.ccr1.modify(|_, w| w.tcie().clear_bit()); // transfer complete interrupt enable

    // For category 2 devices:
    // • DMAMUX channels 0 to 5 are connected to DMA1 channels 1 to 6
    // • DMAMUX channels 6 to 11 are connected to DMA1 channels 1 to 6
    // DMA1 ch1 -> DMAMUX ch6
    perip
        .DMAMUX
        .c0cr
        .modify(|_, w| unsafe { w.dmareq_id().bits(36) }); // Table.91 36:ADC2
    perip.DMAMUX.c0cr.modify(|_, w| w.ege().set_bit()); // Enable generate event

    let adc = &perip.ADC2;
    let adc_data_register_addr = &adc.dr as *const _ as u32;
    // let adc_dma_buf_addr : u32 = adc_dma_buf as *const [u16; 4] as u32;
    // perip.DMA1.cpar1.modify(|_, w| unsafe { w.pa().bits(*adc.dr.as_ptr()) });   // peripheral address
    perip
        .DMA1
        .cpar1
        .modify(|_, w| unsafe { w.pa().bits(adc_data_register_addr) }); // peripheral address
                                                                        // perip.DMA1.cndtr1.modify(|_, w| unsafe { w.ndt().bits(adc_dma_buf.len() as u16) }); // num
    perip.DMA1.cndtr1.modify(|_, w| unsafe { w.ndt().bits(4) }); // num
                                                                 // perip.DMA1.cmar1.modify(|_, w| unsafe { w.ma().bits(adc_dma_buf_addr) });      // memory address
    perip
        .DMA1
        .cmar1
        .modify(|_, w| unsafe { w.ma().bits(address) }); // memory address

    // 割り込み設定
    // unsafe{
    //     core_perip.NVIC.set_priority(Interrupt::DMA1_CH1, 0);
    //     NVIC::unmask(Interrupt::DMA1_CH1);
    //     core_perip.NVIC.set_priority(Interrupt::ADC1_2, 0);
    //     NVIC::unmask(Interrupt::ADC1_2);
    // }
}

pub fn adc2_init(perip: &Peripherals) {
    // GPIOポートの電源投入(クロックの有効化)
    perip.RCC.ahb2enr.modify(|_, w| w.gpioaen().set_bit());
    perip.RCC.ahb2enr.modify(|_, w| w.gpioben().set_bit());

    perip.RCC.ahb2enr.modify(|_, w| w.adc12en().set_bit());
    perip.RCC.ccipr.modify(|_, w| w.adc12sel().system()); // clock source setting

    // gpioモード変更
    perip.GPIOA.moder.modify(|_, w| w.moder5().analog());
    perip.GPIOA.moder.modify(|_, w| w.moder6().analog());
    perip.GPIOA.moder.modify(|_, w| w.moder7().analog());
    perip.GPIOB.moder.modify(|_, w| w.moder2().analog());

    let adc = &perip.ADC2;
    adc.cfgr.modify(|_, w| w.res().bits12()); // Resolution setting
    adc.cfgr.modify(|_, w| w.align().right()); // Data align setting
    adc.cfgr.modify(|_, w| w.ovrmod().overwrite()); // Overrun mode

    adc.cfgr.modify(|_, w| w.cont().single()); // single or continuous
                                               // adc.cfgr.modify(|_, w| w.cont().continuous());   // single or continuous
    adc.cfgr.modify(|_, w| w.discen().disabled()); // single or continuous
                                                   // adc.cfgr.modify(|_, w| w.discen().enabled());   // single or continuous
                                                   // DISCEN = 1 and CONT = 1 is not allowed.
                                                   // adc.cfgr.modify(|_, w| w.discnum().bits(4-1));   // 0 means 1 length

    adc.cfgr.modify(|_, w| w.dmacfg().circular()); // dma oneshot or circular
    adc.cfgr.modify(|_, w| w.dmaen().enabled()); // dma enable
                                                 // 1周は実行したいが，常に変換しつづけるのは困る
    adc.cfgr.modify(|_, w| w.extsel().tim6_trgo()); // dma enable
    adc.cfgr.modify(|_, w| w.exten().rising_edge()); // dma enable

    perip
        .ADC12_COMMON
        .ccr
        .modify(|_, w| unsafe { w.presc().bits(0b0010) }); // Clock prescaler setting

    adc.cr.modify(|_, w| w.deeppwd().disabled()); // Deep power down setting
    adc.cr.modify(|_, w| w.advregen().enabled()); // Voltage regulator setting
                                                  // adc.ier.modify(|_, w| w.eocie().enabled());   // End of regular conversion interrupt setting
    adc.ier.modify(|_, w| w.eocie().disabled()); // End of regular conversion interrupt setting
    adc.ier.modify(|_, w| w.ovrie().enabled()); // Overrun interrupt setting
                                                // // ADC voltage regulator start-up time 20us
    let mut t = perip.TIM3.cnt.read().cnt().bits();
    let prev = t;
    while t.wrapping_sub(prev) >= 10 {
        t = perip.TIM3.cnt.read().cnt().bits();
    }
    // P.604 21.4.8 calibration
    assert!(adc.cr.read().aden().is_enable() == false);
    adc.cr.modify(|_, w| w.adcal().calibration()); // Start calibration
    while !adc.cr.read().adcal().is_complete() {} // Wait for calibration complete

    adc.smpr1.modify(|_, w| w.smp3().cycles24_5()); // sampling time selection
    adc.smpr1.modify(|_, w| w.smp4().cycles24_5()); // sampling time selection
    adc.smpr2.modify(|_, w| w.smp12().cycles24_5()); // sampling time selection
    adc.smpr2.modify(|_, w| w.smp13().cycles24_5()); // sampling time selection

    adc.sqr1.modify(|_, w| w.l().bits(4 - 1)); // Regular channel sequence length. 0 means 1 length
    adc.sqr1.modify(|_, w| unsafe { w.sq1().bits(3) }); // 1st conversion in regular sequence
    adc.sqr1.modify(|_, w| unsafe { w.sq2().bits(4) }); // 1st conversion in regular sequence
    adc.sqr1.modify(|_, w| unsafe { w.sq3().bits(12) }); // 1st conversion in regular sequence
    adc.sqr1.modify(|_, w| unsafe { w.sq4().bits(13) }); // 1st conversion in regular sequence
}

pub fn dma_adc2_start(perip: &Peripherals) {
    // enable DMA
    perip.DMA1.ccr1.modify(|_, w| w.en().set_bit());

    let adc = &perip.ADC2;
    // enable ADC
    adc.isr.modify(|_, w| w.adrdy().set_bit());
    adc.cr.modify(|_, w| w.aden().enable()); // ADC enable control
    while adc.isr.read().adrdy().is_not_ready() {
        // Wait for ADC ready
    }
    let tim6 = &perip.TIM6;
    tim6.cr1.modify(|_, w| w.cen().set_bit());

    // Start ADC
    adc.cr.modify(|_, w| w.adstart().start()); // ADC start
}

pub struct Fsr0<'a> {
    perip: &'a Peripherals,
}

impl<'a> Fsr for Fsr0<'a> {
    fn get_force(&self) -> f32 {
        0.0
    }
}

impl<'a> Fsr0<'a> {
    pub fn new(perip: &'a Peripherals) -> Self {
        Self { perip }
    }
    pub fn sigle_conversion(&self) -> u16 {
        let adc = &self.perip.ADC2;
        adc.cr.modify(|_, w| w.adstart().start()); // ADC start
        while adc.isr.read().eoc().is_not_complete() {
            // Wait for ADC complete
        }
        adc.isr.modify(|_, w| w.eoc().clear()); // clear eoc flag

        adc.dr.read().rdata().bits()
    }
}

pub struct Led0<'a> {
    perip: &'a Peripherals,
}

impl<'a> Indicator for Led0<'a> {
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

impl<'a> Indicator for Led1<'a> {
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

pub struct Led2<'a> {
    perip: &'a Peripherals,
}

impl<'a> Indicator for Led2<'a> {
    fn on(&self) {
        let gpioc = &self.perip.GPIOC;
        gpioc.bsrr.write(|w| w.bs15().set());
    }
    fn off(&self) {
        let gpioc = &self.perip.GPIOC;
        gpioc.bsrr.write(|w| w.br15().reset());
    }
    fn toggle(&self) {
        let gpioc = &self.perip.GPIOC;
        if gpioc.odr.read().odr15().is_low() {
            gpioc.bsrr.write(|w| w.bs15().set());
        } else {
            gpioc.bsrr.write(|w| w.br15().reset());
        }
    }
}

impl<'a> Led2<'a> {
    pub fn new(perip: &'a Peripherals) -> Self {
        // GPIOポートの電源投入(クロックの有効化)
        perip.RCC.ahb2enr.modify(|_, w| w.gpiocen().set_bit());

        // gpioモード変更
        let gpioc = &perip.GPIOC;
        gpioc.moder.modify(|_, w| w.moder15().output());

        Self { perip }
    }
}
