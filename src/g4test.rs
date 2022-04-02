use stm32g4::stm32g431::Peripherals;
use crate::led::Led;

pub trait Potensio {
    fn get_voltage(&self) -> f32;
    fn get_value(&self) -> u16;
}    

pub trait NeoPixelRing {
    fn get_length(&self) -> f32;
    fn get_type(&self);
    fn set_pin(&self);
    fn set_pixel_color(&self);
    fn get_pixel_color(&self);
    fn set_brightness(&self);
    fn get_brightness(&self);
    fn clear(&self);

    // implemeation for private function
    // fn set_length(&self);
    // fn set_type(&self);
    
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
        
    }
}

impl<'a> Led0<'a> {
    pub fn new(perip: &'a Peripherals) -> Self {
        Self{
            perip,
        }
    }
    pub fn init(&self) {
        // GPIOCポートの電源投入(クロックの有効化)
        self.perip.RCC.ahb2enr.modify(|_,w| w.gpiocen().set_bit());

        // gpio初期化(PC13を出力に指定)
        let gpioc = &self.perip.GPIOC;
        gpioc.moder.modify(|_,w| w.moder13().output());
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
        
    }
}
impl<'a> Led1<'a> {
    pub fn new(perip: &'a Peripherals) -> Self {
        Self{
            perip,
        }
    }
    pub fn init(&self) {
        // GPIOCポートの電源投入(クロックの有効化)
        self.perip.RCC.ahb2enr.modify(|_,w| w.gpiocen().set_bit());

        // gpio初期化(PC14を出力に指定)
        let gpioc = &self.perip.GPIOC;
        gpioc.moder.modify(|_,w| w.moder14().output());
    }
}
