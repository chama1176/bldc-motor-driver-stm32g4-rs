// ros::time::now的なのが欲しい。cortexlibでできる？

pub struct Time {
    time: f32,
}

impl Time {
    pub fn now(&self) -> Self {
        
    }
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
