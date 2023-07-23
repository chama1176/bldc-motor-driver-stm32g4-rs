pub struct ThreePhase<T> {
    pub u: T,
    pub v: T,
    pub w: T,
}

pub trait ThreePhaseMotorDriver {
    fn enable(&self);
    fn disable(&self);
    fn set_u_pwm(&self, p: u32);
    fn set_v_pwm(&self, p: u32);
    fn set_w_pwm(&self, p: u32);
}
