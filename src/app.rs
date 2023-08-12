use crate::indicator::Indicator;
use crate::three_phase_motor_driver::ThreePhase;
use crate::three_phase_motor_driver::ThreePhaseMotorDriver;

pub enum State {
    Waiting,
    Calibrating,
    Operating,
}
pub struct App<T0, T1, T2, M>
where
    T0: Indicator,
    T1: Indicator,
    T2: Indicator,
    M: ThreePhaseMotorDriver,
{
    tv: f32,
    count: u64,
    // 
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
            tv: 0.0, 
            count: 0,
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

        match ((self.count as f32/(10.0*1.0)) as u64)%6 {
            0 => tp = ThreePhase{u: 200, v: 0, w: 0},
            1 => tp = ThreePhase{u: 200, v: 200, w: 0},
            2 => tp = ThreePhase{u: 0, v: 200, w: 0},
            3 => tp = ThreePhase{u: 0, v: 200, w: 200},
            4 => tp = ThreePhase{u: 0, v: 0, w: 200},
            5 => tp = ThreePhase{u: 200, v: 0, w: 200},
            6_u64..=u64::MAX => (),
        }
        if self.tv < 0.1 {
            tp = ThreePhase{ u: 0, v: 0, w: 0 };
        }
        self.bldc.set_u_pwm(tp.u);
        self.bldc.set_v_pwm(tp.v);
        self.bldc.set_w_pwm(tp.w);
    }
    pub fn set_target_velocity(&mut self, tv: f32) {
        self.tv = tv;
    }
    pub fn set_count(&mut self, c: u64) {
        self.count = c;
    }
}
