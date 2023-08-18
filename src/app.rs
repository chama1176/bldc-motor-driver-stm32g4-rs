use crate::indicator::Indicator;
use motml::encoder::Encoder;
use motml::motor_driver::OutputStatus;
use motml::motor_driver::ThreePhaseMotorDriver;
use motml::motor_driver::ThreePhaseValue;

pub enum State {
    Waiting,
    Calibrating,
    Operating,
    OperatingForcedCommutation,
    OperatingForcedCommutation2,
}
pub struct App<T0, T1, T2, M, E>
where
    T0: Indicator,
    T1: Indicator,
    T2: Indicator,
    M: ThreePhaseMotorDriver,
    E: Encoder<f32>,
{
    tv: f32,
    count: u64,
    calib_count: u8,    // rad
    state: State,
    encoder_offset: f32,
    //
    led0: T0,
    led1: T1,
    led2: T2,
    bldc: M,
    encoder: E,
}

impl<T0, T1, T2, M, E> App<T0, T1, T2, M, E>
where
    T0: Indicator,
    T1: Indicator,
    T2: Indicator,
    M: ThreePhaseMotorDriver,
    E: Encoder<f32>,
{
    pub fn new(led0: T0, led1: T1, led2: T2, bldc: M, encoder: E) -> Self {
        Self {
            tv: 0.0,
            count: 0,
            calib_count: 0,
            state: State::Waiting,
            encoder_offset: 0.23163112,
            led0,
            led1,
            led2,
            bldc,
            encoder,
        }
    }
    #[rustfmt::skip]
    pub fn periodic_task(&mut self) {
        self.led0.toggle();
        self.led1.toggle();
        self.led2.toggle();

        self.calib_count = 7;
        let mut tp: ThreePhaseValue<f32> = ThreePhaseValue { u: 0., v: 0., w: 0. };
        let mut tpe: ThreePhaseValue<OutputStatus> = ThreePhaseValue { u: OutputStatus::Enable, v: OutputStatus::Enable, w: OutputStatus::Enable };
        match self.state {
            State::OperatingForcedCommutation =>{
                match ((self.count as f32/(10.0*1.0)) as u64)%6 {
                    0 => {
                        tp = ThreePhaseValue{u: 0.25, v: 0., w: 0.};
                        tpe = ThreePhaseValue{u: OutputStatus::Enable, v: OutputStatus::Enable, w: OutputStatus::Disable};
                    },
                    1 => {
                        tp = ThreePhaseValue{u: 0.25, v: 0., w: 0.};
                        tpe = ThreePhaseValue{u: OutputStatus::Enable, v: OutputStatus::Disable, w: OutputStatus::Enable};
                    },
                    2 => {
                        tp = ThreePhaseValue{u: 0., v: 0.25, w: 0.};
                        tpe = ThreePhaseValue{u: OutputStatus::Disable, v: OutputStatus::Enable, w: OutputStatus::Enable};
                    },
                    3 => {
                        tp = ThreePhaseValue{u: 0., v: 0.25, w: 0.};
                        tpe = ThreePhaseValue{u: OutputStatus::Enable, v: OutputStatus::Enable, w: OutputStatus::Disable};
                    },
                    4 => {
                        tp = ThreePhaseValue{u: 0., v: 0., w: 0.25};
                        tpe = ThreePhaseValue{u: OutputStatus::Enable, v: OutputStatus::Disable, w: OutputStatus::Enable};
                    },
                    5 => {
                        tp = ThreePhaseValue{u: 0., v: 0., w: 0.25};
                        tpe = ThreePhaseValue{u: OutputStatus::Disable, v: OutputStatus::Enable, w: OutputStatus::Enable};
                    },
                    6_u64..=u64::MAX => (),
                }
            }
            State::OperatingForcedCommutation2 =>{
                let s =((self.count as f32/(1000.0*1.0)) as u64)%6; 
                match s {
                    0 => tp = ThreePhaseValue{u: 0.25, v: 0., w: 0.},
                    1 => tp = ThreePhaseValue{u: 0.25, v: 0.25, w: 0.},
                    2 => tp = ThreePhaseValue{u: 0., v: 0.25, w: 0.},
                    3 => tp = ThreePhaseValue{u: 0., v: 0.25, w: 0.25},
                    4 => tp = ThreePhaseValue{u: 0., v: 0., w: 0.25},
                    5 => tp = ThreePhaseValue{u: 0.25, v: 0., w: 0.25},
                    6_u64..=u64::MAX => (),
                }
                self.calib_count = s as u8;
            }
            State::Operating => {
                tp = ThreePhaseValue{ u: 0., v: 0., w: 0. };
            }
            State::Calibrating => {
                tp = ThreePhaseValue{ u: 0.25, v: 0., w: 0. };
            }
            _ =>{
                tp = ThreePhaseValue{ u: 0., v: 0., w: 0. };
            }
        }
        self.bldc.set_pwm(tp);
        self.bldc.modify_pwm_output(tpe);
    }
    pub fn set_target_velocity(&mut self, tv: f32) {
        self.tv = tv;
    }
    pub fn set_count(&mut self, c: u64) {
        self.count = c;
    }
    pub fn set_sate(&mut self, s: State) {
        self.state = s
    }
    pub fn calib_count(&mut self) -> u8 {
        self.calib_count
    }
    pub fn read_encoder_data(&mut self) -> f32 {
        self.encoder.get_angle().unwrap() - self.encoder_offset
    }
}
