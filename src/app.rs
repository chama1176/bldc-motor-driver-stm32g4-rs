use crate::indicator::Indicator;
use motml::encoder::Encoder;
use motml::motor::ThreePhaseMotor;
use motml::motor_driver::DQVoltage;
use motml::motor_driver::OutputStatus;
use motml::motor_driver::ThreePhaseMotorDriver;
use motml::motor_driver::ThreePhaseValue;
use motml::motor_driver::ThreePhaseVoltage;
use motml::utils::Deg;

pub enum State {
    Waiting,
    Calibrating,
    Operating,
    OperatingForcedCommutation,
    OperatingForcedCommutation2,
    Operating120DegreeDrive,
    OperatingQPhase,
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
    calib_count: u8, // rad
    state: State,
    encoder_offset: f32,
    //
    led0: T0,
    led1: T1,
    led2: T2,
    bldc: M,
    encoder: E,
    motor: ThreePhaseMotor,
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
            motor: ThreePhaseMotor::new(12),
        }
    }
    #[rustfmt::skip]
    pub fn periodic_task(&mut self) {
        self.led0.toggle();
        self.led1.toggle();
        self.led2.toggle();

        self.calib_count = 7;
        let mut tp = ThreePhaseVoltage::<f32> { v_u: 0., v_v: 0., v_w: 0. };
        let mut tpe: ThreePhaseValue<OutputStatus> = ThreePhaseValue { u: OutputStatus::Enable, v: OutputStatus::Enable, w: OutputStatus::Enable };
        match self.state {
            State::OperatingForcedCommutation =>{
                match ((self.count as f32/(10.0*1.0)) as u64)%6 {
                    0 => {
                        tp = ThreePhaseVoltage{v_u: 0.25, v_v: 0., v_w: 0.};
                        tpe = ThreePhaseValue{u: OutputStatus::Enable, v: OutputStatus::Enable, w: OutputStatus::Disable};
                    },
                    1 => {
                        tp = ThreePhaseVoltage{v_u: 0.25, v_v: 0., v_w: 0.};
                        tpe = ThreePhaseValue{u: OutputStatus::Enable, v: OutputStatus::Disable, w: OutputStatus::Enable};
                    },
                    2 => {
                        tp = ThreePhaseVoltage{v_u: 0., v_v: 0.25, v_w: 0.};
                        tpe = ThreePhaseValue{u: OutputStatus::Disable, v: OutputStatus::Enable, w: OutputStatus::Enable};
                    },
                    3 => {
                        tp = ThreePhaseVoltage{v_u: 0., v_v: 0.25, v_w: 0.};
                        tpe = ThreePhaseValue{u: OutputStatus::Enable, v: OutputStatus::Enable, w: OutputStatus::Disable};
                    },
                    4 => {
                        tp = ThreePhaseVoltage{v_u: 0., v_v: 0., v_w: 0.25};
                        tpe = ThreePhaseValue{u: OutputStatus::Enable, v: OutputStatus::Disable, w: OutputStatus::Enable};
                    },
                    5 => {
                        tp = ThreePhaseVoltage{v_u: 0., v_v: 0., v_w: 0.25};
                        tpe = ThreePhaseValue{u: OutputStatus::Disable, v: OutputStatus::Enable, w: OutputStatus::Enable};
                    },
                    6_u64..=u64::MAX => (),
                }
            }
            State::Operating120DegreeDrive =>{
                let ma = self.read_encoder_data();
                let ea = self.motor.mechanical_angle_to_electrical_angle(ma);
                let eadeg: f32 = ea.rad2deg();
                if 0.0 <= eadeg && eadeg < 60.0 {
                    tp = ThreePhaseVoltage{v_u: 0.25, v_v: 0., v_w: 0.};
                    tpe = ThreePhaseValue{u: OutputStatus::Enable, v: OutputStatus::Enable, w: OutputStatus::Disable};
                }
                else if eadeg < 120.0 {
                    tp = ThreePhaseVoltage{v_u: 0.25, v_v: 0., v_w: 0.};
                    tpe = ThreePhaseValue{u: OutputStatus::Enable, v: OutputStatus::Disable, w: OutputStatus::Enable};
                }
                else if eadeg < 180.0 {
                    tp = ThreePhaseVoltage{v_u: 0., v_v: 0.25, v_w: 0.};
                    tpe = ThreePhaseValue{u: OutputStatus::Disable, v: OutputStatus::Enable, w: OutputStatus::Enable};
                }
                else if eadeg < 240.0 {
                    tp = ThreePhaseVoltage{v_u: 0., v_v: 0.25, v_w: 0.};
                    tpe = ThreePhaseValue{u: OutputStatus::Enable, v: OutputStatus::Enable, w: OutputStatus::Disable};
                }
                else if eadeg < 300.0 {
                    tp = ThreePhaseVoltage{v_u: 0., v_v: 0., v_w: 0.25};
                    tpe = ThreePhaseValue{u: OutputStatus::Enable, v: OutputStatus::Disable, w: OutputStatus::Enable};
                }
                else if eadeg < 360.0 {
                    tp = ThreePhaseVoltage{v_u: 0., v_v: 0., v_w: 0.25};
                    tpe = ThreePhaseValue{u: OutputStatus::Disable, v: OutputStatus::Enable, w: OutputStatus::Enable};
                }
                else {
                    assert!(false);
                }
            }
            State::OperatingQPhase =>{
                let ma = self.read_encoder_data();
                let ea = self.motor.mechanical_angle_to_electrical_angle(ma);

                let tpv = DQVoltage{v_d:0.0, v_q:2.0}.to_three_phase(ea);
                tp = ThreePhaseVoltage{
                    v_u: (tpv.v_u + 6.0) / 12.0,
                    v_v: (tpv.v_v + 6.0) / 12.0,
                    v_w: (tpv.v_w + 6.0) / 12.0
                }
            }
            State::OperatingForcedCommutation2 =>{
                let s =((self.count as f32/(1000.0*1.0)) as u64)%6;
                match s {
                    0 => tp = ThreePhaseVoltage{v_u: 0.25, v_v: 0., v_w: 0.},
                    1 => tp = ThreePhaseVoltage{v_u: 0.25, v_v: 0.25, v_w: 0.},
                    2 => tp = ThreePhaseVoltage{v_u: 0., v_v: 0.25, v_w: 0.},
                    3 => tp = ThreePhaseVoltage{v_u: 0., v_v: 0.25, v_w: 0.25},
                    4 => tp = ThreePhaseVoltage{v_u: 0., v_v: 0., v_w: 0.25},
                    5 => tp = ThreePhaseVoltage{v_u: 0.25, v_v: 0., v_w: 0.25},
                    6_u64..=u64::MAX => (),
                }
                self.calib_count = s as u8;
            }
            State::Operating => {
                tp = ThreePhaseVoltage{ v_u: 0., v_v: 0., v_w: 0. };
            }
            State::Calibrating => {
                tp = ThreePhaseVoltage{ v_u: 0.25, v_v: 0., v_w: 0. };
            }
            _ =>{
                tp = ThreePhaseVoltage{ v_u: 0., v_v: 0., v_w: 0. };
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
        (self.encoder.get_angle().unwrap() - self.encoder_offset).wrap_to_2pi()
    }
}
