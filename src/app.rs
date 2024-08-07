use crate::indicator::Indicator;
use motml::encoder::Encoder;
use motml::motor::ThreePhaseMotor;
use motml::motor_driver::DQVoltage;
use motml::motor_driver::OutputStatus;
use motml::motor_driver::ThreePhaseMotorDriver;
use motml::motor_driver::ThreePhaseValue;
use motml::motor_driver::ThreePhaseVoltage;
use motml::utils::Deg;

pub enum ControlMode {
    Waiting,
    Calibrating,
    Operating,
    OperatingForcedCommutation,
    OperatingForcedCommutation2,
    Operating120DegreeDrive,
    OperatingQPhase,
}
pub struct App<T0, T1, M, E>
//, T2, M>
where
    T0: Indicator,
    T1: Indicator,
    M: ThreePhaseMotorDriver,
    E: Encoder<f32>,
{
    tv: f32,
    count: u32,
    calib_count: u8, // rad
    control_mode: ControlMode,
    encoder_offset: f32,
    //
    led0: T0,
    led1: T1,
    bldc: M,
    encoder: E,
    motor: ThreePhaseMotor,
}

impl<T0, T1, M, E> App<T0, T1, M, E>
where
    T0: Indicator,
    T1: Indicator,
    M: ThreePhaseMotorDriver,
    E: Encoder<f32>,
{
    pub fn new(led0: T0, led1: T1, bldc: M, encoder: E) -> Self {
        Self {
            tv: 0.0,
            count: 0,
            calib_count: 0,
            control_mode: ControlMode::Waiting,
            encoder_offset: 0.23163112,
            led0,
            led1,
            bldc,
            encoder,
            motor: ThreePhaseMotor::new(12),
        }
    }
    #[rustfmt::skip]
    pub fn periodic_task(&mut self) {
        self.led0.toggle();
        self.calib_count = 7;
        let mut tp = ThreePhaseVoltage::<f32> { v_u: 0., v_v: 0., v_w: 0. };
        let mut tpe: ThreePhaseValue<OutputStatus> = ThreePhaseValue { u: OutputStatus::Enable, v: OutputStatus::Enable, w: OutputStatus::Enable };
        match self.control_mode {
            ControlMode::OperatingForcedCommutation =>{
                match ((self.count as f32/(10.0*1.0)) as u32)%6 {
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
                    6_u32..=u32::MAX => (),
                }
            }
            ControlMode::Operating120DegreeDrive =>{
                let ma = self.read_encoder_data();
                // let ea = self.motor.mechanical_angle_to_electrical_angle(ma);
                // let eadeg: f32 = ea.rad2deg();
                // if 0.0 <= eadeg && eadeg < 60.0 {
                //     tp = ThreePhaseVoltage{v_u: 0.25, v_v: 0., v_w: 0.};
                //     tpe = ThreePhaseValue{u: OutputStatus::Enable, v: OutputStatus::Enable, w: OutputStatus::Disable};
                // }
                // else if eadeg < 120.0 {
                //     tp = ThreePhaseVoltage{v_u: 0.25, v_v: 0., v_w: 0.};
                //     tpe = ThreePhaseValue{u: OutputStatus::Enable, v: OutputStatus::Disable, w: OutputStatus::Enable};
                // }
                // else if eadeg < 180.0 {
                //     tp = ThreePhaseVoltage{v_u: 0., v_v: 0.25, v_w: 0.};
                //     tpe = ThreePhaseValue{u: OutputStatus::Disable, v: OutputStatus::Enable, w: OutputStatus::Enable};
                // }
                // else if eadeg < 240.0 {
                //     tp = ThreePhaseVoltage{v_u: 0., v_v: 0.25, v_w: 0.};
                //     tpe = ThreePhaseValue{u: OutputStatus::Enable, v: OutputStatus::Enable, w: OutputStatus::Disable};
                // }
                // else if eadeg < 300.0 {
                //     tp = ThreePhaseVoltage{v_u: 0., v_v: 0., v_w: 0.25};
                //     tpe = ThreePhaseValue{u: OutputStatus::Enable, v: OutputStatus::Disable, w: OutputStatus::Enable};
                // }
                // else if eadeg < 360.0 {
                //     tp = ThreePhaseVoltage{v_u: 0., v_v: 0., v_w: 0.25};
                //     tpe = ThreePhaseValue{u: OutputStatus::Disable, v: OutputStatus::Enable, w: OutputStatus::Enable};
                // }
                // else {
                //     assert!(false);
                // }
            }
            ControlMode::OperatingQPhase =>{
                let ma = self.read_encoder_data();
                let ea = self.motor.mechanical_angle_to_electrical_angle(ma);

                let tpv = DQVoltage{v_d:0.0, v_q:self.tv * 3.0}.to_three_phase(ea);
                tp = ThreePhaseVoltage{
                    v_u: (tpv.v_u + 6.0) / 12.0,
                    v_v: (tpv.v_v + 6.0) / 12.0,
                    v_w: (tpv.v_w + 6.0) / 12.0
                }
            }
            ControlMode::OperatingForcedCommutation2 =>{
                let s =((self.count as f32/(10.0*1.0)) as u32)%6;
                match s {
                    0 => tp = ThreePhaseVoltage{v_u: 0.25, v_v: 0., v_w: 0.},
                    1 => tp = ThreePhaseVoltage{v_u: 0.25, v_v: 0.25, v_w: 0.},
                    2 => tp = ThreePhaseVoltage{v_u: 0., v_v: 0.25, v_w: 0.},
                    3 => tp = ThreePhaseVoltage{v_u: 0., v_v: 0.25, v_w: 0.25},
                    4 => tp = ThreePhaseVoltage{v_u: 0., v_v: 0., v_w: 0.25},
                    5 => tp = ThreePhaseVoltage{v_u: 0.25, v_v: 0., v_w: 0.25},
                    6_u32..=u32::MAX => (),
                }
                self.calib_count = s as u8;
            }
            ControlMode::Operating => {
                tp = ThreePhaseVoltage{ v_u: 0., v_v: 0., v_w: 0. };
            }
            ControlMode::Calibrating => {
                tp = ThreePhaseVoltage{ v_u: 0.3, v_v: 0.3, v_w: 0.3 };
            }
            _ =>{
                tp = ThreePhaseVoltage{ v_u: 0., v_v: 0., v_w: 0. };
            }
        }
        self.bldc.set_pwm(tp);
        self.bldc.modify_pwm_output(tpe);
    }
    pub fn led_tick_task(&mut self) {
        self.led1.toggle();
    }
    pub fn set_target_velocity(&mut self, tv: f32) {
        self.tv = tv;
    }
    pub fn set_count(&mut self, c: u32) {
        self.count = c;
    }
    pub fn set_control_mode(&mut self, c: ControlMode) {
        self.control_mode = c
    }
    pub fn calib_count(&mut self) -> u8 {
        self.calib_count
    }
    pub fn read_encoder_data(&mut self) -> f32 {
        (self.encoder.get_angle().unwrap() - self.encoder_offset).wrap_to_2pi()
    }
}
