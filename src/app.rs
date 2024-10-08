use core::cmp::max;

use crate::bldc_motor_driver_stm32g4::CurrentSensor;
use crate::indicator::Indicator;
use motml::encoder::Encoder;
use motml::motor::ThreePhaseMotor;
use motml::motor_driver::DQCurrent;
use motml::motor_driver::DQVoltage;
use motml::motor_driver::OutputStatus;
use motml::motor_driver::ThreePhaseCurrent;
use motml::motor_driver::ThreePhaseMotorDriver;
use motml::motor_driver::ThreePhaseValue;
use motml::motor_driver::ThreePhaseVoltage;
use motml::utils::Deg;

pub enum ControlMode {
    Waiting,
    Calibrating,
    OperatingForcedCommutation,
    OperatingForcedCommutation2,
    Operating120DegreeDrive,
    OperatingQPhase,
    FieldOrientedControl,
}
pub struct App<T0, T1, M, E>
//, T2, M>
where
    T0: Indicator,
    T1: Indicator,
    M: ThreePhaseMotorDriver,
    E: Encoder<f32>,
{
    // [Param value]
    tv: f32,
    count: u32,
    calib_count: u8, // rad
    control_mode: ControlMode,
    encoder_offset: f32,
    control_err_integral: DQCurrent<f32>,
    // [Debug]
    pub last_electrical_angle: f32, // debug
    pub last_mechanical_angle: f32, // debug
    pub last_current: ThreePhaseCurrent<f32>,
    pub last_dq_current: DQCurrent<f32>,
    pub last_tim_count: u32,
    pub diff_count: u32,
    // [Device]
    led0: T0,
    led1: T1,
    bldc: M,
    encoder: E,
    pub motor: ThreePhaseMotor,
    current_sensor: CurrentSensor,
}

impl<T0, T1, M, E> App<T0, T1, M, E>
where
    T0: Indicator,
    T1: Indicator,
    M: ThreePhaseMotorDriver,
    E: Encoder<f32>,
{
    pub fn new(led0: T0, led1: T1, bldc: M, encoder: E, current_sensor: CurrentSensor) -> Self {
        Self {
            tv: 0.0,
            count: 0,
            calib_count: 0,
            control_mode: ControlMode::Waiting,
            encoder_offset: 0.238,
            control_err_integral: DQCurrent::default(),
            // encoder_offset: 0.0,
            last_electrical_angle: 0.0,
            last_mechanical_angle: 0.0,
            last_current: ThreePhaseCurrent::default(),
            last_dq_current: DQCurrent::default(),
            last_tim_count: 0,
            diff_count: 0,
            led0,
            led1,
            bldc,
            encoder,
            motor: ThreePhaseMotor::new(12),
            current_sensor,
        }
    }
    #[rustfmt::skip]
    pub fn periodic_task(&mut self) {
        self.led0.toggle();
        let ma = self.read_encoder_data();
        let ea = self.motor.mechanical_angle_to_electrical_angle(ma);
        let current = self.current_sensor.get_current();
        let dq_current = current.to_dq(ea);

        self.last_mechanical_angle = ma;    
        self.last_electrical_angle = ea;
        self.last_current = current;
        self.last_dq_current = dq_current;
        self.calib_count = 7;

        let mut tp = ThreePhaseVoltage::<f32> { v_u: 0., v_v: 0., v_w: 0. };
        let mut tpe: ThreePhaseValue<OutputStatus> = ThreePhaseValue { u: OutputStatus::Disable, v: OutputStatus::Disable, w: OutputStatus::Disable };
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
                tpe = ThreePhaseValue { u: OutputStatus::Enable, v: OutputStatus::Enable, w: OutputStatus::Enable };
                let eadeg: f32 = ea.rad2deg();
                if 210.0 <= eadeg && eadeg < 270.0 {
                    tp = ThreePhaseVoltage{v_u: 0.1, v_v: 0., v_w: 0.};
                    tpe = ThreePhaseValue{u: OutputStatus::Enable, v: OutputStatus::Enable, w: OutputStatus::Disable};
                }
                else if 270.0 <= eadeg && eadeg < 330.0 {
                    tp = ThreePhaseVoltage{v_u: 0.1, v_v: 0., v_w: 0.};
                    tpe = ThreePhaseValue{u: OutputStatus::Enable, v: OutputStatus::Disable, w: OutputStatus::Enable};
                }
                else if (330.0 <= eadeg && eadeg < 360.0) || (0.0 <= eadeg && eadeg < 30.0){
                    tp = ThreePhaseVoltage{v_u: 0., v_v: 0.1, v_w: 0.};
                    tpe = ThreePhaseValue{u: OutputStatus::Disable, v: OutputStatus::Enable, w: OutputStatus::Enable};
                }
                else if 30.0 <= eadeg && eadeg < 90.0 {
                    tp = ThreePhaseVoltage{v_u: 0., v_v: 0.1, v_w: 0.};
                    tpe = ThreePhaseValue{u: OutputStatus::Enable, v: OutputStatus::Enable, w: OutputStatus::Disable};
                }
                else if 90.0 <= eadeg && eadeg < 150.0 {
                    tp = ThreePhaseVoltage{v_u: 0., v_v: 0., v_w: 0.1};
                    tpe = ThreePhaseValue{u: OutputStatus::Enable, v: OutputStatus::Disable, w: OutputStatus::Enable};
                }
                else if 150.0 <= eadeg && eadeg < 210.0 {
                    tp = ThreePhaseVoltage{v_u: 0., v_v: 0., v_w: 0.1};
                    tpe = ThreePhaseValue{u: OutputStatus::Disable, v: OutputStatus::Enable, w: OutputStatus::Enable};
                }
                else {
                    assert!(false);
                }
            }
            ControlMode::OperatingQPhase =>{
                tpe = ThreePhaseValue { u: OutputStatus::Enable, v: OutputStatus::Enable, w: OutputStatus::Enable };
                let tpv = DQVoltage{v_d:0.0, v_q:self.tv * 1.0}.to_three_phase(ea);
                tp = ThreePhaseVoltage{
                    v_u: (tpv.v_u + 6.0) / 12.0,
                    v_v: (tpv.v_v + 6.0) / 12.0,
                    v_w: (tpv.v_w + 6.0) / 12.0
                }
            }
            ControlMode::FieldOrientedControl =>{
                let ref_current = DQCurrent{
                    i_d: 0.0,
                    i_q: self.tv * 0.5,
                };
                let kp = 2.2;
                let ki = 0.01;

                let err_current = DQCurrent{
                    i_d: dq_current.i_d - ref_current.i_d,
                    i_q: dq_current.i_q - ref_current.i_q,
                };

                self.control_err_integral.i_d += err_current.i_d;
                self.control_err_integral.i_q += err_current.i_q;

                // self.control_err_integral.i_d = self.control_err_integral.i_d.clamp(-1.0, 1.0);
                // self.control_err_integral.i_q = self.control_err_integral.i_q.clamp(-1.0, 1.0);
                
                tpe = ThreePhaseValue { u: OutputStatus::Enable, v: OutputStatus::Enable, w: OutputStatus::Enable };
                let tpv = DQVoltage{
                    v_d: -kp * err_current.i_d - ki * self.control_err_integral.i_d,
                    v_q: -kp * err_current.i_q - ki * self.control_err_integral.i_q,
                }.to_three_phase(ea);
                tp = ThreePhaseVoltage{
                    v_u: (tpv.v_u + 6.0) / 12.0,
                    v_v: (tpv.v_v + 6.0) / 12.0,
                    v_w: (tpv.v_w + 6.0) / 12.0
                }
            }
            ControlMode::OperatingForcedCommutation2 =>{
                tpe = ThreePhaseValue { u: OutputStatus::Enable, v: OutputStatus::Enable, w: OutputStatus::Enable };
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
            ControlMode::Calibrating => {
                // tpe = ThreePhaseValue { u: OutputStatus::Enable, v: OutputStatus::Enable, w: OutputStatus::Enable };
                // tp = ThreePhaseVoltage{ v_u: 0.6, v_v: 0.4, v_w: 0.4 };
                tpe = ThreePhaseValue { u: OutputStatus::Enable, v: OutputStatus::Enable, w: OutputStatus::Disable };
                tp = ThreePhaseVoltage{ v_u: 0.52, v_v: 0.48, v_w: 0.0 };
            }
            _ =>{
                tpe = ThreePhaseValue { u: OutputStatus::Disable, v: OutputStatus::Disable, w: OutputStatus::Disable };
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
