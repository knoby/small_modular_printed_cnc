//! Configurateion of the HW for the Stepper Motors

#![allow(dead_code)]

use embedded_hal::digital::v2::OutputPin;
use stm32f1xx_hal::gpio::{Input, Output, PullUp, PushPull};

pub type PinEnableSteppers = crate::arduino_shield::D8<Output<PushPull>>;

type PinXDirection = crate::arduino_shield::D14<Output<PushPull>>;
type PinXStep = crate::arduino_shield::D12<Output<PushPull>>;
type PinEndstopX = crate::arduino_shield::D9<Input<PullUp>>;

type PinYDirection = crate::arduino_shield::D6<Output<PushPull>>;
type PinYStep = crate::arduino_shield::D13<Output<PushPull>>;
type PinEndstopY = crate::arduino_shield::D10<Input<PullUp>>;

type PinZDirection = crate::arduino_shield::D7<Output<PushPull>>;
type PinZStep = crate::arduino_shield::D4<Output<PushPull>>;
type PinEndstopZ = crate::arduino_shield::D11<Input<PullUp>>;

pub struct StepperPins {
    x_step: PinXStep,
    y_step: PinYStep,
    z_step: PinZStep,

    x_direction: PinXDirection,
    y_direction: PinYDirection,
    z_direction: PinZDirection,

    enable: PinEnableSteppers,
}

impl StepperPins {
    pub fn new(
        mut x_step: PinXStep,
        mut y_step: PinYStep,
        mut z_step: PinZStep,
        mut x_direction: PinXDirection,
        mut y_direction: PinYDirection,
        mut z_direction: PinZDirection,
        mut enable: PinEnableSteppers,
    ) -> Self {
        x_step.set_high().unwrap();
        y_step.set_high().unwrap();
        z_step.set_high().unwrap();
        x_direction.set_high().unwrap();
        y_direction.set_high().unwrap();
        z_direction.set_high().unwrap();
        enable.set_high().unwrap();
        Self {
            x_step,
            y_step,
            z_step,
            x_direction,
            y_direction,
            z_direction,
            enable,
        }
    }

    pub fn enable(&mut self) {
        self.enable.set_low().unwrap();
    }

    pub fn disable(&mut self) {
        self.enable.set_high().unwrap();
    }

    pub fn start_step(
        &mut self,
        step_x: crate::ramp_gen::Direction,
        step_y: crate::ramp_gen::Direction,
        step_z: crate::ramp_gen::Direction,
    ) {
        match step_x {
            crate::ramp_gen::Direction::Positive => self.x_direction.set_high().unwrap(),
            crate::ramp_gen::Direction::Negative => self.x_direction.set_low().unwrap(),
            crate::ramp_gen::Direction::StandStill => (),
        };
        match step_y {
            crate::ramp_gen::Direction::Positive => self.y_direction.set_high().unwrap(),
            crate::ramp_gen::Direction::Negative => self.y_direction.set_low().unwrap(),
            crate::ramp_gen::Direction::StandStill => {}
        };
        match step_z {
            crate::ramp_gen::Direction::Positive => self.z_direction.set_high().unwrap(),
            crate::ramp_gen::Direction::Negative => self.z_direction.set_low().unwrap(),
            crate::ramp_gen::Direction::StandStill => {}
        }
        if step_x != crate::ramp_gen::Direction::StandStill {
            self.x_step.set_high().unwrap();
        };
        if step_y != crate::ramp_gen::Direction::StandStill {
            self.y_step.set_high().unwrap();
        };
        if step_z != crate::ramp_gen::Direction::StandStill {
            self.z_step.set_high().unwrap();
        };
    }

    pub fn end_step(&mut self) {
        self.x_step.set_low().unwrap();
        self.y_step.set_low().unwrap();
        self.z_step.set_low().unwrap();
    }
}
