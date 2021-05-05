//! Configurateion of the HW for the Stepper Motors

#![allow(dead_code)]

use embedded_hal::digital::v2::OutputPin;
use stm32f1xx_hal::gpio::{Input, Output, PullUp, PushPull};

pub type PinEnableSteppers = crate::arduino_shield::D8<Output<PushPull>>;

type PinXDirection = crate::arduino_shield::D5<Output<PushPull>>;
type PinXStep = crate::arduino_shield::D2<Output<PushPull>>;
type PinEndstopX = crate::arduino_shield::D9<Input<PullUp>>;

type PinYDirection = crate::arduino_shield::D6<Output<PushPull>>;
type PinYStep = crate::arduino_shield::D3<Output<PushPull>>;
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
}

impl StepperPins {
    pub fn new(
        x_step: PinXStep,
        y_step: PinYStep,
        z_step: PinZStep,
        x_direction: PinXDirection,
        y_direction: PinYDirection,
        z_direction: PinZDirection,
    ) -> Self {
        Self {
            x_step,
            y_step,
            z_step,
            x_direction,
            y_direction,
            z_direction,
        }
    }

    pub fn start_step(&mut self, step_x: bool, step_y: bool, step_z: bool) {
        if step_x {
            self.x_step.set_high().unwrap();
        }
        if step_y {
            self.y_step.set_high().unwrap();
        }
        if step_z {
            self.z_step.set_high().unwrap();
        }
    }

    pub fn end_step(&mut self) {
        self.x_step.set_low().unwrap();
        self.y_step.set_low().unwrap();
        self.z_step.set_low().unwrap();
    }
}
