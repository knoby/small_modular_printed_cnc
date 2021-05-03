//! Configurateion of the HW for the Stepper Motors

#![allow(dead_code)]

use stm32f1xx_hal::gpio::{Input, Output, PullUp, PushPull};

pub type PinEnableSteppers = crate::arduino_shield::D8<Output<PushPull>>;

pub type PinXDirection = crate::arduino_shield::D5<Output<PushPull>>;
pub type PinXStep = crate::arduino_shield::D2<Output<PushPull>>;
pub type PinEndstopX = crate::arduino_shield::D9<Input<PullUp>>;

pub type YDirection = crate::arduino_shield::D6<Output<PushPull>>;
pub type PinYStep = crate::arduino_shield::D3<Output<PushPull>>;
pub type PinEndstopY = crate::arduino_shield::D10<Input<PullUp>>;

pub type ZDirection = crate::arduino_shield::D7<Output<PushPull>>;
pub type PinZStep = crate::arduino_shield::D4<Output<PushPull>>;
pub type PinEndstopZ = crate::arduino_shield::D11<Input<PullUp>>;
