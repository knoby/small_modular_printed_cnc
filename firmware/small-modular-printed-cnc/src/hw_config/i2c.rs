#![allow(dead_code)]
#![allow(clippy::clippy::upper_case_acronyms)]

use stm32f1xx_hal::gpio::{Alternate, OpenDrain};

type SCL = crate::arduino_shield::D15<Alternate<OpenDrain>>;
type SDA = crate::arduino_shield::D14<Alternate<OpenDrain>>;

pub type I2CDevice = stm32f1xx_hal::i2c::BlockingI2c<stm32f1xx_hal::pac::I2C1, (SDA, SCL)>;
