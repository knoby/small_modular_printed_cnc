#![allow(dead_code)]

use stm32f1xx_hal::gpio::{Alternate, OpenDrain, PushPull};

type RxPin = stm32f1xx_hal::gpio::gpioa::PA3<Alternate<OpenDrain>>;
type TxPin = stm32f1xx_hal::gpio::gpioa::PA2<Alternate<PushPull>>;

type SerialDevice = stm32f1xx_hal::serial::Serial<stm32f1xx_hal::pac::USART2, (TxPin, RxPin)>;

pub type SerialTx = stm32f1xx_hal::serial::Tx2;
pub type SerialRx = stm32f1xx_hal::serial::Rx2;

type DmaChannel = stm32f1xx_hal::dma::dma1::C5;
