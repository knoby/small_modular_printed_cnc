#![allow(dead_code)]
///! Type alias for the Arduino shied header on the nucleo board

pub type A0<T> = stm32f1xx_hal::gpio::gpioa::PA0<T>;
pub type A1<T> = stm32f1xx_hal::gpio::gpioa::PA1<T>;
pub type A2<T> = stm32f1xx_hal::gpio::gpioa::PA4<T>;
pub type A3<T> = stm32f1xx_hal::gpio::gpiob::PB0<T>;
pub type A4<T> = stm32f1xx_hal::gpio::gpioc::PC1<T>;
pub type A5<T> = stm32f1xx_hal::gpio::gpioc::PC0<T>;

pub type D0<T> = stm32f1xx_hal::gpio::gpioa::PA3<T>;
pub type D1<T> = stm32f1xx_hal::gpio::gpioa::PA2<T>;
pub type D2<T> = stm32f1xx_hal::gpio::gpioa::PA10<T>;
pub type D3<T> = stm32f1xx_hal::gpio::gpiob::PB3<T>;
pub type D4<T> = stm32f1xx_hal::gpio::gpiob::PB5<T>;
pub type D5<T> = stm32f1xx_hal::gpio::gpiob::PB4<T>;
pub type D6<T> = stm32f1xx_hal::gpio::gpiob::PB10<T>;
pub type D7<T> = stm32f1xx_hal::gpio::gpioa::PA8<T>;
pub type D8<T> = stm32f1xx_hal::gpio::gpioa::PA9<T>;
pub type D9<T> = stm32f1xx_hal::gpio::gpioc::PC7<T>;
pub type D10<T> = stm32f1xx_hal::gpio::gpiob::PB6<T>;
pub type D11<T> = stm32f1xx_hal::gpio::gpioa::PA7<T>;
pub type D12<T> = stm32f1xx_hal::gpio::gpioa::PA6<T>;
pub type D13<T> = stm32f1xx_hal::gpio::gpioa::PA5<T>;
pub type D14<T> = stm32f1xx_hal::gpio::gpiob::PB9<T>;
pub type D15<T> = stm32f1xx_hal::gpio::gpiob::PB8<T>;
