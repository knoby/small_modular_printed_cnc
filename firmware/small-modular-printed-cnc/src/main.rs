#![no_std]
#![no_main]

use defmt_rtt as _; // global logger
use panic_probe as _;

use rtic::app;

mod misc;

#[app(device = stm32f1xx_hal::device)]
mod app {}
