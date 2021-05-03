#![no_std]
#![no_main]

use defmt_rtt as _; // global logger
use panic_probe as _;

use rtic::app;

mod arduino_shield;
mod hw_config;
mod misc;

#[app(device = stm32f1xx_hal::device)]
mod app {
    use stm32f1xx_hal::{prelude::_stm32_hal_flash_FlashExt, rcc::RccExt};
    use stm32f1xx_hal::{prelude::*, serial::Config};

    #[resources]
    struct Resources {
        #[task_local]
        serial_rx: crate::hw_config::serial::SerialRx,
        serial_tx: crate::hw_config::serial::SerialTx,
    }

    #[init]
    fn init(cx: init::Context) -> (init::LateResources, init::Monotonics) {
        // Get Peripherals
        let device: stm32f1xx_hal::device::Peripherals = cx.device;

        let mut rcc = device.RCC.constrain();
        let mut flash = device.FLASH.constrain();
        let mut afio = device.AFIO.constrain(&mut rcc.apb2);

        let clocks = rcc
            .cfgr
            .sysclk(72.mhz())
            .pclk1(36.mhz())
            .pclk2(72.mhz())
            .freeze(&mut flash.acr);

        // Init Serial Interface over USB (STLINK)
        let mut gpioa = device.GPIOA.split(&mut rcc.apb2);

        let rx_pin = gpioa.pa3.into_floating_input(&mut gpioa.crl);
        let tx_pin = gpioa.pa2.into_alternate_push_pull(&mut gpioa.crl);

        let (serial_tx, serial_rx) = stm32f1xx_hal::serial::Serial::usart2(
            device.USART2,
            (tx_pin, rx_pin),
            &mut afio.mapr,
            Config::default().baudrate(9600.bps()),
            clocks,
            &mut rcc.apb1,
        )
        .split();

        let dma1_c6 = device.DMA1.split(&mut rcc.ahb).6;

        let serial_rx = serial_rx.with_dma(dma1_c6);

        (
            init::LateResources {
                serial_rx,
                serial_tx,
            },
            init::Monotonics(),
        )
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            cortex_m::asm::nop();
        }
    }
}
