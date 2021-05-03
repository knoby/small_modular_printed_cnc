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

    use core::usize;

    use cortex_m::singleton;
    use stm32f1xx_hal::{prelude::_stm32_hal_flash_FlashExt, rcc::RccExt};
    use stm32f1xx_hal::{prelude::*, serial::Config};

    use crate::hw_config::serial;
    use nb::block;

    #[resources]
    struct Resources {
        #[task_local]
        serial_tx: crate::hw_config::serial::SerialTx,
        #[task_local]
        serial_rx: Option<
            stm32f1xx_hal::dma::Transfer<
                stm32f1xx_hal::dma::W,
                &'static mut [u8; 128],
                crate::hw_config::serial::SerialRx,
            >,
        >,
    }

    #[init]
    fn init(cx: init::Context) -> (init::LateResources, init::Monotonics) {
        // Get Peripherals
        let device: stm32f1xx_hal::device::Peripherals = cx.device;

        let mut rcc = device.RCC.constrain();
        let mut flash = device.FLASH.constrain();
        let mut afio = device.AFIO.constrain(&mut rcc.apb2);

        let clocks = rcc.cfgr.freeze(&mut flash.acr);

        // Init Serial Interface over USB (STLINK)
        let mut gpioa = device.GPIOA.split(&mut rcc.apb2);

        let rx_pin = gpioa.pa3.into_floating_input(&mut gpioa.crl);
        let tx_pin = gpioa.pa2.into_alternate_push_pull(&mut gpioa.crl);

        let serial = stm32f1xx_hal::serial::Serial::usart2(
            device.USART2,
            (tx_pin, rx_pin),
            &mut afio.mapr,
            Config::default()
                .baudrate(9600.bps())
                .parity_none()
                .stopbits(stm32f1xx_hal::serial::StopBits::STOP1),
            clocks,
            &mut rcc.apb1,
        );

        let (mut serial_tx, serial_rx) = serial.split();

        let dma1_c6 = device.DMA1.split(&mut rcc.ahb).6;

        let mut serial_rx = serial_rx.with_dma(dma1_c6);

        serial_rx.listen_idle_line();

        let buffer = singleton!(:[u8; 128] = [0;128]).unwrap();

        let serial_rx = serial_rx.read(buffer);

        // Test Welcom Message
        nb::block!(serial_tx.write(10)).ok();
        use core::fmt::Write;
        serial_tx.write_str("Grbl 1.1h ['$' for help]").ok();

        (
            init::LateResources {
                serial_rx: Some(serial_rx),
                serial_tx,
            },
            init::Monotonics(),
        )
    }

    #[task(binds = USART2, resources = [serial_rx, serial_tx])]
    fn read_buffer(cx: read_buffer::Context) {
        // Get Transfer
        let serial_rx = cx.resources.serial_rx.take().unwrap();

        // Stop Transfer and split in pices
        let (buffer, mut serial_rx) = serial_rx.stop();
        // Reset the Interrupt Event
        serial_rx.reset_idle_line_event();

        let bytes_rec = 128 - serial_rx.get_remaining_transfers();

        defmt::debug!("Idle Line Detected Interrupt");
        defmt::debug!("Recived {:?} bytes", bytes_rec);
        if let Ok(text) = core::str::from_utf8(&buffer[..bytes_rec as usize]) {
            defmt::debug!("Recived Text: {:?}", text);
        } else {
            defmt::debug!("No Valid String");
        }

        // Start Reciving again
        let serial_rx = serial_rx.read(buffer);
        cx.resources.serial_rx.replace(serial_rx);
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            cortex_m::asm::nop();
        }
    }
}
