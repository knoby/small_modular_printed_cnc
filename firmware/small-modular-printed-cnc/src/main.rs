#![no_std]
#![no_main]

const VERSION: &str = env!("CARGO_PKG_VERSION");

use defmt_rtt as _; // global logger
use panic_probe as _;

use rtic::app;

mod arduino_shield;
mod hw_config;
mod misc;

#[app(device = stm32f1xx_hal::device)]
mod app {

    use defmt::debug;
    use stm32f1xx_hal::{prelude::_stm32_hal_flash_FlashExt, rcc::RccExt};
    use stm32f1xx_hal::{prelude::*, serial::Config};

    #[resources]
    struct Resources {
        #[task_local]
        serial_tx: crate::hw_config::serial::SerialTx, // Sending Data over Serial
        #[task_local]
        serial_rx: crate::hw_config::serial::SerialRx, // Reciving Data over Serial
        #[task_local]
        buffer_queue_in: heapless::spsc::Producer<'static, ([u8; 64], usize), 4>,
        #[task_local]
        buffer_queue_out: heapless::spsc::Consumer<'static, ([u8; 64], usize), 4>,
    }

    #[init]
    fn init(cx: init::Context) -> (init::LateResources, init::Monotonics) {
        static mut SERIAL_QUEUE: heapless::spsc::Queue<([u8; 64], usize), 4> =
            heapless::spsc::Queue::new();
        let (buffer_queue_in, buffer_queue_out) = SERIAL_QUEUE.split();

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

        let (mut serial_tx, mut serial_rx) = serial.split();

        // Enable the RX Interrupt
        serial_rx.listen();

        // Send Welcome Message
        use core::fmt::Write;
        serial_tx.write_str("SMPCNC Version ").ok();
        serial_tx.write_str(crate::VERSION).ok();
        nb::block!(serial_tx.write(b'\n')).ok();
        serial_tx.write_str("Grbl 1.1h ['$' for help]\n").ok();

        (
            init::LateResources {
                serial_rx,
                serial_tx,
                buffer_queue_in,
                buffer_queue_out,
            },
            init::Monotonics(),
        )
    }

    #[idle(resources = [buffer_queue_out, serial_tx])]
    fn protocol_handler(cx: protocol_handler::Context) -> ! {
        loop {
            while let Some((buffer, length)) = cx.resources.buffer_queue_out.dequeue() {
                if buffer[0] == b'$' && length >= 2 {
                    debug!("System Command Recived");
                } else {
                    debug!("G-Code Recived");
                }
            }
        }
    }

    #[task(binds = USART2, resources = [serial_rx, buffer_queue_in])]
    fn read_serial_buffer(cx: read_serial_buffer::Context) {
        /// Serial Buffer for holding until a new line is detected
        static mut SERIAL_BUFFER: heapless::Vec<u8, 64> = heapless::Vec::new();

        while let Ok(data_byte) = cx.resources.serial_rx.read() {
            match data_byte {
                b'?' => {
                    // Status Query
                    debug!("Status Query recived");
                }
                b'~' => unimplemented!(), // Start/Resume
                b'!' => unimplemented!(), // Feed Hold
                0x18 => cortex_m::peripheral::SCB::sys_reset(), // Soft Reset
                0x0A => {
                    // Line Ended
                    debug!("Line End recived");
                    if SERIAL_BUFFER.len() > 0 {
                        let mut data = [0; 64];
                        for (place, data) in data.iter_mut().zip(SERIAL_BUFFER.iter()) {
                            *place = *data;
                        }
                        cx.resources
                            .buffer_queue_in
                            .enqueue((data, SERIAL_BUFFER.len()))
                            .ok();
                        SERIAL_BUFFER.clear();
                    }
                }
                _ => {
                    // Normal Byte is Send to buffer
                    SERIAL_BUFFER.push(data_byte).ok();
                }
            };
        }
    }
}
