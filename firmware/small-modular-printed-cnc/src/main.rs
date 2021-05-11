#![no_std]
#![no_main]

const VERSION: &str = env!("CARGO_PKG_VERSION");

use defmt_rtt as _; // global logger
use panic_probe as _;

use rtic::app;

mod arduino_shield;
mod gcode_helper;
mod hw_config;
mod misc;
mod ramp_gen;

#[app(device = stm32f1xx_hal::device, dispatchers = [SPI1, SPI2, DMA1_CHANNEL1, DMA1_CHANNEL2, DMA1_CHANNEL3])]
mod app {

    use defmt::debug;
    use dwt_systick_monotonic::DwtSystick;
    use rtic::rtic_monotonic::{Microseconds, Milliseconds, Nanoseconds};
    use stm32f1xx_hal::{prelude::_stm32_hal_flash_FlashExt, rcc::RccExt};
    use stm32f1xx_hal::{prelude::*, serial::Config};

    use crate::hw_config;

    #[resources]
    struct Resources {
        #[task_local]
        serial_tx: crate::hw_config::serial::SerialTx, // Sending Data over Serial
        #[task_local]
        serial_rx: crate::hw_config::serial::SerialRx, // Reciving Data over Serial
        #[task_local]
        buffer_serial_in: heapless::spsc::Producer<'static, ([u8; 64], usize), 4>,
        #[task_local]
        buffer_serial_out: heapless::spsc::Consumer<'static, ([u8; 64], usize), 4>,
        stepper_pins: crate::hw_config::stepper::StepperPins,
        #[task_local]
        buffer_gcode_in: heapless::spsc::Producer<'static, crate::gcode_helper::GCode, 8>,
        #[task_local]
        buffer_gcode_out: heapless::spsc::Consumer<'static, crate::gcode_helper::GCode, 8>,
        ramp_gen: crate::ramp_gen::RampGenerator,
    }

    #[monotonic(binds = SysTick, default = true)]
    type MyMono = DwtSystick<32_000_000>; // 64 MHz

    #[init]
    fn init(mut cx: init::Context) -> (init::LateResources, init::Monotonics) {
        static mut SERIAL_QUEUE: heapless::spsc::Queue<([u8; 64], usize), 4> =
            heapless::spsc::Queue::new();
        static mut GCODE_QUEUE: heapless::spsc::Queue<crate::gcode_helper::GCode, 8> =
            heapless::spsc::Queue::new();

        let (buffer_serial_in, buffer_serial_out) = SERIAL_QUEUE.split();
        let (buffer_gcode_in, buffer_gcode_out) = GCODE_QUEUE.split();

        // Get Peripherals
        let device: stm32f1xx_hal::device::Peripherals = cx.device;

        let mut rcc = device.RCC.constrain();
        let mut flash = device.FLASH.constrain();
        let mut afio = device.AFIO.constrain(&mut rcc.apb2);

        let clocks = rcc
            .cfgr
            .sysclk(32.mhz())
            .pclk1(16.mhz())
            .pclk2(32.mhz())
            .freeze(&mut flash.acr);

        // Activate GPIOs
        let mut gpioa = device.GPIOA.split(&mut rcc.apb2);
        let mut gpiob = device.GPIOB.split(&mut rcc.apb2);

        // Disable JTAG (Not used in Application as ST-Link V2 is avaible)
        //let (_pa15, pb3, pb4) = afio.mapr.disable_jtag(gpioa.pa15, gpiob.pb3, gpiob.pb4);

        // Init Serial Interface over USB (STLINK)
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

        // Init Serial Port Steppe Pins
        let x_step = gpioa.pa6.into_push_pull_output(&mut gpioa.crl);
        let y_step = gpioa.pa5.into_push_pull_output(&mut gpioa.crl);
        let z_step = gpiob.pb5.into_push_pull_output(&mut gpiob.crl);
        let x_direction = gpiob.pb9.into_push_pull_output(&mut gpiob.crh);
        let y_direction = gpiob.pb10.into_push_pull_output(&mut gpiob.crh);
        let z_direction = gpioa.pa8.into_push_pull_output(&mut gpioa.crh);
        let enable = gpioa.pa9.into_push_pull_output(&mut gpioa.crh);
        let mut stepper_pins = hw_config::stepper::StepperPins::new(
            x_step,
            y_step,
            z_step,
            x_direction,
            y_direction,
            z_direction,
            enable,
        );

        // Create Systic Timer
        let mono = DwtSystick::new(
            &mut cx.core.DCB,
            cx.core.DWT,
            cx.core.SYST,
            clocks.sysclk().0,
        );

        // Enable the RX Interrupt
        serial_rx.listen();

        // Create Profile Generator
        let mut ramp_gen = crate::ramp_gen::RampGenerator::default();
        ramp_gen.set_acceleration(32_000.0);
        ramp_gen.set_max_speed(8_000.0);
        ramp_gen.set_target(1_600, 0.0);
        stepper_pins.enable();

        // Send Welcome Message
        use core::fmt::Write;
        serial_tx.write_str("SMPCNC Version ").ok();
        serial_tx.write_str(crate::VERSION).ok();
        nb::block!(serial_tx.write(b'\n')).ok();
        serial_tx.write_str("Grbl 1.1h ['$' for help]\n").ok();

        step_calc::spawn_after(Milliseconds(500_u32)).ok();

        (
            init::LateResources {
                serial_rx,
                serial_tx,
                buffer_serial_in,
                buffer_serial_out,
                stepper_pins,
                buffer_gcode_in,
                buffer_gcode_out,
                ramp_gen,
            },
            init::Monotonics(mono),
        )
    }

    #[idle(resources = [buffer_serial_out, buffer_gcode_in, serial_tx])]
    fn protocol_handler(cx: protocol_handler::Context) -> ! {
        loop {
            while let Some((buffer, length)) = cx.resources.buffer_serial_out.dequeue() {
                if let Ok(line) = core::str::from_utf8(&buffer[..length]) {
                    debug!("Recived Line {:?}", line);
                    if buffer[0] == b'$' && length >= 2 {
                        debug!("System Command Recived");
                        use core::fmt::Write;
                        cx.resources.serial_tx.write_str("ok\n").ok();
                    } else {
                        debug!("G-Code Recived");
                        use core::fmt::Write;
                        cx.resources.serial_tx.write_str("ok\n").ok();
                        let g_code_parser: gcode::Parser<gcode::Nop, crate::gcode_helper::Buffer> =
                            gcode::Parser::new(line, gcode::Nop);
                        for line in g_code_parser {
                            for gcode in line.gcodes() {
                                cx.resources.buffer_gcode_in.enqueue(gcode.clone()).ok();
                            }
                        }
                        // Start processing of gcodes
                        gcode_interpreter::spawn().ok();
                    }
                }
            }
        }
    }

    #[task(priority = 5, resources = [buffer_gcode_out])]
    fn gcode_interpreter(cx: gcode_interpreter::Context) {
        let buffer: &mut heapless::spsc::Consumer<'static, crate::gcode_helper::GCode, 8> =
            cx.resources.buffer_gcode_out;

        while let Some(gcode) = buffer.dequeue() {
            match gcode.mnemonic() {
                gcode::Mnemonic::General => {
                    debug!("G")
                }
                gcode::Mnemonic::Miscellaneous => {
                    debug!("M")
                }
                gcode::Mnemonic::ProgramNumber => {
                    debug!("P")
                }
                gcode::Mnemonic::ToolChange => {
                    debug!("T")
                }
            }
            for arg in gcode.arguments().iter().map(|arg| arg.letter) {
                debug!("Argument {:?}", arg);
            }
        }
    }

    #[task(priority = 10, binds = USART2, resources = [serial_rx, buffer_serial_in])]
    fn read_serial_buffer(cx: read_serial_buffer::Context) {
        /// Serial Buffer for holding until a new line is detected
        static mut SERIAL_BUFFER: heapless::Vec<u8, 64> = heapless::Vec::new();

        while let Ok(data_byte) = cx.resources.serial_rx.read() {
            match data_byte {
                b'?' | b'~' | b'!' | 0x18 => {
                    debug!("Realtime Command Recived");
                    handle_realtime_command::spawn(data_byte).ok();
                }
                0x0A => {
                    // Line Ended
                    debug!("Line End recived");
                    if SERIAL_BUFFER.len() > 0 {
                        let mut data = [0; 64];
                        for (place, data) in data.iter_mut().zip(SERIAL_BUFFER.iter()) {
                            *place = *data;
                        }
                        cx.resources
                            .buffer_serial_in
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

    #[task(priority = 14,  resources = [ramp_gen])]
    fn step_calc(mut cx: step_calc::Context) {
        static mut NEXT_STEP_TIME: u32 = 0;

        if *NEXT_STEP_TIME == 0 {
            // No more step
            step_calc::spawn_after(Milliseconds(100_u32)).unwrap();
        } else {
            // Do a step
            start_step_pulse::spawn(
                cx.resources
                    .ramp_gen
                    .lock(|ramp_gen| ramp_gen.get_current_dir()),
                crate::ramp_gen::Direction::StandStill,
                crate::ramp_gen::Direction::StandStill,
            )
            .unwrap();
            step_calc::spawn_after(Nanoseconds(*NEXT_STEP_TIME)).unwrap();
            end_step_pulse::spawn_after(Microseconds(10_u32)).unwrap();
        }

        // Calculate the next step time from profile generator
        *NEXT_STEP_TIME = cx
            .resources
            .ramp_gen
            .lock(|ramp_gen| ramp_gen.get_next_step()) as u32;
    }

    #[task(priority = 15, resources = [stepper_pins])]
    fn start_step_pulse(
        mut cx: start_step_pulse::Context,
        x: crate::ramp_gen::Direction,
        y: crate::ramp_gen::Direction,
        z: crate::ramp_gen::Direction,
    ) {
        cx.resources.stepper_pins.lock(|pins| {
            pins.start_step(x, y, z);
        })
    }

    #[task(priority = 15, resources = [stepper_pins])]
    fn end_step_pulse(mut cx: end_step_pulse::Context) {
        cx.resources.stepper_pins.lock(|pins| {
            pins.end_step();
        });
    }

    #[task(priority = 5)]
    fn handle_realtime_command(_: handle_realtime_command::Context, command: u8) {
        if 0x18 == command {
            cortex_m::peripheral::SCB::sys_reset()
        }
    }
}
