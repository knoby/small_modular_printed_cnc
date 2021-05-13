//!! Motion kernal for the koordinated motion of the three axis (X, Y and Z)

use defmt::debug;
use num_traits::Float;

use crate::ramp_gen::{Direction, RampGenerator};
///!
///! There are different operationg Modes for the Motion Kernel:
///! - Manual Mode (Jogging)
///! - Homeing (Set the Reference Position)
///! - Disabled (Steppers are disabled and no movement posible)  
///! - Positioning (To Target Position)
///! - Continous Moving (Moving with constant Velocity)

#[derive(Debug, Clone, Copy, defmt::Format, PartialEq, PartialOrd)]
pub enum Axis {
    XAxis,
    YAxis,
    ZAxis,
}

#[derive(Debug, Clone, Copy, PartialEq, PartialOrd, Default)]
pub struct Acceleration {
    x: f32,
    y: f32,
    z: f32,
}

impl Acceleration {
    pub fn new(x: f32, y: f32, z: f32) -> Self {
        Self { x, y, z }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, PartialOrd, Default)]
pub struct Velocity {
    x: f32,
    y: f32,
    z: f32,
}

impl Velocity {
    pub fn new(x: f32, y: f32, z: f32) -> Self {
        Self { x, y, z }
    }

    pub fn zero() -> Self {
        Self {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        }
    }
}

#[derive(Debug, Clone, Copy, defmt::Format, PartialEq, PartialOrd, Default)]
pub struct Position {
    x: i64,
    y: i64,
    z: i64,
}

impl Position {
    pub fn new(x: i64, y: i64, z: i64) -> Self {
        Self { x, y, z }
    }
}

pub struct MotionKernel {
    ramp_gen: crate::ramp_gen::RampGenerator,
    target_pos: Position,
    current_pos: Position,
    cfg: MotionSettings,
    primary_axis: Axis,
    secondary_axis: (Axis, Axis),
    delta_primary: i64,
    delta_secondary: (i64, i64),
    signum_primary: i64,
    signum_secondary: (i64, i64),
    error: (i64, i64),
}

#[derive(Debug, Default, Clone, Copy, PartialEq, PartialOrd)]
pub struct MotionSettings {
    pub max_acc: Acceleration,
    pub max_vel: Velocity,
    pub max_path_acc: f32,
    pub max_path_vel: f32,
}

impl MotionKernel {
    pub fn new(cfg: MotionSettings) -> Self {
        let ramp_gen = RampGenerator::default();

        Self {
            cfg,
            ramp_gen,
            target_pos: Position::new(0, 0, 0),
            current_pos: Position::new(0, 0, 0),
            primary_axis: Axis::XAxis,
            secondary_axis: (Axis::YAxis, Axis::ZAxis),
            delta_primary: 0,
            delta_secondary: (0, 0),
            signum_primary: 0,
            signum_secondary: (0, 0),
            error: (0, 0),
        }
    }

    /// Get Next Step
    pub fn calc_next_step(&mut self) -> ((Direction, Direction, Direction), i64) {
        let next_stepsize = self.ramp_gen.get_next_step();

        if next_stepsize != 0 {
            let last_pos = self.current_pos;
            let x_step = match self.current_pos.x.cmp(&last_pos.x) {
                core::cmp::Ordering::Less => Direction::Negative,
                core::cmp::Ordering::Equal => Direction::StandStill,
                core::cmp::Ordering::Greater => Direction::Positive,
            };
            let y_step = match self.current_pos.y.cmp(&last_pos.y) {
                core::cmp::Ordering::Less => Direction::Negative,
                core::cmp::Ordering::Equal => Direction::StandStill,
                core::cmp::Ordering::Greater => Direction::Positive,
            };
            let z_step = match self.current_pos.z.cmp(&last_pos.z) {
                core::cmp::Ordering::Less => Direction::Negative,
                core::cmp::Ordering::Equal => Direction::StandStill,
                core::cmp::Ordering::Greater => Direction::Positive,
            };
            self.step_bresenham();

            ((x_step, y_step, z_step), next_stepsize)
        } else {
            (
                (
                    Direction::StandStill,
                    Direction::StandStill,
                    Direction::StandStill,
                ),
                next_stepsize,
            )
        }
    }

    /// Move Command to move to a position, with a defined endspeed
    pub fn move_to(&mut self, target_pos: Position) {
        // Setup target Position
        self.target_pos = target_pos;
        // Prepare Multi Axis Movement
        self.setup_bresenham();
        // Prepare Ramp Generator
        match self.primary_axis {
            Axis::XAxis => {
                self.ramp_gen.set_current_pos(self.current_pos.x);
                let max_vel = (self.cfg.max_path_vel.powi(2) - self.cfg.max_vel.y.powi(2)
                    + self.cfg.max_vel.z.powi(2))
                .sqrt()
                .min(self.cfg.max_vel.x);
                let max_acc = (self.cfg.max_path_acc.powi(2) - self.cfg.max_acc.y.powi(2)
                    + self.cfg.max_acc.z.powi(2))
                .sqrt()
                .min(self.cfg.max_acc.x);
                self.ramp_gen.set_max_speed(max_vel);
                self.ramp_gen.set_acceleration(max_acc);
                self.ramp_gen.set_target(self.target_pos.x, 0.0);
            }
            Axis::YAxis => {
                let max_vel = (self.cfg.max_path_vel.powi(2) - self.cfg.max_vel.x.powi(2)
                    + self.cfg.max_vel.z.powi(2))
                .sqrt()
                .min(self.cfg.max_vel.y);
                let max_acc = (self.cfg.max_path_acc.powi(2) - self.cfg.max_acc.x.powi(2)
                    + self.cfg.max_acc.z.powi(2))
                .sqrt()
                .min(self.cfg.max_acc.y);
                self.ramp_gen.set_max_speed(max_vel);
                self.ramp_gen.set_acceleration(max_acc);
                self.ramp_gen.set_current_pos(self.current_pos.y);
                self.ramp_gen.set_target(self.target_pos.y, 0.0);
            }
            Axis::ZAxis => {
                let max_vel = (self.cfg.max_path_vel.powi(2) - self.cfg.max_vel.x.powi(2)
                    + self.cfg.max_vel.y.powi(2))
                .sqrt()
                .min(self.cfg.max_vel.z);
                let max_acc = (self.cfg.max_path_acc.powi(2) - self.cfg.max_acc.x.powi(2)
                    + self.cfg.max_acc.y.powi(2))
                .sqrt()
                .min(self.cfg.max_acc.z);
                self.ramp_gen.set_max_speed(max_vel);
                self.ramp_gen.set_acceleration(max_acc);
                self.ramp_gen.set_current_pos(self.current_pos.z);
                self.ramp_gen.set_target(self.target_pos.z, 0.0);
            }
        }
    }

    fn setup_bresenham(&mut self) {
        let dx = self.target_pos.x - self.current_pos.x;
        let dy = self.target_pos.y - self.current_pos.y;
        let dz = self.target_pos.z - self.current_pos.z;

        let sx = dx.signum();
        let sy = dy.signum();
        let sz = dz.signum();

        debug!("Setup Bresenham");
        debug!("Current Pos: {:?}", self.current_pos);
        debug!("Target Pos: {:?}", self.target_pos);

        // Detect Primary Move Axis
        let primary = if dx.abs() >= dy.abs() && dx.abs() >= dz.abs() {
            Axis::XAxis
        } else if dy.abs() >= dx.abs() && dy.abs() >= dz.abs() {
            Axis::YAxis
        } else if dz.abs() >= dx.abs() && dz.abs() >= dy.abs() {
            Axis::ZAxis
        } else {
            unreachable!("No Primary Axis found")
        };

        // Translate to primary and secondary
        let (dp, ds1, ds2) = match primary {
            Axis::XAxis => (dx, dy, dz),
            Axis::YAxis => (dy, dx, dz),
            Axis::ZAxis => (dz, dx, dy),
        };

        let (sp, ss1, ss2) = match primary {
            Axis::XAxis => (sx, sy, sz),
            Axis::YAxis => (sy, sx, sz),
            Axis::ZAxis => (sz, sx, sy),
        };

        let secondary_axis = match primary {
            Axis::XAxis => (Axis::YAxis, Axis::ZAxis),
            Axis::YAxis => (Axis::XAxis, Axis::ZAxis),
            Axis::ZAxis => (Axis::XAxis, Axis::YAxis),
        };

        // Init the Error
        let error = (dp.abs() / 2, dp.abs() / 2);

        // Safe parameter
        self.error = error;
        self.delta_secondary = (ds1, ds2);
        self.delta_primary = dp;
        self.signum_primary = sp;
        self.signum_secondary = (ss1, ss2);
        self.primary_axis = primary;
        self.secondary_axis = secondary_axis;
    }

    fn step_bresenham(&mut self) {
        // Update Error
        self.error.0 -= self.delta_secondary.0.abs();
        if self.error.0 < 0 {
            self.error.0 += self.delta_primary.abs();
            match self.secondary_axis.0 {
                Axis::XAxis => self.current_pos.x += self.signum_secondary.0,
                Axis::YAxis => self.current_pos.y += self.signum_secondary.0,
                Axis::ZAxis => self.current_pos.z += self.signum_secondary.0,
            }
        }
        self.error.1 -= self.delta_secondary.1.abs();
        if self.error.1 < 0 {
            self.error.1 += self.delta_primary.abs();
            match self.secondary_axis.1 {
                Axis::XAxis => self.current_pos.x += self.signum_secondary.1,
                Axis::YAxis => self.current_pos.y += self.signum_secondary.1,
                Axis::ZAxis => self.current_pos.z += self.signum_secondary.1,
            }
        }
        match self.primary_axis {
            Axis::XAxis => self.current_pos.x += self.signum_primary,
            Axis::YAxis => self.current_pos.y += self.signum_primary,
            Axis::ZAxis => self.current_pos.z += self.signum_primary,
        }
        defmt::debug!("{:?}", self.current_pos);
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum Error {}
