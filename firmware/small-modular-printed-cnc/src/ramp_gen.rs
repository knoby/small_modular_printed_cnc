use core::f32::EPSILON;

use libm::sqrtf;

#[derive(Clone, Copy, Debug, Eq, PartialEq, PartialOrd, Ord, defmt::Format)]
pub enum State {
    Standstill,
    Accelerating,
    Coasting,
    ConstVel,
}
#[derive(Clone, Copy, Debug, Eq, PartialEq, PartialOrd, Ord, defmt::Format)]
pub enum Direction {
    Positive,
    Negative,
    StandStill,
}

pub struct RampGenerator {
    target_position: i64, // [Steps ]
    target_speed: f32,

    max_speed: f32,
    acceleration: f32,

    current_position: i64,
    current_speed: f32,

    step_interval: i64,

    /// The step counter or speed calculations
    step_counter: i64,
    initial_step_size: i64,
    last_step_size: i64,

    /// Min step size based on `max_speed`.
    min_step_size: i64,

    state: State,
    direction: Direction,
}

impl RampGenerator {
    /// Get the number of steps to go until reaching the target position.
    pub fn distance_to_go(&self) -> i64 {
        self.target_position - self.current_position
    }

    /// Calculate the steps to reach the target velocity
    fn steps_to_target_speed(&self) -> i64 {
        let steps_from_act =
            libm::ceilf((self.current_speed * self.current_speed) / (2.0 * self.acceleration))
                as i64;
        let steps_from_target =
            libm::ceilf((self.target_speed * self.target_speed) / (2.0 * self.acceleration)) as i64;
        (steps_from_act - steps_from_target).abs()
    }

    /// Calculates the next step time
    fn update_state(&mut self) {
        let steps_to = self.target_position - self.current_position;
        let steps_to_target_speed = self.steps_to_target_speed();

        //defmt::debug!(
        //    "Steps to: {:?}, Steps to Speed {:?}, StepCounter: {:?}, Direction: {:?}",
        //    steps_to,
        //    steps_to_target_speed,
        //    self.step_counter,
        //    self.direction
        //);

        // Check if profile at endpoint
        if steps_to == 0 && steps_to_target_speed <= 1 {
            // We are at the target and its time to stop
            if self.target_speed != 0.0 {
                self.set_speed(self.target_speed);
            } else {
                self.step_interval = 0;
                self.current_speed = 0.0;
                self.step_counter = 0;
                self.state = State::Standstill;
                self.direction = Direction::StandStill;
            }
            return;
        }

        // Decide about holding speed, accelerating, decelerating
        match (&self.direction, steps_to) {
            (Direction::Positive, steps_to) if steps_to >= 0 => {
                // Check for acc/dec
                if (steps_to.abs()) <= steps_to_target_speed {
                    self.step_counter = -self.step_counter.abs();
                    self.state = State::Accelerating;
                } else {
                    self.step_counter = self.step_counter.abs();
                }
            }
            (Direction::Negative, steps_to) if steps_to >= 0 => {
                // Need to turn around}
                self.step_counter = -self.step_counter.abs();
                self.state = State::Accelerating;
            }
            (Direction::Positive, steps_to) if steps_to < 0 => {
                // Need to turn around
                self.step_counter = -self.step_counter.abs();
                self.state = State::Accelerating;
            }
            (Direction::Negative, steps_to) if steps_to < 0 => {
                // Check for acc/dec}
                if (steps_to.abs()) <= steps_to_target_speed {
                    self.step_counter = -self.step_counter.abs();
                    self.state = State::Accelerating;
                } else {
                    self.step_counter = self.step_counter.abs();
                }
            }
            (Direction::StandStill, _) => {}
            (_, _) => unreachable!(),
        }

        // Update Stepsize
        self.calc_stepsize();

        // Increase Step Counter
        if self.state == State::Accelerating {
            self.step_counter += 1;
            if self.step_counter == 0 {
                if self.distance_to_go() > 0 {
                    self.direction = Direction::Positive;
                } else {
                    self.direction = Direction::Negative;
                }
            }
        }
        self.step_interval = self.last_step_size;
        self.current_speed = 1_000_000_000.0 / self.last_step_size as f32;

        // Change sign of calculated speed
        if self.direction == Direction::Negative {
            self.current_speed *= -1.0;
        }
    }

    /// Calculate the stepsize for the next step
    fn calc_stepsize(&mut self) {
        // Calculate the next step size
        if self.step_counter == 0 {
            // This is the first step after having stopped
            self.last_step_size = self.initial_step_size;
            if self.target_position > self.current_position {
                self.direction = Direction::Positive;
            } else {
                self.direction = Direction::Negative;
            }
        } else {
            let last_step_size = self.last_step_size as f32 / 1_000_000_000.0;
            let last_step_size =
                last_step_size - last_step_size * 2.0 / ((4.0 * self.step_counter as f32) + 1.0);
            self.last_step_size = (last_step_size * 1_000_000_000.0) as i64;
            // Check if max. velocity is reached and set coasting mode
            if self.last_step_size < self.min_step_size {
                self.last_step_size = self.min_step_size;
                self.state = State::Coasting;
            }
        }
    }

    /// Set the desired constant speed in `steps/sec`.
    pub fn set_speed(&mut self, speed: f32) {
        if libm::fabsf(speed - self.current_speed) < EPSILON {
            return;
        }

        let speed = libm::fminf(libm::fmaxf(speed, -self.max_speed), self.max_speed);

        if speed == 0.0 || !speed.is_finite() {
            self.step_interval = 0;
        } else {
            self.step_interval = libm::roundf(libm::fabsf(1e9 / speed)) as i64;
        }

        self.state = State::ConstVel;
        if speed > 0.0 {
            self.direction = Direction::Positive;
        } else {
            self.direction = Direction::Negative;
        }

        self.current_speed = speed;
    }

    /// Get the current moving direction
    pub fn get_current_dir(&self) -> Direction {
        self.direction
    }

    /// Sets the targetparameter for the profile generation
    /// Caution:
    /// If the target speed is internaly limited to the max. speed.
    /// Additionaly the target speed is limited to the speed that can be  reached with the current acceleration
    pub fn set_target(&mut self, target: i64, speed: f32) {
        // Set Target Position
        self.target_position = target;
        // Check the speed that is ok to be reached
        let max_speed_delta = sqrtf(self.distance_to_go().abs() as f32 * 2.0 * self.acceleration);
        // Set the possible speed
        self.target_speed = speed.min(self.max_speed).min(max_speed_delta);

        defmt::debug!(
            "New Target: {:?}, New Speed: {:?}",
            self.target_position,
            self.target_speed,
        );

        self.state = State::Accelerating;
    }

    /// Gets the next Step Infos
    pub fn get_next_step(&mut self) -> i64 {
        match (&self.state, &self.direction) {
            (State::Standstill, _) => {}
            (_, Direction::Positive) => self.current_position += 1,
            (_, Direction::Negative) => self.current_position -= 1,
            (_, _) => {}
        }

        match self.state {
            State::Standstill => {}
            State::Accelerating | State::Coasting => self.update_state(),
            State::ConstVel => {}
        }

        //defmt::debug!(
        //    "Pos: {:?}, Speed: {:?}, StepTime: {:?}, State {:?}",
        //    self.current_position,
        //    self.current_speed,
        //    self.step_interval,
        //    self.state
        //);

        self.step_interval
    }

    /// Gets the current position of the profile Stepper
    pub fn get_current_pos(&self) -> i64 {
        self.current_position
    }

    pub fn set_acceleration(&mut self, acceleration: f32) {
        if acceleration == 0.0 {
            return;
        }

        let acceleration = libm::fabsf(acceleration);

        if libm::fabsf(self.acceleration - acceleration) > core::f32::EPSILON {
            // Recompute step_counter per Equation 17
            self.step_counter =
                (self.step_counter as f32 * self.acceleration / acceleration) as i64;
            // New initial_step_size per Equation 7, with correction per
            // Equation 15
            let initial_step_size = 0.676 * libm::sqrtf(2.0 / acceleration as f32);
            self.initial_step_size = (initial_step_size * 1_000_000_000.0) as i64;
            self.acceleration = acceleration;
            // defmt::debug!(
            //     "New Acc: {:?}, New Init Stepsize: {:?}",
            //     self.acceleration,
            //     self.initial_step_size
            // );
            self.update_state();
        }
    }

    /// Set the maximum permitted speed in `steps/second`.
    ///
    /// # Caution
    ///
    /// the maximum speed achievable depends on your processor and clock speed.
    /// The default max speed is `1.0` step per second.
    pub fn set_max_speed(&mut self, steps_per_second: f32) {
        debug_assert!(steps_per_second > 0.0);

        self.max_speed = steps_per_second;
        self.min_step_size = (1_000_000_000.0 / steps_per_second) as i64;
        defmt::debug!(
            "New Max. Speed: {:?}, New Min Step Size: {:?}",
            self.max_speed,
            self.min_step_size
        );
    }
}

impl Default for RampGenerator {
    fn default() -> Self {
        Self {
            acceleration: 0.0,
            max_speed: 0.0,

            current_position: 0,
            current_speed: 0.0,

            initial_step_size: 0,
            last_step_size: 0,

            min_step_size: 0,

            target_position: 0,
            target_speed: 0.0,

            step_counter: 0,
            step_interval: 0,

            state: State::Standstill,
            direction: Direction::StandStill,
        }
    }
}
