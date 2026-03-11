//! 运动引擎层：仅负责单舵机状态推进。

use super::profile::{
    AngleDeg, PhysicalParams, PulseWidthUs, ServoId, ALL_SERVOS, DEFAULT_MID_PWM_US, SERVO_COUNT,
};

pub type EasingFn = fn(f32) -> f32;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum RunState {
    Idle,
    Moving,
    Paused,
}

#[derive(Clone, Copy, Debug)]
struct ServoState {
    start_pwm: PulseWidthUs,
    current_pwm: PulseWidthUs,
    target_pwm: PulseWidthUs,
    steps_total: u32,
    steps_left: u32,
    run_state: RunState,
}

impl Default for ServoState {
    fn default() -> Self {
        Self {
            start_pwm: PulseWidthUs(DEFAULT_MID_PWM_US),
            current_pwm: PulseWidthUs(DEFAULT_MID_PWM_US),
            target_pwm: PulseWidthUs(DEFAULT_MID_PWM_US),
            steps_total: 0,
            steps_left: 0,
            run_state: RunState::Idle,
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct MotionPlan {
    pub target_pwm: PulseWidthUs,
    pub duration_ms: u32,
    pub easing: Option<EasingFn>,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum EngineError {
    InvalidProfile,
    InvalidDuration,
    InvalidAngle,
}

pub struct MotionEngine {
    profiles: [PhysicalParams; SERVO_COUNT],
    states: [ServoState; SERVO_COUNT],
    default_easing: EasingFn,
}

impl MotionEngine {
    pub fn new() -> Self {
        Self {
            profiles: [PhysicalParams::default(); SERVO_COUNT],
            states: [ServoState::default(); SERVO_COUNT],
            default_easing: default_easing,
        }
    }

    pub fn with_profiles(profiles: [PhysicalParams; SERVO_COUNT]) -> Result<Self, EngineError> {
        for p in &profiles {
            if p.min_pwm_us >= p.max_pwm_us || p.min_angle_deg >= p.max_angle_deg {
                return Err(EngineError::InvalidProfile);
            }
        }

        Ok(Self {
            profiles,
            states: [ServoState::default(); SERVO_COUNT],
            default_easing: default_easing,
        })
    }

    pub fn set_easing_function(&mut self, easing: EasingFn) {
        self.default_easing = easing;
    }

    pub fn start_motion(&mut self, id: ServoId, plan: MotionPlan) -> Result<(), EngineError> {
        if plan.duration_ms == 0 {
            let state = &mut self.states[id.index()];
            let clamped = clamp_pwm_by_profile(plan.target_pwm, self.profiles[id.index()]);
            state.current_pwm = clamped;
            state.target_pwm = state.current_pwm;
            state.start_pwm = state.current_pwm;
            state.steps_total = 0;
            state.steps_left = 0;
            state.run_state = RunState::Idle;
            return Ok(());
        }

        let idx = id.index();
        let target = clamp_pwm_by_profile(plan.target_pwm, self.profiles[idx]);
        let s = &mut self.states[idx];
        s.start_pwm = s.current_pwm;
        s.target_pwm = target;
        s.steps_total = plan.duration_ms;
        s.steps_left = plan.duration_ms;
        s.run_state = RunState::Moving;
        Ok(())
    }

    pub fn move_pwm(&mut self, id: ServoId, target: PulseWidthUs, duration_ms: u32) -> Result<(), EngineError> {
        self.start_motion(
            id,
            MotionPlan {
                target_pwm: target,
                duration_ms,
                easing: None,
            },
        )
    }

    pub fn move_angle(&mut self, id: ServoId, angle: f32, duration_ms: u32) -> Result<(), EngineError> {
        let angle = AngleDeg::new(angle).map_err(|_| EngineError::InvalidAngle)?;
        let target = self.profiles[id.index()].angle_to_pwm(angle);
        self.move_pwm(id, target, duration_ms)
    }

    pub fn pause(&mut self, id: ServoId) {
        let s = &mut self.states[id.index()];
        if s.run_state == RunState::Moving {
            s.run_state = RunState::Paused;
        }
    }

    pub fn resume(&mut self, id: ServoId) {
        let s = &mut self.states[id.index()];
        if s.run_state == RunState::Paused && s.steps_left > 0 {
            s.run_state = RunState::Moving;
        }
    }

    pub fn stop(&mut self, id: ServoId) {
        let s = &mut self.states[id.index()];
        s.run_state = RunState::Idle;
        s.steps_left = 0;
        s.steps_total = 0;
    }

    pub fn tick_1ms(&mut self) -> u32 {
        let mut completed_mask = 0u32;

        for id in ALL_SERVOS {
            let idx = id.index();
            let profile = self.profiles[idx];
            let s = &mut self.states[idx];

            if s.run_state != RunState::Moving {
                continue;
            }

            if s.steps_left > 0 {
                s.steps_left -= 1;
            }

            let progress = if s.steps_total == 0 {
                1.0
            } else {
                1.0 - (s.steps_left as f32 / s.steps_total as f32)
            };

            let eased = (self.default_easing)(progress.clamp(0.0, 1.0));
            s.current_pwm = interpolate_pwm(s.start_pwm, s.target_pwm, eased);
            s.current_pwm = clamp_pwm_by_profile(s.current_pwm, profile);

            if s.steps_left == 0 {
                s.current_pwm = s.target_pwm;
                s.run_state = RunState::Idle;
                completed_mask |= 1 << idx;
            }
        }

        completed_mask
    }

    pub fn is_moving(&self, id: ServoId) -> bool {
        self.states[id.index()].run_state == RunState::Moving
    }

    pub fn moving_mask(&self) -> u32 {
        let mut mask = 0;
        for id in ALL_SERVOS {
            if self.is_moving(id) {
                mask |= 1 << id.index();
            }
        }
        mask
    }

    pub fn current_pwm(&self, id: ServoId) -> PulseWidthUs {
        self.states[id.index()].current_pwm
    }

    pub fn target_pwm(&self, id: ServoId) -> PulseWidthUs {
        self.states[id.index()].target_pwm
    }

    pub fn remaining_ms(&self, id: ServoId) -> u32 {
        let s = &self.states[id.index()];
        if matches!(s.run_state, RunState::Moving | RunState::Paused) {
            s.steps_left
        } else {
            0
        }
    }

    pub fn progress(&self, id: ServoId) -> Option<f32> {
        let s = &self.states[id.index()];
        match s.run_state {
            RunState::Moving | RunState::Paused if s.steps_total > 0 => {
                Some(1.0 - s.steps_left as f32 / s.steps_total as f32)
            }
            _ => None,
        }
    }

    pub fn profile(&self, id: ServoId) -> PhysicalParams {
        self.profiles[id.index()]
    }

    pub fn sync_to_hardware<F, E>(&self, mut output: F) -> Result<(), E>
    where
        F: FnMut(ServoId, PulseWidthUs) -> Result<(), E>,
    {
        for id in ALL_SERVOS {
            output(id, self.states[id.index()].current_pwm)?;
        }
        Ok(())
    }
}

fn clamp_pwm_by_profile(pwm: PulseWidthUs, profile: PhysicalParams) -> PulseWidthUs {
    PulseWidthUs(pwm.0.clamp(profile.min_pwm_us, profile.max_pwm_us))
}

fn interpolate_pwm(start: PulseWidthUs, target: PulseWidthUs, t: f32) -> PulseWidthUs {
    let delta = target.0 as i32 - start.0 as i32;
    let next = start.0 as i32 + (delta as f32 * t.clamp(0.0, 1.0)) as i32;
    PulseWidthUs(next.clamp(0, u16::MAX as i32) as u16)
}

fn default_easing(t: f32) -> f32 {
    let t = t.clamp(0.0, 1.0);
    t * t * (3.0 - 2.0 * t)
}

pub fn linear_easing(t: f32) -> f32 {
    t.clamp(0.0, 1.0)
}

pub fn quadratic_in_easing(t: f32) -> f32 {
    let t = t.clamp(0.0, 1.0);
    t * t
}

pub fn quadratic_out_easing(t: f32) -> f32 {
    let t = t.clamp(0.0, 1.0);
    t * (2.0 - t)
}
