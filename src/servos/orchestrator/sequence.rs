//! 姿态序列编排器
//!
//! `SequenceManager` 负责按顺序触发一组姿态（Pose），并支持循环播放。

use super::super::config::{ConfigManager, Storage};
use super::super::controller::{Callback, ControlError, ControlUnit, Target};
use super::super::engine::MotionEngine;
use super::super::profile::{AngleDeg, PulseWidthUs, ServoId, SERVO_COUNT};

#[derive(Clone, Copy, Debug)]
pub struct Pose<'a> {
    pub ids: &'a [ServoId],
    pub angles_deg: &'a [f32],
    pub duration_ms: u32,
}

impl<'a> Pose<'a> {
    pub const fn new(ids: &'a [ServoId], angles_deg: &'a [f32], duration_ms: u32) -> Self {
        Self {
            ids,
            angles_deg,
            duration_ms,
        }
    }
}

#[derive(Debug, PartialEq)]
pub enum SequenceError {
    EmptyPoses,
    InvalidPose,
    Control(ControlError),
}

impl From<ControlError> for SequenceError {
    fn from(value: ControlError) -> Self {
        Self::Control(value)
    }
}

pub struct SequenceManager<'a> {
    poses: &'a [Pose<'a>],
    current_idx: usize,
    loops_total: u32,
    loops_done: u32,
    active: bool,
}

impl<'a> SequenceManager<'a> {
    pub const fn new() -> Self {
        Self {
            poses: &[],
            current_idx: 0,
            loops_total: 0,
            loops_done: 0,
            active: false,
        }
    }

    pub fn run<S: Storage>(
        &mut self,
        unit: &mut ControlUnit,
        engine: &mut MotionEngine,
        config: &ConfigManager<S>,
        poses: &'a [Pose<'a>],
        loops: u32,
        callback: Option<Callback>,
    ) -> Result<(), SequenceError> {
        if poses.is_empty() {
            return Err(SequenceError::EmptyPoses);
        }
        for pose in poses {
            if pose.ids.is_empty() || pose.ids.len() != pose.angles_deg.len() {
                return Err(SequenceError::InvalidPose);
            }
        }

        self.poses = poses;
        self.current_idx = 0;
        self.loops_total = loops.max(1);
        self.loops_done = 0;
        self.active = true;

        self.start_current_pose(unit, engine, config, callback)
    }

    pub fn tick<S: Storage>(
        &mut self,
        unit: &mut ControlUnit,
        engine: &mut MotionEngine,
        config: &ConfigManager<S>,
        completed_mask: u32,
        final_callback: Option<Callback>,
    ) -> Result<(), SequenceError> {
        if !self.active {
            return Ok(());
        }

        unit.on_completed_mask(completed_mask);
        if unit.is_active() {
            return Ok(());
        }

        self.current_idx += 1;
        if self.current_idx >= self.poses.len() {
            self.current_idx = 0;
            self.loops_done += 1;
            if self.loops_done >= self.loops_total {
                self.active = false;
                if let Some(cb) = final_callback {
                    (cb.func)(cb.user_data);
                }
                return Ok(());
            }
        }

        self.start_current_pose(unit, engine, config, None)
    }

    pub const fn is_active(&self) -> bool {
        self.active
    }

    fn start_current_pose<S: Storage>(
        &mut self,
        unit: &mut ControlUnit,
        engine: &mut MotionEngine,
        config: &ConfigManager<S>,
        callback: Option<Callback>,
    ) -> Result<(), SequenceError> {
        let pose = self.poses[self.current_idx];

        *unit = ControlUnit::new(pose.ids)?;

        let mut targets = [Target::Pwm(PulseWidthUs(0)); SERVO_COUNT];
        for (i, angle) in pose.angles_deg.iter().enumerate() {
            let deg = AngleDeg::new(*angle).map_err(|_| ControlError::InvalidTarget)?;
            targets[i] = Target::Angle(deg);
        }
        unit.start(engine, config, &targets[..pose.angles_deg.len()], pose.duration_ms, callback)?;
        Ok(())
    }
}
