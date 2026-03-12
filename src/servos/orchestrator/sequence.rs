//! 姿态序列编排器。
//!
//! `SequenceManager` 负责按顺序触发一组 `Pose`，并在每个姿态完成后自动推进到下一个。
//! 当序列走到结尾时，可以按配置循环多次，最后再触发收尾回调。

use super::super::config::{ConfigManager, Storage};
use super::super::controller::{Callback, ControlError, ControlUnit, Target};
use super::super::engine::MotionEngine;
use super::super::profile::{AngleDeg, PulseWidthUs, SERVO_COUNT, ServoId};

/// 一个离散姿态片段。
///
/// `ids` 与 `angles_deg` 需要按相同顺序一一对应，`duration_ms` 定义这一姿态的执行时长。
#[derive(Clone, Copy, Debug)]
pub struct Pose<'a> {
    pub ids: &'a [ServoId],
    pub angles_deg: &'a [f32],
    pub duration_ms: u32,
}

impl<'a> Pose<'a> {
    /// 构造一个姿态描述。
    pub const fn new(ids: &'a [ServoId], angles_deg: &'a [f32], duration_ms: u32) -> Self {
        Self {
            ids,
            angles_deg,
            duration_ms,
        }
    }
}

/// 序列编排阶段的错误。
#[derive(Debug, PartialEq)]
pub enum SequenceError {
    /// 输入姿态数组为空，无法启动序列。
    EmptyPoses,
    /// 某个姿态的 `ids` / `angles_deg` 非法或长度不匹配。
    InvalidPose,
    /// 下层控制单元启动或推进失败。
    Control(ControlError),
}

impl From<ControlError> for SequenceError {
    fn from(value: ControlError) -> Self {
        Self::Control(value)
    }
}

/// 按顺序执行多个姿态的状态机。
///
/// 状态机只在 `active == true` 时推进；每次 `tick()` 会根据控制单元是否完成来决定
/// 保持当前姿态还是切换到下一个姿态。
pub struct SequenceManager<'a> {
    poses: &'a [Pose<'a>],
    current_idx: usize,
    loops_total: u32,
    loops_done: u32,
    active: bool,
}

impl<'a> SequenceManager<'a> {
    /// 创建空闲的序列管理器。
    pub const fn new() -> Self {
        Self {
            poses: &[],
            current_idx: 0,
            loops_total: 0,
            loops_done: 0,
            active: false,
        }
    }

    /// 启动一组姿态序列。
    ///
    /// `loops == 0` 会被视为执行 1 次，避免出现“已启动但永不执行”的歧义状态。
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

    /// 用控制单元的完成状态推进序列。
    ///
    /// 只有当当前姿态对应的 `ControlUnit` 已经完成时，才会切换到下一个姿态。
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
                // 最后一轮最后一个姿态完成后才触发收尾回调。
                self.active = false;
                if let Some(cb) = final_callback {
                    cb.invoke();
                }
                return Ok(());
            }
        }

        self.start_current_pose(unit, engine, config, None)
    }

    /// 当前是否仍有未执行完的姿态。
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

        // 每个 pose 都重建一个只覆盖本姿态目标舵机集合的控制单元。
        *unit = ControlUnit::new(pose.ids)?;

        let mut targets = [Target::Pwm(PulseWidthUs(0)); SERVO_COUNT];
        for (i, angle) in pose.angles_deg.iter().enumerate() {
            let deg = AngleDeg::new(*angle).map_err(|_| ControlError::InvalidAngle)?;
            targets[i] = Target::Angle(deg);
        }
        unit.start(
            engine,
            config,
            &targets[..pose.angles_deg.len()],
            pose.duration_ms,
            callback,
        )?;
        Ok(())
    }
}
