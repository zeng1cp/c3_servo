//! 舵机运动插值与运行时状态管理。
//!
//! `MotionEngine` 以 1ms 为时间粒度推进每个舵机的目标脉宽，负责：
//! - 接受角度或 PWM 形式的目标；
//! - 按配置的 easing 函数插值；
//! - 维护每个舵机的运行状态并在完成时上报 bitmask。

use super::profile::{
    ALL_SERVOS, DEFAULT_MID_PWM_US, PhysicalParams, PulseWidthUs, SERVO_COUNT, ServoId,
};

/// `t ∈ [0, 1]` 的 easing 函数签名。
pub type EasingFn = fn(f32) -> f32;

/// 运动引擎在准备或接受目标时返回的错误。
#[derive(Debug, PartialEq)]
pub enum EngineError {
    /// 舵机物理参数自相矛盾，无法用于限幅或换算。
    InvalidProfile,
    /// 输入角度不是有限浮点数。
    InvalidAngle,
}

/// 对外暴露的单舵机运行状态。
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum MotionStatus {
    /// 当前没有活动任务。
    Idle,
    /// 正在朝目标推进。
    Moving,
    /// 任务被暂停，剩余步数保留。
    Paused,
    /// 最近一次任务已经完成。
    Completed,
}

/// 引擎内部的单舵机状态机。
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum ServoRunState {
    /// 当前没有挂起的运动任务。
    Idle,
    /// `update_1ms()` 会继续推进插值。
    Moving,
    /// 任务被上层暂停，直到收到 `restart()`。
    Paused,
    /// 目标已到达，但尚未被新任务覆盖。
    Completed,
}

impl From<ServoRunState> for MotionStatus {
    fn from(value: ServoRunState) -> Self {
        match value {
            ServoRunState::Idle => Self::Idle,
            ServoRunState::Moving => Self::Moving,
            ServoRunState::Paused => Self::Paused,
            ServoRunState::Completed => Self::Completed,
        }
    }
}

/// 单个舵机的运行时插值上下文。
#[derive(Clone, Copy, Debug)]
struct ServoState {
    current_pwm: PulseWidthUs,
    target_pwm: PulseWidthUs,
    start_pwm: PulseWidthUs,
    steps_total: u32,
    steps_left: u32,
    run_state: ServoRunState,
}

impl Default for ServoState {
    fn default() -> Self {
        Self {
            current_pwm: PulseWidthUs(DEFAULT_MID_PWM_US),
            target_pwm: PulseWidthUs(DEFAULT_MID_PWM_US),
            start_pwm: PulseWidthUs(DEFAULT_MID_PWM_US),
            steps_total: 0,
            steps_left: 0,
            run_state: ServoRunState::Idle,
        }
    }
}

/// 多舵机共享的运动插值引擎。
///
/// 该类型不直接操作硬件，而是维护目标和当前脉宽，随后由调用方通过
/// `sync_to_hardware()` 将结果写入 PWM 输出层。
pub struct MotionEngine {
    profiles: [PhysicalParams; SERVO_COUNT],
    states: [ServoState; SERVO_COUNT],
    easing: EasingFn,
}

impl MotionEngine {
    /// 使用默认物理参数和默认 easing 创建引擎。
    pub fn new() -> Self {
        Self {
            profiles: [PhysicalParams::default(); SERVO_COUNT],
            states: [ServoState::default(); SERVO_COUNT],
            easing: default_easing,
        }
    }

    /// 使用每个舵机的物理参数创建引擎。
    ///
    /// # Errors
    ///
    /// 当任一舵机的 PWM 范围或角度范围非法时，返回 `EngineError::InvalidProfile`。
    pub fn with_profiles(profiles: [PhysicalParams; SERVO_COUNT]) -> Result<Self, EngineError> {
        for profile in profiles.iter() {
            if profile.min_pwm_us >= profile.max_pwm_us
                || profile.min_angle_deg >= profile.max_angle_deg
            {
                return Err(EngineError::InvalidProfile);
            }
        }

        Ok(Self {
            profiles,
            states: [ServoState::default(); SERVO_COUNT],
            easing: default_easing,
        })
    }

    /// 替换用于插值推进的 easing 函数。
    pub fn set_easing_function(&mut self, easing: EasingFn) {
        self.easing = easing;
    }

    /// 为指定舵机下发一个 PWM 目标。
    ///
    /// `duration_ms == 0` 时会立即到达目标，不经过中间插值。
    pub fn move_pwm(
        &mut self,
        id: ServoId,
        target: PulseWidthUs,
        duration_ms: u32,
    ) -> Result<(), EngineError> {
        let idx = id.index();
        let profile = self.profiles[idx];
        let target = self.clamp_pwm_by_profile(target, &profile);

        let state = &mut self.states[idx];
        if target == state.current_pwm {
            state.start_pwm = state.current_pwm;
            state.target_pwm = target;
            state.steps_total = 0;
            state.steps_left = 0;
            state.run_state = ServoRunState::Completed;
            return Ok(());
        }

        if duration_ms == 0 {
            state.start_pwm = state.current_pwm;
            state.target_pwm = target;
            state.current_pwm = target;
            state.steps_total = 0;
            state.steps_left = 0;
            state.run_state = ServoRunState::Completed;
            return Ok(());
        }

        state.start_pwm = state.current_pwm;
        state.target_pwm = target;
        state.steps_total = duration_ms;
        state.steps_left = duration_ms;
        state.run_state = ServoRunState::Moving;
        Ok(())
    }

    /// 为指定舵机下发一个角度目标。
    ///
    /// 角度会先按物理参数换算成 PWM，再复用 `move_pwm()` 的调度逻辑。
    pub fn move_angle(
        &mut self,
        id: ServoId,
        angle: f32,
        duration_ms: u32,
    ) -> Result<(), EngineError> {
        if !angle.is_finite() {
            return Err(EngineError::InvalidAngle);
        }

        let idx = id.index();
        let profile = self.profiles[idx];
        let target_pwm = self.angle_to_pwm_internal(angle, &profile);
        self.move_pwm(id, target_pwm, duration_ms)
    }

    /// 停止指定舵机的当前任务，并把当前位置视为新的静止点。
    pub fn stop(&mut self, id: ServoId) {
        let state = &mut self.states[id.index()];
        state.start_pwm = state.current_pwm;
        state.target_pwm = state.current_pwm;
        state.steps_total = 0;
        state.steps_left = 0;
        state.run_state = ServoRunState::Idle;
    }

    /// 暂停正在执行的任务。
    ///
    /// 对非 `Moving` 状态调用时是无副作用的。
    pub fn pause(&mut self, id: ServoId) {
        let state = &mut self.states[id.index()];
        if state.run_state == ServoRunState::Moving {
            state.run_state = ServoRunState::Paused;
        }
    }

    /// 恢复先前被暂停的任务。
    pub fn restart(&mut self, id: ServoId) {
        let state = &mut self.states[id.index()];
        if state.run_state == ServoRunState::Paused {
            state.run_state = if state.steps_left > 0 {
                ServoRunState::Moving
            } else {
                ServoRunState::Completed
            };
        }
    }

    /// 推进所有舵机 1ms，并返回本 tick 内刚完成任务的 bitmask。
    ///
    /// 返回值的第 `n` 位对应 `ServoId::Sn` 是否在本次推进后进入 `Completed`。
    pub fn update_1ms(&mut self) -> u32 {
        let mut completed_mask = 0u32;

        for id in ALL_SERVOS {
            let idx = id.index();
            if self.states[idx].run_state != ServoRunState::Moving {
                continue;
            }

            let profile = self.profiles[idx];
            let state = &mut self.states[idx];

            if state.steps_left > 0 {
                state.steps_left -= 1;
            }

            if state.steps_left == 0 {
                // 最后一步直接贴到目标值，避免累积舍入误差导致无法精确到达。
                state.current_pwm = state.target_pwm;
                state.run_state = ServoRunState::Completed;
                completed_mask |= 1 << idx;
                continue;
            }

            let t = 1.0 - (state.steps_left as f32 / state.steps_total as f32);
            let eased = (self.easing)(t);
            let delta = state.target_pwm.0 as i32 - state.start_pwm.0 as i32;
            let next = state.start_pwm.0 as i32 + (delta as f32 * eased) as i32;
            let next_pwm = next.clamp(0, u16::MAX as i32) as u16;
            // 即使 easing 输出异常，也在物理边界内再做一次夹紧。
            let clamped = next_pwm.clamp(profile.min_pwm_us, profile.max_pwm_us);
            state.current_pwm = PulseWidthUs(clamped);
        }

        completed_mask
    }

    /// 将当前缓存的 PWM 值批量写给硬件层。
    ///
    /// 调用方提供闭包以决定具体的输出目标，因此该方法不依赖某个特定 HAL。
    pub fn sync_to_hardware<F>(&self, mut output: F) -> Result<(), ()>
    where
        F: FnMut(ServoId, PulseWidthUs) -> Result<(), ()>,
    {
        for id in ALL_SERVOS {
            output(id, self.states[id.index()].current_pwm)?;
        }
        Ok(())
    }

    /// 判断指定舵机是否仍处于推进中。
    pub fn is_moving(&self, id: ServoId) -> bool {
        self.states[id.index()].run_state == ServoRunState::Moving
    }

    /// 查询指定舵机对外可见的运行状态。
    pub fn status(&self, id: ServoId) -> MotionStatus {
        self.states[id.index()].run_state.into()
    }

    /// 遍历当前仍在移动的舵机。
    pub fn moving_servos(&self) -> impl Iterator<Item = ServoId> + '_ {
        ALL_SERVOS
            .iter()
            .copied()
            .filter(|&id| self.status(id) == MotionStatus::Moving)
    }

    /// 以 bitmask 形式返回所有仍在移动的舵机。
    pub fn moving_mask(&self) -> u32 {
        let mut mask = 0u32;
        for id in ALL_SERVOS {
            if self.is_moving(id) {
                mask |= 1 << id.index();
            }
        }
        mask
    }

    /// 读取当前插值后的 PWM 值。
    pub fn current_pwm(&self, id: ServoId) -> PulseWidthUs {
        self.states[id.index()].current_pwm
    }

    /// 读取当前任务的目标 PWM。
    ///
    /// 对 `Idle` 状态，返回当前位置而不是历史目标，避免把已失效的目标暴露给上层。
    pub fn target_pwm(&self, id: ServoId) -> PulseWidthUs {
        let state = &self.states[id.index()];
        if state.run_state == ServoRunState::Idle {
            state.current_pwm
        } else {
            state.target_pwm
        }
    }

    /// 查询剩余时间，单位为毫秒。
    pub fn remaining_ms(&self, id: ServoId) -> u32 {
        let state = &self.states[id.index()];
        match state.run_state {
            ServoRunState::Moving | ServoRunState::Paused => state.steps_left,
            ServoRunState::Idle | ServoRunState::Completed => 0,
        }
    }

    /// 查询当前任务进度。
    ///
    /// - `Idle` 返回 `None`，表示当前没有活动任务；
    /// - 其余状态返回 `0.0..=1.0` 的进度值。
    pub fn progress(&self, id: ServoId) -> Option<f32> {
        let state = &self.states[id.index()];
        match state.run_state {
            ServoRunState::Idle => None,
            ServoRunState::Completed => Some(1.0),
            ServoRunState::Moving | ServoRunState::Paused => {
                if state.steps_total == 0 {
                    Some(1.0)
                } else {
                    Some(1.0 - (state.steps_left as f32 / state.steps_total as f32))
                }
            }
        }
    }

    /// 读取指定舵机的物理参数副本。
    pub fn profile(&self, id: ServoId) -> PhysicalParams {
        self.profiles[id.index()]
    }

    #[inline]
    fn clamp_pwm_by_profile(&self, pwm: PulseWidthUs, profile: &PhysicalParams) -> PulseWidthUs {
        PulseWidthUs(pwm.0.clamp(profile.min_pwm_us, profile.max_pwm_us))
    }

    #[inline]
    fn angle_to_pwm_internal(&self, angle: f32, profile: &PhysicalParams) -> PulseWidthUs {
        let angle = angle.clamp(profile.min_angle_deg, profile.max_angle_deg);
        let range_angle = profile.max_angle_deg - profile.min_angle_deg;
        if range_angle <= f32::EPSILON {
            return PulseWidthUs(profile.mid_pwm_us);
        }

        let ratio = (angle - profile.min_angle_deg) / range_angle;
        let pwm =
            profile.min_pwm_us as f32 + (profile.max_pwm_us - profile.min_pwm_us) as f32 * ratio;
        PulseWidthUs((pwm + 0.5) as u16)
    }
}

fn default_easing(t: f32) -> f32 {
    let t = t.clamp(0.0, 1.0);
    t * t * (3.0 - 2.0 * t)
}

/// 线性 easing，不做额外曲线整形。
pub fn linear_easing(t: f32) -> f32 {
    t.clamp(0.0, 1.0)
}

/// 二次缓入 easing，起步更平缓。
pub fn quadratic_in_easing(t: f32) -> f32 {
    let t = t.clamp(0.0, 1.0);
    t * t
}

/// 二次缓出 easing，临近结束时减速。
pub fn quadratic_out_easing(t: f32) -> f32 {
    let t = t.clamp(0.0, 1.0);
    t * (2.0 - t)
}
