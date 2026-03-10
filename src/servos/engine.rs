//! 运动引擎层
//!
//! 本模块负责每个舵机的运动规划、状态管理和时间步进。
//! 通过 `MotionEngine` 结构体，可以启动、暂停、停止舵机运动，
//! 并每毫秒更新一次位置（由上层调用 `update_1ms`）。
//!
//! # 设计要点
//! - 物理参数（`PhysicalParams`）在引擎创建时设置，运行中不可修改。
//! - 缓动函数可通过 `set_easing_function` 在运行时配置，默认为三次缓动（ease-in-out）。
//! - 不直接输出到硬件，只更新内部状态；上层需通过 `current_pwm` 读取并输出。
//! - 运动时长以毫秒为单位，内部转换为步数（每步1ms）。
//! - 提供丰富的状态查询接口，便于上层反馈。

use super::profile::{DEFAULT_MID_PWM_US, PhysicalParams, PulseWidthUs, ServoId, ALL_SERVOS, SERVO_COUNT};

/// 缓动函数类型：输入 t (0.0~1.0)，输出插值后的进度（通常也在 0.0~1.0 之间）
pub type EasingFn = fn(f32) -> f32;

/// 运动引擎错误类型
#[derive(Debug, PartialEq)]
pub enum EngineError {
    /// 无效的物理参数配置（来自 profile 验证）
    InvalidProfile,
    /// 无效的运动时长（例如 0？但允许 0 立即完成）
    InvalidDuration,
    /// 舵机ID无效（不应发生，但保留）
    InvalidServoId,
    /// 角度转换错误（例如 NaN）
    InvalidAngle,
}

/// 舵机动态状态
#[derive(Clone, Copy, Debug)]
struct ServoState {
    /// 当前实际脉冲宽度（经过插值后的最新值）
    current_pwm: PulseWidthUs,
    /// 目标脉冲宽度
    target_pwm: PulseWidthUs,
    /// 起始脉冲宽度（本次运动开始时）
    start_pwm: PulseWidthUs,
    /// 总步数（总毫秒数）
    steps_total: u32,
    /// 剩余步数
    steps_left: u32,
    /// 是否正在运动中
    moving: bool,
}

impl Default for ServoState {
    fn default() -> Self {
        Self {
            current_pwm: PulseWidthUs(DEFAULT_MID_PWM_US),
            target_pwm: PulseWidthUs(DEFAULT_MID_PWM_US),
            start_pwm: PulseWidthUs(DEFAULT_MID_PWM_US),
            steps_total: 0,
            steps_left: 0,
            moving: false,
        }
    }
}

/// 运动引擎
///
/// 管理所有舵机的物理参数和运动状态。物理参数在创建时确定，运行中不可修改。
/// 缓动函数可在运行时通过 `set_easing_function` 修改。
pub struct MotionEngine {
    /// 每个舵机的物理参数（只读，由构造函数初始化）
    profiles: [PhysicalParams; SERVO_COUNT],
    /// 每个舵机的动态状态
    states: [ServoState; SERVO_COUNT],
    /// 当前使用的缓动函数
    easing: EasingFn,
}

impl MotionEngine {
    /// 创建新的运动引擎，使用所有舵机的默认物理参数
    pub fn new() -> Self {
        Self {
            profiles: [PhysicalParams::default(); SERVO_COUNT],
            states: [ServoState::default(); SERVO_COUNT],
            easing: default_easing,
        }
    }

    /// 使用自定义物理参数数组创建运动引擎
    ///
    /// # 参数
    /// * `profiles` - 长度等于 `SERVO_COUNT` 的数组，依次对应每个舵机的物理参数
    ///
    /// # 错误
    /// 如果任何一个 profile 验证失败，返回 `EngineError::InvalidProfile`
    pub fn with_profiles(profiles: [PhysicalParams; SERVO_COUNT]) -> Result<Self, EngineError> {
        // 验证所有 profile 的有效性
        for profile in profiles.iter() {
            if profile.min_pwm_us >= profile.max_pwm_us || profile.min_angle_deg >= profile.max_angle_deg {
                return Err(EngineError::InvalidProfile);
            }
        }

        Ok(Self {
            profiles,
            states: [ServoState::default(); SERVO_COUNT],
            easing: default_easing,
        })
    }

    /// 设置缓动函数
    ///
    /// # 参数
    /// * `easing` - 新的缓动函数，接受 t (0.0~1.0) 并返回插值进度
    ///
    /// # 注意
    /// 如果在运动中修改缓动函数，当前运动的插值会立即受影响（可能导致曲线不连续）。
    /// 建议在启动新运动前修改缓动函数。
    pub fn set_easing_function(&mut self, easing: EasingFn) {
        self.easing = easing;
    }

    /// 启动一次 PWM 目标运动
    ///
    /// # 参数
    /// - `id`: 舵机ID
    /// - `target`: 目标脉冲宽度（应已通过上层校正）
    /// - `duration_ms`: 运动持续时间（毫秒）
    ///
    /// # 注意
    /// - 如果 `target` 超出物理范围，会被自动钳位到范围内。
    /// - 如果 `duration_ms == 0`，则立即将当前位置设置为目标，并标记为完成（但不会触发回调）。
    /// - 如果当前舵机正在运动，调用此方法会立即中断原运动，开始新运动。
    pub fn move_pwm(
        &mut self,
        id: ServoId,
        target: PulseWidthUs,
        duration_ms: u32,
    ) -> Result<(), EngineError> {
        let idx = id.index();
        let profile = self.profiles[idx];

        // 钳位目标到物理范围
        let target = self.clamp_pwm_by_profile(target, &profile);

        let state = &mut self.states[idx];

        // 如果目标与当前位置相同，直接完成（不进入运动）
        if target == state.current_pwm {
            // 清除运动状态（如果之前正在运动）
            if state.moving {
                state.moving = false;
                state.steps_left = 0;
            }
            return Ok(());
        }

        // 处理 duration_ms == 0 的情况：立即跳转到目标
        if duration_ms == 0 {
            state.current_pwm = target;
            state.moving = false;
            state.steps_left = 0;
            return Ok(());
        }

        // 启动新运动
        state.start_pwm = state.current_pwm;
        state.target_pwm = target;
        state.steps_total = duration_ms;
        state.steps_left = duration_ms;
        state.moving = true;
        Ok(())
    }

    /// 启动一次角度目标运动
    ///
    /// # 参数
    /// - `id`: 舵机ID
    /// - `angle`: 目标角度（度）
    /// - `duration_ms`: 运动持续时间（毫秒）
    ///
    /// # 注意
    /// - 角度将通过当前舵机的物理参数转换为 PWM，并自动钳位。
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

    /// 停止指定舵机的运动
    ///
    /// 将舵机标记为非运动状态，剩余步数清零，当前位置保持不变。
    pub fn stop(&mut self, id: ServoId) {
        let idx = id.index();
        let state = &mut self.states[idx];
        if state.moving {
            state.moving = false;
            state.steps_left = 0;
        }
    }

    /// 暂停指定舵机的运动
    ///
    /// 将舵机标记为非运动状态，但保留剩余步数，以便后续恢复。
    pub fn pause(&mut self, id: ServoId) {
        let idx = id.index();
        self.states[idx].moving = false;
    }

    /// 恢复指定舵机的运动
    ///
    /// 如果舵机有剩余步数且未完成，则重新标记为运动状态。
    pub fn restart(&mut self, id: ServoId) {
        let idx = id.index();
        let state = &mut self.states[idx];
        if state.steps_left > 0 && !state.moving {
            state.moving = true;
        }
    }

    /// 每毫秒更新一次所有正在运动的舵机位置
    ///
    /// # 返回值
    /// 返回一个 `u32` 掩码，表示本次更新中完成运动的舵机（即剩余步数变为0的舵机）。
    ///
    /// # 注意
    /// - 此函数不直接输出到硬件，只更新内部 `current_pwm`。
    /// - 上层应在调用此函数后，根据需要读取新的 `current_pwm` 并输出到硬件。
    pub fn update_1ms(&mut self) -> u32 {
        let mut completed_mask = 0u32;

        for id in ALL_SERVOS {
            let idx = id.index();
            if !self.states[idx].moving {
                continue;
            }

            // 提前复制 profile，避免后续借用冲突
            let profile = self.profiles[idx];

            let state = &mut self.states[idx];

            if state.steps_left > 0 {
                state.steps_left -= 1;
            }

            if state.steps_left == 0 {
                // 运动完成，直接设置为目标值
                state.current_pwm = state.target_pwm;
                state.moving = false;
                completed_mask |= 1 << idx;
                continue;
            }

            // 计算当前插值进度 t = 1 - (steps_left / steps_total)
            let t = 1.0 - (state.steps_left as f32 / state.steps_total as f32);
            let eased = (self.easing)(t); // 使用当前缓动函数

            let delta = state.target_pwm.0 as i32 - state.start_pwm.0 as i32;
            let next = state.start_pwm.0 as i32 + (delta as f32 * eased) as i32;
            let next_pwm = next.max(0).min(u16::MAX as i32) as u16;
            let next_pwm = PulseWidthUs(next_pwm);

            // 使用提前复制的 profile 进行钳位
            let clamped = next_pwm.0.clamp(profile.min_pwm_us, profile.max_pwm_us);
            state.current_pwm = PulseWidthUs(clamped);
        }

        completed_mask
    }

    /// 同步当前所有舵机的脉冲值到硬件（由上层调用）
    /// 此方法仅用于一次性输出所有当前值，通常在初始化后使用。
    /// 正常运行时，上层应在每次 `update_1ms` 后输出变化的值。
    pub fn sync_to_hardware<F>(&self, mut output: F) -> Result<(), ()>
    where
        F: FnMut(ServoId, PulseWidthUs) -> Result<(), ()>,
    {
        for id in ALL_SERVOS {
            output(id, self.states[id.index()].current_pwm)?;
        }
        Ok(())
    }

    // ---------- 状态查询方法 ----------

    /// 查询指定舵机是否正在运动
    pub fn is_moving(&self, id: ServoId) -> bool {
        self.states[id.index()].moving
    }

    /// 获取当前正在运动的舵机迭代器
    pub fn moving_servos(&self) -> impl Iterator<Item = ServoId> + '_ {
        ALL_SERVOS.iter().copied().filter(|&id| self.is_moving(id))
    }

    /// 获取当前正在运动的舵机掩码（兼容旧接口，但建议直接使用 `moving_servos`）
    pub fn moving_mask(&self) -> u32 {
        let mut mask = 0;
        for id in ALL_SERVOS {
            if self.is_moving(id) {
                mask |= 1 << id.index();
            }
        }
        mask
    }

    /// 获取指定舵机的当前脉冲宽度
    pub fn current_pwm(&self, id: ServoId) -> PulseWidthUs {
        self.states[id.index()].current_pwm
    }

    /// 获取指定舵机的目标脉冲宽度（如果正在运动，否则返回当前值）
    pub fn target_pwm(&self, id: ServoId) -> PulseWidthUs {
        let state = &self.states[id.index()];
        if state.moving {
            state.target_pwm
        } else {
            state.current_pwm
        }
    }

    /// 获取指定舵机当前运动的剩余毫秒数（如果未运动，返回0）
    pub fn remaining_ms(&self, id: ServoId) -> u32 {
        let state = &self.states[id.index()];
        if state.moving {
            state.steps_left
        } else {
            0
        }
    }

    /// 获取指定舵机当前运动的进度（0.0 ~ 1.0），如果未运动返回 0.0
    pub fn progress(&self, id: ServoId) -> f32 {
        let state = &self.states[id.index()];
        if state.moving && state.steps_total > 0 {
            1.0 - (state.steps_left as f32 / state.steps_total as f32)
        } else {
            0.0
        }
    }

    /// 获取指定舵机的物理参数（只读）
    pub fn profile(&self, id: ServoId) -> PhysicalParams {
        self.profiles[id.index()]
    }

    // ---------- 内部辅助方法 ----------

    /// 将脉冲宽度钳位到指定 profile 的物理范围内
    #[inline]
    fn clamp_pwm_by_profile(&self, pwm: PulseWidthUs, profile: &PhysicalParams) -> PulseWidthUs {
        let clamped = pwm.0.clamp(profile.min_pwm_us, profile.max_pwm_us);
        PulseWidthUs(clamped)
    }

    /// 将角度转换为脉冲宽度（使用给定的 profile，不钳位，但内部已钳位角度）
    #[inline]
    fn angle_to_pwm_internal(&self, angle: f32, profile: &PhysicalParams) -> PulseWidthUs {
        let angle = angle.clamp(profile.min_angle_deg, profile.max_angle_deg);
        let range_angle = profile.max_angle_deg - profile.min_angle_deg;
        if range_angle <= f32::EPSILON {
            return PulseWidthUs(profile.mid_pwm_us);
        }
        let ratio = (angle - profile.min_angle_deg) / range_angle;
        let pwm = profile.min_pwm_us as f32
            + (profile.max_pwm_us - profile.min_pwm_us) as f32 * ratio;
        // 手动四舍五入（适用于正数）
        PulseWidthUs((pwm + 0.5) as u16)
    }
}

/// 默认缓动函数：三次缓动（ease-in-out）
fn default_easing(t: f32) -> f32 {
    let t = t.clamp(0.0, 1.0);
    t * t * (3.0 - 2.0 * t)
}

/// 预定义的线性缓动函数
pub fn linear_easing(t: f32) -> f32 {
    t.clamp(0.0, 1.0)
}

/// 预定义的二次缓动（ease-in）
pub fn quadratic_in_easing(t: f32) -> f32 {
    let t = t.clamp(0.0, 1.0);
    t * t
}

/// 预定义的二次缓动（ease-out）
pub fn quadratic_out_easing(t: f32) -> f32 {
    let t = t.clamp(0.0, 1.0);
    t * (2.0 - t)
}

// ----------------------------------------------------------------------------
// 测试
// ----------------------------------------------------------------------------
#[cfg(test)]
mod tests {
    use super::*;
    use crate::profile::{DEFAULT_MID_PWM_US, PhysicalParams};

    #[test]
    fn test_move_pwm_immediate() {
        let mut engine = MotionEngine::new();
        let id = ServoId::S0;
        let target = PulseWidthUs(2000);
        engine.move_pwm(id, target, 0).unwrap();
        assert_eq!(engine.current_pwm(id).0, 2000);
        assert!(!engine.is_moving(id));
    }

    #[test]
    fn test_move_pwm_normal() {
        let mut engine = MotionEngine::new();
        let id = ServoId::S0;
        let target = PulseWidthUs(2000);
        engine.move_pwm(id, target, 10).unwrap();
        assert!(engine.is_moving(id));
        assert_eq!(engine.remaining_ms(id), 10);

        for _ in 0..10 {
            engine.update_1ms();
        }
        assert!(!engine.is_moving(id));
        assert_eq!(engine.current_pwm(id).0, 2000);
    }

    #[test]
    fn test_move_angle() {
        let mut engine = MotionEngine::new();
        let id = ServoId::S0;
        engine.move_angle(id, 90.0, 10).unwrap();
        // 默认 profile: min=0, max=270, min_pwm=500, max_pwm=2500, mid_pwm=1500
        // 90度对应 (90/270)*2000 + 500 = 500 + 666.67 ≈ 1167
        let expected_pwm = ((90.0 / 270.0) * 2000.0 + 500.0).round() as u16;
        assert_eq!(engine.target_pwm(id).0, expected_pwm);
    }

    #[test]
    fn test_with_profiles() {
        let custom_profile = PhysicalParams {
            min_pwm_us: 600,
            mid_pwm_us: 1500,
            max_pwm_us: 2400,
            min_angle_deg: 10.0,
            mid_angle_deg: 135.0,
            max_angle_deg: 260.0,
        };
        let profiles = [custom_profile; SERVO_COUNT];
        let engine = MotionEngine::with_profiles(profiles).unwrap();

        for id in ALL_SERVOS {
            let p = engine.profile(id);
            assert_eq!(p.min_pwm_us, 600);
            assert_eq!(p.max_pwm_us, 2400);
        }
    }

    #[test]
    fn test_easing_function() {
        let mut engine = MotionEngine::new();
        let id = ServoId::S0;

        engine.set_easing_function(linear_easing);
        engine.move_pwm(id, PulseWidthUs(2000), 10).unwrap();
        engine.update_1ms();
        assert_eq!(engine.current_pwm(id).0, 1550);

        engine.set_easing_function(default_easing);
        engine.move_pwm(id, PulseWidthUs(2000), 10).unwrap();
        engine.update_1ms();
        let cur = engine.current_pwm(id).0;
        assert!(cur > 1510 && cur < 1520);
    }

    #[test]
    fn test_pause_restart() {
        let mut engine = MotionEngine::new();
        let id = ServoId::S0;
        engine.move_pwm(id, PulseWidthUs(2000), 10).unwrap();
        engine.pause(id);
        assert!(!engine.is_moving(id));
        assert_eq!(engine.remaining_ms(id), 10);

        engine.restart(id);
        assert!(engine.is_moving(id));

        for _ in 0..10 {
            engine.update_1ms();
        }
        assert!(!engine.is_moving(id));
    }

    #[test]
    fn test_stop() {
        let mut engine = MotionEngine::new();
        let id = ServoId::S0;
        engine.move_pwm(id, PulseWidthUs(2000), 10).unwrap();
        engine.stop(id);
        assert!(!engine.is_moving(id));
        assert_eq!(engine.remaining_ms(id), 0);
        assert_eq!(engine.current_pwm(id).0, DEFAULT_MID_PWM_US);
    }

    #[test]
    fn test_moving_mask() {
        let mut engine = MotionEngine::new();
        let id1 = ServoId::S0;
        let id2 = ServoId::S1;
        assert_eq!(engine.moving_mask(), 0);

        engine.move_pwm(id1, PulseWidthUs(2000), 10).unwrap();
        assert_eq!(engine.moving_mask(), 1 << 0);

        engine.move_pwm(id2, PulseWidthUs(1800), 5).unwrap();
        assert_eq!(engine.moving_mask(), (1 << 0) | (1 << 1));

        engine.stop(id1);
        assert_eq!(engine.moving_mask(), 1 << 1);

        engine.stop(id2);
        assert_eq!(engine.moving_mask(), 0);
    }
}