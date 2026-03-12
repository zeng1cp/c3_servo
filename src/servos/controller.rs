//! 面向多舵机组合动作的高层控制单元。
//!
//! `ControlUnit` 基于一组 `ServoId` 构建批量运动请求，负责：
//! - 把 `Angle` / `Pwm` 目标转换成最终脉宽；
//! - 统一启动、暂停、恢复、停止一批舵机；
//! - 根据 `MotionEngine` 的完成 bitmask 驱动自身状态机；
//! - 在整组动作完成时触发可选回调。

use core::ffi::c_void;

use super::config::{ConfigManager, Storage};
use super::engine::MotionEngine;
use super::profile::{
    ALL_SERVOS, AngleDeg, CorrectionParams, PhysicalParams, PulseWidthUs, SERVO_COUNT, ServoId,
};

/// 控制完成后的回调形式。
#[derive(Clone, Copy)]
pub enum Callback {
    /// 无上下文的函数指针回调。
    Simple(fn()),
    /// 带裸指针上下文的 C 风格回调。
    Context {
        func: fn(*mut c_void),
        user_data: *mut c_void,
    },
}

impl Callback {
    /// 从普通函数指针构造回调。
    pub const fn from_fn(func: fn()) -> Self {
        Self::Simple(func)
    }

    /// 从函数指针和用户上下文构造回调。
    pub const fn from_context(func: fn(*mut c_void), user_data: *mut c_void) -> Self {
        Self::Context { func, user_data }
    }

    /// 立即执行回调。
    pub fn invoke(self) {
        match self {
            Self::Simple(func) => func(),
            Self::Context { func, user_data } => func(user_data),
        }
    }
}

pub type ControlCompleteCb = fn();

/// 批量控制流程中的错误。
#[derive(Debug, PartialEq)]
pub enum ControlError {
    /// 目标舵机集合为空，无法构造控制单元。
    EmptyMask,
    /// 目标数组长度与控制单元包含的舵机数量不一致。
    TargetCountMismatch,
    /// 当前状态机不允许执行请求的操作，例如在 `Idle` 状态下暂停。
    InvalidState,
    /// 传入的角度目标不是有限浮点数。
    InvalidAngle,
    /// 底层运动引擎拒绝请求。
    Engine(super::engine::EngineError),
    /// 校正参数或持久化层返回错误。
    Config(super::config::ConfigError),
}

impl From<super::engine::EngineError> for ControlError {
    fn from(value: super::engine::EngineError) -> Self {
        Self::Engine(value)
    }
}

impl From<super::config::ConfigError> for ControlError {
    fn from(value: super::config::ConfigError) -> Self {
        Self::Config(value)
    }
}

/// 单个目标既可以直接给 PWM，也可以给角度。
#[derive(Clone, Copy)]
pub enum Target {
    Pwm(PulseWidthUs),
    Angle(AngleDeg),
}

/// `ControlUnit` 的内部状态机。
#[derive(Clone, Copy, Debug, PartialEq)]
enum ControlUnitState {
    /// 当前没有挂起的批量任务。
    Idle,
    /// 正在等待 `pending_mask` 覆盖的所有舵机完成。
    Moving { pending_mask: u32 },
    /// 任务已暂停，但仍保留尚未完成的舵机集合。
    Paused { pending_mask: u32 },
}

/// 管理一组舵机的批量控制单元。
///
/// `mask` 确定控制范围，状态机只追踪这组舵机的完成情况，不管理外部舵机。
pub struct ControlUnit {
    mask: u32,
    state: ControlUnitState,
    callback: Option<Callback>,
}

impl ControlUnit {
    /// 根据一组舵机构造控制单元。
    ///
    /// 重复的 `ServoId` 会自然折叠到同一个 bit，不会重复计数。
    pub fn new(ids: &[ServoId]) -> Result<Self, ControlError> {
        if ids.is_empty() {
            return Err(ControlError::EmptyMask);
        }

        let mut mask = 0u32;
        for &id in ids {
            mask |= 1 << id.index();
        }

        Ok(Self {
            mask,
            state: ControlUnitState::Idle,
            callback: None,
        })
    }

    /// 为当前控制范围内的所有舵机启动一次批量动作。
    ///
    /// # Errors
    ///
    /// - 当 `targets` 长度与控制范围不匹配时返回 `TargetCountMismatch`；
    /// - 当任一目标转换或引擎下发失败时返回对应错误。
    pub fn start<S: Storage>(
        &mut self,
        engine: &mut MotionEngine,
        config: &ConfigManager<S>,
        targets: &[Target],
        duration_ms: u32,
        callback: Option<Callback>,
    ) -> Result<(), ControlError> {
        let mut ids_buf = [ALL_SERVOS[0]; SERVO_COUNT];
        let ids = self.selected_ids(&mut ids_buf);
        if targets.len() != ids.len() {
            return Err(ControlError::TargetCountMismatch);
        }

        self.stop(engine);

        let mut target_pwms = [PulseWidthUs(0); SERVO_COUNT];
        for (&id, target) in ids.iter().zip(targets.iter()) {
            let phys = engine.profile(id);
            let corr = config.get_correction(id);
            target_pwms[id.index()] = Self::convert_target(target, &phys, &corr)?;
        }

        let mut started = 0usize;
        for &id in ids.iter() {
            if let Err(err) = engine.move_pwm(id, target_pwms[id.index()], duration_ms) {
                // 某个舵机启动失败时回滚此前已启动的舵机，避免形成半成功状态。
                for &started_id in ids[..started].iter() {
                    engine.stop(started_id);
                }
                self.state = ControlUnitState::Idle;
                self.callback = None;
                return Err(err.into());
            }
            started += 1;
        }

        self.state = ControlUnitState::Moving {
            pending_mask: self.mask,
        };
        self.callback = callback;
        self.check_immediate_completion(engine);
        Ok(())
    }

    /// 暂停当前批量任务。
    pub fn pause(&mut self, engine: &mut MotionEngine) -> Result<(), ControlError> {
        match self.state {
            ControlUnitState::Moving { pending_mask } => {
                let mut ids_buf = [ALL_SERVOS[0]; SERVO_COUNT];
                for &id in self.selected_ids(&mut ids_buf) {
                    engine.pause(id);
                }
                self.state = ControlUnitState::Paused { pending_mask };
                Ok(())
            }
            _ => Err(ControlError::InvalidState),
        }
    }

    /// 恢复先前暂停的批量任务。
    pub fn restart(&mut self, engine: &mut MotionEngine) -> Result<(), ControlError> {
        match self.state {
            ControlUnitState::Paused { pending_mask } => {
                let mut ids_buf = [ALL_SERVOS[0]; SERVO_COUNT];
                for &id in self.selected_ids(&mut ids_buf) {
                    engine.restart(id);
                }
                self.state = ControlUnitState::Moving { pending_mask };
                Ok(())
            }
            _ => Err(ControlError::InvalidState),
        }
    }

    /// 停止当前批量任务，并清空完成回调。
    pub fn stop(&mut self, engine: &mut MotionEngine) {
        let mut ids_buf = [ALL_SERVOS[0]; SERVO_COUNT];
        for &id in self.selected_ids(&mut ids_buf) {
            engine.stop(id);
        }
        self.state = ControlUnitState::Idle;
        self.callback = None;
    }

    /// 消费来自 `MotionEngine::update_1ms()` 的完成 bitmask。
    ///
    /// 只有处于 `Moving` 状态时才会推进状态机；暂停状态不会消费完成事件。
    pub fn on_completed_mask(&mut self, completed_mask: u32) {
        match self.state {
            ControlUnitState::Moving { pending_mask } => {
                let new_pending = pending_mask & !completed_mask;
                if new_pending == 0 {
                    self.state = ControlUnitState::Idle;
                    if let Some(cb) = self.callback.take() {
                        cb.invoke();
                    }
                } else {
                    self.state = ControlUnitState::Moving {
                        pending_mask: new_pending,
                    };
                }
            }
            ControlUnitState::Paused { .. } | ControlUnitState::Idle => {}
        }
    }

    /// 当前是否仍有挂起任务。
    pub fn is_active(&self) -> bool {
        !matches!(self.state, ControlUnitState::Idle)
    }

    /// 当前是否处于推进中而非暂停。
    pub fn is_moving(&self) -> bool {
        matches!(self.state, ControlUnitState::Moving { .. })
    }

    /// 返回仍未完成的舵机 bitmask。
    pub fn pending_mask(&self) -> Option<u32> {
        match self.state {
            ControlUnitState::Moving { pending_mask }
            | ControlUnitState::Paused { pending_mask } => Some(pending_mask),
            ControlUnitState::Idle => None,
        }
    }

    fn convert_target(
        target: &Target,
        phys: &PhysicalParams,
        corr: &CorrectionParams,
    ) -> Result<PulseWidthUs, ControlError> {
        match target {
            Target::Pwm(pwm) => {
                // 直接给 PWM 时仍然要经过校正参数，保证反向与偏移一致生效。
                let corrected = corr.apply(pwm.0 as i32, phys.mid_pwm_us);
                Ok(PulseWidthUs(corrected))
            }
            Target::Angle(angle) => {
                let raw_pwm = phys.angle_to_pwm(*angle);
                let corrected = corr.apply(raw_pwm.0 as i32, phys.mid_pwm_us);
                Ok(PulseWidthUs(corrected))
            }
        }
    }

    fn check_immediate_completion(&mut self, engine: &MotionEngine) {
        if let ControlUnitState::Moving { pending_mask } = self.state {
            let mut still_pending = pending_mask;
            let mut ids_buf = [ALL_SERVOS[0]; SERVO_COUNT];
            for &id in self.selected_ids(&mut ids_buf) {
                if (pending_mask >> id.index()) & 1 == 0 {
                    continue;
                }
                if !engine.is_moving(id) {
                    still_pending &= !(1 << id.index());
                }
            }

            if still_pending == 0 {
                // `duration_ms == 0` 等场景下，任务可能在 `start()` 当下已经结束。
                self.state = ControlUnitState::Idle;
                if let Some(cb) = self.callback.take() {
                    cb.invoke();
                }
            } else {
                self.state = ControlUnitState::Moving {
                    pending_mask: still_pending,
                };
            }
        }
    }

    /// 将 `mask` 展开成稳定顺序的 `ServoId` 列表。
    ///
    /// 返回顺序始终与 `ALL_SERVOS` 一致，确保 `targets` 的位置语义稳定。
    fn selected_ids<'a>(&self, buf: &'a mut [ServoId; SERVO_COUNT]) -> &'a [ServoId] {
        let mut count = 0usize;
        for id in ALL_SERVOS {
            if (self.mask >> id.index()) & 1 != 0 {
                buf[count] = id;
                count += 1;
            }
        }
        &buf[..count]
    }
}
