//! 控制器层
//!
//! 提供统一的对外控制接口 `ControlUnit`，代表一次舵机控制操作。
//! `ControlUnit` 可作用于单个或多个舵机，自动应用校正参数（反向、偏移），
//! 并跟踪运动完成状态。上层需在每毫秒更新后，将 `engine` 返回的完成掩码
//! 传递给 `on_completed_mask` 以推进状态。

use core::ffi::c_void;

use super::profile::{ServoId, PulseWidthUs, AngleDeg, PhysicalParams, CorrectionParams, ALL_SERVOS, SERVO_COUNT};
use super::engine::MotionEngine;
use super::config::{ConfigManager, Storage};

/// 回调结构（简化版，直接定义在此处）
pub struct Callback {
    pub func: fn(*mut c_void),
    pub user_data: *mut c_void,
}

/// 控制单元完成回调类型
pub type ControlCompleteCb = fn(*mut c_void);

/// 控制单元错误
#[derive(Debug, PartialEq)]
pub enum ControlError {
    /// 舵机掩码为空（没有指定任何舵机）
    EmptyMask,
    /// 目标数量与掩码不匹配
    TargetCountMismatch,
    /// 当前状态下不允许该操作（例如未启动时暂停）
    InvalidState,
    /// 角度值无效（例如 NaN）
    InvalidAngle,
    /// 引擎错误
    Engine(super::engine::EngineError),
    /// 配置错误（例如校正参数读取失败）
    Config(super::config::ConfigError),
}

impl From<super::engine::EngineError> for ControlError {
    fn from(e: super::engine::EngineError) -> Self {
        Self::Engine(e)
    }
}

impl From<super::config::ConfigError> for ControlError {
    fn from(e: super::config::ConfigError) -> Self {
        Self::Config(e)
    }
}

/// 目标类型（用户输入）
#[derive(Clone, Copy)]
pub enum Target {
    /// 直接指定脉冲宽度（微秒）
    Pwm(PulseWidthUs),
    /// 指定角度（度）
    Angle(AngleDeg),
}

/// 控制单元内部状态
#[derive(Clone, Copy, Debug, PartialEq)]
enum ControlUnitState {
    /// 空闲（未启动或已完成）
    Idle,
    /// 运动中，记录尚未完成的舵机掩码
    Moving { pending_mask: u32 },
    /// 已暂停，记录尚未完成的舵机掩码
    Paused { pending_mask: u32 },
}

/// 控制单元
///
/// 代表一次舵机控制操作，可作用于多个舵机。使用前需通过 `new` 指定参与舵机，
/// 然后调用 `start` 启动运动。之后，上层应在每毫秒调用 `engine.update_1ms()`
/// 后，将返回的完成掩码传递给 `on_completed_mask` 方法，该方法会更新内部状态
/// 并在全部完成时触发回调。
///
/// # 示例
/// ```
/// let mut unit = ControlUnit::new(&[ServoId::S0, ServoId::S1])?;
/// unit.start(
///     &mut engine,
///     &config,
///     &[Target::Angle(AngleDeg(90.0)), Target::Pwm(PulseWidthUs(2000))],
///     1000,
///     Some(Callback { func: my_callback, user_data: ptr })
/// )?;
///
/// loop {
///     let completed = engine.update_1ms();
///     unit.on_completed_mask(completed);
///     // ... 输出到硬件 ...
/// }
/// ```
pub struct ControlUnit {
    /// 参与运动的舵机掩码（每个位对应一个舵机）
    mask: u32,
    /// 当前状态
    state: ControlUnitState,
    /// 完成回调
    callback: Option<Callback>,
}

impl ControlUnit {
    /// 创建一个新的控制单元，指定参与运动的舵机。
    ///
    /// # 参数
    /// * `ids` - 参与运动的舵机ID列表（至少一个）
    ///
    /// # 错误
    /// 如果 `ids` 为空，返回 `ControlError::EmptyMask`。
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

    /// 启动运动。
    ///
    /// # 参数
    /// * `engine` - 运动引擎的可变引用
    /// * `config` - 配置管理器，用于获取当前校正参数
    /// * `targets` - 每个舵机的目标（必须与 `new` 时指定的 `ids` 长度一致）
    /// * `duration_ms` - 运动持续时间（毫秒）
    /// * `callback` - 运动完成时的回调（可选）
    ///
    /// # 行为
    /// - 如果控制单元当前处于运动中或暂停状态，会先中止当前运动，然后开始新运动。
    /// - 每个目标会结合当前校正参数转换为实际脉冲宽度，然后调用 `engine.move_pwm`。
    /// - 如果所有舵机立即完成（duration=0 或目标与当前位置相同），回调会同步触发。
    ///
    /// # 错误
    /// - 如果 `targets` 长度与 `ids` 不一致，返回 `TargetCountMismatch`。
    /// - 如果角度转换失败或引擎调用失败，返回相应错误，并停止已启动的舵机。
    pub fn start<S: Storage>(
        &mut self,
        engine: &mut MotionEngine,
        config: &ConfigManager<S>,
        targets: &[Target],
        duration_ms: u32,
        callback: Option<Callback>,
    ) -> Result<(), ControlError> {
        // 检查目标数量
        let count = self.mask.count_ones() as usize;
        if targets.len() != count {
            return Err(ControlError::TargetCountMismatch);
        }

        // 中止当前运动（如果存在）
        self.stop(engine);

        // 第一遍：将目标转换为校正后的脉冲，存储到临时数组
        let mut target_pwms = [PulseWidthUs(0); SERVO_COUNT];
        let mut idx_iter = ALL_SERVOS.iter().enumerate().filter(|(_, id)| (self.mask >> id.index()) & 1 != 0);
        for (target, (_, id)) in targets.iter().zip(idx_iter.by_ref()) {
            let phys = engine.profile(*id);
            let corr = config.get_correction(*id);
            target_pwms[id.index()] = Self::convert_target(target, &phys, &corr)?;
        }

        // 第二遍：启动每个舵机的运动
        let mut started = 0u32;
        for id in ALL_SERVOS {
            if (self.mask >> id.index()) & 1 == 0 {
                continue;
            }
            if let Err(e) = engine.move_pwm(id, target_pwms[id.index()], duration_ms) {
                // 停止已启动的舵机
                for id2 in ALL_SERVOS {
                    if (started >> id2.index()) & 1 != 0 {
                        engine.stop(id2);
                    }
                }
                self.state = ControlUnitState::Idle;
                self.callback = None;
                return Err(e.into());
            }
            started |= 1 << id.index();
        }

        // 设置新状态
        self.state = ControlUnitState::Moving { pending_mask: self.mask };
        self.callback = callback;

        // 检查是否有立即完成的舵机
        self.check_immediate_completion(engine);

        Ok(())
    }

    /// 暂停运动。
    ///
    /// 如果控制单元正在运动，则暂停所有参与舵机，并保持内部状态。
    /// 如果已暂停或空闲，返回 `ControlError::InvalidState`。
    pub fn pause(&mut self, engine: &mut MotionEngine) -> Result<(), ControlError> {
        match self.state {
            ControlUnitState::Moving { pending_mask } => {
                for id in ALL_SERVOS {
                    if (self.mask >> id.index()) & 1 != 0 {
                        engine.pause(id);
                    }
                }
                self.state = ControlUnitState::Paused { pending_mask };
                Ok(())
            }
            _ => Err(ControlError::InvalidState),
        }
    }

    /// 恢复运动。
    ///
    /// 如果控制单元已暂停，则恢复所有参与舵机。
    /// 如果正在运动或空闲，返回 `ControlError::InvalidState`。
    pub fn restart(&mut self, engine: &mut MotionEngine) -> Result<(), ControlError> {
        match self.state {
            ControlUnitState::Paused { pending_mask } => {
                for id in ALL_SERVOS {
                    if (self.mask >> id.index()) & 1 != 0 {
                        engine.restart(id);
                    }
                }
                self.state = ControlUnitState::Moving { pending_mask };
                Ok(())
            }
            _ => Err(ControlError::InvalidState),
        }
    }

    /// 停止运动。
    ///
    /// 无论当前状态如何，停止所有参与舵机的运动，并将控制单元重置为空闲状态。
    pub fn stop(&mut self, engine: &mut MotionEngine) {
        for id in ALL_SERVOS {
            if (self.mask >> id.index()) & 1 != 0 {
                engine.stop(id);
            }
        }
        self.state = ControlUnitState::Idle;
        self.callback = None;
    }

    /// 处理引擎更新返回的完成掩码。
    ///
    /// 此方法应由上层在每次 `engine.update_1ms()` 后调用，传入返回的 `completed_mask`。
    /// 它会检查掩码中与本控制单元相关的完成位，更新内部状态，并在所有舵机完成时触发回调。
    pub fn on_completed_mask(&mut self, completed_mask: u32) {
        match self.state {
            ControlUnitState::Moving { pending_mask } => {
                let new_pending = pending_mask & !completed_mask;
                if new_pending == 0 {
                    // 全部完成
                    self.state = ControlUnitState::Idle;
                    if let Some(cb) = self.callback.take() {
                        (cb.func)(cb.user_data);
                    }
                } else {
                    self.state = ControlUnitState::Moving { pending_mask: new_pending };
                }
            }
            ControlUnitState::Paused { pending_mask } => {
                // 暂停状态不处理完成事件
            }
            ControlUnitState::Idle => {}
        }
    }

    /// 检查控制单元是否处于活跃状态（运动中或暂停）。
    pub fn is_active(&self) -> bool {
        !matches!(self.state, ControlUnitState::Idle)
    }

    /// 检查控制单元是否正在运动（未暂停）。
    pub fn is_moving(&self) -> bool {
        matches!(self.state, ControlUnitState::Moving { .. })
    }

    /// 获取当前未完成的舵机掩码（仅在 Moving 或 Paused 时有意义）。
    pub fn pending_mask(&self) -> Option<u32> {
        match self.state {
            ControlUnitState::Moving { pending_mask } | ControlUnitState::Paused { pending_mask } => Some(pending_mask),
            _ => None,
        }
    }

    // ---------- 内部辅助方法 ----------

    /// 将用户目标转换为校正后的脉冲宽度
    fn convert_target(
        target: &Target,
        phys: &PhysicalParams,
        corr: &CorrectionParams,
    ) -> Result<PulseWidthUs, ControlError> {
        match target {
            Target::Pwm(pwm) => {
                let corrected = corr.apply(pwm.0 as i32, phys.mid_pwm_us);
                Ok(PulseWidthUs(corrected))
            }
            Target::Angle(angle) => {
                // 使用 PhysicalParams 提供的角度转 PWM 方法
                let raw_pwm = phys.angle_to_pwm(*angle);
                let corrected = corr.apply(raw_pwm.0 as i32, phys.mid_pwm_us);
                Ok(PulseWidthUs(corrected))
            }
        }
    }

    /// 检查是否有舵机立即完成（duration=0 或目标与当前位置相同）
    fn check_immediate_completion(&mut self, engine: &MotionEngine) {
        if let ControlUnitState::Moving { pending_mask } = self.state {
            let mut still_pending = pending_mask;
            for id in ALL_SERVOS {
                if (pending_mask >> id.index()) & 1 == 0 {
                    continue;
                }
                // 如果舵机不在运动中，说明已停止（可能因为目标与当前相同或 duration=0）
                if !engine.is_moving(id) {
                    still_pending &= !(1 << id.index());
                }
            }
            if still_pending == 0 {
                // 全部立即完成
                self.state = ControlUnitState::Idle;
                if let Some(cb) = self.callback.take() {
                    (cb.func)(cb.user_data);
                }
            } else {
                self.state = ControlUnitState::Moving { pending_mask: still_pending };
            }
        }
    }
}

// ----------------------------------------------------------------------------
// 测试
// ----------------------------------------------------------------------------
#[cfg(test)]
mod tests {
    use super::*;
    use super::profile::{PhysicalParams, DEFAULT_MID_PWM_US, CorrectionParams};
    use super::config::{ConfigManager, Storage, StorageError};

    // 模拟存储
    struct MockStorage;
    impl Storage for MockStorage {
        fn read(&mut self, _offset: usize, _buf: &mut [u8]) -> Result<usize, StorageError> { unimplemented!() }
        fn write(&mut self, _offset: usize, _buf: &[u8]) -> Result<usize, StorageError> { unimplemented!() }
    }

    // 辅助函数：创建测试用的引擎和配置
    fn setup() -> (MotionEngine, ConfigManager<MockStorage>) {
        let engine = MotionEngine::new();
        let storage = MockStorage;
        let config = ConfigManager::new(storage).unwrap();
        (engine, config)
    }

    #[test]
    fn test_new_empty() {
        assert!(ControlUnit::new(&[]).is_err());
    }

    #[test]
    fn test_start_stop() {
        let (mut engine, config) = setup();
        let mut unit = ControlUnit::new(&[ServoId::S0]).unwrap();

        let target = Target::Pwm(PulseWidthUs(2000));
        unit.start(&mut engine, &config, &[target], 10, None).unwrap();
        assert!(unit.is_moving());

        unit.stop(&mut engine);
        assert!(!unit.is_active());
    }

    #[test]
    fn test_pause_restart() {
        let (mut engine, config) = setup();
        let mut unit = ControlUnit::new(&[ServoId::S0]).unwrap();

        let target = Target::Pwm(PulseWidthUs(2000));
        unit.start(&mut engine, &config, &[target], 10, None).unwrap();
        assert!(unit.is_moving());

        unit.pause(&mut engine).unwrap();
        assert!(!unit.is_moving());
        assert!(unit.is_active());

        unit.restart(&mut engine).unwrap();
        assert!(unit.is_moving());
    }

    #[test]
    fn test_on_completed_mask() {
        let (mut engine, config) = setup();
        let mut unit = ControlUnit::new(&[ServoId::S0, ServoId::S1]).unwrap();

        let targets = [
            Target::Pwm(PulseWidthUs(2000)),
            Target::Pwm(PulseWidthUs(1800)),
        ];
        unit.start(&mut engine, &config, &targets, 10, None).unwrap();
        assert_eq!(unit.pending_mask(), Some(0b11));

        // 模拟 S0 完成
        unit.on_completed_mask(1 << 0);
        assert_eq!(unit.pending_mask(), Some(1 << 1));
        assert!(unit.is_moving());

        // 模拟 S1 完成
        unit.on_completed_mask(1 << 1);
        assert!(!unit.is_active());
        assert_eq!(unit.pending_mask(), None);
    }
}