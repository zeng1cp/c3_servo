//! 舵机子系统的统一错误类型。
//!
//! 该模块将 `profile`、`config`、`engine`、`controller`、`orchestrator`
//! 与 `hal` 的错误收敛到一个枚举中，便于上层在应用边界统一处理。

use super::{config, controller, engine, hal, orchestrator, profile};

/// 汇总 `servos` 子模块错误的顶层枚举。
///
/// 该类型保留底层子模块的错误语义，适合在应用层继续向上传播。
#[derive(Debug)]
pub enum Error {
    /// 舵机参数或校正参数非法。
    Profile(profile::ConfigError),
    /// PWM 输出硬件层失败，例如底层驱动拒绝设置占空比。
    Hal(hal::HalError),
    /// 运动引擎拒绝本次运动请求。
    Engine(engine::EngineError),
    /// 校正参数持久化或反序列化失败。
    Config(config::ConfigError),
    /// 控制单元状态机或批量下发过程失败。
    Control(controller::ControlError),
    /// 序列编排输入非法，或序列执行阶段的控制调用失败。
    Sequence(orchestrator::sequence::SequenceError),
    /// 未细分到具体模块的静态错误。
    Other(&'static str),
}

impl From<profile::ConfigError> for Error {
    fn from(value: profile::ConfigError) -> Self {
        Self::Profile(value)
    }
}

impl From<hal::HalError> for Error {
    fn from(value: hal::HalError) -> Self {
        Self::Hal(value)
    }
}

impl From<engine::EngineError> for Error {
    fn from(value: engine::EngineError) -> Self {
        Self::Engine(value)
    }
}

impl From<config::ConfigError> for Error {
    fn from(value: config::ConfigError) -> Self {
        Self::Config(value)
    }
}

impl From<controller::ControlError> for Error {
    fn from(value: controller::ControlError) -> Self {
        Self::Control(value)
    }
}

impl From<orchestrator::sequence::SequenceError> for Error {
    fn from(value: orchestrator::sequence::SequenceError) -> Self {
        Self::Sequence(value)
    }
}

/// 舵机子系统统一 `Result` 别名。
pub type Result<T> = core::result::Result<T, Error>;
