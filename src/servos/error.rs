//! 统一的错误类型，汇集各模块的错误变体

use super::profile::ConfigError;

/// 硬件抽象层错误（预留）
#[derive(Debug)]
pub enum HalError {
    /// 硬件驱动错误（例如 PWM 设置失败）
    Driver,
    /// 无效的通道
    InvalidChannel,
}

/// 运动引擎层错误（预留）
#[derive(Debug)]
pub enum MotionError {
    /// 运动时长无效（例如 duration_ms 过长导致步数溢出）
    InvalidDuration,
    /// 其他引擎错误
    Other,
}

/// 控制层错误（预留）
#[derive(Debug)]
pub enum ControlError {
    /// 控制单元 ID 无效
    InvalidUnitId,
    /// 舵机列表长度与目标列表长度不匹配
    MismatchedLength,
    /// 控制单元未启动
    NotStarted,
}

/// 编排层错误（预留）
#[derive(Debug)]
pub enum SequenceError {
    /// 无效的序列 ID
    InvalidSequenceId,
    /// 序列配置错误（例如姿态数为0）
    InvalidConfig,
    /// 没有空闲的序列槽位
    NoFreeSlot,
}

/// 整个舵机库的顶层错误类型
#[derive(Debug)]
pub enum Error {
    /// 配置层错误
    Config(ConfigError),
    /// 硬件抽象层错误
    Hal(HalError),
    /// 运动引擎层错误
    Motion(MotionError),
    /// 控制层错误
    Control(ControlError),
    /// 编排层错误
    Sequence(SequenceError),
    /// 其他未分类的错误
    Other(&'static str),
}

// 实现从各模块错误到顶层错误的自动转换
impl From<ConfigError> for Error {
    fn from(e: ConfigError) -> Self {
        Self::Config(e)
    }
}

impl From<HalError> for Error {
    fn from(e: HalError) -> Self {
        Self::Hal(e)
    }
}

impl From<MotionError> for Error {
    fn from(e: MotionError) -> Self {
        Self::Motion(e)
    }
}

impl From<ControlError> for Error {
    fn from(e: ControlError) -> Self {
        Self::Control(e)
    }
}

impl From<SequenceError> for Error {
    fn from(e: SequenceError) -> Self {
        Self::Sequence(e)
    }
}

/// 库内常用的 Result 类型
pub type Result<T> = core::result::Result<T, Error>;