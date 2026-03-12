//! 舵机子系统的顶层模块。
//!
//! 该目录按职责拆分为：
//! - `profile`：舵机物理参数、校正参数与基础类型。
//! - `config`：校正参数的持久化读写。
//! - `engine`：按 1ms tick 推进的运动插值引擎。
//! - `controller`：面向一组舵机的批量控制与完成回调。
//! - `orchestrator`：更高层的序列化动作编排。
//! - `hal`：PWM 输出硬件抽象。
//! - `error`：跨子模块的统一错误出口。

/// 校正参数的存储格式与持久化管理。
pub mod config;
/// 多舵机协同控制与回调管理。
pub mod controller;
/// 运动插值与运行时状态管理。
pub mod engine;
/// 统一错误类型与 `Result` 别名。
pub mod error;
/// 舵机 PWM 输出的硬件抽象层。
pub mod hal;
/// 更高层的动作序列编排逻辑。
pub mod orchestrator;
/// 舵机物理参数、校正参数与基础类型定义。
pub mod profile;
