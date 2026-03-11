//! servos 层统一错误（不包含 orchestrator）。

#[derive(Debug)]
pub enum ServoError {
    Config(super::config::ConfigError),
    Engine(super::engine::EngineError),
    Control(super::controller::ControlError),
    Hal(super::hal::HalError),
}

impl From<super::config::ConfigError> for ServoError {
    fn from(e: super::config::ConfigError) -> Self {
        Self::Config(e)
    }
}

impl From<super::engine::EngineError> for ServoError {
    fn from(e: super::engine::EngineError) -> Self {
        Self::Engine(e)
    }
}

impl From<super::controller::ControlError> for ServoError {
    fn from(e: super::controller::ControlError) -> Self {
        Self::Control(e)
    }
}

impl From<super::hal::HalError> for ServoError {
    fn from(e: super::hal::HalError) -> Self {
        Self::Hal(e)
    }
}

pub type Result<T> = core::result::Result<T, ServoError>;
