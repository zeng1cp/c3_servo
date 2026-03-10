//! 统一错误类型，汇总各模块错误。

use super::{config, controller, engine, hal, orchestrator, profile};

#[derive(Debug)]
pub enum Error {
    Profile(profile::ConfigError),
    Hal(hal::HalError),
    Engine(engine::EngineError),
    Config(config::ConfigError),
    Control(controller::ControlError),
    Sequence(orchestrator::sequence::SequenceError),
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

pub type Result<T> = core::result::Result<T, Error>;
