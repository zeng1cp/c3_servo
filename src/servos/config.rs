//! 配置管理器与持久化存储。 

use super::config_codec::{ConfigBlob, ConfigCodec, LoadOutcome};
use super::profile::{self, CorrectionParams, ServoId, SERVO_COUNT};

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum StorageError {
    ReadFailed,
    WriteFailed,
    EraseFailed,
    InvalidLength,
}

pub trait Storage {
    fn read_exact(&mut self, offset: usize, buf: &mut [u8]) -> Result<(), StorageError>;
    fn write_all(&mut self, offset: usize, buf: &[u8]) -> Result<(), StorageError>;

    fn erase_range(&mut self, offset: usize, len: usize) -> Result<(), StorageError> {
        let _ = (offset, len);
        Ok(())
    }

    fn requires_erase_before_write(&self) -> bool {
        false
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ConfigError {
    Profile(profile::ConfigError),
    Storage(StorageError),
    InvalidHeader,
    InvalidVersion,
    InvalidChecksum,
    InvalidDataLength,
}

impl From<profile::ConfigError> for ConfigError {
    fn from(e: profile::ConfigError) -> Self {
        Self::Profile(e)
    }
}

impl From<StorageError> for ConfigError {
    fn from(e: StorageError) -> Self {
        Self::Storage(e)
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum InitStatus {
    Loaded,
    Defaulted,
}

pub struct ConfigManager<S: Storage> {
    storage: S,
    corrections: [CorrectionParams; SERVO_COUNT],
    dirty: bool,
}

impl<S: Storage> ConfigManager<S> {
    pub fn new_default(storage: S) -> Self {
        Self {
            storage,
            corrections: [CorrectionParams::default(); SERVO_COUNT],
            dirty: false,
        }
    }

    pub fn new_with_load(storage: S) -> Result<(Self, InitStatus), ConfigError> {
        let mut mgr = Self::new_default(storage);
        match mgr.load()? {
            LoadOutcome::Loaded(_) => Ok((mgr, InitStatus::Loaded)),
            LoadOutcome::Empty => Ok((mgr, InitStatus::Defaulted)),
        }
    }

    pub fn get_correction(&self, id: ServoId) -> CorrectionParams {
        self.corrections[id.index()]
    }

    pub fn set_correction(&mut self, id: ServoId, params: CorrectionParams) {
        if self.corrections[id.index()] != params {
            self.corrections[id.index()] = params;
            self.dirty = true;
        }
    }

    pub fn corrections(&self) -> &[CorrectionParams; SERVO_COUNT] {
        &self.corrections
    }

    pub fn save(&mut self) -> Result<(), ConfigError> {
        if !self.dirty {
            return Ok(());
        }

        let mut buf = [0u8; ConfigBlob::MAX_SIZE];
        let used = ConfigCodec::encode(&self.corrections, &mut buf)?;

        if self.storage.requires_erase_before_write() {
            self.storage.erase_range(0, used)?;
        }
        self.storage.write_all(0, &buf[..used])?;

        self.dirty = false;
        Ok(())
    }

    pub fn load(&mut self) -> Result<LoadOutcome, ConfigError> {
        let mut buf = [0u8; ConfigBlob::MAX_SIZE];
        self.storage.read_exact(0, &mut buf)?;

        match ConfigCodec::decode(&buf)? {
            LoadOutcome::Loaded(corrections) => {
                self.corrections = corrections;
                self.dirty = false;
                Ok(LoadOutcome::Loaded(corrections))
            }
            LoadOutcome::Empty => Ok(LoadOutcome::Empty),
        }
    }

    pub fn is_dirty(&self) -> bool {
        self.dirty
    }

    pub fn storage_mut(&mut self) -> &mut S {
        &mut self.storage
    }
}
