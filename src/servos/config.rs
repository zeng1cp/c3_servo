//! 舵机校正参数的持久化管理。
//!
//! 该模块只持久化 `CorrectionParams`，不持久化物理参数：
//! 物理参数通常随固件编译产物提供，而校正参数需要在设备运行后调试并保存。

use super::profile::{self, CorrectionParams, SERVO_COUNT, ServoId};

/// 单个 `CorrectionParams` 的固定序列化长度。
const PARAM_SIZE: usize = 8;
const PAYLOAD_SIZE: usize = SERVO_COUNT * PARAM_SIZE;
const HEADER_SIZE: usize = 16;
const STORED_SIZE: usize = HEADER_SIZE + PAYLOAD_SIZE;
const FORMAT_MAGIC: [u8; 4] = *b"SCFG";
const FORMAT_VERSION: u8 = 1;

/// 存储层访问失败时返回的错误。
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum StorageError {
    /// 读取底层存储介质失败。
    ReadFailed,
    /// 写入底层存储介质失败。
    WriteFailed,
    /// 擦除底层存储介质失败。
    EraseFailed,
    /// 底层返回的读写长度与请求不符。
    InvalidLength,
}

/// 面向字节数组的最小存储抽象。
///
/// 偏移量和长度均由 `ConfigManager` 控制，具体介质可以是 EEPROM、Flash 或测试桩。
pub trait Storage {
    fn read(&mut self, offset: usize, buf: &mut [u8]) -> Result<usize, StorageError>;

    fn write(&mut self, offset: usize, buf: &[u8]) -> Result<usize, StorageError>;

    fn erase(&mut self, offset: usize, len: usize) -> Result<(), StorageError> {
        let _ = (offset, len);
        Ok(())
    }
}

/// 配置管理器在解析或保存配置时返回的错误。
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ConfigError {
    /// 单个校正项的字节内容非法。
    Profile(profile::ConfigError),
    /// 底层存储介质访问失败。
    Storage(StorageError),
    /// 存储中的总长度与当前格式定义不一致。
    InvalidDataLength,
    /// 头部魔数不匹配，通常表示该区域未写入本模块格式的数据。
    InvalidMagic,
    /// 读取到受支持范围之外的配置版本。
    UnsupportedVersion(u8),
    /// CRC 校验失败，说明存储内容已损坏或未写完整。
    ChecksumMismatch,
}

/// `load()` 的结果状态。
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum LoadStatus {
    /// 成功读取并解析已有配置。
    Loaded,
    /// 存储区域为空或处于擦除态。
    Empty,
    /// 读取失败后回退到默认值，并携带触发回退的错误。
    Defaulted(ConfigError),
}

impl From<profile::ConfigError> for ConfigError {
    fn from(value: profile::ConfigError) -> Self {
        Self::Profile(value)
    }
}

impl From<StorageError> for ConfigError {
    fn from(value: StorageError) -> Self {
        Self::Storage(value)
    }
}

/// 管理校正参数在内存与存储层之间同步的组件。
pub struct ConfigManager<S: Storage> {
    storage: S,
    corrections: [CorrectionParams; SERVO_COUNT],
    dirty: bool,
}

impl<S: Storage> ConfigManager<S> {
    /// 创建配置管理器，并要求存储中的数据必须可成功加载。
    pub fn new(storage: S) -> Result<Self, ConfigError> {
        let mut mgr = Self::default_with_storage(storage);
        match mgr.load()? {
            LoadStatus::Loaded | LoadStatus::Empty => Ok(mgr),
            LoadStatus::Defaulted(_) => unreachable!("load() never returns Defaulted"),
        }
    }

    /// 创建配置管理器，并在加载失败时回退到默认校正参数。
    ///
    /// 返回的 `LoadStatus` 可让上层区分是正常加载、空存储还是错误后回退。
    pub fn new_with_default_fallback(storage: S) -> Result<(Self, LoadStatus), ConfigError> {
        let mut mgr = Self::default_with_storage(storage);
        let status = match mgr.load() {
            Ok(status) => status,
            Err(err) => {
                mgr.dirty = true;
                LoadStatus::Defaulted(err)
            }
        };
        Ok((mgr, status))
    }

    /// 读取指定舵机的校正参数副本。
    pub fn get_correction(&self, id: ServoId) -> CorrectionParams {
        self.corrections[id.index()]
    }

    /// 更新指定舵机的校正参数，并标记配置为 dirty。
    pub fn set_correction(&mut self, id: ServoId, params: CorrectionParams) {
        self.corrections[id.index()] = params;
        self.dirty = true;
    }

    /// 将当前校正参数写回存储层。
    ///
    /// 只有 `dirty == true` 时才会真正触发擦除与写入。
    pub fn save(&mut self) -> Result<(), ConfigError> {
        if !self.dirty {
            return Ok(());
        }

        let payload = self.serialize_payload();
        let checksum = crc32(&payload);

        let mut buf = [0u8; STORED_SIZE];
        buf[..4].copy_from_slice(&FORMAT_MAGIC);
        buf[4] = FORMAT_VERSION;
        buf[8..12].copy_from_slice(&(PAYLOAD_SIZE as u32).to_le_bytes());
        buf[12..16].copy_from_slice(&checksum.to_le_bytes());
        buf[HEADER_SIZE..].copy_from_slice(&payload);

        // 先擦后写，避免旧内容残留导致 CRC 与长度字段不一致。
        self.storage.erase(0, STORED_SIZE)?;

        let written = self.storage.write(0, &buf)?;
        if written != STORED_SIZE {
            return Err(StorageError::InvalidLength.into());
        }

        self.dirty = false;
        Ok(())
    }

    /// 从存储层加载校正参数。
    ///
    /// 该方法会校验魔数、版本、长度与 CRC；任一校验失败都会拒绝接纳该数据。
    pub fn load(&mut self) -> Result<LoadStatus, ConfigError> {
        let mut buf = [0u8; STORED_SIZE];
        let read = self.storage.read(0, &mut buf)?;

        if read == 0 {
            return Ok(LoadStatus::Empty);
        }

        if read != STORED_SIZE {
            if is_erased(&buf[..read]) {
                return Ok(LoadStatus::Empty);
            }
            return Err(ConfigError::InvalidDataLength);
        }

        if is_erased(&buf) {
            return Ok(LoadStatus::Empty);
        }

        if buf[..4] != FORMAT_MAGIC {
            return Err(ConfigError::InvalidMagic);
        }

        let version = buf[4];
        if version != FORMAT_VERSION {
            return Err(ConfigError::UnsupportedVersion(version));
        }

        let payload_len = u32::from_le_bytes([buf[8], buf[9], buf[10], buf[11]]) as usize;
        if payload_len != PAYLOAD_SIZE {
            return Err(ConfigError::InvalidDataLength);
        }

        let expected_crc = u32::from_le_bytes([buf[12], buf[13], buf[14], buf[15]]);
        let payload = &buf[HEADER_SIZE..];
        if crc32(payload) != expected_crc {
            return Err(ConfigError::ChecksumMismatch);
        }

        let mut corrections = [CorrectionParams::default(); SERVO_COUNT];
        for (index, chunk) in payload.chunks_exact(PARAM_SIZE).enumerate() {
            corrections[index] = CorrectionParams::from_bytes(chunk)?;
        }

        self.corrections = corrections;
        self.dirty = false;
        Ok(LoadStatus::Loaded)
    }

    /// 当前内存中的校正参数是否尚未持久化。
    pub fn is_dirty(&self) -> bool {
        self.dirty
    }

    /// 暴露底层存储对象的可变引用，便于上层做介质级操作或测试注入。
    pub fn storage_mut(&mut self) -> &mut S {
        &mut self.storage
    }

    fn default_with_storage(storage: S) -> Self {
        Self {
            storage,
            corrections: [CorrectionParams::default(); SERVO_COUNT],
            dirty: false,
        }
    }

    fn serialize_payload(&self) -> [u8; PAYLOAD_SIZE] {
        let mut payload = [0u8; PAYLOAD_SIZE];
        for (index, params) in self.corrections.iter().enumerate() {
            let offset = index * PARAM_SIZE;
            payload[offset..offset + PARAM_SIZE].copy_from_slice(&params.to_bytes());
        }
        payload
    }
}

fn is_erased(buf: &[u8]) -> bool {
    !buf.is_empty()
        && (buf.iter().all(|&byte| byte == 0x00) || buf.iter().all(|&byte| byte == 0xFF))
}

/// 计算 payload 的 CRC32，用于检测部分写入或位翻转。
fn crc32(bytes: &[u8]) -> u32 {
    let mut crc = 0xFFFF_FFFFu32;
    for &byte in bytes {
        crc ^= u32::from(byte);
        for _ in 0..8 {
            let mask = (crc & 1).wrapping_neg() & 0xEDB8_8320;
            crc = (crc >> 1) ^ mask;
        }
    }
    !crc
}
