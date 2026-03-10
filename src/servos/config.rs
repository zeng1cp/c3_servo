//! 配置管理器与持久化存储
//!
//! 本模块提供舵机校正参数的统一管理，并支持通过抽象的 `Storage` trait 进行非易失存储。
//! 上层可通过 `ConfigManager` 读取/修改单个舵机的校正参数，并调用 `save()` 持久化。

use super::profile::{self, CorrectionParams, ServoId, SERVO_COUNT};

// ----------------------------------------------------------------------------
// 存储抽象
// ----------------------------------------------------------------------------

/// 存储操作错误类型
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum StorageError {
    /// 读取失败（例如地址无效、硬件错误）
    ReadFailed,
    /// 写入失败
    WriteFailed,
    /// 擦除失败（对于 Flash 等介质）
    EraseFailed,
    /// 数据长度不匹配
    InvalidLength,
}

/// 存储介质抽象 trait
///
/// 实现此 trait 即可将配置管理器适配到不同的非易失存储（EEPROM、Flash、文件等）。
pub trait Storage {
    /// 从指定偏移读取数据到缓冲区，返回实际读取的字节数。
    ///
    /// 如果偏移或长度超出介质范围，应返回错误。
    fn read(&mut self, offset: usize, buf: &mut [u8]) -> Result<usize, StorageError>;

    /// 将缓冲区数据写入指定偏移，返回实际写入的字节数。
    ///
    /// 对于某些介质（如 Flash），可能需要在写入前确保相应区域已擦除。
    fn write(&mut self, offset: usize, buf: &[u8]) -> Result<usize, StorageError>;

    /// 擦除指定区域（如果需要）。对于不需要擦除的介质（如 EEPROM），可直接返回成功。
    fn erase(&mut self, offset: usize, len: usize) -> Result<(), StorageError> {
        // 默认实现：无操作，返回成功
        let _ = (offset, len);
        Ok(())
    }
}

// ----------------------------------------------------------------------------
// 配置管理器错误
// ----------------------------------------------------------------------------

/// 配置管理器错误类型
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ConfigError {
    /// 校正参数解析错误（来自 profile 模块）
    Profile(profile::ConfigError),
    /// 存储操作错误
    Storage(StorageError),
    /// 存储数据长度不匹配（读取的数据长度不等于期望值）
    InvalidDataLength,
}

// 允许从 profile::ConfigError 和 StorageError 自动转换
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

// ----------------------------------------------------------------------------
// 配置管理器
// ----------------------------------------------------------------------------

/// 舵机校正参数管理器
///
/// 内部维护所有舵机的 `CorrectionParams`，并提供读取、修改、保存、加载功能。
/// 保存和加载通过传入的 `Storage` 实现完成。
pub struct ConfigManager<S: Storage> {
    storage: S,
    corrections: [CorrectionParams; SERVO_COUNT],
    dirty: bool, // 标记是否有未保存的修改
}

impl<S: Storage> ConfigManager<S> {
    /// 创建新的配置管理器，并从存储中加载已有数据。
    ///
    /// 如果存储中无有效数据或读取失败，则使用默认参数（`CorrectionParams::default()`）。
    pub fn new(storage: S) -> Result<Self, ConfigError> {
        let mut mgr = Self {
            storage,
            corrections: [CorrectionParams::default(); SERVO_COUNT],
            dirty: false,
        };
        // 尝试加载，忽略错误（使用默认值）
        let _ = mgr.load();
        Ok(mgr)
    }

    /// 获取指定舵机的校正参数副本。
    pub fn get_correction(&self, id: ServoId) -> CorrectionParams {
        self.corrections[id.index()]
    }

    /// 设置指定舵机的校正参数，并标记脏数据。
    pub fn set_correction(&mut self, id: ServoId, params: CorrectionParams) {
        self.corrections[id.index()] = params;
        self.dirty = true;
    }

    /// 将当前所有校正参数保存到存储介质。
    ///
    /// 如果 `dirty` 为 `false`，则直接返回成功，不执行实际写入。
    pub fn save(&mut self) -> Result<(), ConfigError> {
        if !self.dirty {
            return Ok(());
        }

        // 计算总数据大小
        const PARAM_SIZE: usize = 8; // CorrectionParams::to_bytes() 返回 [u8;8]
        const TOTAL_SIZE: usize = SERVO_COUNT * PARAM_SIZE;

        // 在栈上分配缓冲区（SERVO_COUNT 较小，如6时仅48字节）
        let mut buf = [0u8; TOTAL_SIZE];

        // 将所有校正参数序列化到缓冲区
        for (i, params) in self.corrections.iter().enumerate() {
            let offset = i * PARAM_SIZE;
            buf[offset..offset + PARAM_SIZE].copy_from_slice(&params.to_bytes());
        }

        // 写入存储（假设从偏移0开始）
        let written = self.storage.write(0, &buf)?;
        if written != TOTAL_SIZE {
            return Err(StorageError::InvalidLength.into());
        }

        self.dirty = false;
        Ok(())
    }

    /// 从存储介质加载校正参数，覆盖当前内存中的值。
    ///
    /// 如果存储数据长度不足或解析失败，将保留原有值（不修改），并返回错误。
    pub fn load(&mut self) -> Result<(), ConfigError> {
        const PARAM_SIZE: usize = 8;
        const TOTAL_SIZE: usize = SERVO_COUNT * PARAM_SIZE;

        let mut buf = [0u8; TOTAL_SIZE];
        let read = self.storage.read(0, &mut buf)?;
        if read != TOTAL_SIZE {
            return Err(ConfigError::InvalidDataLength);
        }

        // 解析每个校正参数
        let mut new_corrections = [CorrectionParams::default(); SERVO_COUNT];
        for i in 0..SERVO_COUNT {
            let offset = i * PARAM_SIZE;
            let slice = &buf[offset..offset + PARAM_SIZE];
            new_corrections[i] = CorrectionParams::from_bytes(slice)?;
        }

        self.corrections = new_corrections;
        self.dirty = false;
        Ok(())
    }

    /// 检查是否有未保存的修改。
    pub fn is_dirty(&self) -> bool {
        self.dirty
    }

    /// 获取内部存储的可变引用（如需直接操作存储介质）。
    pub fn storage_mut(&mut self) -> &mut S {
        &mut self.storage
    }
}

// ----------------------------------------------------------------------------
// 测试示例（可选，需要 std 环境）
// ----------------------------------------------------------------------------
#[cfg(test)]
mod tests {
    use super::*;
    use crate::servos::profile::CorrectionParams;

    // 一个简单的内存存储模拟器，用于测试
    struct MemStorage {
        data: [u8; 48],
    }

    impl MemStorage {
        fn new() -> Self {
            Self { data: [0; 48] }
        }
    }

    impl Storage for MemStorage {
        fn read(&mut self, offset: usize, buf: &mut [u8]) -> Result<usize, StorageError> {
            if offset + buf.len() > self.data.len() {
                return Err(StorageError::InvalidLength);
            }
            buf.copy_from_slice(&self.data[offset..offset + buf.len()]);
            Ok(buf.len())
        }

        fn write(&mut self, offset: usize, buf: &[u8]) -> Result<usize, StorageError> {
            if offset + buf.len() > self.data.len() {
                return Err(StorageError::InvalidLength);
            }
            self.data[offset..offset + buf.len()].copy_from_slice(buf);
            Ok(buf.len())
        }
    }

    #[test]
    fn test_config_manager() {
        let storage = MemStorage::new();
        let mut mgr = ConfigManager::new(storage).unwrap();

        // 默认参数应全为默认值
        let id = ServoId::S0;
        assert_eq!(mgr.get_correction(id), CorrectionParams::default());

        // 修改参数
        let mut new_corr = CorrectionParams::default();
        new_corr.reverse = true;
        new_corr.offset_pwm = 10;
        new_corr.deadband_us = 5;
        mgr.set_correction(id, new_corr);

        assert!(mgr.is_dirty());

        // 保存并重新加载
        mgr.save().unwrap();

        // 新建一个管理器（模拟重启后重新加载）
        let storage2 = MemStorage::new();
        // 需要将之前存储的数据复制到新存储，这里简化：直接使用 mgr 的 storage 数据
        // 更真实的测试应重新从相同介质读取，但我们在此省略

        // 直接验证存储的数据
        let mut buf = [0u8; 48];
        mgr.storage.read(0, &mut buf).unwrap();
        // 可以解析验证，略

        // 创建新管理器并加载
        let mut mgr2 = ConfigManager::new(mgr.storage).unwrap();
        assert_eq!(mgr2.get_correction(id), new_corr);
    }
}
