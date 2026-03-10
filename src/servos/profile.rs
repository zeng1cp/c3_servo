//! 舵机配置与基础类型定义
//!
//! # 用户可配置参数（请根据您的机械臂修改此处）
//! 以下常量定义了舵机的基本参数，您可以根据实际硬件进行调整。
//! 修改后请确保与您的机械臂物理特性匹配，否则可能导致运动异常或损坏。

// =============================================================================
// 用户可配置区域 - 请根据实际硬件修改
// =============================================================================

/// 舵机 PWM 周期（微秒），通常为 20000（20ms）
pub const SERVO_PERIOD_US: u32 = 20_000;
/// 舵机数量
///
/// **注意**：修改此值后，必须同步调整下方的 `ServoId` 枚举（增加或删除变体）
/// 以及 `ALL_SERVOS` 数组，确保三者数量一致。
pub const SERVO_COUNT: usize = 6;

/// 默认物理参数（脉冲宽度范围及角度范围）
/// 这些值用于 `PhysicalParams::default()`，您可以根据常用舵机型号修改。
pub const DEFAULT_MIN_PWM_US: u16 = 500;
pub const DEFAULT_MID_PWM_US: u16 = 1500;
pub const DEFAULT_MAX_PWM_US: u16 = 2500;
pub const DEFAULT_MIN_ANGLE_DEG: f32 = 0.0;
pub const DEFAULT_MID_ANGLE_DEG: f32 = 135.0;
pub const DEFAULT_MAX_ANGLE_DEG: f32 = 270.0;

/// 默认校正参数（反向、偏移、死区）
/// 这些值用于 `CorrectionParams::default()`，您可以根据需要调整初始值。
pub const DEFAULT_REVERSE: bool = false;
pub const DEFAULT_OFFSET_PWM: i16 = 0;
pub const DEFAULT_DEADBAND_US: u16 = 0;

// =============================================================================
// 以下为核心代码，一般无需修改
// =============================================================================

use core::fmt;

/// 舵机ID枚举
///
/// **注意**：此枚举的变体数量必须与上面的 `SERVO_COUNT` 一致。
/// 如果需要增加或减少舵机，请在此处添加或删除对应的变体（例如 S6, S7...），
/// 并同步修改下方的 `ALL_SERVOS` 数组。
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ServoId {
    S0,
    S1,
    S2,
    S3,
    S4,
    S5,
}

impl ServoId {
    /// 转换为数组索引（0..SERVO_COUNT-1）
    #[inline]
    pub const fn index(self) -> usize {
        match self {
            Self::S0 => 0,
            Self::S1 => 1,
            Self::S2 => 2,
            Self::S3 => 3,
            Self::S4 => 4,
            Self::S5 => 5,
        }
    }
}

/// 所有舵机ID的常量数组
///
/// **注意**：此数组的长度必须等于 `SERVO_COUNT`，且元素与上面的 `ServoId` 枚举变体一一对应。
/// 修改枚举后请同步更新此数组。
pub const ALL_SERVOS: [ServoId; SERVO_COUNT] = [
    ServoId::S0,
    ServoId::S1,
    ServoId::S2,
    ServoId::S3,
    ServoId::S4,
    ServoId::S5,
];

/// 脉冲宽度（微秒），无范围限制，具体有效性由配置保证
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct PulseWidthUs(pub u16);

impl PulseWidthUs {
    /// 创建一个新的脉冲宽度值
    #[inline]
    pub const fn new(us: u16) -> Self {
        Self(us)
    }

    /// 返回内部的微秒值
    #[inline]
    pub const fn as_u16(self) -> u16 {
        self.0
    }
}

/// 角度值（度），仅保证浮点有限性
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AngleDeg(f32);

impl AngleDeg {
    /// 创建一个新的角度值，如果输入为 `NaN` 或无限则返回 `ConfigError::InvalidAngle`
    pub fn new(deg: f32) -> Result<Self, ConfigError> {
        if deg.is_finite() {
            Ok(Self(deg))
        } else {
            Err(ConfigError::InvalidAngle)
        }
    }

    /// 返回内部的浮点值
    #[inline]
    pub const fn as_f32(self) -> f32 {
        self.0
    }
}

// ----------------------------------------------------------------------------
// 物理参数（运行中不可修改）
// ----------------------------------------------------------------------------

/// 舵机的物理参数，反映其固有特性，通常在舵机安装后固定不变。
#[derive(Clone, Copy, Debug)]
pub struct PhysicalParams {
    /// 最小脉冲宽度（微秒）
    pub min_pwm_us: u16,
    /// 中点脉冲宽度（微秒）
    pub mid_pwm_us: u16,
    /// 最大脉冲宽度（微秒）
    pub max_pwm_us: u16,
    /// 最小角度（度）
    pub min_angle_deg: f32,
    /// 中点角度（度）
    pub mid_angle_deg: f32,
    /// 最大角度（度）
    pub max_angle_deg: f32,
}

impl PhysicalParams {
    /// 创建一个新的物理参数实例，并进行基本验证。
    pub fn new(
        min_pwm_us: u16,
        mid_pwm_us: u16,
        max_pwm_us: u16,
        min_angle_deg: f32,
        mid_angle_deg: f32,
        max_angle_deg: f32,
    ) -> Result<Self, ConfigError> {
        let params = Self {
            min_pwm_us,
            mid_pwm_us,
            max_pwm_us,
            min_angle_deg,
            mid_angle_deg,
            max_angle_deg,
        };
        params.validate()?;
        Ok(params)
    }

    /// 验证参数的合理性：
    /// - 脉冲宽度：`min_pwm_us < max_pwm_us`
    /// - 角度范围：`min_angle_deg < max_angle_deg` 且均为有限值
    /// - 允许中点值任意，不强制在 min/max 之间
    fn validate(&self) -> Result<(), ConfigError> {
        if self.min_pwm_us >= self.max_pwm_us {
            return Err(ConfigError::InvalidPwmRange);
        }
        if self.min_angle_deg >= self.max_angle_deg {
            return Err(ConfigError::InvalidAngleRange);
        }
        if !(self.min_angle_deg.is_finite()
            && self.mid_angle_deg.is_finite()
            && self.max_angle_deg.is_finite())
        {
            return Err(ConfigError::InvalidAngle);
        }
        Ok(())
    }

    /// 将角度转换为脉冲宽度（线性插值）。
    ///
    /// 输入角度会被限制到 `[min_angle_deg, max_angle_deg]`。
    pub fn angle_to_pwm(&self, angle: AngleDeg) -> PulseWidthUs {
        let angle = angle.0.clamp(self.min_angle_deg, self.max_angle_deg);
        let range_angle = self.max_angle_deg - self.min_angle_deg;
        if range_angle <= f32::EPSILON {
            return PulseWidthUs(self.mid_pwm_us);
        }
        let ratio = (angle - self.min_angle_deg) / range_angle;
        let pwm = self.min_pwm_us as f32 + (self.max_pwm_us - self.min_pwm_us) as f32 * ratio;
        PulseWidthUs((pwm + 0.5) as u16)
    }

    /// 将脉冲宽度转换为角度（线性插值）。
    ///
    /// 输入脉冲会被限制到 `[min_pwm_us, max_pwm_us]`。
    pub fn pwm_to_angle(&self, pwm: PulseWidthUs) -> AngleDeg {
        let pwm = pwm.0.clamp(self.min_pwm_us, self.max_pwm_us) as f32;
        let range_pwm = (self.max_pwm_us - self.min_pwm_us) as f32;
        if range_pwm <= f32::EPSILON {
            return AngleDeg(self.mid_angle_deg);
        }
        let ratio = (pwm - self.min_pwm_us as f32) / range_pwm;
        let angle = self.min_angle_deg + (self.max_angle_deg - self.min_angle_deg) * ratio;
        AngleDeg(angle)
    }
}

impl Default for PhysicalParams {
    /// 返回默认物理参数（使用顶部定义的常量）
    fn default() -> Self {
        Self {
            min_pwm_us: DEFAULT_MIN_PWM_US,
            mid_pwm_us: DEFAULT_MID_PWM_US,
            max_pwm_us: DEFAULT_MAX_PWM_US,
            min_angle_deg: DEFAULT_MIN_ANGLE_DEG,
            mid_angle_deg: DEFAULT_MID_ANGLE_DEG,
            max_angle_deg: DEFAULT_MAX_ANGLE_DEG,
        }
    }
}

// ----------------------------------------------------------------------------
// 校正参数（运行时可修改，需持久化）
// ----------------------------------------------------------------------------

/// 舵机的校正参数，可在运行时调整以补偿安装偏差或机械特性。
///
/// 这些参数应支持持久化存储（例如保存到 EEPROM 或 Flash），
/// 通过 `to_bytes` 和 `from_bytes` 方法进行序列化和反序列化。
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CorrectionParams {
    /// 是否反向安装（true 表示舵机旋转方向与默认相反）
    pub reverse: bool,
    /// 脉冲偏移量（微秒），用于微调中点，范围通常为 -100..100
    pub offset_pwm: i16,
    /// 死区（微秒），当目标与当前位置差小于此值时认为到达，避免微小抖动
    pub deadband_us: u16,
    /// 保留字段，便于未来扩展
    _reserved: [u8; 3],
}

impl Default for CorrectionParams {
    /// 返回默认校正参数（使用顶部定义的常量）
    fn default() -> Self {
        Self {
            reverse: DEFAULT_REVERSE,
            offset_pwm: DEFAULT_OFFSET_PWM,
            deadband_us: DEFAULT_DEADBAND_US,
            _reserved: [0; 3],
        }
    }
}

impl CorrectionParams {
    /// 创建一个新的校正参数实例（使用默认值）
    pub fn new() -> Self {
        Self::default()
    }

    /// 将校正参数序列化为固定长度的字节数组（8字节）
    ///
    /// 格式：
    /// - 字节0: reverse (0/1)
    /// - 字节1-2: offset_pwm (i16, 小端)
    /// - 字节3-4: deadband_us (u16, 小端)
    /// - 字节5-7: 保留 (全0)
    pub fn to_bytes(&self) -> [u8; 8] {
        let mut buf = [0u8; 8];
        buf[0] = self.reverse as u8;
        buf[1..3].copy_from_slice(&self.offset_pwm.to_le_bytes());
        buf[3..5].copy_from_slice(&self.deadband_us.to_le_bytes());
        // 保留字段已初始化为0
        buf
    }

    /// 从字节数组反序列化校正参数
    ///
    /// 如果输入切片长度不足8字节，则返回 `ConfigError::InvalidCorrectionData`。
    pub fn from_bytes(bytes: &[u8]) -> Result<Self, ConfigError> {
        if bytes.len() < 8 {
            return Err(ConfigError::InvalidCorrectionData);
        }
        let reverse = bytes[0] != 0;
        let offset_pwm = i16::from_le_bytes([bytes[1], bytes[2]]);
        let deadband_us = u16::from_le_bytes([bytes[3], bytes[4]]);
        // 忽略保留字节
        Ok(Self {
            reverse,
            offset_pwm,
            deadband_us,
            _reserved: [0; 3],
        })
    }

    /// 应用校正到目标脉冲宽度（考虑反向和偏移）
    pub(crate) fn apply(&self, mut pwm: i32, mid_pwm: u16) -> u16 {
        if self.reverse {
            // 反向：以中点为轴翻转
            let mid = mid_pwm as i32;
            pwm = mid + (mid - pwm);
        }
        pwm = pwm.wrapping_add(self.offset_pwm as i32);
        if pwm < 0 {
            pwm = 0;
        }
        if pwm > u16::MAX as i32 {
            pwm = u16::MAX as i32;
        }
        pwm as u16
    }
}

// ----------------------------------------------------------------------------
// 完整舵机配置（物理参数 + 校正参数）
// ----------------------------------------------------------------------------

/// 单个舵机的完整配置，包含不可变的物理参数和可变的校正参数。
#[derive(Clone, Copy, Debug)]
pub struct ServoProfile {
    /// 物理参数（不可变）
    pub physical: PhysicalParams,
    /// 校正参数（可变）
    pub correction: CorrectionParams,
}

impl ServoProfile {
    /// 使用物理参数和校正参数创建新的配置。
    pub fn new(physical: PhysicalParams, correction: CorrectionParams) -> Self {
        Self {
            physical,
            correction,
        }
    }

    /// 创建默认配置（物理参数默认，校正参数默认）
    pub fn default() -> Self {
        Self {
            physical: PhysicalParams::default(),
            correction: CorrectionParams::default(),
        }
    }

    /// 将脉冲宽度限制在物理范围内，并应用校正参数（反向、偏移）。
    pub fn clamp_and_correct_pwm(&self, pwm: PulseWidthUs) -> PulseWidthUs {
        let clamped = pwm.0.clamp(self.physical.min_pwm_us, self.physical.max_pwm_us);
        let corrected = self.correction.apply(clamped as i32, self.physical.mid_pwm_us);
        // 确保不超过物理范围
        let final_pwm = corrected.clamp(self.physical.min_pwm_us, self.physical.max_pwm_us);
        PulseWidthUs(final_pwm)
    }

    /// 将角度转换为脉冲宽度（线性插值），并应用校正。
    pub fn angle_to_pwm(&self, angle: AngleDeg) -> PulseWidthUs {
        let angle = angle.0.clamp(self.physical.min_angle_deg, self.physical.max_angle_deg);
        let range_angle = self.physical.max_angle_deg - self.physical.min_angle_deg;
        if range_angle <= f32::EPSILON {
            return PulseWidthUs(self.physical.mid_pwm_us);
        }
        let ratio = (angle - self.physical.min_angle_deg) / range_angle;
        let pwm = self.physical.min_pwm_us as f32
            + (self.physical.max_pwm_us - self.physical.min_pwm_us) as f32 * ratio;
        // 手动四舍五入（适用于正数），避免依赖 round 方法
        let pwm_us = (pwm + 0.5) as u16;
        self.clamp_and_correct_pwm(PulseWidthUs(pwm_us))
    }

    /// 将脉冲宽度转换为角度（线性插值），不考虑校正（校正只影响输出，不影响角度反馈）。
    pub fn pwm_to_angle(&self, pwm: PulseWidthUs) -> AngleDeg {
        // 注意：这里使用物理参数直接转换，不考虑校正，因为校正仅用于输出到舵机。
        let pwm = pwm.0.clamp(self.physical.min_pwm_us, self.physical.max_pwm_us) as f32;
        let range_pwm = (self.physical.max_pwm_us - self.physical.min_pwm_us) as f32;
        if range_pwm <= f32::EPSILON {
            return AngleDeg(self.physical.mid_angle_deg);
        }
        let ratio = (pwm - self.physical.min_pwm_us as f32) / range_pwm;
        let angle = self.physical.min_angle_deg
            + (self.physical.max_angle_deg - self.physical.min_angle_deg) * ratio;
        AngleDeg(angle)
    }

    /// 获取当前校正后的目标脉冲宽度（用于运动引擎），
    /// 即考虑反向和偏移后的值，但保持为 `PulseWidthUs`。
    pub fn corrected_target(&self, target: PulseWidthUs) -> PulseWidthUs {
        self.clamp_and_correct_pwm(target)
    }
}

// ----------------------------------------------------------------------------
// 错误定义
// ----------------------------------------------------------------------------

/// 配置相关的错误类型
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ConfigError {
    /// 无效的脉冲宽度范围（min >= max）
    InvalidPwmRange,
    /// 无效的角度范围（min >= max）
    InvalidAngleRange,
    /// 角度值不是有限数（NaN 或无穷）
    InvalidAngle,
    /// 校正参数数据无效（例如反序列化时长度不足）
    InvalidCorrectionData,
}

impl fmt::Display for ConfigError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidPwmRange => write!(f, "Invalid PWM range: min >= max"),
            Self::InvalidAngleRange => write!(f, "Invalid angle range: min >= max"),
            Self::InvalidAngle => write!(f, "Angle value is NaN or infinite"),
            Self::InvalidCorrectionData => write!(f, "Invalid correction data"),
        }
    }
}
