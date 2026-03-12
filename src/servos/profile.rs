//! 舵机参数模型与基础类型定义。
//!
//! 这里区分两类参数：
//! - `PhysicalParams`：描述舵机及机构本身的物理边界，通常随硬件确定后不再变化。
//! - `CorrectionParams`：用于补偿安装方向、中点偏移与死区，适合在运行后调试并持久化。

use core::fmt;

/// 舵机 PWM 周期，单位为微秒。
///
/// 常见模拟舵机使用 20ms 周期，因此默认值为 `20_000`。
pub const SERVO_PERIOD_US: u32 = 20_000;

/// 系统中舵机的总数量。
///
/// 修改该值时，需要同步更新 `ServoId` 与 `ALL_SERVOS`，确保三者保持一致。
pub const SERVO_COUNT: usize = 6;

/// 默认最小 PWM 脉宽。
pub const DEFAULT_MIN_PWM_US: u16 = 500;
/// 默认中点 PWM 脉宽。
pub const DEFAULT_MID_PWM_US: u16 = 1500;
/// 默认最大 PWM 脉宽。
pub const DEFAULT_MAX_PWM_US: u16 = 2500;
/// 默认最小角度。
pub const DEFAULT_MIN_ANGLE_DEG: f32 = 0.0;
/// 默认中点角度。
pub const DEFAULT_MID_ANGLE_DEG: f32 = 135.0;
/// 默认最大角度。
pub const DEFAULT_MAX_ANGLE_DEG: f32 = 270.0;

/// 默认安装方向不反转。
pub const DEFAULT_REVERSE: bool = false;
/// 默认 PWM 偏移为 0。
pub const DEFAULT_OFFSET_PWM: i16 = 0;
/// 默认不额外设置死区。
pub const DEFAULT_DEADBAND_US: u16 = 0;

/// 舵机逻辑编号。
///
/// 该枚举的变体数量必须与 `SERVO_COUNT` 对齐。
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
    /// 转换成数组索引。
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

/// 所有舵机编号的稳定枚举顺序。
///
/// 上层若依赖位置语义，应以该顺序为准。
pub const ALL_SERVOS: [ServoId; SERVO_COUNT] = [
    ServoId::S0,
    ServoId::S1,
    ServoId::S2,
    ServoId::S3,
    ServoId::S4,
    ServoId::S5,
];

/// 脉宽值，单位为微秒。
///
/// 该类型本身不带范围约束，是否有效由具体 `PhysicalParams` 决定。
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct PulseWidthUs(pub u16);

impl PulseWidthUs {
    /// 创建一个新的脉宽值。
    #[inline]
    pub const fn new(us: u16) -> Self {
        Self(us)
    }

    /// 取出底层 `u16`。
    #[inline]
    pub const fn as_u16(self) -> u16 {
        self.0
    }
}

/// 角度值，单位为度。
///
/// 该类型只保证值为有限浮点数，不保证落在任一舵机的物理范围内。
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AngleDeg(f32);

impl AngleDeg {
    /// 创建新的角度值。
    ///
    /// # Errors
    ///
    /// 当输入为 `NaN` 或无穷大时返回 `ConfigError::InvalidAngle`。
    pub fn new(deg: f32) -> Result<Self, ConfigError> {
        if deg.is_finite() {
            Ok(Self(deg))
        } else {
            Err(ConfigError::InvalidAngle)
        }
    }

    /// 取出底层 `f32`。
    #[inline]
    pub const fn as_f32(self) -> f32 {
        self.0
    }
}

/// 舵机的物理参数。
///
/// 这些参数描述 PWM 与角度的物理映射边界，通常在设备装配完成后保持固定。
#[derive(Clone, Copy, Debug)]
pub struct PhysicalParams {
    /// 最小允许 PWM 脉宽。
    pub min_pwm_us: u16,
    /// 物理中点 PWM 脉宽。
    pub mid_pwm_us: u16,
    /// 最大允许 PWM 脉宽。
    pub max_pwm_us: u16,
    /// 最小物理角度。
    pub min_angle_deg: f32,
    /// 物理中点角度。
    pub mid_angle_deg: f32,
    /// 最大物理角度。
    pub max_angle_deg: f32,
}

impl PhysicalParams {
    /// 创建并校验一组物理参数。
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

    /// 校验 PWM 范围和角度范围是否构成单调区间。
    ///
    /// `mid_*` 字段允许不处于中间位置，因为某些机构会故意采用偏置中点。
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

    /// 将角度线性映射到 PWM。
    ///
    /// 输入角度会先被夹紧到 `[min_angle_deg, max_angle_deg]`。
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

    /// 将 PWM 线性映射回角度。
    ///
    /// 输入脉宽会先被夹紧到 `[min_pwm_us, max_pwm_us]`。
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
    /// 使用模块默认常量构造物理参数。
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

/// 舵机校正参数。
///
/// 这些参数用于补偿装配方向、中点偏移与停止抖动，适合在运行时调节并持久化。
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CorrectionParams {
    /// `true` 表示以物理中点为轴做方向反转。
    pub reverse: bool,
    /// 输出 PWM 偏移量，单位为微秒。
    pub offset_pwm: i16,
    /// 死区阈值，单位为微秒。
    pub deadband_us: u16,
    /// 为后续格式扩展预留的字节。
    _reserved: [u8; 3],
}

impl Default for CorrectionParams {
    /// 使用模块默认常量构造校正参数。
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
    /// 创建默认校正参数。
    pub fn new() -> Self {
        Self::default()
    }

    /// 将校正参数序列化为固定 8 字节格式。
    ///
    /// 字段布局如下：
    /// - byte 0: `reverse`
    /// - byte 1..=2: `offset_pwm`，小端
    /// - byte 3..=4: `deadband_us`，小端
    /// - byte 5..=7: 预留
    pub fn to_bytes(&self) -> [u8; 8] {
        let mut buf = [0u8; 8];
        buf[0] = self.reverse as u8;
        buf[1..3].copy_from_slice(&self.offset_pwm.to_le_bytes());
        buf[3..5].copy_from_slice(&self.deadband_us.to_le_bytes());
        buf
    }

    /// 从字节数组反序列化校正参数。
    ///
    /// # Errors
    ///
    /// 当输入不足 8 字节时返回 `ConfigError::InvalidCorrectionData`。
    pub fn from_bytes(bytes: &[u8]) -> Result<Self, ConfigError> {
        if bytes.len() < 8 {
            return Err(ConfigError::InvalidCorrectionData);
        }
        let reverse = bytes[0] != 0;
        let offset_pwm = i16::from_le_bytes([bytes[1], bytes[2]]);
        let deadband_us = u16::from_le_bytes([bytes[3], bytes[4]]);
        Ok(Self {
            reverse,
            offset_pwm,
            deadband_us,
            _reserved: [0; 3],
        })
    }

    /// 将校正参数应用到目标 PWM。
    ///
    /// 反向操作以 `mid_pwm` 为镜像轴，随后再叠加 `offset_pwm`。
    pub(crate) fn apply(&self, mut pwm: i32, mid_pwm: u16) -> u16 {
        if self.reverse {
            // 方向反转必须以物理中点为轴，否则会改变可用行程的对称性。
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

/// 单个舵机的完整配置视图。
///
/// 该类型把固定物理参数与可调校正参数组合起来，方便上层统一做目标换算。
#[derive(Clone, Copy, Debug)]
pub struct ServoProfile {
    /// 舵机及机构本身的物理边界。
    pub physical: PhysicalParams,
    /// 安装补偿与停止策略。
    pub correction: CorrectionParams,
}

impl ServoProfile {
    /// 使用物理参数和校正参数构造配置。
    pub fn new(physical: PhysicalParams, correction: CorrectionParams) -> Self {
        Self {
            physical,
            correction,
        }
    }

    /// 使用默认物理参数和默认校正参数构造配置。
    pub fn default() -> Self {
        Self {
            physical: PhysicalParams::default(),
            correction: CorrectionParams::default(),
        }
    }

    /// 先按物理范围夹紧，再应用校正参数。
    pub fn clamp_and_correct_pwm(&self, pwm: PulseWidthUs) -> PulseWidthUs {
        let clamped = pwm
            .0
            .clamp(self.physical.min_pwm_us, self.physical.max_pwm_us);
        let corrected = self
            .correction
            .apply(clamped as i32, self.physical.mid_pwm_us);
        // 校正后的结果仍需回到物理边界内，避免偏移把输出推到非法区间。
        let final_pwm = corrected.clamp(self.physical.min_pwm_us, self.physical.max_pwm_us);
        PulseWidthUs(final_pwm)
    }

    /// 将角度转换为最终输出 PWM，并应用校正参数。
    pub fn angle_to_pwm(&self, angle: AngleDeg) -> PulseWidthUs {
        let pwm = self.physical.angle_to_pwm(angle);
        self.clamp_and_correct_pwm(pwm)
    }

    /// 将 PWM 转回角度。
    ///
    /// 这里故意不逆向应用校正参数，因为校正只影响输出命令，不描述真实反馈模型。
    pub fn pwm_to_angle(&self, pwm: PulseWidthUs) -> AngleDeg {
        self.physical.pwm_to_angle(pwm)
    }

    /// 获取经校正后的目标 PWM。
    pub fn corrected_target(&self, target: PulseWidthUs) -> PulseWidthUs {
        self.clamp_and_correct_pwm(target)
    }
}

/// 参数与反序列化相关的错误。
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ConfigError {
    /// PWM 范围不是严格递增区间。
    InvalidPwmRange,
    /// 角度范围不是严格递增区间。
    InvalidAngleRange,
    /// 角度值为 `NaN` 或无穷大。
    InvalidAngle,
    /// 校正参数字节内容不完整或不合法。
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
