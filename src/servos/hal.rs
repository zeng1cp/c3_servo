//! 硬件抽象层（HAL）
//!
//! 本模块定义了 PWM 输出 trait 以及基于 ESP32-C3 LEDC 的具体实现。
//! 提供 `ServoDrivers` 结构，使用 const 泛型支持任意数量舵机，通过动态分发数组管理硬件通道。

use core::marker::PhantomData;
use embedded_hal::pwm::SetDutyCycle;
use esp_hal::ledc::channel::ChannelIFace;
use esp_hal::ledc::LowSpeed;

use super::profile::{DEFAULT_MID_PWM_US, PulseWidthUs, ServoId, SERVO_PERIOD_US};

/// 硬件抽象层错误类型
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum HalError {
    /// 驱动错误（例如 PWM 设置失败）
    Driver,
    /// 无效的通道或舵机 ID
    InvalidChannel,
}

/// PWM 输出抽象 trait
///
/// 任何实现了此 trait 的类型都可以作为舵机的硬件输出通道。
pub trait PwmOutput {
    /// 设置脉冲宽度（微秒）
    fn set_pulse_width_us(&mut self, us: u16) -> Result<(), HalError>;
}

/// ESP32-C3 LEDC PWM 通道适配器
///
/// 包装 esp-hal 的 LEDC 通道，实现 `PwmOutput` trait。
pub struct LedcPwmChannel<'a, C>
where
    C: ChannelIFace<'a, LowSpeed> + SetDutyCycle,
{
    ch: C,
    _marker: PhantomData<&'a ()>,
}

impl<'a, C> LedcPwmChannel<'a, C>
where
    C: ChannelIFace<'a, LowSpeed> + SetDutyCycle,
{
    /// 创建新的 LEDC 通道适配器
    pub fn new(ch: C) -> Self {
        Self {
            ch,
            _marker: PhantomData,
        }
    }

    /// 将脉冲宽度（微秒）转换为占空比计数值
    #[inline]
    fn pulse_to_duty(max_duty: u16, pulse_us: u16) -> u16 {
        let duty = (u32::from(pulse_us) * u32::from(max_duty)) / SERVO_PERIOD_US;
        duty.min(u32::from(max_duty)) as u16
    }
}

impl<'a, C> PwmOutput for LedcPwmChannel<'a, C>
where
    C: ChannelIFace<'a, LowSpeed> + SetDutyCycle,
{
    #[inline]
    fn set_pulse_width_us(&mut self, us: u16) -> Result<(), HalError> {
        let max_duty = self.ch.max_duty_cycle();
        let duty = Self::pulse_to_duty(max_duty, us);
        self.ch.set_duty_cycle(duty).map_err(|_| HalError::Driver)
    }
}

/// 舵机驱动器聚合体（支持任意数量舵机）
///
/// 通过 const 泛型 `N` 指定舵机数量，内部存储一个动态分发数组 `[&mut dyn PwmOutput; N]`。
/// 这种设计允许轻松修改舵机数量，只需调整 `SERVO_COUNT` 和 `ServoId` 枚举即可。
pub struct ServoDrivers<'a, const N: usize> {
    drivers: [&'a mut dyn PwmOutput; N],
    cache: [PulseWidthUs; N],
}

impl<'a, const N: usize> ServoDrivers<'a, N> {
    /// 创建新的舵机驱动器实例
    ///
    /// # 参数
    /// * `drivers` - 一个长度等于舵机数量的数组，每个元素是对应 PWM 通道的可变引用。
    pub fn new(drivers: [&'a mut dyn PwmOutput; N]) -> Self {
        // 缓存默认初始化为默认中点脉冲值，但此时不会写入硬件
        let cache = [PulseWidthUs(DEFAULT_MID_PWM_US); N];
        Self { drivers, cache }
    }

    /// 初始化所有舵机：将缓存中的脉冲值写入硬件
    pub fn init_all(&mut self) -> Result<(), HalError> {
        for i in 0..N {
            self.drivers[i].set_pulse_width_us(self.cache[i].0)?;
        }
        Ok(())
    }

    /// 设置指定舵机的脉冲宽度（微秒），并更新缓存
    ///
    /// # 注意
    /// 调用者需确保 `id.index()` 在 `0..N` 范围内，通常由 `ServoId` 的定义保证。
    pub fn set_pulse_us(&mut self, id: ServoId, us: u16) -> Result<(), HalError> {
        let pulse = PulseWidthUs(us);
        self.set_pulse(id, pulse)
    }

    /// 设置指定舵机的脉冲宽度（`PulseWidthUs`），并更新缓存
    pub fn set_pulse_width(&mut self, id: ServoId, pulse: PulseWidthUs) -> Result<(), HalError> {
        self.set_pulse(id, pulse)
    }

    /// 获取指定舵机上次设置的脉冲宽度（缓存值）
    pub fn cached_pulse(&self, id: ServoId) -> PulseWidthUs {
        self.cache[id.index()]
    }

    /// 内部方法：实际写入硬件并更新缓存
    fn set_pulse(&mut self, id: ServoId, pulse: PulseWidthUs) -> Result<(), HalError> {
        let idx = id.index();
        self.drivers[idx].set_pulse_width_us(pulse.0)?;
        self.cache[idx] = pulse;
        Ok(())
    }
}

// ----------------------------------------------------------------------------
// 测试（需要 std 环境）
// ----------------------------------------------------------------------------
#[cfg(test)]
mod tests {
    use super::*;
    use crate::servos::profile::DEFAULT_MID_PWM_US;

    struct MockPwm {
        last_pulse: u16,
    }

    impl PwmOutput for MockPwm {
        fn set_pulse_width_us(&mut self, us: u16) -> Result<(), HalError> {
            self.last_pulse = us;
            Ok(())
        }
    }

    #[test]
    fn test_servo_drivers() {
        let mut p0 = MockPwm { last_pulse: 0 };
        let mut p1 = MockPwm { last_pulse: 0 };
        let mut p2 = MockPwm { last_pulse: 0 };
        let mut p3 = MockPwm { last_pulse: 0 };
        let mut p4 = MockPwm { last_pulse: 0 };
        let mut p5 = MockPwm { last_pulse: 0 };

        let drivers = [
            &mut p0 as &mut dyn PwmOutput,
            &mut p1,
            &mut p2,
            &mut p3,
            &mut p4,
            &mut p5,
        ];

        let mut drivers = ServoDrivers::<6>::new(drivers);
        drivers.init_all().unwrap();

        assert_eq!(drivers.cached_pulse(ServoId::S0).0, DEFAULT_MID_PWM_US);

        drivers.set_pulse_us(ServoId::S0, 2000).unwrap();
        assert_eq!(drivers.cached_pulse(ServoId::S0).0, 2000);
        assert_eq!(p0.last_pulse, 2000);
    }
}
