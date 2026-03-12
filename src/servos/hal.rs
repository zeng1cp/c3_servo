//! 舵机 PWM 输出的硬件抽象层。
//!
//! 该模块把具体 LEDC 通道与上层运动控制逻辑隔离开：
//! 上层只关心脉宽，硬件层负责把微秒脉宽换算成底层 PWM 占空比。

use core::marker::PhantomData;
use embedded_hal::pwm::SetDutyCycle;
use esp_hal::ledc::LowSpeed;
use esp_hal::ledc::channel::ChannelIFace;

use super::profile::{DEFAULT_MID_PWM_US, PulseWidthUs, SERVO_PERIOD_US, ServoId};

/// 硬件抽象层可能返回的错误。
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum HalError {
    /// 底层 PWM 驱动调用失败。
    Driver,
    /// `ServoId` 对应的驱动槽位不存在。
    InvalidChannel,
}

/// 以脉宽为中心的 PWM 输出抽象。
///
/// 该 trait 将具体定时器/通道实现隐藏在后面，使运动层不依赖特定 HAL。
pub trait PwmOutput {
    fn set_pulse_width_us(&mut self, us: u16) -> Result<(), HalError>;
}

/// 基于 `esp-hal` LEDC 通道的 `PwmOutput` 适配器。
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
    /// 包装一个已初始化的 LEDC 通道。
    pub fn new(ch: C) -> Self {
        Self {
            ch,
            _marker: PhantomData,
        }
    }

    #[inline]
    fn pulse_to_duty(max_duty: u16, pulse_us: u16) -> u16 {
        // 舵机以固定周期工作，因此这里只需在线性区间内换算高电平脉宽。
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

/// 一组按 `ServoId::index()` 映射的舵机驱动。
///
/// `cache` 记录最近一次成功下发到硬件的脉宽，便于上层在不访问硬件寄存器的情况下读取状态。
pub struct ServoDrivers<'a, const N: usize> {
    drivers: [&'a mut dyn PwmOutput; N],
    cache: [PulseWidthUs; N],
}

impl<'a, const N: usize> ServoDrivers<'a, N> {
    /// 创建一组按 `ServoId::index()` 对齐的舵机驱动。
    pub fn new(drivers: [&'a mut dyn PwmOutput; N]) -> Self {
        Self {
            drivers,
            cache: [PulseWidthUs(DEFAULT_MID_PWM_US); N],
        }
    }

    /// 以缓存中的初始脉宽同步所有输出通道。
    pub fn init_all(&mut self) -> Result<(), HalError> {
        for index in 0..N {
            self.drivers[index].set_pulse_width_us(self.cache[index].0)?;
        }
        Ok(())
    }

    /// 按微秒单位更新指定舵机的目标脉宽。
    pub fn set_pulse_us(&mut self, id: ServoId, us: u16) -> Result<(), HalError> {
        self.set_pulse(id, PulseWidthUs(us))
    }

    /// 使用 `PulseWidthUs` 更新指定舵机的目标脉宽。
    pub fn set_pulse_width(&mut self, id: ServoId, pulse: PulseWidthUs) -> Result<(), HalError> {
        self.set_pulse(id, pulse)
    }

    /// 读取最近一次成功写入硬件的缓存值。
    pub fn cached_pulse(&self, id: ServoId) -> Result<PulseWidthUs, HalError> {
        self.cache
            .get(id.index())
            .copied()
            .ok_or(HalError::InvalidChannel)
    }

    fn set_pulse(&mut self, id: ServoId, pulse: PulseWidthUs) -> Result<(), HalError> {
        let index = id.index();
        self.drivers
            .get_mut(index)
            .ok_or(HalError::InvalidChannel)?
            .set_pulse_width_us(pulse.0)?;
        // 只有在底层驱动成功接受更新后才刷新缓存，避免缓存与硬件状态失配。
        *self.cache.get_mut(index).ok_or(HalError::InvalidChannel)? = pulse;
        Ok(())
    }
}
