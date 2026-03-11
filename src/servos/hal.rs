//! 硬件抽象层（HAL）。

#[cfg(target_arch = "riscv32")]
use core::marker::PhantomData;
#[cfg(target_arch = "riscv32")]
use embedded_hal::pwm::SetDutyCycle;
#[cfg(target_arch = "riscv32")]
use esp_hal::ledc::channel::ChannelIFace;
#[cfg(target_arch = "riscv32")]
use esp_hal::ledc::LowSpeed;

use super::profile::{DEFAULT_MID_PWM_US, PulseWidthUs, ServoId};
#[cfg(target_arch = "riscv32")]
use super::profile::SERVO_PERIOD_US;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum HalError {
    Driver,
    InvalidChannel,
}

pub trait PwmOutput {
    fn set_pulse_width_us(&mut self, us: u16) -> Result<(), HalError>;
}

#[cfg(target_arch = "riscv32")]
pub struct LedcPwmChannel<'a, C>
where
    C: ChannelIFace<'a, LowSpeed> + SetDutyCycle,
{
    ch: C,
    _marker: PhantomData<&'a ()>,
}

#[cfg(target_arch = "riscv32")]
impl<'a, C> LedcPwmChannel<'a, C>
where
    C: ChannelIFace<'a, LowSpeed> + SetDutyCycle,
{
    pub fn new(ch: C) -> Self {
        Self {
            ch,
            _marker: PhantomData,
        }
    }

    fn pulse_to_duty(max_duty: u16, pulse_us: u16) -> u16 {
        let duty = (u32::from(pulse_us) * u32::from(max_duty)) / SERVO_PERIOD_US;
        duty.min(u32::from(max_duty)) as u16
    }
}

#[cfg(target_arch = "riscv32")]
impl<'a, C> PwmOutput for LedcPwmChannel<'a, C>
where
    C: ChannelIFace<'a, LowSpeed> + SetDutyCycle,
{
    fn set_pulse_width_us(&mut self, us: u16) -> Result<(), HalError> {
        let max_duty = self.ch.max_duty_cycle();
        let duty = Self::pulse_to_duty(max_duty, us);
        self.ch.set_duty_cycle(duty).map_err(|_| HalError::Driver)
    }
}

pub struct ServoDrivers<'a, const N: usize> {
    drivers: [&'a mut dyn PwmOutput; N],
    cache: [PulseWidthUs; N],
}

impl<'a, const N: usize> ServoDrivers<'a, N> {
    pub fn new(drivers: [&'a mut dyn PwmOutput; N]) -> Self {
        Self {
            drivers,
            cache: [PulseWidthUs(DEFAULT_MID_PWM_US); N],
        }
    }

    pub fn init_all(&mut self) -> Result<(), HalError> {
        for i in 0..N {
            self.drivers[i].set_pulse_width_us(self.cache[i].0)?;
        }
        Ok(())
    }

    pub fn set_pulse_us(&mut self, id: ServoId, us: u16) -> Result<(), HalError> {
        self.set_pulse(id, PulseWidthUs(us))
    }

    pub fn set_pulse_width(&mut self, id: ServoId, pulse: PulseWidthUs) -> Result<(), HalError> {
        self.set_pulse(id, pulse)
    }

    pub fn cached_pulse(&self, id: ServoId) -> Result<PulseWidthUs, HalError> {
        let idx = id.index();
        if idx >= N {
            return Err(HalError::InvalidChannel);
        }
        Ok(self.cache[idx])
    }

    fn set_pulse(&mut self, id: ServoId, pulse: PulseWidthUs) -> Result<(), HalError> {
        let idx = id.index();
        if idx >= N {
            return Err(HalError::InvalidChannel);
        }
        self.drivers[idx].set_pulse_width_us(pulse.0)?;
        self.cache[idx] = pulse;
        Ok(())
    }
}
