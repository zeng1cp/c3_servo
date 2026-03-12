#![no_std]
#![no_main]

use c3_servo::servos::config::{ConfigManager, Storage, StorageError};
use c3_servo::servos::controller::{Callback, ControlUnit, Target};
use c3_servo::servos::engine::{MotionEngine, quadratic_in_easing};
use c3_servo::servos::hal::{LedcPwmChannel, PwmOutput, ServoDrivers};
use c3_servo::servos::profile::{ALL_SERVOS, AngleDeg, PulseWidthUs, SERVO_COUNT, ServoId};
use defmt::println;
use esp_hal::clock::CpuClock;
use esp_hal::delay::Delay;
use esp_hal::gpio::DriveMode;
use esp_hal::ledc::{
    LSGlobalClkSource, Ledc, LowSpeed,
    channel::{self, ChannelIFace},
    timer::{self, TimerIFace},
};
use esp_hal::main;
use esp_hal::time::Rate;
use esp_println as _;

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {}
}

esp_bootloader_esp_idf::esp_app_desc!();

const CONFIG_BYTES: usize = 64;
const DEMO_DURATION_MS: u32 = 2_200;
const DEMO_PAUSE_MS: u32 = 400;

const DEMO_IDS: [ServoId; SERVO_COUNT] = ALL_SERVOS;

const POSE_A: [f32; SERVO_COUNT] = [0.0, 135.0, 135.0, 135.0, 135.0, 135.0];
const POSE_B: [f32; SERVO_COUNT] = [270.0, 135.0, 135.0, 135.0, 135.0, 135.0];

fn on_motion_done() {
    println!("controller motion completed");
}

struct RamStorage {
    buf: [u8; CONFIG_BYTES],
    len: usize,
}

impl RamStorage {
    const fn new() -> Self {
        Self {
            buf: [0xFF; CONFIG_BYTES],
            len: 0,
        }
    }
}

impl Storage for RamStorage {
    fn read(&mut self, offset: usize, out: &mut [u8]) -> Result<usize, StorageError> {
        if offset > self.buf.len() {
            return Err(StorageError::ReadFailed);
        }

        let available = self.len.saturating_sub(offset);
        let read_len = available.min(out.len());
        out[..read_len].copy_from_slice(&self.buf[offset..offset + read_len]);
        Ok(read_len)
    }

    fn write(&mut self, offset: usize, input: &[u8]) -> Result<usize, StorageError> {
        let end = offset.saturating_add(input.len());
        if end > self.buf.len() {
            return Err(StorageError::WriteFailed);
        }

        self.buf[offset..end].copy_from_slice(input);
        if end > self.len {
            self.len = end;
        }
        Ok(input.len())
    }

    fn erase(&mut self, offset: usize, len: usize) -> Result<(), StorageError> {
        let end = offset.saturating_add(len);
        if end > self.buf.len() {
            return Err(StorageError::EraseFailed);
        }

        for byte in &mut self.buf[offset..end] {
            *byte = 0xFF;
        }
        if offset == 0 && end >= self.len {
            self.len = 0;
        }
        Ok(())
    }
}

fn build_targets(angles: &[f32; SERVO_COUNT]) -> [Target; SERVO_COUNT] {
    let mut targets = [Target::Pwm(PulseWidthUs::new(1500)); SERVO_COUNT];
    let mut index = 0usize;

    while index < SERVO_COUNT {
        let angle = AngleDeg::new(angles[index]).unwrap();
        targets[index] = Target::Angle(angle);
        index += 1;
    }

    targets
}

#[main]
fn main() -> ! {
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    let delay = Delay::new();

    let mut ledc = Ledc::new(peripherals.LEDC);
    ledc.set_global_slow_clock(LSGlobalClkSource::APBClk);

    let mut timer0 = ledc.timer::<LowSpeed>(timer::Number::Timer0);
    timer0
        .configure(timer::config::Config {
            duty: timer::config::Duty::Duty13Bit,
            clock_source: timer::LSClockSource::APBClk,
            frequency: Rate::from_hz(50),
        })
        .unwrap();

    let mut ch0 = ledc.channel(channel::Number::Channel0, peripherals.GPIO1);
    let mut ch1 = ledc.channel(channel::Number::Channel1, peripherals.GPIO2);
    let mut ch2 = ledc.channel(channel::Number::Channel2, peripherals.GPIO3);
    let mut ch3 = ledc.channel(channel::Number::Channel3, peripherals.GPIO4);
    let mut ch4 = ledc.channel(channel::Number::Channel4, peripherals.GPIO5);
    let mut ch5 = ledc.channel(channel::Number::Channel5, peripherals.GPIO6);

    for ch in [&mut ch0, &mut ch1, &mut ch2, &mut ch3, &mut ch4, &mut ch5] {
        ch.configure(channel::config::Config {
            timer: &timer0,
            duty_pct: 7,
            drive_mode: DriveMode::PushPull,
        })
        .unwrap();
    }

    let mut pwm0 = LedcPwmChannel::new(ch0);
    let mut pwm1 = LedcPwmChannel::new(ch1);
    let mut pwm2 = LedcPwmChannel::new(ch2);
    let mut pwm3 = LedcPwmChannel::new(ch3);
    let mut pwm4 = LedcPwmChannel::new(ch4);
    let mut pwm5 = LedcPwmChannel::new(ch5);

    let drivers: [&mut dyn PwmOutput; SERVO_COUNT] = [
        &mut pwm0, &mut pwm1, &mut pwm2, &mut pwm3, &mut pwm4, &mut pwm5,
    ];
    let mut outputs = ServoDrivers::new(drivers);
    outputs.init_all().unwrap();

    let storage = RamStorage::new();
    let (config_mgr, load_status) = ConfigManager::new_with_default_fallback(storage).unwrap();
    match load_status {
        c3_servo::servos::config::LoadStatus::Loaded => println!("config loaded from storage"),
        c3_servo::servos::config::LoadStatus::Empty => println!("config storage is empty"),
        c3_servo::servos::config::LoadStatus::Defaulted(_) => {
            println!("config fallback to defaults")
        }
    }

    let mut engine = MotionEngine::new();
    engine.set_easing_function(quadratic_in_easing);

    let mut controller = ControlUnit::new(&DEMO_IDS).unwrap();
    let targets_a = build_targets(&POSE_A);
    let targets_b = build_targets(&POSE_B);

    let mut next_is_pose_b = true;
    let mut cooldown_ms = 0u32;

    controller
        .start(
            &mut engine,
            &config_mgr,
            &targets_a,
            DEMO_DURATION_MS,
            Some(Callback::from_fn(on_motion_done)),
        )
        .unwrap();

    loop {
        delay.delay_millis(1);

        let completed_mask = engine.update_1ms();
        controller.on_completed_mask(completed_mask);

        engine
            .sync_to_hardware(|id, pulse| outputs.set_pulse_width(id, pulse).map_err(|_| ()))
            .unwrap();

        if controller.is_active() {
            continue;
        }

        if cooldown_ms < DEMO_PAUSE_MS {
            cooldown_ms += 1;
            continue;
        }

        cooldown_ms = 0;
        let next_targets = if next_is_pose_b {
            &targets_b
        } else {
            &targets_a
        };
        next_is_pose_b = !next_is_pose_b;

        controller
            .start(
                &mut engine,
                &config_mgr,
                next_targets,
                DEMO_DURATION_MS,
                Some(Callback::from_fn(on_motion_done)),
            )
            .unwrap();
    }
}
