#![no_std]
#![no_main]

use esp_hal::clock::CpuClock;
use esp_hal::gpio::DriveMode;
use esp_hal::ledc::{
    LSGlobalClkSource, Ledc, LowSpeed, channel::{self, ChannelIFace},
    timer::{self, TimerIFace},
};
use esp_hal::main;
use esp_hal::time::Rate;
use esp_hal::delay::Delay;
use esp_println as _;
use defmt::{println};

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {}
}

esp_bootloader_esp_idf::esp_app_desc!();

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
    
    loop {

    }
}
