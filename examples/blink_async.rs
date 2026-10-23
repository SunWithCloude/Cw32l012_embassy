//! 异步 GPIO 闪烁示例（PC13，板上 LED 常见位置）。
//!
//! cargo build --target thumbv6m-none-eabi --release --example blink_async

#![no_std]
#![no_main]

use embassy_executor::main;
use embassy_time::Timer;

use embassy_cw32::{
    Config, Peripherals,
    gpio::{Level, Output, Pin},
    sysctrl_enable_gpioc,
};

#[main]
async fn main(_spawner: embassy_executor::Spawner) {
    embassy_cw32::init(Config::default()); // 复位默认 SysClk = 4MHz

    let p = Peripherals::take().unwrap();
    sysctrl_enable_gpioc(p.sysctrl);

    let mut led = Output::new(Pin::new(p.gpioc, 13), Level::High);

    loop {
        Timer::after_millis(500).await;
        led.toggle();
    }
}

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {
        core::hint::spin_loop();
    }
}
