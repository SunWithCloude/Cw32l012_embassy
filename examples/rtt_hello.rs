//! RTT 日志示例（L012）：defmt 经 RTT 输出，`probe-rs` 原生可读
//! （`probe-rs logs` / VS Code 插件的输出面板），不占 UART 引脚。
//!
//! cargo build --target thumbv6m-none-eabi --release --features rt --example rtt_hello
//! cargo run --release --bin ...
//! 观测:probe-rs attach --log-format oneline 时 defmt 输出直接进终端,
//! 或 VS Code 里 cw32-tools 的调试会话(DEFMT_LOG=info)。
//!
//! ⚠ 本示例的 panic handler 由 panic-probe 提供(带 defmt 打印),
//! 不再自定义 #[panic_handler]。

#![no_std]
#![no_main]

use defmt_rtt as _; // RTT 日志通道(全局 logger)
use panic_probe as _; // defmt 版 panic handler

/// defmt 的时间戳(毫秒 uptime):embassy-time 的 Instant 提供
#[unsafe(no_mangle)]
fn _defmt_timestamp() -> u64 {
    embassy_time::Instant::now().as_micros() / 1_000
}

use embassy_executor::main;
use embassy_time::Timer;

use embassy_cw32::{
    Config, Peripherals,
    gpio::{Level, Output},
};

#[main]
async fn main(_spawner: embassy_executor::Spawner) {
    embassy_cw32::init(Config::default());
    defmt::info!("embassy-cw32 RTT hello — 复位默认时钟 4MHz");

    let p = Peripherals::take().unwrap();
    embassy_cw32::sysctrl_enable_gpioc(p.sysctrl);
    let mut led = Output::new(Pin::new(p.gpioc, 13), Level::High);

    let mut n: u32 = 0;
    loop {
        n += 1;
        defmt::info!("heartbeat #{} — LED 翻转", n);
        led.toggle();
        Timer::after_millis(500).await;
    }
}

use embassy_cw32::gpio::Pin;
