//! IWDT 看门狗演示（L012）：行为设计成**可观测的复位循环**（阻塞版
//! wdt_demo 实机配方移植）：
//!
//! 1. IWDT 超时 ≈ 1.5s（RC10K ÷4 档，ARR 按公式自动取；RC10K 有 ±% 级
//!    漂移，喂狗间隔留足裕量）
//! 2. `FEEDS` 静态量计数喂狗（PC13 每次喂狗翻转，肉眼可见快闪 5 次）
//! 3. 喂满 5 次后**故意停喂** → 1.5s 后看门狗复位 → 重启
//! 4. `FEEDS` 在看门狗复位后归零重新爬升——这就是"复位计数"观测面
//!
//! 预期观测（调试器内存窗口 / probe-rs read，地址以 nm 输出为准）：
//! - `FEEDS` 从 0 爬到 5 → 停喂 → 芯片复位 → 回到 0 再爬
//! - 周期 ≈ 5×600ms + 1.5s ≈ 4.5s 一轮，PC13 快闪 5 下、灭 1.5s
//! - 探针 SWD 短暂掉线再恢复 = 看门狗复位生效的硬证据
//!
//! cargo build --target thumbv6m-none-eabi --release --features rt --example wdt_demo

#![no_std]
#![no_main]

use embassy_executor::main;
use embassy_time::Timer;

use embassy_cw32::{
    Config, Peripherals,
    gpio::{Level, Output, Pin},
    wdt::{Iwdt, TimeoutMs},
};

/// 本次开机喂狗次数（看门狗复位后归零重新计数——复位循环的观测面）。
/// 单任务独写 + probe-rs 只读，static mut 可接受。
#[used]
#[unsafe(no_mangle)]
pub static mut FEEDS: u32 = 0;

#[main]
async fn main(_spawner: embassy_executor::Spawner) {
    embassy_cw32::init(Config::default()); // 复位默认 SysClk = 4MHz

    let p = Peripherals::take().unwrap();
    embassy_cw32::sysctrl_enable_gpioc(p.sysctrl);
    let mut led = Output::new(Pin::new(p.gpioc, 13), Level::High);

    let mut wdt = Iwdt::new(p.iwdt);
    // 1.5s 超时（喂狗间隔 600ms < 1.5s ✓）；范围外才会 Err，1.5s 必可诊
    if wdt.start(TimeoutMs(1500)).is_err() {
        loop {
            Timer::after_secs(1).await; // 不可达路径：便于观测配置失败态
        }
    }

    loop {
        unsafe {
            FEEDS += 1;
            // 只喂前 5 次；之后故意停喂 → 1.5s 后看门狗复位
            if FEEDS <= 5 {
                led.toggle();
                wdt.feed();
            }
        }
        // embassy 的 Timer 睡 WFI，IWDT 的 RC10K 独立时钟域照常计数
        Timer::after_millis(600).await;
    }
}

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {
        core::hint::spin_loop();
    }
}
