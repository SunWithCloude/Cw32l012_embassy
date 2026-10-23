//! # embassy-cw32
//!
//! 武汉芯源半导体（CMSemicon）**CW32L012** 的 embassy 异步 HAL。
//!
//! 基于 cw32l012-pac（手册 V1.4 逐项核对的寄存器层），为 Cortex-M0+
//! 提供 embassy 异步运行时支持：
//!
//! - [`init`]：SysTick 时间驱动（1MHz tick，动态单次闹钟，空闲不醒）
//! - [`bind_interrupts!`](crate::bind_interrupts)：编译期中断-驱动绑定
//! - [`gpio`]：异步 `Input`（等边沿 / 等电平）+ `Output`
//! - [`uart`]：异步 UART（中断驱动，`embedded-io-async` 接口）
//! - [`i2c`] / [`spi`]：`embedded-hal-async` 主机接口
//!
//! ## 最小示例
//!
//! ```ignore
//! #![no_std]
//! #![no_main]
//!
//! use embassy_executor::main;
//! use embassy_time::Timer;
//! use embassy_cw32::{gpio::{Level, Output, Pin}, Config, Peripherals};
//!
//! #[main]
//! async fn main() -> () {
//!     embassy_cw32::init(Config::default()); // 复位默认 SysClk = 4MHz
//!     let p = Peripherals::take().unwrap();
//!     embassy_cw32::sysctrl_enable_gpioa(p.sysctrl); // 打开 GPIOA 时钟
//!     let mut led = Output::new(Pin::new(p.gpioa, 13), Level::High);
//!     loop {
//!         Timer::after_millis(500).await;
//!         led.toggle();
//!     }
//! }
//! ```
//!
//! ## 时钟约定
//!
//! v0.1 不做时钟树配置：`Config` 只是**声明**当前 SysClk / PCLK 频率
//! （复位默认均为 4MHz），用于 SysTick 计时与波特率换算。需要变频的
//! 固件请先用寄存器层配好时钟再 `init`，并传入对应频率。
//!
//! ## 中断绑定
//!
//! 异步外设需要把对应 IRQ 绑定到驱动（编译期检查，杜绝绑错）：
//!
//! ```ignore
//! embassy_cw32::bind_interrupts!(struct Irqs {
//!     UART1 => uart::Uart1Irq;
//!     GPIOA => gpio::GpioAIrq;
//! });
//! ```

#![no_std]
#![allow(rustdoc::broken_intra_doc_links)]

pub mod gpio;
pub mod i2c;
pub mod interrupt;
pub mod spi;
pub mod time_driver;
pub mod uart;

pub use cw32l012_pac as pac;
pub use pac::Peripherals;

use core::sync::atomic::{AtomicU32, Ordering};

/// 全局配置（时间驱动与外设波特率换算使用）
pub(crate) static HCLK_HZ: AtomicU32 = AtomicU32::new(4_000_000);
pub(crate) static PCLK_HZ: AtomicU32 = AtomicU32::new(4_000_000);

/// embassy 初始化配置。
#[derive(Debug, Clone, Copy)]
pub struct Config {
    /// HCLK 频率（SysTick 时钟源）。复位默认 4MHz（HSI 96MHz / 24）。
    pub hclk_hz: u32,
    /// PCLK 频率（UART/SPI/I2C 波特率时钟）。复位默认与 HCLK 相同。
    pub pclk_hz: u32,
}

impl Default for Config {
    fn default() -> Self {
        Config {
            hclk_hz: 4_000_000,
            pclk_hz: 4_000_000,
        }
    }
}

/// embassy 初始化：启动 SysTick 时间驱动。
///
/// 必须在执行器运行任何带时间依赖的任务之前调用（`#[main]` 的第一行）。
pub fn init(config: Config) {
    HCLK_HZ.store(config.hclk_hz, Ordering::Relaxed);
    PCLK_HZ.store(config.pclk_hz, Ordering::Relaxed);
    time_driver::init();
}

// ================= critical-section 实现 =================

/// PRIMASK 临界区（feature `critical-section-impl`，默认开启）。
///
/// 单核 M0+ 语义：关可屏蔽中断；嵌套进入只记录"已关"标记，最外层退出
/// 时恢复。**一个固件只能有一个 critical-section 实现**——不要与
/// cw32l012-hal 的 `critical-section-impl` 或 cortex-m 的实现同时启用。
#[cfg(all(
    feature = "critical-section-impl",
    target_arch = "arm",
    target_os = "none"
))]
mod cs_impl {
    struct Cs;

    critical_section::set_impl!(Cs);

    unsafe impl critical_section::Impl for Cs {
        unsafe fn acquire() -> critical_section::RawRestoreState {
            let primask: u32;
            unsafe {
                core::arch::asm!("mrs {0}, PRIMASK", "cpsid i", out(reg) primask);
            }
            (primask & 1) as u8
        }

        unsafe fn release(token: critical_section::RawRestoreState) {
            if token == 0 {
                unsafe { core::arch::asm!("cpsie i") };
            }
        }
    }
}

/// 宿主机（cargo test）下的退化 critical-section 实现：全局位自旋。
/// 单线程测试环境足够；不用于任何真实目标。
#[cfg(all(
    feature = "critical-section-impl",
    not(all(target_arch = "arm", target_os = "none"))
))]
mod cs_impl_host {
    use core::sync::atomic::{AtomicBool, Ordering};

    struct Cs;

    critical_section::set_impl!(Cs);

    unsafe impl critical_section::Impl for Cs {
        unsafe fn acquire() -> critical_section::RawRestoreState {
            while LOCK
                .compare_exchange_weak(false, true, Ordering::Acquire, Ordering::Relaxed)
                .is_err()
            {}
            0
        }

        unsafe fn release(_token: critical_section::RawRestoreState) {
            LOCK.store(false, Ordering::Release);
        }
    }

    static LOCK: AtomicBool = AtomicBool::new(false);
}

// ================= 便捷时钟门控 =================

/// 打开 GPIOA 端口时钟（AHBEN，KEY 保护写）。
pub fn sysctrl_enable_gpioa(sysctrl: &'static pac::Sysctrl) {
    sysctrl
        .ahben
        .write(pac::sysctrl::KEY | pac::sysctrl::AHBEN_GPIOA);
}

/// 打开 GPIOB 端口时钟。
pub fn sysctrl_enable_gpiob(sysctrl: &'static pac::Sysctrl) {
    sysctrl
        .ahben
        .write(pac::sysctrl::KEY | pac::sysctrl::AHBEN_GPIOB);
}

/// 打开 GPIOC 端口时钟。
pub fn sysctrl_enable_gpioc(sysctrl: &'static pac::Sysctrl) {
    sysctrl
        .ahben
        .write(pac::sysctrl::KEY | pac::sysctrl::AHBEN_GPIOC);
}

/// 打开 GPIOF 端口时钟。
pub fn sysctrl_enable_gpiof(sysctrl: &'static pac::Sysctrl) {
    sysctrl
        .ahben
        .write(pac::sysctrl::KEY | pac::sysctrl::AHBEN_GPIOF);
}
