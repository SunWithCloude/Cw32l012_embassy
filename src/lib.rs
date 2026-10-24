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

// 家族互斥/必选检查
#[cfg(all(feature = "cw32l012", feature = "cw32f030"))]
compile_error!("cw32l012 与 cw32f030 feature 互斥,一个固件只选一个家族");
#[cfg(not(any(feature = "cw32l012", feature = "cw32f030")))]
compile_error!("必须启用一个家族 feature:cw32l012 或 cw32f030");

pub mod gpio;
pub mod i2c;
pub mod interrupt;
#[cfg(feature = "cw32f030")]
pub mod rcc;
pub mod spi;
pub mod time_driver;
pub mod uart;

#[cfg(feature = "cw32f030")]
pub use cw32f030_pac as pac;
#[cfg(feature = "cw32l012")]
pub use cw32l012_pac as pac;
pub use pac::Peripherals;

/// 两套 PAC 的寄存器原语统一别名(字段访问签名用)
pub mod pac_types {
    /// 可读可写 32 位寄存器
    pub type RwU32 = crate::pac::Rw<u32>;
    /// 只读 32 位寄存器
    pub type RoU32 = crate::pac::Ro<u32>;
    /// 只写 32 位寄存器
    pub type WoU32 = crate::pac::Wo<u32>;
}

/// 家族统一的寄存器类型别名(手写 PAC 用 Gpio/Sysctrl/Uart,
/// 生成 PAC 用头文件原名 GPIO/SYSCTRL/UART)
#[cfg(feature = "cw32l012")]
pub type Gpio = pac::Gpio;
#[cfg(feature = "cw32l012")]
pub type Sysctrl = pac::Sysctrl;
#[cfg(feature = "cw32l012")]
pub type Uart = pac::Uart;
#[cfg(feature = "cw32l012")]
pub type Spi = pac::Spi;
#[cfg(feature = "cw32l012")]
pub type I2c = pac::I2c;
#[cfg(feature = "cw32f030")]
pub type Gpio = pac::GPIO;
#[cfg(feature = "cw32f030")]
pub type Sysctrl = pac::SYSCTRL;
#[cfg(feature = "cw32f030")]
pub type Uart = pac::UART;
#[cfg(feature = "cw32f030")]
pub type Spi = pac::SPI;
#[cfg(feature = "cw32f030")]
pub type I2c = pac::I2C;

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
        // 复位默认时钟:L012 = HSI 96MHz/24 = 4MHz;F030 = HSI 48MHz/6 = 8MHz
        // (pdsc processor Dclock=8MHz 佐证;上板实测 48MHz 假设下 SysTick 慢 6 倍)
        #[cfg(feature = "cw32l012")]
        let hz = 4_000_000;
        #[cfg(feature = "cw32f030")]
        let hz = 8_000_000;
        Config {
            hclk_hz: hz,
            pclk_hz: hz,
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

/// 切频后更新全局频率声明(配 [] 使用;否则 Timer/波特率全部错拍)
#[cfg(feature = "cw32f030")]
pub fn set_frequencies(hclk_hz: u32, pclk_hz: u32) {
    HCLK_HZ.store(hclk_hz, Ordering::Relaxed);
    PCLK_HZ.store(pclk_hz, Ordering::Relaxed);
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
// 两代的 AHBEN GPIO 位:A/B/C 相同(4/5/6),F 不同(L012=7,F030=9);
// 且生成版 PAC 无位常量,统一用位掩码写。

/// SYSCTRL 写保护键。⚠ 仅 L012 需要:其 KEY 保护寄存器须高 16 位
/// 0x5A5A 才生效。**F030 经典家族的 AHBEN/APBEN 无 KEY 保护**(厂商库
/// `CW_SYSCTRL->APBEN2 |= Periph` 直接写,2026-10-24 上板实测带 KEY 写
/// 会顶掉使能位),仅 IER/复位类寄存器带 KEY。
#[cfg(feature = "cw32l012")]
pub const SYSCTRL_KEY: u32 = 0x5A5A << 16;

/// 打开 GPIOA 端口时钟(AHBEN bit4,两代相同)。
pub fn sysctrl_enable_gpioa(sysctrl: &'static crate::Sysctrl) {
    #[cfg(feature = "cw32l012")]
    sysctrl.ahben.write(SYSCTRL_KEY | (1 << 4));
    #[cfg(feature = "cw32f030")]
    sysctrl.ahben.modify(|v| v | (1 << 4));
}

/// 打开 GPIOB 端口时钟(AHBEN bit5,两代相同)。
pub fn sysctrl_enable_gpiob(sysctrl: &'static crate::Sysctrl) {
    #[cfg(feature = "cw32l012")]
    sysctrl.ahben.write(SYSCTRL_KEY | (1 << 5));
    #[cfg(feature = "cw32f030")]
    sysctrl.ahben.modify(|v| v | (1 << 5));
}

/// 打开 GPIOC 端口时钟(AHBEN bit6,两代相同)。
pub fn sysctrl_enable_gpioc(sysctrl: &'static crate::Sysctrl) {
    #[cfg(feature = "cw32l012")]
    sysctrl.ahben.write(SYSCTRL_KEY | (1 << 6));
    #[cfg(feature = "cw32f030")]
    sysctrl.ahben.modify(|v| v | (1 << 6));
}

/// 打开 GPIOF 端口时钟(L012:AHBEN bit7;F030:bit9)。
pub fn sysctrl_enable_gpiof(sysctrl: &'static crate::Sysctrl) {
    #[cfg(feature = "cw32l012")]
    sysctrl.ahben.write(SYSCTRL_KEY | (1 << 7));
    #[cfg(feature = "cw32f030")]
    sysctrl.ahben.modify(|v| v | (1 << 9));
}
