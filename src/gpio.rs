//! GPIO：异步 `Input`（等边沿 / 等电平）与 `Output`。
//!
//! CW32L012 每个端口共享一个中断（IRQ5..8 对应 A/B/C/F），上升沿 /
//! 下降沿使能按位独立（`RISEIE` / `FALLIE`），标志在 `ISR`，写 `ICR`
//! 清除。等待期间注册 per-pin waker，端口中断里按位唤醒。
//!
//! ```ignore
//! embassy_cw32::bind_interrupts!(struct Irqs { GPIOA => embassy_cw32::gpio::GpioAIrq; });
//! let btn = Input::new(Pin::new(p.gpioa, 0), Pull::Up);
//! btn.wait_for_falling_edge().await;
//! ```

use core::future::poll_fn;
use core::sync::atomic::Ordering;
use core::task::Poll;

use embassy_sync::waitqueue::AtomicWaker;

use crate::interrupt::{IrqHandler, interrupt};
use crate::pac;

/// 引脚（运行时绑定端口与编号）
pub struct Pin {
    gpio: &'static pac::Gpio,
    pin: u8,
}

impl Pin {
    /// 绑定端口与引脚号（0..15，须与封装实际可用引脚一致）
    pub fn new(gpio: &'static pac::Gpio, pin: u8) -> Self {
        assert!(pin < 16);
        Self { gpio, pin }
    }

    /// 引脚位掩码
    fn mask(&self) -> u32 {
        1 << self.pin
    }

    /// 设置复用功能（AFRL pin0..7 / AFRH pin8..15，每脚 4bit）
    pub fn set_af(&self, af: u8) {
        let m4 = 0xF << ((self.pin % 8) * 4);
        let v4 = ((af as u32) & 0xF) << ((self.pin % 8) * 4);
        if self.pin < 8 {
            self.gpio.afrl.modify(|v| (v & !m4) | v4);
        } else {
            self.gpio.afrh.modify(|v| (v & !m4) | v4);
        }
    }

    /// 端口索引（0..3 = A/B/C/F，用于 waker 表）
    fn bank(&self) -> usize {
        let addr = self.gpio as *const _ as usize;
        match addr {
            pac::GPIOA_BASE => 0,
            pac::GPIOB_BASE => 1,
            pac::GPIOC_BASE => 2,
            pac::GPIOF_BASE => 3,
            _ => unreachable!("非 GPIO 端口寄存器块"),
        }
    }
}

/// 上/下拉配置。
///
/// ⚠ 下拉仅 PF3 有实际电阻（UM V1.4 §9.6.3，复位值 `PDR=0x0000_0008`），
/// 其余引脚 [`Pull::Down`] 等同 [`Pull::None`]。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Pull {
    /// 浮空
    None,
    /// 上拉
    Up,
    /// 下拉（仅 PF3 有效）
    Down,
}

/// 输出电平
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Level {
    /// 低电平
    Low,
    /// 高电平
    High,
}

impl Level {
    /// 翻转
    pub fn toggle(&mut self) {
        *self = match *self {
            Level::Low => Level::High,
            Level::High => Level::Low,
        };
    }
}

/// 数字输出（推挽）。
pub struct Output {
    pin: Pin,
}

impl Output {
    /// 配置为推挽输出（方向 DIR=0、数字模式 ANALOG=0），并置初始电平。
    pub fn new(pin: Pin, initial: Level) -> Self {
        pin.gpio.analog.modify(|v| v & !pin.mask());
        pin.gpio.dir.modify(|v| v & !pin.mask());
        match initial {
            Level::Low => pin.gpio.brr.write(pin.mask()),
            Level::High => pin.gpio.bsrr.write(pin.mask()),
        }
        Self { pin }
    }

    /// 置高（BSRR 原子置位）
    pub fn set_high(&mut self) {
        self.pin.gpio.bsrr.write(self.pin.mask());
    }

    /// 置低（BRR 原子清零）
    pub fn set_low(&mut self) {
        self.pin.gpio.brr.write(self.pin.mask());
    }

    /// 翻转（TOG 原子翻转）
    pub fn toggle(&mut self) {
        self.pin.gpio.tog.write(self.pin.mask());
    }

    /// 设置电平
    pub fn set_level(&mut self, level: Level) {
        match level {
            Level::Low => self.set_low(),
            Level::High => self.set_high(),
        }
    }

    /// 读回输出锁存电平（ODR）
    pub fn get_output_level(&self) -> Level {
        if self.pin.gpio.odr.read() & self.pin.mask() != 0 {
            Level::High
        } else {
            Level::Low
        }
    }

    /// 释放引脚
    pub fn into_pin(self) -> Pin {
        self.pin
    }
}

impl embedded_hal::digital::ErrorType for Output {
    type Error = core::convert::Infallible;
}

impl embedded_hal::digital::OutputPin for Output {
    fn set_low(&mut self) -> Result<(), Self::Error> {
        Output::set_low(self);
        Ok(())
    }
    fn set_high(&mut self) -> Result<(), Self::Error> {
        Output::set_high(self);
        Ok(())
    }
}

impl embedded_hal::digital::StatefulOutputPin for Output {
    fn is_set_high(&mut self) -> Result<bool, Self::Error> {
        Ok(self.get_output_level() == Level::High)
    }
    fn is_set_low(&mut self) -> Result<bool, Self::Error> {
        Ok(self.get_output_level() == Level::Low)
    }
}

// ================= 异步输入 =================

/// 数字输入（可上/下拉）。异步等待边沿与电平。
pub struct Input {
    pin: Pin,
}

/// 每端口 16 引脚的 waker 表
static WAKERS: [[AtomicWaker; 16]; 4] = [const { [const { AtomicWaker::new() }; 16] }; 4];

/// 每端口的软件触发位图（IRQ 清硬件标志后置位，任务侧消费；
/// 全程 critical section，避免丢唤醒）
static TRIGGERED: [core::sync::atomic::AtomicU32; 4] =
    [const { core::sync::atomic::AtomicU32::new(0) }; 4];

impl Input {
    /// 配置为数字输入（DIR=1、ANALOG=0）并设置上/下拉。
    pub fn new(pin: Pin, pull: Pull) -> Self {
        pin.gpio.analog.modify(|v| v & !pin.mask());
        pin.gpio.dir.modify(|v| v | pin.mask());
        match pull {
            Pull::None => {
                pin.gpio.pur.modify(|v| v & !pin.mask());
                pin.gpio.pdr.modify(|v| v & !pin.mask());
            }
            Pull::Up => {
                pin.gpio.pur.modify(|v| v | pin.mask());
                pin.gpio.pdr.modify(|v| v & !pin.mask());
            }
            Pull::Down => {
                // 仅 PF3 有实际下拉（见 Pull::Down 文档）
                pin.gpio.pur.modify(|v| v & !pin.mask());
                pin.gpio.pdr.modify(|v| v | pin.mask());
            }
        }
        Self { pin }
    }

    /// 当前电平（IDR）
    pub fn get_level(&self) -> Level {
        if self.pin.gpio.idr.read() & self.pin.mask() != 0 {
            Level::High
        } else {
            Level::Low
        }
    }

    /// 释放引脚
    pub fn into_pin(self) -> Pin {
        self.pin
    }

    /// 等待任一边沿（上升或下降）。
    pub async fn wait_for_any_edge(&mut self) {
        self.wait_edge(true, true).await;
    }

    /// 等待上升沿。
    pub async fn wait_for_rising_edge(&mut self) {
        self.wait_edge(true, false).await;
    }

    /// 等待下降沿。
    pub async fn wait_for_falling_edge(&mut self) {
        self.wait_edge(false, true).await;
    }

    /// 边沿等待公共实现。
    async fn wait_edge(&mut self, rise: bool, fall: bool) {
        let m = self.pin.mask();
        let bank = self.pin.bank();
        let gpio = self.pin.gpio;
        // 使能 NVIC 中断线（v0.1 遗漏：向量有了但 ISER 没开）
        match bank {
            0 => crate::pac::Interrupt::GpioA.enable(),
            1 => crate::pac::Interrupt::GpioB.enable(),
            2 => crate::pac::Interrupt::GpioC.enable(),
            _ => crate::pac::Interrupt::GpioF.enable(),
        }
        // 清历史标志、清软件触发位，再挂边沿中断
        critical_section::with(|_| {
            gpio.icr.write(m);
            TRIGGERED[bank].store(
                TRIGGERED[bank].load(Ordering::Relaxed) & !m,
                Ordering::Relaxed,
            );
        });
        if rise {
            gpio.riseie.modify(|v| v | m);
        }
        if fall {
            gpio.fallie.modify(|v| v | m);
        }
        poll_fn(|cx| {
            WAKERS[bank][self.pin.pin as usize].register(cx.waker());
            let hit = critical_section::with(|_| {
                let t = TRIGGERED[bank].load(Ordering::Relaxed);
                if t & m != 0 {
                    TRIGGERED[bank].store(t & !m, Ordering::Relaxed);
                    true
                } else {
                    false
                }
            });
            if hit { Poll::Ready(()) } else { Poll::Pending }
        })
        .await;
        // 摘除中断使能并清标志
        gpio.riseie.modify(|v| v & !m);
        gpio.fallie.modify(|v| v & !m);
        gpio.icr.write(m);
    }

    /// 等待高电平（边沿触发 + 电平复查）。
    pub async fn wait_for_high(&mut self) {
        loop {
            if self.get_level() == Level::High {
                return;
            }
            self.wait_for_any_edge().await;
        }
    }

    /// 等待低电平（边沿触发 + 电平复查）。
    pub async fn wait_for_low(&mut self) {
        loop {
            if self.get_level() == Level::Low {
                return;
            }
            self.wait_for_any_edge().await;
        }
    }
}

impl embedded_hal::digital::ErrorType for Input {
    type Error = core::convert::Infallible;
}

impl embedded_hal::digital::InputPin for Input {
    fn is_high(&mut self) -> Result<bool, Self::Error> {
        Ok(self.get_level() == Level::High)
    }
    fn is_low(&mut self) -> Result<bool, Self::Error> {
        Ok(self.get_level() == Level::Low)
    }
}

// ================= 端口中断派发 =================

/// 端口中断公共处理：置软件触发位、清硬件标志、按位唤醒。
fn on_irq(bank: usize, gpio: &'static pac::Gpio) {
    let pending = gpio.isr.read();
    critical_section::with(|_| {
        let t = TRIGGERED[bank].load(Ordering::Relaxed);
        TRIGGERED[bank].store(t | pending, Ordering::Relaxed);
    });
    gpio.icr.write(pending); // 写 1 清除
    for (i, w) in WAKERS[bank].iter().enumerate() {
        if pending & (1 << i) != 0 {
            w.wake();
        }
    }
}

/// GPIOA 中断标记（IRQ5）
pub struct GpioAIrq;
impl IrqHandler<interrupt::GPIOA> for GpioAIrq {
    fn call() {
        let gpio = unsafe { &*(pac::GPIOA_BASE as *const pac::Gpio) };
        on_irq(0, gpio);
    }
}

/// GPIOB 中断标记（IRQ6）
pub struct GpioBIrq;
impl IrqHandler<interrupt::GPIOB> for GpioBIrq {
    fn call() {
        let gpio = unsafe { &*(pac::GPIOB_BASE as *const pac::Gpio) };
        on_irq(1, gpio);
    }
}

/// GPIOC 中断标记（IRQ7）
pub struct GpioCIrq;
impl IrqHandler<interrupt::GPIOC> for GpioCIrq {
    fn call() {
        let gpio = unsafe { &*(pac::GPIOC_BASE as *const pac::Gpio) };
        on_irq(2, gpio);
    }
}

/// GPIOF 中断标记（IRQ8）
pub struct GpioFIrq;
impl IrqHandler<interrupt::GPIOF> for GpioFIrq {
    fn call() {
        let gpio = unsafe { &*(pac::GPIOF_BASE as *const pac::Gpio) };
        on_irq(3, gpio);
    }
}
