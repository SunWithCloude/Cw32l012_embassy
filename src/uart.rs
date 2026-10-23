//! 异步 UART（中断驱动，逐字节收发；`embedded-io-async` 接口）。
//!
//! 发送等待 TXBE（发送缓冲空）、接收等待 RC（接收完成）。等待时才打开
//! 对应 IER 位，避免空闲时中断风暴。
//!
//! 引脚组合（UM 表 9-2 摘要）：
//!
//! | UART  | 常用组合                       |
//! |-------|--------------------------------|
//! | UART1 | PA02=TXD / PA03=RXD（AF1）     |
//! | UART2 | PA07=TXD / PA06=RXD（AF1）     |
//! | UART3 | PA04=TXD / PA05=RXD（AF1）     |

use core::future::poll_fn;
use core::task::Poll;

use embassy_sync::waitqueue::AtomicWaker;
use embedded_io_async as embedded_io;

use crate::gpio::Pin;
use crate::interrupt::{Binding, IrqHandler, interrupt};
use crate::pac::{self, uart as ub};

/// UART 配置（固定 8-N-1）
#[derive(Debug, Clone, Copy)]
pub struct Config {
    /// 波特率
    pub baudrate: u32,
}

impl Default for Config {
    fn default() -> Self {
        Config { baudrate: 115_200 }
    }
}

/// UART 错误
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum UartError {
    /// 接收溢出
    Overrun,
    /// 帧错误（波特率不匹配？）
    Framing,
    /// 奇偶校验错
    Parity,
    /// 噪声
    Noise,
}

impl core::fmt::Display for UartError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            UartError::Overrun => write!(f, "receiver overrun (data lost)"),
            UartError::Framing => write!(f, "framing error (baud rate mismatch?)"),
            UartError::Parity => write!(f, "parity error"),
            UartError::Noise => write!(f, "noise error"),
        }
    }
}

impl core::error::Error for UartError {}

impl embedded_io::Error for UartError {
    fn kind(&self) -> embedded_io::ErrorKind {
        embedded_io::ErrorKind::Other
    }
}

/// 实例索引（0/1/2 = UART1/2/3）
fn instance_of(regs: &'static pac::Uart) -> usize {
    match regs as *const _ as usize {
        pac::UART1_BASE => 0,
        pac::UART2_BASE => 1,
        pac::UART3_BASE => 2,
        _ => unreachable!("非 UART 寄存器块"),
    }
}

/// per-实例 waker（0=TX 可写，1=RX 可读）
static TX_WAKER: [AtomicWaker; 3] = [const { AtomicWaker::new() }; 3];
static RX_WAKER: [AtomicWaker; 3] = [const { AtomicWaker::new() }; 3];

/// 异步 UART（整体）
pub struct Uart {
    regs: &'static pac::Uart,
    idx: usize,
}

/// 异步 UART 发送半部
pub struct UartTx {
    regs: &'static pac::Uart,
    idx: usize,
}

/// 异步 UART 接收半部
pub struct UartRx {
    regs: &'static pac::Uart,
    idx: usize,
}

/// 计算波特率分频：`Baud = UCLK / (BRRI*16 + BRRF)`（四舍五入）。
fn calc_brr(uclk_hz: u32, baud: u32) -> (u32, u32) {
    let div = (uclk_hz as u64 * 2 + baud as u64) / (baud as u64 * 2);
    let brri = (div / 16).clamp(1, 0xFFFF);
    let brrf = div % 16;
    (brri as u32, brrf as u32)
}

impl Uart {
    /// 初始化 UART（8-N-1，PCLK 时钟源）并配置引脚复用。
    ///
    /// * `regs` — UART1/2/3 寄存器块
    /// * `tx` / `rx` — 引脚（内部自动 `set_af(af)`）
    /// * `_irqs` — 中断绑定证明（`bind_interrupts!` 生成的结构体）
    pub fn new(
        regs: &'static pac::Uart,
        tx: Pin,
        rx: Pin,
        af: u8,
        _irqs: impl Binding<interrupt::UART1> + Binding<interrupt::UART2> + Binding<interrupt::UART3>,
        config: Config,
    ) -> Self {
        Self::new_inner(regs, tx, rx, af, config)
    }

    /// 初始化 UART1（IRQ 绑定类型收窄到 UART1）。
    pub fn new_uart1(
        regs: &'static pac::Uart,
        tx: Pin,
        rx: Pin,
        af: u8,
        _irqs: impl Binding<interrupt::UART1>,
        config: Config,
    ) -> Self {
        Self::new_inner(regs, tx, rx, af, config)
    }

    /// 初始化 UART2（IRQ 绑定类型收窄到 UART2）。
    pub fn new_uart2(
        regs: &'static pac::Uart,
        tx: Pin,
        rx: Pin,
        af: u8,
        _irqs: impl Binding<interrupt::UART2>,
        config: Config,
    ) -> Self {
        Self::new_inner(regs, tx, rx, af, config)
    }

    /// 初始化 UART3（IRQ 绑定类型收窄到 UART3）。
    pub fn new_uart3(
        regs: &'static pac::Uart,
        tx: Pin,
        rx: Pin,
        af: u8,
        _irqs: impl Binding<interrupt::UART3>,
        config: Config,
    ) -> Self {
        Self::new_inner(regs, tx, rx, af, config)
    }

    fn new_inner(regs: &'static pac::Uart, tx: Pin, rx: Pin, af: u8, config: Config) -> Self {
        let idx = instance_of(regs);
        // 使能 APB1 时钟（KEY 保护写）
        let sysctrl = unsafe { &*(pac::SYSCTRL_BASE as *const pac::Sysctrl) };
        let gate = match idx {
            0 => pac::sysctrl::APBEN1_UART1,
            1 => pac::sysctrl::APBEN1_UART2,
            _ => pac::sysctrl::APBEN1_UART3,
        };
        sysctrl.apben1.write(pac::sysctrl::KEY | gate);

        // 引脚复用（数字模式、方向交给外设）
        tx.set_af(af);
        rx.set_af(af);

        // 8-N-1、PCLK 时钟源、收发使能
        regs.cr1.write(ub::CR1_TXEN | ub::CR1_RXEN);
        let (brri, brrf) = calc_brr(
            crate::PCLK_HZ.load(core::sync::atomic::Ordering::Relaxed),
            config.baudrate,
        );
        regs.brri.write(brri);
        regs.brrf.write(brrf);
        // 清残留错误标志（R1W0：对应位写 0）
        regs.icr.write(ub::ICR_CLEAR_RX_ERRORS);

        Self { regs, idx }
    }

    /// 拆分收发半部
    pub fn split(self) -> (UartTx, UartRx) {
        (
            UartTx {
                regs: self.regs,
                idx: self.idx,
            },
            UartRx {
                regs: self.regs,
                idx: self.idx,
            },
        )
    }

    /// 异步写单字节（等 TXBE）
    pub async fn write_byte(&mut self, b: u8) {
        UartTx {
            regs: self.regs,
            idx: self.idx,
        }
        .write_byte(b)
        .await;
    }

    /// 异步读单字节（等 RC）
    pub async fn read_byte(&mut self) -> Result<u8, UartError> {
        UartRx {
            regs: self.regs,
            idx: self.idx,
        }
        .read_byte()
        .await
    }
}

impl UartTx {
    /// 异步写单字节
    pub async fn write_byte(&mut self, b: u8) {
        poll_fn(|cx| {
            TX_WAKER[self.idx].register(cx.waker());
            if self.regs.isr.read() & ub::ISR_TXBE != 0 {
                Poll::Ready(())
            } else {
                // 等待期间打开 TXBE 中断（IRQ 里唤醒后由本 poll 复查）
                self.regs.ier.modify(|v| v | ub::ISR_TXBE);
                Poll::Pending
            }
        })
        .await;
        self.regs.tdr.write(b as u32);
    }
}

impl UartRx {
    /// 异步读单字节；错误经 ICR 清除后返回
    pub async fn read_byte(&mut self) -> Result<u8, UartError> {
        poll_fn(|cx| {
            RX_WAKER[self.idx].register(cx.waker());
            let isr = self.regs.isr.read();
            if isr & (ub::ISR_RC | ub::ISR_ORE | ub::ISR_NE | ub::ISR_PE | ub::ISR_FE) != 0 {
                Poll::Ready(())
            } else {
                self.regs.ier.modify(|v| v | ub::ISR_RC);
                Poll::Pending
            }
        })
        .await;
        let isr = self.regs.isr.read();
        let err = if isr & ub::ISR_ORE != 0 {
            Some(UartError::Overrun)
        } else if isr & ub::ISR_FE != 0 {
            Some(UartError::Framing)
        } else if isr & ub::ISR_PE != 0 {
            Some(UartError::Parity)
        } else if isr & ub::ISR_NE != 0 {
            Some(UartError::Noise)
        } else {
            None
        };
        if let Some(e) = err {
            self.regs.icr.write(ub::ICR_CLEAR_RX_ERRORS);
            return Err(e);
        }
        Ok(self.regs.rdr.read() as u8)
    }
}

// ================= embedded-io-async =================

impl embedded_io::ErrorType for Uart {
    type Error = UartError;
}
impl embedded_io::ErrorType for UartTx {
    type Error = UartError;
}
impl embedded_io::ErrorType for UartRx {
    type Error = UartError;
}

impl embedded_io_async::Write for Uart {
    async fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        for b in buf {
            self.write_byte(*b).await;
        }
        Ok(buf.len())
    }
    async fn flush(&mut self) -> Result<(), Self::Error> {
        // 等 TC（发送完成）
        poll_fn(|cx| {
            TX_WAKER[self.idx].register(cx.waker());
            if self.regs.isr.read() & ub::ISR_TC != 0 {
                Poll::Ready(())
            } else {
                self.regs.ier.modify(|v| v | ub::ISR_TC);
                Poll::Pending
            }
        })
        .await;
        Ok(())
    }
}

impl embedded_io_async::Write for UartTx {
    async fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        for b in buf {
            self.write_byte(*b).await;
        }
        Ok(buf.len())
    }
    async fn flush(&mut self) -> Result<(), Self::Error> {
        poll_fn(|cx| {
            TX_WAKER[self.idx].register(cx.waker());
            if self.regs.isr.read() & ub::ISR_TC != 0 {
                Poll::Ready(())
            } else {
                self.regs.ier.modify(|v| v | ub::ISR_TC);
                Poll::Pending
            }
        })
        .await;
        Ok(())
    }
}

impl embedded_io_async::Read for Uart {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        if buf.is_empty() {
            return Ok(0);
        }
        buf[0] = self.read_byte().await?;
        Ok(1)
    }
}

impl embedded_io_async::Read for UartRx {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        if buf.is_empty() {
            return Ok(0);
        }
        buf[0] = self.read_byte().await?;
        Ok(1)
    }
}

// ================= 中断派发 =================

/// 单实例中断处理：唤醒对应方向的等待者，清接收错误。
fn on_irq(idx: usize, regs: &'static pac::Uart) {
    let isr = regs.isr.read();
    if isr & (ub::ISR_RC | ub::ISR_ORE | ub::ISR_NE | ub::ISR_PE | ub::ISR_FE) != 0 {
        RX_WAKER[idx].wake();
    }
    if isr & (ub::ISR_TXBE | ub::ISR_TC) != 0 {
        TX_WAKER[idx].wake();
    }
    if isr & (ub::ISR_ORE | ub::ISR_NE | ub::ISR_PE | ub::ISR_FE) != 0 {
        regs.icr.write(ub::ICR_CLEAR_RX_ERRORS);
    }
    // 无等待者时压掉中断使能，避免风暴（下次 poll 会重新打开）
    if isr & ub::ISR_TXBE != 0 {
        regs.ier.modify(|v| v & !ub::ISR_TXBE);
    }
    if isr & ub::ISR_RC != 0 {
        regs.ier.modify(|v| v & !ub::ISR_RC);
    }
}

/// UART1 中断标记（IRQ27）
pub struct Uart1Irq;
impl IrqHandler<interrupt::UART1> for Uart1Irq {
    fn call() {
        on_irq(0, unsafe { &*(pac::UART1_BASE as *const pac::Uart) });
    }
}

/// UART2 中断标记（IRQ28）
pub struct Uart2Irq;
impl IrqHandler<interrupt::UART2> for Uart2Irq {
    fn call() {
        on_irq(1, unsafe { &*(pac::UART2_BASE as *const pac::Uart) });
    }
}

/// UART3 中断标记（IRQ29）
pub struct Uart3Irq;
impl IrqHandler<interrupt::UART3> for Uart3Irq {
    fn call() {
        on_irq(2, unsafe { &*(pac::UART3_BASE as *const pac::Uart) });
    }
}
