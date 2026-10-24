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

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::pipe::Pipe;
use embassy_sync::waitqueue::AtomicWaker;
use embedded_io_async as embedded_io;

use crate::gpio::{GpioPort, Pin};
use crate::interrupt::{Binding, IrqHandler, interrupt};
use crate::pac;

// ================= 家族位常量面 =================
// 两代共用位:TXBE=0 / TC=1 / RC=2;错误位不同(L012 FE8/PE9/NE10/ORE11,
// F030 经典 FE3/PE4 且无 NE/ORE 标志)。ICR 两代均为 R1W0(写 0 清除):
// L012 依据 UM V1.4;F030 依据厂商库 cw32f030_uart.c 的 REGBITS_CLR(ICR, flag)。
#[cfg(feature = "cw32l012")]
pub(crate) mod ub {
    pub use crate::pac::uart::*;
    /// 全部接收错误位掩码(L012:ORE|NE|PE|FE)
    pub const ERR_ALL: u32 = ISR_ORE | ISR_NE | ISR_PE | ISR_FE;
    /// 接收就绪 + 全部错误位
    pub const RX_ALL: u32 = ISR_RC | ERR_ALL;
    /// 错误种类解码(L012 含 NE/ORE)
    pub fn decode_error(isr: u32) -> Option<super::UartError> {
        if isr & ISR_ORE != 0 {
            Some(super::UartError::Overrun)
        } else if isr & ISR_FE != 0 {
            Some(super::UartError::Framing)
        } else if isr & ISR_PE != 0 {
            Some(super::UartError::Parity)
        } else if isr & ISR_NE != 0 {
            Some(super::UartError::Noise)
        } else {
            None
        }
    }
}

#[cfg(feature = "cw32f030")]
pub(crate) mod ub {
    use crate::pac::uart as raw;
    /// 发送缓冲空(ISR/IER bit0)
    pub const ISR_TXBE: u32 = raw::isr::txe_MASK;
    /// 发送完成(bit1)
    pub const ISR_TC: u32 = raw::isr::tc_MASK;
    /// 接收完成(bit2)
    pub const ISR_RC: u32 = raw::isr::rc_MASK;
    /// 帧错误(bit3)
    pub const ISR_FE: u32 = raw::isr::fe_MASK;
    /// 校验错(bit4)
    pub const ISR_PE: u32 = raw::isr::pe_MASK;
    /// 发送使能
    pub const CR1_TXEN: u32 = raw::cr1::txen_MASK;
    /// 接收使能
    pub const CR1_RXEN: u32 = raw::cr1::rxen_MASK;
    /// R1W0 全 1(无动作)基准
    pub const ICR_NONE: u32 = 0x1FFF;
    /// 全部接收错误位掩码(经典:FE|PE)
    pub const ERR_ALL: u32 = ISR_FE | ISR_PE;
    /// 接收就绪 + 全部错误位
    pub const RX_ALL: u32 = ISR_RC | ERR_ALL;
    /// 清全部接收错误(R1W0:错误位写 0)
    pub const ICR_CLEAR_RX_ERRORS: u32 = ICR_NONE & !(ISR_FE | ISR_PE);

    /// 错误种类解码(经典 UART 仅 FE/PE)
    pub fn decode_error(isr: u32) -> Option<super::UartError> {
        if isr & ISR_FE != 0 {
            Some(super::UartError::Framing)
        } else if isr & ISR_PE != 0 {
            Some(super::UartError::Parity)
        } else {
            None
        }
    }
}

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
fn instance_of(regs: &'static crate::Uart) -> usize {
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
    regs: &'static crate::Uart,
    idx: usize,
}

/// 异步 UART 发送半部
pub struct UartTx {
    regs: &'static crate::Uart,
    idx: usize,
}

/// 异步 UART 接收半部
pub struct UartRx {
    regs: &'static crate::Uart,
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
        regs: &'static crate::Uart,
        tx: Pin<impl GpioPort>,
        rx: Pin<impl GpioPort>,
        af: u8,
        _irqs: impl Binding<interrupt::UART1> + Binding<interrupt::UART2> + Binding<interrupt::UART3>,
        config: Config,
    ) -> Self {
        Self::new_inner(regs, tx, rx, af, config)
    }

    /// 初始化 UART1（IRQ 绑定类型收窄到 UART1）。
    pub fn new_uart1(
        regs: &'static crate::Uart,
        tx: Pin<impl GpioPort>,
        rx: Pin<impl GpioPort>,
        af: u8,
        _irqs: impl Binding<interrupt::UART1>,
        config: Config,
    ) -> Self {
        Self::new_inner(regs, tx, rx, af, config)
    }

    /// 初始化 UART2（IRQ 绑定类型收窄到 UART2）。
    pub fn new_uart2(
        regs: &'static crate::Uart,
        tx: Pin<impl GpioPort>,
        rx: Pin<impl GpioPort>,
        af: u8,
        _irqs: impl Binding<interrupt::UART2>,
        config: Config,
    ) -> Self {
        Self::new_inner(regs, tx, rx, af, config)
    }

    /// 初始化 UART3（IRQ 绑定类型收窄到 UART3）。
    pub fn new_uart3(
        regs: &'static crate::Uart,
        tx: Pin<impl GpioPort>,
        rx: Pin<impl GpioPort>,
        af: u8,
        _irqs: impl Binding<interrupt::UART3>,
        config: Config,
    ) -> Self {
        Self::new_inner(regs, tx, rx, af, config)
    }

    fn new_inner(
        regs: &'static crate::Uart,
        tx: Pin<impl GpioPort>,
        rx: Pin<impl GpioPort>,
        af: u8,
        config: Config,
    ) -> Self {
        let idx = instance_of(regs);
        // 使能 UART 时钟（KEY 保护写）;两代门控位不同:
        // L012 全在 APBEN1(bit3/4/8);F030:UART1=APBEN2.9、UART2=APBEN1.7、UART3=APBEN1.8
        let sysctrl = unsafe { &*(pac::SYSCTRL_BASE as *const crate::Sysctrl) };
        #[cfg(feature = "cw32l012")]
        {
            let gate = match idx {
                0 => pac::sysctrl::APBEN1_UART1,
                1 => pac::sysctrl::APBEN1_UART2,
                _ => pac::sysctrl::APBEN1_UART3,
            };
            sysctrl.apben1.write(pac::sysctrl::KEY | gate);
        }
        #[cfg(feature = "cw32f030")]
        {
            match idx {
                0 => sysctrl.apben2.write(crate::SYSCTRL_KEY | (1 << 9)),
                1 => sysctrl.apben1.write(crate::SYSCTRL_KEY | (1 << 7)),
                _ => sysctrl.apben1.write(crate::SYSCTRL_KEY | (1 << 8)),
            }
        }

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

        // v0.1 遗漏修复：使能 NVIC 中断线（向量有了但 ISER 没开，
        // 任何 UART 中断都进不来——2026-10-24 移植缓冲驱动时发现）
        match idx {
            0 => crate::interrupt::nvic_enable::<interrupt::UART1>(),
            1 => crate::interrupt::nvic_enable::<interrupt::UART2>(),
            _ => crate::interrupt::nvic_enable::<interrupt::UART3>(),
        }

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
            if isr & (ub::RX_ALL) != 0 {
                Poll::Ready(())
            } else {
                self.regs.ier.modify(|v| v | ub::ISR_RC);
                Poll::Pending
            }
        })
        .await;
        let isr = self.regs.isr.read();
        let err = ub::decode_error(isr);
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

// ================= 缓冲式 UART（设计吸收自 fan2nd/cw32-hal2） =================

/// 缓冲收发容量（字节，收/发各一份，三个实例共用规格）
pub const BUFFER_CAPACITY: usize = 64;

/// 缓冲模式标记（IRQ 派发据此选择缓冲排空或逐字节唤醒路径）
static BUFFERED: [core::sync::atomic::AtomicBool; 3] =
    [const { core::sync::atomic::AtomicBool::new(false) }; 3];

/// 收/发管道（IRQ 生产 RX、排空 TX；任务侧 await 读写）
static RX_PIPE: [Pipe<CriticalSectionRawMutex, BUFFER_CAPACITY>; 3] = [const { Pipe::new() }; 3];
static TX_PIPE: [Pipe<CriticalSectionRawMutex, BUFFER_CAPACITY>; 3] = [const { Pipe::new() }; 3];

/// 发送活跃标记（kick 置位，IRQ 排空后清零）
static TX_ACTIVE: [core::sync::atomic::AtomicBool; 3] =
    [const { core::sync::atomic::AtomicBool::new(false) }; 3];

/// flush 等待者（TC 到点时唤醒）
static FLUSH_WAKER: [AtomicWaker; 3] = [const { AtomicWaker::new() }; 3];

/// 缓冲模式下被丢弃/出错的接收计数（诊断用）
static RX_DROPPED: [core::sync::atomic::AtomicU32; 3] =
    [const { core::sync::atomic::AtomicU32::new(0) }; 3];

/// 缓冲式 UART：后台环形缓冲收发。
///
/// 接收方向 IRQ 常开（含错误位），数据进 RX 管道，多任务并发读；
/// 发送方向写入 TX 管道后 kick 一次，IRQ 排空到 TDR，`flush` 等 TC。
///
/// 设计吸收自 [fan2nd/cw32-hal2](https://github.com/fan2nd/cw32-hal2)
/// （MIT OR Apache-2.0）的 buffered USART：IER 单发屏蔽、R1W0 清标志
/// 纪律、写 TDR 前清 TC 防陈旧标志误触 flush；环形缓冲改用
/// embassy-sync `Pipe` 实现。
pub struct BufferedUart {
    regs: &'static crate::Uart,
    idx: usize,
}

/// 缓冲式发送半部
pub struct BufferedUartTx {
    regs: &'static crate::Uart,
    idx: usize,
}

/// 缓冲式接收半部
pub struct BufferedUartRx {
    idx: usize,
}

impl BufferedUart {
    /// 初始化 UART1 缓冲式驱动（8-N-1）。
    pub fn new_uart1(
        regs: &'static crate::Uart,
        tx: Pin<impl GpioPort>,
        rx: Pin<impl GpioPort>,
        af: u8,
        _irqs: impl Binding<interrupt::UART1>,
        config: Config,
    ) -> Self {
        let _ = Uart::new_inner(regs, tx, rx, af, config);
        Self::finish(regs, 0)
    }

    /// 初始化 UART2 缓冲式驱动。
    pub fn new_uart2(
        regs: &'static crate::Uart,
        tx: Pin<impl GpioPort>,
        rx: Pin<impl GpioPort>,
        af: u8,
        _irqs: impl Binding<interrupt::UART2>,
        config: Config,
    ) -> Self {
        let _ = Uart::new_inner(regs, tx, rx, af, config);
        Self::finish(regs, 1)
    }

    /// 初始化 UART3 缓冲式驱动。
    pub fn new_uart3(
        regs: &'static crate::Uart,
        tx: Pin<impl GpioPort>,
        rx: Pin<impl GpioPort>,
        af: u8,
        _irqs: impl Binding<interrupt::UART3>,
        config: Config,
    ) -> Self {
        let _ = Uart::new_inner(regs, tx, rx, af, config);
        Self::finish(regs, 2)
    }

    /// 缓冲模式收尾：置标记 + 常开接收中断
    fn finish(regs: &'static crate::Uart, idx: usize) -> Self {
        BUFFERED[idx].store(true, core::sync::atomic::Ordering::Relaxed);
        regs.ier.write(ub::RX_ALL);
        Self { regs, idx }
    }

    /// 拆分收发半部（管道是全局静态，两半部可分属不同任务）
    pub fn split(self) -> (BufferedUartTx, BufferedUartRx) {
        (
            BufferedUartTx {
                regs: self.regs,
                idx: self.idx,
            },
            BufferedUartRx { idx: self.idx },
        )
    }

    /// 缓冲区内已有的接收字节数（可非阻塞窥视）
    pub fn rx_ready(&self) -> usize {
        RX_PIPE[self.idx].len()
    }

    /// 被丢弃的接收字节计数（缓冲满或错误，诊断用）
    pub fn rx_dropped(&self) -> u32 {
        RX_DROPPED[self.idx].load(core::sync::atomic::Ordering::Relaxed)
    }
}

impl BufferedUartTx {
    /// 写入发送管道并 kick 后台排空
    pub async fn write(&mut self, buf: &[u8]) {
        TX_PIPE[self.idx].write_all(buf).await;
        self.kick();
    }

    /// kick：置活跃标记并打开 TXBE 中断（若已在排空则无事可做）
    fn kick(&self) {
        critical_section::with(|_| {
            if !TX_ACTIVE[self.idx].load(core::sync::atomic::Ordering::Relaxed) {
                TX_ACTIVE[self.idx].store(true, core::sync::atomic::Ordering::Relaxed);
                self.regs.ier.modify(|v| v | ub::ISR_TXBE);
                // TXBE 可能已置位（管道有货就该马上发），立即排一次
                buffered_tx_drain(self.idx, self.regs);
            }
        });
    }

    /// 等待全部发出（管道空 + TC 置位）
    pub async fn flush(&mut self) {
        poll_fn(|cx| {
            FLUSH_WAKER[self.idx].register(cx.waker());
            if TX_PIPE[self.idx].is_empty() && self.regs.isr.read() & ub::ISR_TC != 0 {
                Poll::Ready(())
            } else {
                self.regs.ier.modify(|v| v | ub::ISR_TC);
                Poll::Pending
            }
        })
        .await;
    }
}

impl BufferedUartRx {
    /// 读至少 1 字节（阻塞到有数据）
    pub async fn read(&mut self, buf: &mut [u8]) -> usize {
        RX_PIPE[self.idx].read(buf).await
    }

    /// 缓冲区内已有的接收字节数
    pub fn ready(&self) -> usize {
        RX_PIPE[self.idx].len()
    }
}

impl embedded_io::ErrorType for BufferedUart {
    type Error = UartError;
}
impl embedded_io::ErrorType for BufferedUartTx {
    type Error = UartError;
}
impl embedded_io::ErrorType for BufferedUartRx {
    type Error = core::convert::Infallible;
}

impl embedded_io_async::Write for BufferedUart {
    async fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        TX_PIPE[self.idx].write_all(buf).await;
        critical_section::with(|_| {
            if !TX_ACTIVE[self.idx].load(core::sync::atomic::Ordering::Relaxed) {
                TX_ACTIVE[self.idx].store(true, core::sync::atomic::Ordering::Relaxed);
                self.regs.ier.modify(|v| v | ub::ISR_TXBE);
                buffered_tx_drain(self.idx, self.regs);
            }
        });
        Ok(buf.len())
    }
    async fn flush(&mut self) -> Result<(), Self::Error> {
        BufferedUartTx {
            regs: self.regs,
            idx: self.idx,
        }
        .flush()
        .await;
        Ok(())
    }
}

impl embedded_io_async::Write for BufferedUartTx {
    async fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        BufferedUartTx::write(self, buf).await;
        Ok(buf.len())
    }
    async fn flush(&mut self) -> Result<(), Self::Error> {
        BufferedUartTx::flush(self).await;
        Ok(())
    }
}

impl embedded_io_async::Read for BufferedUart {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        if buf.is_empty() {
            return Ok(0);
        }
        RX_PIPE[self.idx].read(buf).await;
        Ok(1)
    }
}

impl embedded_io_async::Read for BufferedUartRx {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        if buf.is_empty() {
            return Ok(0);
        }
        RX_PIPE[self.idx].read(buf).await;
        Ok(1)
    }
}

/// 缓冲模式 IRQ：RX 排空入管道 / TX 排空管道入 TDR / TC 唤醒 flush。
/// 临界区内调用（kick 与本函数共用一套状态）。
fn buffered_irq(idx: usize, regs: &'static crate::Uart) {
    // RX：排空 RDR（读 RDR 清 RC；错误位经 ICR R1W0 清）
    loop {
        let isr = regs.isr.read();
        if isr & ub::ISR_RC != 0 {
            let b = regs.rdr.read() as u8;
            let one = [b];
            if RX_PIPE[idx].try_write(&one).is_err() {
                {
                    // v6m 无 fetch_add（无 CAS）；本函数全程临界区，load+store 安全
                    let d = RX_DROPPED[idx].load(core::sync::atomic::Ordering::Relaxed);
                    RX_DROPPED[idx].store(d + 1, core::sync::atomic::Ordering::Relaxed);
                }
            }
        } else {
            if isr & (ub::ERR_ALL) != 0 {
                regs.icr.write(ub::ICR_CLEAR_RX_ERRORS);
            }
            break;
        }
    }
    buffered_tx_drain(idx, regs);

    // TC：发送完成 → 单发屏蔽并唤醒 flush 等待者
    let ier = regs.ier.read();
    if ier & ub::ISR_TC != 0 && regs.isr.read() & ub::ISR_TC != 0 {
        regs.ier.modify(|v| v & !ub::ISR_TC);
        FLUSH_WAKER[idx].wake();
    }
}

/// TX 排空：管道有货且 TXBE 时写 TDR（写前清 TC，防陈旧标志把
/// flush 提前唤醒——纪律吸收自 cw32-hal2 的 transmit()）。
fn buffered_tx_drain(idx: usize, regs: &'static crate::Uart) {
    let mut byte = [0u8; 1];
    while TX_ACTIVE[idx].load(core::sync::atomic::Ordering::Relaxed)
        && regs.isr.read() & ub::ISR_TXBE != 0
        && TX_PIPE[idx]
            .try_read(&mut byte)
            .map(|n| n > 0)
            .unwrap_or(false)
    {
        regs.icr.write(ub::ICR_NONE & !ub::ISR_TC); // R1W0：清 TC
        regs.tdr.write(byte[0] as u32);
    }
    if TX_PIPE[idx].is_empty() {
        // 排空完毕：关 TXBE、清活跃标记（TC 稍后到点唤醒 flush）
        regs.ier.modify(|v| v & !ub::ISR_TXBE);
        TX_ACTIVE[idx].store(false, core::sync::atomic::Ordering::Relaxed);
    }
}

// ================= 中断派发 =================

/// 单实例中断处理：缓冲模式走管道排空，逐字节模式唤醒等待者。
fn on_irq(idx: usize, regs: &'static crate::Uart) {
    if BUFFERED[idx].load(core::sync::atomic::Ordering::Relaxed) {
        critical_section::with(|_| buffered_irq(idx, regs));
        return;
    }
    let isr = regs.isr.read();
    if isr & (ub::RX_ALL) != 0 {
        RX_WAKER[idx].wake();
    }
    if isr & (ub::ISR_TXBE | ub::ISR_TC) != 0 {
        TX_WAKER[idx].wake();
    }
    if isr & (ub::ERR_ALL) != 0 {
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
        on_irq(0, unsafe { &*(pac::UART1_BASE as *const crate::Uart) });
    }
}

/// UART2 中断标记（IRQ28）
pub struct Uart2Irq;
impl IrqHandler<interrupt::UART2> for Uart2Irq {
    fn call() {
        on_irq(1, unsafe { &*(pac::UART2_BASE as *const crate::Uart) });
    }
}

/// UART3 中断标记（IRQ29）
pub struct Uart3Irq;
impl IrqHandler<interrupt::UART3> for Uart3Irq {
    fn call() {
        on_irq(2, unsafe { &*(pac::UART3_BASE as *const crate::Uart) });
    }
}
