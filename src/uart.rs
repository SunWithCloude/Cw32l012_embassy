//! 异步 UART（中断驱动，逐字节收发；`embedded-io-async` 接口）。
//!
//! 发送等待 TXBE（发送缓冲空）、接收等待 RC（接收完成）。等待时才打开
//! 对应 IER 位，避免空闲时中断风暴。
//!
//! 引脚组合（摘要；L012 来自其 UM 表 9-2，l083 来自数据手册表 5-3..5-8
//! 摘要，全表见 cw32-hal2 数据仓库 `af/cw32l083-serial.yaml`）：
//!
//! | UART  | 常用组合（L012）               |
//! |-------|--------------------------------|
//! | UART1 | PA02=TXD / PA03=RXD（AF1）     |
//! | UART2 | PA07=TXD / PA06=RXD（AF1）     |
//! | UART3 | PA04=TXD / PA05=RXD（AF1）     |
//!
//! l083 六实例常用组合（TX/RX 同组同 AF，厂商头 `PAxx_AFx_UARTxTXD()` 宏）：
//!
//! | UART  | 常用组合（l083）                                                                                                             |
//! |-------|------------------------------------------------------------------------------------------------------------------------------|
//! | UART1 | PA08/PA09（AF2）、PC04/PC05（AF1）、PC02/PC03（AF5）、PD08/PD09（AF3）、PE08/PE09（AF1）、PF04/PF05（AF1）、PA14/PA13（AF3） |
//! | UART2 | PA06/PA07（AF2）、PB03/PB04（AF4）、PD14/PD15（AF1）、PE14/PE15（AF1）、PF09/PF10（AF1）、PA13/PA14（AF4）                    |
//! | UART3 | PA02/PA03（AF1）、PA09/PA10（AF1）、PA14/PA15（AF1）、PC06/PC07（AF2）、PD05/PD06（AF3）、PE12/PE13（AF1）、PF10/PF09（AF2） |
//! | UART4 | PB11/PB10（AF2）、PC03/PC02（AF1）、PC07/PC06（AF1）、PC10/PC11（AF1）、PC14/PC15（AF4）、PD13/PD12（AF2）、PE10/PE11（AF1） |
//! | UART5 | PB01/PB00（AF1）、PB08/PB09（AF3）、PC01/PC00（AF5）、PD10/PD11（AF1）、PE04/PE03（AF2）、PF05/PF04（AF4）                    |
//! | UART6 | PB06/PB07（AF1）、PC08/PC09（AF2）、PC05/PC04（AF2）、PD01/PD00（AF1）、PE06/PE05（AF3）、PA02/PA03（AF2）                    |
//!
//! # l083 中断共享向量
//!
//! l083 的 UART 两两共享一条 NVIC 线：UART1/UART4 = IRQ27（typelevel 标记
//! `UART1_UART4`）、UART2/UART5 = IRQ28（`UART2_UART5`）、UART3/UART6 =
//! IRQ29（`UART3_UART6`）。`bind_interrupts!` 绑定共享向量后，派发标记
//! （[`Uart1Uart4Irq`] 等）会依次检查组内两个实例——未使用的实例时钟未开、
//! 无中断标志，检查即刻返回（与 interrupt.rs 文档 SPI2/3 共享 `SPI23` 的
//! 做法一致）。
//!
//! # DMA 收发（L012 与经典组，单次全量搬运；语义级实现待实板验证）
//!
//! [`Uart::write_dma`]/[`Uart::read_dma`] 用 [`crate::dma`] 的通道抽象一次
//! 搬完整段缓冲：发送 mem→TDR、接收 RDR→mem，硬件触发 UARTx_TX/UARTx_RX
//! （表 8-8）。**DMA 模式与逐字节/Buffered 管道模式互斥**——同一实例同时
//! 只能用一种传输方式（三套方式的 IER/CR2 状态机互不感知，混用前先让
//! 另一方式收尾）。
//!
//! 两代位面：L012 的 DMA 请求使能在 CR2[7]/CR2[6]（厂商库
//! `UART_DMAReq_Tx/Rx`）；经典组（f030/f020/l031/l052/l083/r031/w031）
//! **同位布局**——f030 厂商库 `USART_DMAReq_Tx=0x0080`/`USART_DMAReq_Rx=
//! 0x0040` 同写 CR2（`USART_DMACmd`），其余家族厂商头
//! `UARTx_CR2_DMATX_Msk=0x80`/`DMARX_Msk=0x40` 同值（逐家抽验，见
//! [`Uart::write_dma`] 文档），触发源取 [`crate::dma::hs::UART1_TX`] 等
//! （全经典组 §8.8.4 请求表同段同值）。
//!
//! ⚠ 诚实标注：阻塞版 cw32l012-hal 没有 UART/SPI 的 DMA 实机配方，本实现
//! 按 PAC `dma::hs` 触发表（表 8-8）+ 厂商库 `UART_TransmitDMA`/
//! `UART_ReceiveDMA`/`UART_DMACmd`（cw32l012_uart.c/.h；f030 同名
//! `USART_DMACmd`）的位面与次序编写，**语义级实现待实板验证**。
//!
//! l083 范围：`write_dma`/`read_dma` 六实例全接线（UART1..6）——触发源
//! 按手册 §8.8.4 请求表：UART1..3 取 0x00..0x05（全经典组同值）、UART4..6
//! 取 0x2B..0x30（**仅 l083 手册表有 UART4-6 行**，l083=CN_V2.0 p147-149），
//! 与 [`crate::dma::hs::UART4_RX`] 等常量一一对应；UART2/3 实例仅 f030/l083
//! 有（其余经典家族单实例只走 UART1），调用 `new_uart2..6` 构造即用。

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
// 全家族共用位:TXBE=0 / TC=1 / RC=2。错误位两代不同:
// L012 代组(L010/L011/L012) FE8/PE9/NE10/ORE11;经典组(F030/F020/F003)
// FE3/PE4 且无 NE/ORE(逐 PAC 抽验:F020/F003 的 isr::fe_POS=3/pe_POS=4 同
// f030)。ICR 两代均为 R1W0(写 0 清除):L012 依据 UM V1.4;经典组依据厂商库
// cw32f030_uart.c 的 REGBITS_CLR(ICR, flag)。
// L012 用手写 PAC 的 ISR_* 常量名;生成 PAC 是小写字段 _MASK 风格,
// 在此适配成统一名(寄存器布局与各家族手册同源)。
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

#[cfg(any(feature = "cw32l010", feature = "cw32l011"))]
pub(crate) mod ub {
    use crate::pac::uart as raw;
    // 生成 PAC 的位常量按寄存器嵌套(uart::isr::txe_MASK 风格)
    /// 发送缓冲空(ISR/IER bit0)
    pub const ISR_TXBE: u32 = raw::isr::txe_MASK;
    /// 发送完成(bit1)
    pub const ISR_TC: u32 = raw::isr::tc_MASK;
    /// 接收完成(bit2)
    pub const ISR_RC: u32 = raw::isr::rc_MASK;
    /// 帧错误(bit8)
    pub const ISR_FE: u32 = raw::isr::fe_MASK;
    /// 校验错(bit9)
    pub const ISR_PE: u32 = raw::isr::pe_MASK;
    /// 噪声(bit10)
    pub const ISR_NE: u32 = raw::isr::ne_MASK;
    /// 溢出(bit11)
    pub const ISR_ORE: u32 = raw::isr::ore_MASK;
    /// 发送使能
    pub const CR1_TXEN: u32 = raw::cr1::txen_MASK;
    /// 接收使能
    pub const CR1_RXEN: u32 = raw::cr1::rxen_MASK;
    /// R1W0 全 1(无动作)基准 = ICR 已定义位并集
    /// (tc..rxmatch,bit1..12;ICR 无 TXE 位)
    pub const ICR_NONE: u32 = 0x1FFC;
    /// 全部接收错误位掩码(ORE|NE|PE|FE)
    pub const ERR_ALL: u32 = ISR_ORE | ISR_NE | ISR_PE | ISR_FE;
    /// 接收就绪 + 全部错误位
    pub const RX_ALL: u32 = ISR_RC | ERR_ALL;
    /// 清全部接收错误(R1W0:错误位写 0)
    pub const ICR_CLEAR_RX_ERRORS: u32 = ICR_NONE & !ERR_ALL;

    /// 错误种类解码(优先级同 L012:ORE > FE > PE > NE)
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

// 经典组(F030/F020/F003/L031/L052/L083/R031/W031 逐个接入;位面逐 PAC 核对
// 一致:ISR TXE0/TC1/RC2/FE3/PE4,无 NE/ORE;ICR 为 R1W0,已定义位至 bit11——
// l083 例外:ICR 已定义位仅至 bit6(CTS),ICR_NONE 多写的 1 落在保留位
// (R1W0 无动作),CTS 清零无副作用——驱动不用硬件流控)
#[cfg(any(
    feature = "cw32f030",
    feature = "cw32l031",
    feature = "cw32l052",
    feature = "cw32l083",
    feature = "cw32r031",
    feature = "cw32w031",
    feature = "cw32f020",
    feature = "cw32f003"
))]
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
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
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
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
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

/// 实例索引（0..3 = UART1..UART3；l083 为 0..6 = UART1..UART6，按基址 match）
fn instance_of(regs: &'static crate::Uart) -> usize {
    // L012/F030 三实例;l083 六实例;L010/L011/F003/F020 单实例(PAC 无
    // UART2/3 基址常量——F003 硬件有 UART2@0x40004400 但生成 PAC 缺实例,
    // 仅接 UART1)
    match regs as *const _ as usize {
        pac::UART1_BASE => 0,
        #[cfg(any(feature = "cw32l012", feature = "cw32f030", feature = "cw32l083"))]
        pac::UART2_BASE => 1,
        #[cfg(any(feature = "cw32l012", feature = "cw32f030", feature = "cw32l083"))]
        pac::UART3_BASE => 2,
        #[cfg(feature = "cw32l083")]
        pac::UART4_BASE => 3,
        #[cfg(feature = "cw32l083")]
        pac::UART5_BASE => 4,
        #[cfg(feature = "cw32l083")]
        pac::UART6_BASE => 5,
        _ => unreachable!("非 UART 寄存器块"),
    }
}

/// 本家族 UART 实例总数（l083 六实例，其余 0..3）
#[cfg(feature = "cw32l083")]
const INSTANCE_COUNT: usize = 6;
#[cfg(not(feature = "cw32l083"))]
const INSTANCE_COUNT: usize = 3;

/// per-实例 waker（0=TX 可写，1=RX 可读）
static TX_WAKER: [AtomicWaker; INSTANCE_COUNT] = [const { AtomicWaker::new() }; INSTANCE_COUNT];
static RX_WAKER: [AtomicWaker; INSTANCE_COUNT] = [const { AtomicWaker::new() }; INSTANCE_COUNT];

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
    #[cfg(any(feature = "cw32l012", feature = "cw32f030"))]
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

    /// 初始化 UART1（单实例家族与经典代 l031 组的入口；L010/L011/
    /// L031 组/F003/F020 的 PAC 只有 UART1_BASE，UART2/3 仅有中断号
    /// 无硬件——F003 的 UART2 硅片存在但生成 PAC 缺实例，待 PAC 补）。
    #[cfg(any(
        feature = "cw32l010",
        feature = "cw32l011",
        feature = "cw32l031",
        feature = "cw32l052",
        feature = "cw32r031",
        feature = "cw32w031",
        feature = "cw32f020",
        feature = "cw32f003"
    ))]
    pub fn new(
        regs: &'static crate::Uart,
        tx: Pin<impl GpioPort>,
        rx: Pin<impl GpioPort>,
        af: u8,
        _irqs: impl Binding<interrupt::UART1>,
        config: Config,
    ) -> Self {
        Self::new_inner(regs, tx, rx, af, config)
    }

    /// 初始化 UART1（l083 入口；UART1..UART6 的 PAC 寄存器块均已收编，
    /// UART2..6 分别用 `new_uart2`..`new_uart6`。中断与 UART4 共享向量，
    /// typelevel 标记为 `UART1_UART4`（IRQ27）——`bind_interrupts!` 侧
    /// 写作 `UART1_UART4 => uart::Uart1Uart4Irq;`）。
    #[cfg(feature = "cw32l083")]
    pub fn new(
        regs: &'static crate::Uart,
        tx: Pin<impl GpioPort>,
        rx: Pin<impl GpioPort>,
        af: u8,
        _irqs: impl Binding<interrupt::UART1_UART4>,
        config: Config,
    ) -> Self {
        Self::new_inner(regs, tx, rx, af, config)
    }

    /// 初始化 UART1（IRQ 绑定类型收窄到 UART1）。
    #[cfg(not(feature = "cw32l083"))]
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

    /// 初始化 UART1（l083：IRQ 绑定类型为共享向量 UART1_UART4）。
    #[cfg(feature = "cw32l083")]
    pub fn new_uart1(
        regs: &'static crate::Uart,
        tx: Pin<impl GpioPort>,
        rx: Pin<impl GpioPort>,
        af: u8,
        _irqs: impl Binding<interrupt::UART1_UART4>,
        config: Config,
    ) -> Self {
        Self::new_inner(regs, tx, rx, af, config)
    }

    /// 初始化 UART2（l083：与 UART5 共享向量 UART2_UART5，IRQ28）。
    #[cfg(feature = "cw32l083")]
    pub fn new_uart2(
        regs: &'static crate::Uart,
        tx: Pin<impl GpioPort>,
        rx: Pin<impl GpioPort>,
        af: u8,
        _irqs: impl Binding<interrupt::UART2_UART5>,
        config: Config,
    ) -> Self {
        Self::new_inner(regs, tx, rx, af, config)
    }

    /// 初始化 UART3（l083：与 UART6 共享向量 UART3_UART6，IRQ29）。
    #[cfg(feature = "cw32l083")]
    pub fn new_uart3(
        regs: &'static crate::Uart,
        tx: Pin<impl GpioPort>,
        rx: Pin<impl GpioPort>,
        af: u8,
        _irqs: impl Binding<interrupt::UART3_UART6>,
        config: Config,
    ) -> Self {
        Self::new_inner(regs, tx, rx, af, config)
    }

    /// 初始化 UART4（l083：与 UART1 共享向量 UART1_UART4，IRQ27）。
    #[cfg(feature = "cw32l083")]
    pub fn new_uart4(
        regs: &'static crate::Uart,
        tx: Pin<impl GpioPort>,
        rx: Pin<impl GpioPort>,
        af: u8,
        _irqs: impl Binding<interrupt::UART1_UART4>,
        config: Config,
    ) -> Self {
        Self::new_inner(regs, tx, rx, af, config)
    }

    /// 初始化 UART5（l083：与 UART2 共享向量 UART2_UART5，IRQ28）。
    #[cfg(feature = "cw32l083")]
    pub fn new_uart5(
        regs: &'static crate::Uart,
        tx: Pin<impl GpioPort>,
        rx: Pin<impl GpioPort>,
        af: u8,
        _irqs: impl Binding<interrupt::UART2_UART5>,
        config: Config,
    ) -> Self {
        Self::new_inner(regs, tx, rx, af, config)
    }

    /// 初始化 UART6（l083：与 UART3 共享向量 UART3_UART6，IRQ29）。
    #[cfg(feature = "cw32l083")]
    pub fn new_uart6(
        regs: &'static crate::Uart,
        tx: Pin<impl GpioPort>,
        rx: Pin<impl GpioPort>,
        af: u8,
        _irqs: impl Binding<interrupt::UART3_UART6>,
        config: Config,
    ) -> Self {
        Self::new_inner(regs, tx, rx, af, config)
    }

    #[cfg(any(feature = "cw32l012", feature = "cw32f030"))]
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

    #[cfg(any(feature = "cw32l012", feature = "cw32f030"))]
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
        // 使能 UART 时钟;两代门控位不同:
        // L012 全在 APBEN1(bit3/4/8,带 KEY);经典组:UART1=APBEN2.9、
        // UART2=APBEN1.7、UART3=APBEN1.8(F030 实测布局;F020/F003 的
        // PAC apben2::uart1/apben1::uart2 同位抽验一致),无 KEY 读改写
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
        #[cfg(any(feature = "cw32l010", feature = "cw32l011"))]
        {
            assert!(idx == 0, "L010/L011 仅 UART1 实例");
            // uart1en = APBEN1.3（l011 SDK 头 cw32l011.h:6030
            // SYSCTRL_APBEN1_UART1_Pos=3，l010 头 cw32l010.h:5670 同位）。
            // KEY 协议查证（2026-11-14）：这两家族的 SYSCTRL **没有独立
            // KEY 寄存器**——KEY 内嵌于各受保护寄存器的 [31:16]
            // （cw32l011.h:6018-6019 SYSCTRL_APBEN1_KEY_Pos=16；cw32l010.h
            // :5662-5663 同；PAC SYSCTRL 结构体首字段是 CR0 而非 KEY，
            // 独立 KEY@0x00 的是 RTC），与 L012 代组同协议。
            // 读改写 + KEY 一并写入：保留同寄存器其它使能位（ADC/VC/SPI1/
            // UART2/ATIM/GTIM1/GTIM2——与 PWM 门控共存，整字写会顶掉）
            sysctrl
                .apben1
                .modify(|v| crate::SYSCTRL_KEY | (v & 0xFFFF) | (1 << 3));
        }
        // 经典代单 UART 家族(l031 组):uart1en = APBEN2.9,无 KEY,读改写
        #[cfg(any(
            feature = "cw32l031",
            feature = "cw32l052",
            feature = "cw32r031",
            feature = "cw32w031"
        ))]
        {
            assert!(idx == 0, "该家族仅 UART1 实例");
            sysctrl.apben2.modify(|v| v | (1 << 9));
        }
        // l083 六实例门控查表(PAC apben1/apben2 位域):UART1=APBEN2.9、
        // UART2=APBEN1.7、UART3=APBEN1.8、UART4=APBEN1.9、UART5=APBEN1.10、
        // UART6=APBEN2.1。APBEN 无 KEY 保护,读改写防顶掉其它使能位
        #[cfg(feature = "cw32l083")]
        match idx {
            0 => sysctrl.apben2.modify(|v| v | (1 << 9)),
            1 => sysctrl.apben1.modify(|v| v | (1 << 7)),
            2 => sysctrl.apben1.modify(|v| v | (1 << 8)),
            3 => sysctrl.apben1.modify(|v| v | (1 << 9)),
            4 => sysctrl.apben1.modify(|v| v | (1 << 10)),
            _ => sysctrl.apben2.modify(|v| v | (1 << 1)), // UART6
        }
        #[cfg(any(feature = "cw32f030", feature = "cw32f020", feature = "cw32f003"))]
        {
            // f030:UART1=APBEN2.9、UART2=APBEN1.7、UART3=APBEN1.8(F030 PAC
            // apben2::uart1/apben1::uart2/uart3 位域);F020/F003 单实例(PAC
            // 无 UART2/3 基址常量,instance_of 只会给 0)。APBEN 无 KEY 保护,
            // 读改写防顶掉其它使能位
            match idx {
                0 => sysctrl.apben2.modify(|v| v | (1 << 9)),
                1 => sysctrl.apben1.modify(|v| v | (1 << 7)),
                _ => sysctrl.apben1.modify(|v| v | (1 << 8)),
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
        #[cfg(any(
            feature = "cw32l010",
            feature = "cw32l011",
            feature = "cw32l031",
            feature = "cw32l052",
            feature = "cw32r031",
            feature = "cw32w031",
            feature = "cw32f020",
            feature = "cw32f003"
        ))]
        crate::interrupt::nvic_enable::<interrupt::UART1>();
        // l083 六实例,向量按组共享:UART1/4=UART1_UART4(IRQ27)、
        // UART2/5=UART2_UART5(IRQ28)、UART3/6=UART3_UART6(IRQ29)
        #[cfg(feature = "cw32l083")]
        match idx {
            0 | 3 => crate::interrupt::nvic_enable::<interrupt::UART1_UART4>(),
            1 | 4 => crate::interrupt::nvic_enable::<interrupt::UART2_UART5>(),
            _ => crate::interrupt::nvic_enable::<interrupt::UART3_UART6>(),
        }
        #[cfg(any(feature = "cw32l012", feature = "cw32f030"))]
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

/// 缓冲收发容量（字节，收/发各一份，全部实例共用规格）
pub const BUFFER_CAPACITY: usize = 64;

/// 缓冲模式标记（IRQ 派发据此选择缓冲排空或逐字节唤醒路径）
static BUFFERED: [core::sync::atomic::AtomicBool; INSTANCE_COUNT] =
    [const { core::sync::atomic::AtomicBool::new(false) }; INSTANCE_COUNT];

/// 收/发管道（IRQ 生产 RX、排空 TX；任务侧 await 读写）
static RX_PIPE: [Pipe<CriticalSectionRawMutex, BUFFER_CAPACITY>; INSTANCE_COUNT] =
    [const { Pipe::new() }; INSTANCE_COUNT];
static TX_PIPE: [Pipe<CriticalSectionRawMutex, BUFFER_CAPACITY>; INSTANCE_COUNT] =
    [const { Pipe::new() }; INSTANCE_COUNT];

/// 发送活跃标记（kick 置位，IRQ 排空后清零）
static TX_ACTIVE: [core::sync::atomic::AtomicBool; INSTANCE_COUNT] =
    [const { core::sync::atomic::AtomicBool::new(false) }; INSTANCE_COUNT];

/// flush 等待者（TC 到点时唤醒）
static FLUSH_WAKER: [AtomicWaker; INSTANCE_COUNT] = [const { AtomicWaker::new() }; INSTANCE_COUNT];

/// 缓冲模式下被丢弃/出错的接收计数（诊断用）
static RX_DROPPED: [core::sync::atomic::AtomicU32; INSTANCE_COUNT] =
    [const { core::sync::atomic::AtomicU32::new(0) }; INSTANCE_COUNT];

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
    #[cfg(not(feature = "cw32l083"))]
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

    /// 初始化 UART1 缓冲式驱动（8-N-1；l083：绑定共享向量 UART1_UART4）。
    #[cfg(feature = "cw32l083")]
    pub fn new_uart1(
        regs: &'static crate::Uart,
        tx: Pin<impl GpioPort>,
        rx: Pin<impl GpioPort>,
        af: u8,
        _irqs: impl Binding<interrupt::UART1_UART4>,
        config: Config,
    ) -> Self {
        let _ = Uart::new_inner(regs, tx, rx, af, config);
        Self::finish(regs, 0)
    }

    /// 初始化 UART2 缓冲式驱动（8-N-1；l083：绑定共享向量 UART2_UART5）。
    #[cfg(feature = "cw32l083")]
    pub fn new_uart2(
        regs: &'static crate::Uart,
        tx: Pin<impl GpioPort>,
        rx: Pin<impl GpioPort>,
        af: u8,
        _irqs: impl Binding<interrupt::UART2_UART5>,
        config: Config,
    ) -> Self {
        let _ = Uart::new_inner(regs, tx, rx, af, config);
        Self::finish(regs, 1)
    }

    /// 初始化 UART3 缓冲式驱动（8-N-1；l083：绑定共享向量 UART3_UART6）。
    #[cfg(feature = "cw32l083")]
    pub fn new_uart3(
        regs: &'static crate::Uart,
        tx: Pin<impl GpioPort>,
        rx: Pin<impl GpioPort>,
        af: u8,
        _irqs: impl Binding<interrupt::UART3_UART6>,
        config: Config,
    ) -> Self {
        let _ = Uart::new_inner(regs, tx, rx, af, config);
        Self::finish(regs, 2)
    }

    /// 初始化 UART4 缓冲式驱动（8-N-1；l083：绑定共享向量 UART1_UART4）。
    #[cfg(feature = "cw32l083")]
    pub fn new_uart4(
        regs: &'static crate::Uart,
        tx: Pin<impl GpioPort>,
        rx: Pin<impl GpioPort>,
        af: u8,
        _irqs: impl Binding<interrupt::UART1_UART4>,
        config: Config,
    ) -> Self {
        let _ = Uart::new_inner(regs, tx, rx, af, config);
        Self::finish(regs, 3)
    }

    /// 初始化 UART5 缓冲式驱动（8-N-1；l083：绑定共享向量 UART2_UART5）。
    #[cfg(feature = "cw32l083")]
    pub fn new_uart5(
        regs: &'static crate::Uart,
        tx: Pin<impl GpioPort>,
        rx: Pin<impl GpioPort>,
        af: u8,
        _irqs: impl Binding<interrupt::UART2_UART5>,
        config: Config,
    ) -> Self {
        let _ = Uart::new_inner(regs, tx, rx, af, config);
        Self::finish(regs, 4)
    }

    /// 初始化 UART6 缓冲式驱动（8-N-1；l083：绑定共享向量 UART3_UART6）。
    #[cfg(feature = "cw32l083")]
    pub fn new_uart6(
        regs: &'static crate::Uart,
        tx: Pin<impl GpioPort>,
        rx: Pin<impl GpioPort>,
        af: u8,
        _irqs: impl Binding<interrupt::UART3_UART6>,
        config: Config,
    ) -> Self {
        let _ = Uart::new_inner(regs, tx, rx, af, config);
        Self::finish(regs, 5)
    }

    #[cfg(any(feature = "cw32l012", feature = "cw32f030"))]
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

    #[cfg(any(feature = "cw32l012", feature = "cw32f030"))]
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

// ================= DMA 收发（L012，单次全量搬运；语义级实现待实板验证） =================
// 位面与次序按厂商库 cw32l012_uart.c 编写（阻塞版 HAL 无 UART DMA 配方）：
// 先配 DMA 通道（EN=1）再开外设侧 DMA 请求，TC 后关请求——与厂商
// UART_TransmitDMA/UART_ReceiveDMA 的顺序一致。

/// UART DMA 请求使能位与触发源选择（L012）
///
/// PAC 位定义未收编 UART 的 DMA 使能位——按厂商库补录：cw32l012_uart.h
/// `UART_DMAReq_Tx = 0x0080`（CR2[7]）/ `UART_DMAReq_Rx = 0x0040`（CR2[6]），
/// `UART_DMACmd()` 把它们写进 `UARTx->CR2`（手册 §21 UART 寄存器组）。
#[cfg(feature = "cw32l012")]
mod udma {
    /// DMA 发送请求使能（CR2[7]，厂商库 UART_DMAReq_Tx）
    pub const CR2_DMATX: u32 = 1 << 7;
    /// DMA 接收请求使能（CR2[6]，厂商库 UART_DMAReq_Rx）
    pub const CR2_DMARX: u32 = 1 << 6;

    /// 发送方向硬件触发源（表 8-8 / 厂商库 DMA_TRIGGER_SRC_UARTx_TX）
    pub fn tx_hs(idx: usize) -> u8 {
        match idx {
            0 => crate::pac::dma::hs::UART1_TX,
            1 => crate::pac::dma::hs::UART2_TX,
            _ => crate::pac::dma::hs::UART3_TX,
        }
    }

    /// 接收方向硬件触发源（表 8-8 / 厂商库 DMA_TRIGGER_SRC_UARTx_RX）
    pub fn rx_hs(idx: usize) -> u8 {
        match idx {
            0 => crate::pac::dma::hs::UART1_RX,
            1 => crate::pac::dma::hs::UART2_RX,
            _ => crate::pac::dma::hs::UART3_RX,
        }
    }
}

#[cfg(feature = "cw32l012")]
impl Uart {
    /// DMA 发送：整段 buf 搬进 TDR（单次全量搬运，占用 DMA 通道 2）
    ///
    /// 触发源 UARTx_TX（TXBE"发送缓冲空"事件，表 8-8）；mem→periph：
    /// 源自增、TDR 固定、8bit、BLOCK——厂商库 UART_TransmitDMA 同款位面。
    /// 次序：通道 `configure`（EN=1）→ 置 `CR2.DMATX`（空闲时 TXBE 已
    /// 置位，首个请求立即到）→ `wait` 等 DMA TC → 关 `CR2.DMATX`。
    ///
    /// DMA TC = 全部字节已写入 TDR；此时行上最后一字节可能仍在移位，
    /// 要等物理发完再收尾（如半双工转向）的调用方随后
    /// `embedded_io_async::Write::flush`（等 ISR.TC）。
    ///
    /// **与逐字节/Buffered 模式互斥**（见模块文档）；空缓冲或 >65535
    /// 字节返回 [`crate::dma::DmaError::BadLength`]。⚠ 语义级实现待实板
    /// 验证。
    pub async fn write_dma(
        &mut self,
        dma: &mut crate::dma::Dma,
        buf: &[u8],
    ) -> Result<(), crate::dma::DmaError> {
        if buf.is_empty() || buf.len() > 0xFFFF {
            return Err(crate::dma::DmaError::BadLength);
        }
        let cfg = crate::dma::Config::mem_to_periph(
            buf.as_ptr() as u32,
            core::ptr::addr_of!(self.regs.tdr) as u32,
            buf.len() as u16,
            crate::dma::Width::B8,
            udma::tx_hs(self.idx),
        );
        let mut ch = dma.channel(2);
        ch.configure(&cfg); // EN=1，等 TXBE 请求
        self.regs.cr2.modify(|v| v | udma::CR2_DMATX);
        let r = ch.wait().await;
        self.regs.cr2.modify(|v| v & !udma::CR2_DMATX);
        ch.stop(); // 收尾同 ADC 序列 DMA：通道 EN 清零防残留请求
        r
    }

    /// DMA 接收：恰好收满 buf.len() 字节（单次全量搬运，占用 DMA 通道 1）
    ///
    /// 触发源 UARTx_RX（RC"接收完成"事件，表 8-8）；periph→mem：RDR 固定、
    /// 目的自增、8bit、BLOCK——厂商库 UART_ReceiveDMA 同款位面。启动前清
    /// 残留 RC 与错误标志（R1W0：对应位写 0），保证触发请求从干净电平
    /// 开始——启动前已到达但未被读走的字节会被丢弃。
    ///
    /// DMA 路径不逐字节上报接收错误（FE/PE/NE/ORE 只置 ISR，不影响搬运）；
    /// 需要错误感知的收流用逐字节 `read_byte`/`Read::read`。
    ///
    /// **与逐字节/Buffered 模式互斥**（见模块文档）；空缓冲或 >65535
    /// 字节返回 [`crate::dma::DmaError::BadLength`]。⚠ 语义级实现待实板
    /// 验证。
    pub async fn read_dma(
        &mut self,
        dma: &mut crate::dma::Dma,
        buf: &mut [u8],
    ) -> Result<(), crate::dma::DmaError> {
        if buf.is_empty() || buf.len() > 0xFFFF {
            return Err(crate::dma::DmaError::BadLength);
        }
        // 清残留 RC + 错误位（R1W0：写 0 清除，其余位写 1 无动作）
        let isr_clear = ub::ICR_NONE & !(ub::ISR_RC | ub::ERR_ALL);
        self.regs.icr.write(isr_clear);
        let cfg = crate::dma::Config::periph_to_mem(
            core::ptr::addr_of!(self.regs.rdr) as u32,
            buf.as_mut_ptr() as u32,
            buf.len() as u16,
            crate::dma::Width::B8,
            udma::rx_hs(self.idx),
        );
        let mut ch = dma.channel(1);
        ch.configure(&cfg); // EN=1，等 RC 请求
        self.regs.cr2.modify(|v| v | udma::CR2_DMARX);
        let r = ch.wait().await;
        self.regs.cr2.modify(|v| v & !udma::CR2_DMARX);
        ch.stop();
        r
    }
}

// ================= DMA 收发（经典组，单次全量搬运；语义级实现待实板验证） =================
// 位面与次序按厂商库编写：cw32f030_uart.h:188-189 USART_DMAReq_Tx=0x0080
// （CR2[7]）/ USART_DMAReq_Rx=0x0040（CR2[6]），cw32f030_uart.c:462-467
// `USART_DMACmd` 写 UARTx->CR2——与 L012 同位布局，且经典组生成 PAC 已收编
// 位常量（uart::cr2::dmatx/dmarx_MASK；其余家族厂商头逐家抽验同值：
// cw32f020.h:7032-7035、cw32l031.h:6001-6004、cw32l052.h:8047-8050、
// cw32l083.h:8663-8666、cw32r031.h:6001-6004、cw32w031.h:6353-6356）。
// 触发源 dma::hs::UARTx_RX/TX（全经典组手册 §8.8.4 请求表同段同值；厂商库
// cw32f030_dma.h:105-110 DMA_HardTrig_UARTx_RXBufferNE=0x00<<2 …
// UART3_TXBufferE=0x05<<2）。

/// UART 经典组 DMA 请求使能位与触发源选择
#[cfg(any(
    feature = "cw32f030",
    feature = "cw32f020",
    feature = "cw32l031",
    feature = "cw32l052",
    feature = "cw32l083",
    feature = "cw32r031",
    feature = "cw32w031"
))]
mod cudma {
    /// DMA 发送请求使能（CR2[7]，厂商库 USART_DMAReq_Tx=0x0080；PAC 位常量）
    pub const CR2_DMATX: u32 = crate::pac::uart::cr2::dmatx_MASK;
    /// DMA 接收请求使能（CR2[6]，厂商库 USART_DMAReq_Rx=0x0040）
    pub const CR2_DMARX: u32 = crate::pac::uart::cr2::dmarx_MASK;

    /// 发送方向硬件触发源（§8.8.4 / 厂商库 DMA_HardTrig_UARTx_TXBufferE）。
    ///
    /// 全经典组 l083 六实例 + f030 三实例**全部接线**（idx 由 `instance_of`
    /// 按寄存器基址产生，不会越界）：UART1..3 取
    /// UARTx_TX=0x01/03/05（全经典组请求表同值），UART4..6 取
    /// UART4_TX..UART6_TX=0x2C/2E/30（仅 l083 手册表有，l083=CN_V2.0
    /// p147-149），与 [`crate::dma::hs`] 常量一一对应。`_` 分支是结构性
    /// 兜底——单实例家族（f020/l031/l052/r031/w031）`instance_of` 只产 0。
    pub fn tx_hs(idx: usize) -> u8 {
        match idx {
            0 => crate::dma::hs::UART1_TX,
            #[cfg(any(feature = "cw32f030", feature = "cw32l083"))]
            1 => crate::dma::hs::UART2_TX,
            #[cfg(any(feature = "cw32f030", feature = "cw32l083"))]
            2 => crate::dma::hs::UART3_TX,
            #[cfg(feature = "cw32l083")]
            3 => crate::dma::hs::UART4_TX,
            #[cfg(feature = "cw32l083")]
            4 => crate::dma::hs::UART5_TX,
            #[cfg(feature = "cw32l083")]
            5 => crate::dma::hs::UART6_TX,
            _ => unreachable!("UART 实例号超出本家族实例数（instance_of 只产 0..N-1）"),
        }
    }

    /// 接收方向硬件触发源（§8.8.4 / 厂商库 DMA_HardTrig_UARTx_RXBufferNE）
    ///
    /// 接线范围同 [`tx_hs`]：UART1..3=0x00/02/04、UART4..6=0x2B/2D/2F
    /// （仅 l083 手册表有）。
    pub fn rx_hs(idx: usize) -> u8 {
        match idx {
            0 => crate::dma::hs::UART1_RX,
            #[cfg(any(feature = "cw32f030", feature = "cw32l083"))]
            1 => crate::dma::hs::UART2_RX,
            #[cfg(any(feature = "cw32f030", feature = "cw32l083"))]
            2 => crate::dma::hs::UART3_RX,
            #[cfg(feature = "cw32l083")]
            3 => crate::dma::hs::UART4_RX,
            #[cfg(feature = "cw32l083")]
            4 => crate::dma::hs::UART5_RX,
            #[cfg(feature = "cw32l083")]
            5 => crate::dma::hs::UART6_RX,
            _ => unreachable!("UART 实例号超出本家族实例数（instance_of 只产 0..N-1）"),
        }
    }
}

#[cfg(any(
    feature = "cw32f030",
    feature = "cw32f020",
    feature = "cw32l031",
    feature = "cw32l052",
    feature = "cw32l083",
    feature = "cw32r031",
    feature = "cw32w031"
))]
impl Uart {
    /// DMA 发送：整段 buf 搬进 TDR（单次全量搬运，占用经典组 DMA 通道 2）
    ///
    /// 触发源 UARTx_TX（TXBE"发送缓冲空"事件，§8.8.4 / 厂商库
    /// `DMA_HardTrig_UARTx_TXBufferE`）；mem→periph：源自增、TDR 固定、
    /// 8bit、BLOCK。次序：通道 `configure`（EN=1）→ 置 `CR2.DMATX`
    /// （空闲时 TXBE 已置位，首个请求立即到）→ `wait` 等 DMA TC → 关
    /// `CR2.DMATX`——厂商库 `USART_DMACmd` 同款位面（CR2[7]）。
    ///
    /// DMA TC = 全部字节已写入 TDR；此时行上最后一字节可能仍在移位，
    /// 要等物理发完再收尾（如半双工转向）的调用方随后
    /// `embedded_io_async::Write::flush`（等 ISR.TC）。
    ///
    /// **与逐字节/Buffered 模式互斥**（见模块文档）；空缓冲或 >65535
    /// 字节返回 [`crate::dma::DmaErrorClassic::BadLength`]。⚠ 语义级实现
    /// 待实板验证。l083：UART1..6 全接线（触发源 UART1_TX..UART6_TX，
    /// §8.8.4 请求表，见模块文档 DMA 节）。
    pub async fn write_dma(
        &mut self,
        dma: &mut crate::dma::ClassicDma,
        buf: &[u8],
    ) -> Result<(), crate::dma::DmaErrorClassic> {
        if buf.is_empty() || buf.len() > 0xFFFF {
            return Err(crate::dma::DmaErrorClassic::BadLength);
        }
        let cfg = crate::dma::ClassicConfig::mem_to_periph(
            buf.as_ptr() as u32,
            core::ptr::addr_of!(self.regs.tdr) as u32,
            buf.len() as u16,
            crate::dma::ClassicWidth::B8,
            cudma::tx_hs(self.idx),
        );
        let mut ch = dma.channel(2);
        ch.configure(&cfg); // EN=1，等 TXBE 请求
        self.regs.cr2.modify(|v| v | cudma::CR2_DMATX);
        let r = ch.wait().await;
        self.regs.cr2.modify(|v| v & !cudma::CR2_DMATX);
        ch.stop(); // 收尾清通道 EN 防残留请求（ADC read_dma 同款）
        r
    }

    /// DMA 接收：恰好收满 buf.len() 字节（单次全量搬运，占用经典组 DMA 通道 1）
    ///
    /// 触发源 UARTx_RX（RC"接收完成"事件，§8.8.4 / 厂商库
    /// `DMA_HardTrig_UARTx_RXBufferNE`）；periph→mem：RDR 固定、目的自增、
    /// 8bit、BLOCK。启动前清残留 RC 与错误标志（ICR R1W0：对应位写 0），
    /// 保证触发请求从干净电平开始——启动前已到达但未被读走的字节会被丢弃。
    ///
    /// DMA 路径不逐字节上报接收错误（经典 UART 仅 FE/PE，只置 ISR，不影响
    /// 搬运）；需要错误感知的收流用逐字节 `read_byte`/`Read::read`。
    ///
    /// **与逐字节/Buffered 模式互斥**（见模块文档）；空缓冲或 >65535
    /// 字节返回 [`crate::dma::DmaErrorClassic::BadLength`]。⚠ 语义级实现
    /// 待实板验证。l083：UART1..6 全接线（触发源 UART1_RX..UART6_RX，
    /// §8.8.4 请求表，见模块文档 DMA 节）。
    pub async fn read_dma(
        &mut self,
        dma: &mut crate::dma::ClassicDma,
        buf: &mut [u8],
    ) -> Result<(), crate::dma::DmaErrorClassic> {
        if buf.is_empty() || buf.len() > 0xFFFF {
            return Err(crate::dma::DmaErrorClassic::BadLength);
        }
        // 清残留 RC + 错误位（R1W0：写 0 清除，其余位写 1 无动作）
        let isr_clear = ub::ICR_NONE & !(ub::ISR_RC | ub::ERR_ALL);
        self.regs.icr.write(isr_clear);
        let cfg = crate::dma::ClassicConfig::periph_to_mem(
            core::ptr::addr_of!(self.regs.rdr) as u32,
            buf.as_mut_ptr() as u32,
            buf.len() as u16,
            crate::dma::ClassicWidth::B8,
            cudma::rx_hs(self.idx),
        );
        let mut ch = dma.channel(1);
        ch.configure(&cfg); // EN=1，等 RC 请求
        self.regs.cr2.modify(|v| v | cudma::CR2_DMARX);
        let r = ch.wait().await;
        self.regs.cr2.modify(|v| v & !cudma::CR2_DMARX);
        ch.stop();
        r
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
#[cfg(not(feature = "cw32l083"))]
pub struct Uart1Irq;
#[cfg(not(feature = "cw32l083"))]
impl IrqHandler<interrupt::UART1> for Uart1Irq {
    fn call() {
        on_irq(0, unsafe { &*(pac::UART1_BASE as *const crate::Uart) });
    }
}

// ---- l083：UART 两两共享向量，标记类型按组派发 ----
// UART1/UART4、UART2/UART5、UART3/UART6 各共享一条 NVIC 线（IRQ27/28/29，
// 向量符号 UART1_UART4/UART2_UART5/UART3_UART6）。与 interrupt.rs 文档的
// SPI2/3 共享 `SPI23` 做法一致：共享向量的每个标记类型都依次检查组内
// 全部实例——未使用的实例时钟未开、无中断标志，检查即刻返回（不会额外
// 拉起中断线；门控外设也拉不起共享线）。

/// UART1+UART4 共享向量中断标记（l083 IRQ27）。
///
/// `bind_interrupts!` 侧写作 `UART1_UART4 => uart::Uart1Uart4Irq;`，
/// 可同时服务 UART1（`new_uart1`）与 UART4（`new_uart4`）。
#[cfg(feature = "cw32l083")]
pub struct Uart1Uart4Irq;
#[cfg(feature = "cw32l083")]
impl IrqHandler<interrupt::UART1_UART4> for Uart1Uart4Irq {
    fn call() {
        on_irq(0, unsafe { &*(pac::UART1_BASE as *const crate::Uart) });
        on_irq(3, unsafe { &*(pac::UART4_BASE as *const crate::Uart) });
    }
}

/// UART2+UART5 共享向量中断标记（l083 IRQ28）。
///
/// `bind_interrupts!` 侧写作 `UART2_UART5 => uart::Uart2Uart5Irq;`，
/// 可同时服务 UART2（`new_uart2`）与 UART5（`new_uart5`）。
#[cfg(feature = "cw32l083")]
pub struct Uart2Uart5Irq;
#[cfg(feature = "cw32l083")]
impl IrqHandler<interrupt::UART2_UART5> for Uart2Uart5Irq {
    fn call() {
        on_irq(1, unsafe { &*(pac::UART2_BASE as *const crate::Uart) });
        on_irq(4, unsafe { &*(pac::UART5_BASE as *const crate::Uart) });
    }
}

/// UART3+UART6 共享向量中断标记（l083 IRQ29）。
///
/// `bind_interrupts!` 侧写作 `UART3_UART6 => uart::Uart3Uart6Irq;`，
/// 可同时服务 UART3（`new_uart3`）与 UART6（`new_uart6`）。
#[cfg(feature = "cw32l083")]
pub struct Uart3Uart6Irq;
#[cfg(feature = "cw32l083")]
impl IrqHandler<interrupt::UART3_UART6> for Uart3Uart6Irq {
    fn call() {
        on_irq(2, unsafe { &*(pac::UART3_BASE as *const crate::Uart) });
        on_irq(5, unsafe { &*(pac::UART6_BASE as *const crate::Uart) });
    }
}

#[cfg(any(feature = "cw32l012", feature = "cw32f030"))]
/// UART2 中断标记（IRQ28）
pub struct Uart2Irq;
#[cfg(any(feature = "cw32l012", feature = "cw32f030"))]
impl IrqHandler<interrupt::UART2> for Uart2Irq {
    fn call() {
        on_irq(1, unsafe { &*(pac::UART2_BASE as *const crate::Uart) });
    }
}

#[cfg(any(feature = "cw32l012", feature = "cw32f030"))]
/// UART3 中断标记（IRQ29）
pub struct Uart3Irq;
#[cfg(any(feature = "cw32l012", feature = "cw32f030"))]
impl IrqHandler<interrupt::UART3> for Uart3Irq {
    fn call() {
        on_irq(2, unsafe { &*(pac::UART3_BASE as *const crate::Uart) });
    }
}
