//! 异步 DMA 控制器（§8）：4 通道、软件/硬件触发、TC 中断驱动的异步等待。
//!
//! **L012 家族先行**（`cw32l012` feature）；经典组（F030/F020 等虽然也有
//! DMA_BASE = 0x4002_0000）的 DMA 是另一套寄存器面——通道数更多、
//! ISR/ICR 的 TCy/TEy 标志按每通道 4bit 打包（L012 是 2bit）、且无
//! CCNT/CSRC/CDST 回读寄存器——不能直接复用本模块的位常量面，待实板
//! 回归后单独接入（届时解除模块级 cfg）。
//!
//! 寄存器配方与阻塞版 HAL **同源**（`cw32l012-hal/src/dma.rs`，逐行移植；
//! 其 `examples/dma_demo.rs` 已上板验证——存储器→存储器 + ADC 硬件触发
//! 搬运，2026-10-23 通过，见其 TESTING.md「dma_demo」一节）。
//!
//! # 一次传输要配什么（§8.4.1 步骤 1~14）
//!
//! | 步骤 | 寄存器 | 本模块入口 |
//! |---|---|---|
//! | 1 传输模式 | `CSR.TRANS` | [`Mode`] |
//! | 2 触发方式 | `TRIG.TYPE` | [`Trigger`] |
//! | 3 位宽 | `CSR.SIZE` | [`Width`] |
//! | 4 数据块数量 | `CNT.CNT` | [`Config::len`] |
//! | 5 数据块大小 | `CNT.REPEAT` | [`Config::repeat`]（建议 1） |
//! | 6/7 源/目的地址 | `SRCADDR` / `DSTADDR` | [`Config::src`] / [`Config::dst`] |
//! | 8/9 地址增量 | `CSR.SRCINC` / `DSTINC` | [`Config::src_inc`] / [`Config::dst_inc`] |
//! | 10/11 地址重载 | `CSR.SRCLOAD` / `DSTLOAD` | [`Config::src_reload`] / [`Config::dst_reload`] |
//! | 12 自动重传 | `CSR.RESTART` | [`Config::restart`] |
//! | 13 使能 | `CSR.EN` | [`Channel::configure`] 内部置位 |
//! | 14 启动 | `TRIG.SOFTSRC` | [`Channel::start`] |
//!
//! # 四个容易踩的点（阻塞版实机验证结论）
//!
//! - **`CNT.REPEAT` 每次传输后被硬件清 0**（§8.5.2），所以每轮传输前都要
//!   重新 [`Channel::configure`]，不能只改 `CNT`。
//! - **`ICR` 是 R1W0**：写 1 无动作、写 0 清标志（复位值 0xFFFF_FFFF）。
//!   清一个通道的 TC/TE 要写 `!TC_bit`（本模块已封装）。
//! - **软件触发下 `SOFTSRC` 传输完成后自动清零**，所以"再传一次"必须重新
//!   写它；硬件触发则只需外设继续发请求（配合 `RESTART` 可做无限环形搬运）。
//! - **地址自增量由位宽决定**（8/16/32bit → 1/2/4 字节，§8）。ADC 的多通道
//!   结果寄存器步进是 4 字节（RESULT0..7 @ +0x30..+0x4C），16 位宽的自增
//!   量对不上——要么配 32 位宽，要么用 `REPEAT` 整块搬（阻塞版 TESTING.md
//!   坑 4；本 crate 的 [`crate::adc::Adc::read_sequence_dma`] 用后者）。
//!
//! # 中断模型（异步等待）
//!
//! L012 的 DMA 两条 NVIC 线（PAC typelevel 名）：**IRQ9 = `DMA12`（通道
//! 1/2 共享）、IRQ10 = `DMA34`（通道 3/4 共享）**。[`Channel::wait`] 等
//! TC/TE：等待时才开该通道的 `CSR.TCIE|TEIE`，IRQ 唤醒即压掉（防风暴，
//! 纪律同 [`crate::uart`]）；ISR 标志由 poll 命中后经 `ICR`（R1W0）清除。
//!
//! ```ignore
//! embassy_cw32::bind_interrupts!(struct Irqs {
//!     DMA12 => embassy_cw32::dma::Dma12Irq;
//!     DMA34 => embassy_cw32::dma::Dma34Irq;
//! });
//! let mut dma = Dma::new(p.dma, Irqs);
//! let mut dst = [0u32; 64];
//! dma.mem_to_mem(&SRC, &mut dst).await?; // 存储器→存储器，异步
//!
//! // 外设→存储器：外设每次事件搬一个结果（硬件触发）
//! dma.periph_to_mem(
//!     pac::ADC1_BASE as u32 + 0x30,      // ADC1 RESULT0
//!     &mut buf,
//!     Width::B16,
//!     pac::dma::hs::ADC1_SINGLE,         // 表 8-8 硬件触发源
//! ).await?;
//! ```
//!
//! ⚠ `wait` 依赖对应中断已 `bind_interrupts!` 绑定到 [`Dma12Irq`]/[`Dma34Irq`]
//! （[`Dma::new`] 的 `_irqs` 参数做编译期证明）；未绑定则 NVIC 线落到弱默认
//! 向量，首个 TC 事件即挂死。

use core::future::poll_fn;
use core::task::Poll;

use embassy_sync::waitqueue::AtomicWaker;

use crate::interrupt::{Binding, IrqHandler, interrupt};
use crate::pac;
use crate::pac::dma as dc;

/// 传输数据位宽（`CSR.SIZE`，§8.8；同时决定地址自增量 1/2/4 字节）
#[allow(missing_docs)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum Width {
    B8 = 0,
    B16 = 1,
    B32 = 2,
}

impl Width {
    /// 寄存器编码
    #[inline]
    pub const fn code(self) -> u32 {
        self as u32
    }

    /// 单次搬运字节数（自增量与之一致，§8）
    #[inline]
    pub const fn bytes(self) -> usize {
        1 << (self as usize)
    }

    /// 按元素大小选位宽
    pub const fn for_size(size: usize) -> Option<Self> {
        match size {
            1 => Some(Width::B8),
            2 => Some(Width::B16),
            4 => Some(Width::B32),
            _ => None,
        }
    }
}

/// 传输模式（`CSR.TRANS`，§8.8 表 8-1）
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Mode {
    /// 块传输：每搬完一个数据块插入传输间隙，允许 CPU / 高优先级通道抢占
    Block,
    /// 批量传输：一口气搬完，期间不让出总线（数据量别太大）
    Bulk,
}

/// 触发方式（`TRIG.TYPE` + `TRIG.HARDSRC`，§8.8 表 8-8）
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Trigger {
    /// 软件触发：由 [`Channel::start`] 写 `SOFTSRC` 启动
    Software,
    /// 硬件触发：由外设事件启动，取值为 [`pac::dma::hs`] 中的常量
    Hardware(u8),
}

/// 通道配置（对应手册 §8.4.1 的步骤 1~12）
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Config {
    /// 源地址（不得指向 DMA/FLASH/RAM 控制器自己的寄存器，§8.8.6 注）
    pub src: u32,
    /// 目的地址（同上限制）
    pub dst: u32,
    /// 传输数据块数量（1..=65535）
    pub len: u16,
    /// 数据块大小（1..=15，建议 1；每次传输后硬件清 0，§8.5.2）
    pub repeat: u8,
    /// 位宽
    pub width: Width,
    /// BLOCK / BULK
    pub mode: Mode,
    /// 触发方式
    pub trigger: Trigger,
    /// 源地址自增
    pub src_inc: bool,
    /// 目的地址自增
    pub dst_inc: bool,
    /// 每个数据块完成时重载源地址（环形缓冲）
    pub src_reload: bool,
    /// 每个数据块完成时重载目的地址（环形缓冲）
    pub dst_reload: bool,
    /// 自动重传
    pub restart: bool,
}

impl Config {
    /// 存储器→存储器（地址自增、BULK、软件触发）
    pub const fn mem_to_mem(src: u32, dst: u32, len: u16, width: Width) -> Self {
        Self {
            src,
            dst,
            len,
            repeat: 1,
            width,
            mode: Mode::Bulk,
            trigger: Trigger::Software,
            src_inc: true,
            dst_inc: true,
            src_reload: false,
            dst_reload: false,
            restart: false,
        }
    }

    /// 外设→存储器（源固定、目的自增、BLOCK、硬件触发）
    pub const fn periph_to_mem(src: u32, dst: u32, len: u16, width: Width, hs: u8) -> Self {
        Self {
            src,
            dst,
            len,
            repeat: 1,
            width,
            mode: Mode::Block,
            trigger: Trigger::Hardware(hs),
            src_inc: false,
            dst_inc: true,
            src_reload: false,
            dst_reload: false,
            restart: false,
        }
    }

    /// 存储器→外设（源自增、目的固定、BLOCK、硬件触发）
    pub const fn mem_to_periph(src: u32, dst: u32, len: u16, width: Width, hs: u8) -> Self {
        Self {
            src,
            dst,
            len,
            repeat: 1,
            width,
            mode: Mode::Block,
            trigger: Trigger::Hardware(hs),
            src_inc: true,
            dst_inc: false,
            src_reload: false,
            dst_reload: false,
            restart: false,
        }
    }

    /// 编码 `CSR` 的可写位（不含 `EN`；`REPEAT` 在 `CNT` 里，不参与）
    pub const fn encode_csr(&self) -> u32 {
        let mut csr = self.width.code() << dc::CSR_SIZE_POS;
        if self.src_inc {
            csr |= dc::CSR_SRCINC;
        }
        if self.dst_inc {
            csr |= dc::CSR_DSTINC;
        }
        if let Mode::Block = self.mode {
            csr |= dc::CSR_TRANS;
        }
        if self.src_reload {
            csr |= dc::CSR_SRCLOAD;
        }
        if self.dst_reload {
            csr |= dc::CSR_DSTLOAD;
        }
        if self.restart {
            csr |= dc::CSR_RESTART;
        }
        csr
    }

    /// 编码 `CNT`（CNT[15:0] + REPEAT[19:16]）
    pub const fn encode_cnt(&self) -> u32 {
        // 手册：REPEAT 不可为 0，合法范围 1..=15（const fn 里不能调 clamp）
        let r = if self.repeat == 0 {
            1
        } else if self.repeat > 15 {
            15
        } else {
            self.repeat
        };
        (self.len as u32 & dc::CNT_CNT_MASK) | ((r as u32) << dc::CNT_REPEAT_POS)
    }

    /// 编码 `TRIG`（TYPE + HARDSRC）
    pub const fn encode_trig(&self) -> u32 {
        match self.trigger {
            Trigger::Software => 0,
            Trigger::Hardware(hs) => dc::TRIG_TYPE | (((hs as u32) & 0x3F) << dc::TRIG_HARDSRC_POS),
        }
    }
}

/// DMA 错误（语义同阻塞版 `dma::DmaError`）
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum DmaError {
    /// 通道号非法（应为 1..=4）
    BadChannel,
    /// 元素宽度不是 1/2/4 字节（或与缓冲元素类型不匹配）
    UnsupportedWidth,
    /// 源/目的长度不一致、为 0 或超过 65535
    BadLength,
    /// `STATUS=001`：传输地址超出寻址范围（表 8-2）
    AddressOutOfRange,
    /// `STATUS=010`：传输停止请求引起中止
    Aborted,
    /// `STATUS=011`：访问源地址出错
    SourceError,
    /// `STATUS=100`：访问目的地址出错
    DestError,
}

/// per-通道 waker（0..3 = 通道 1..4；`wait` 用）
static WAKERS: [AtomicWaker; 4] = [const { AtomicWaker::new() }; 4];

/// DMA 控制器（§8，DMA_BASE = 0x4002_0000）
pub struct Dma {
    regs: &'static pac::Dma,
}

impl Dma {
    /// 初始化（门控 AHBEN.DMA bit0 + 清残留标志）
    ///
    /// * `regs` — DMA 寄存器块（`p.dma` 单例）
    /// * `_irqs` — 中断绑定证明（`bind_interrupts!` 生成的结构体）：两条
    ///   NVIC 线都必须绑到本模块的 [`Dma12Irq`]/[`Dma34Irq`]（通道 1/2、
    ///   3/4 各占一条，[`Channel::wait`] 依赖对应向量）。
    pub fn new(
        regs: &'static pac::Dma,
        _irqs: impl Binding<interrupt::DMA12> + Binding<interrupt::DMA34>,
    ) -> Self {
        // AHBEN.DMA（bit0，§4）：L012 代组 KEY 写保护，读改写保留其它外设
        // 的使能位（阻塞版 rcc.rs set_bits 配方 / 本 crate timer.rs 同款）
        let sysctrl = unsafe { &*(pac::SYSCTRL_BASE as *const crate::Sysctrl) };
        sysctrl
            .ahben
            .modify(|v| pac::sysctrl::KEY | (v & 0xFFFF) | pac::sysctrl::AHBEN_DMA);
        // 清掉所有通道的残留标志（上电/软复位后 ICR 复位值为全 1；
        // ICR 是 R1W0——写 0 才清，阻塞版 Dma::new 配方）
        regs.icr.write(0);
        Dma { regs }
    }

    /// 取通道（`n` = 1..=4，数字越小优先级越高，§8.5.4）
    ///
    /// 返回的 [`Channel`] 借用本控制器（同一条通道不可被重复取出）。
    pub fn channel(&mut self, n: u8) -> Channel<'_> {
        assert!((1..=4).contains(&n), "DMA 通道号必须为 1..=4（Got {n}）");
        Channel {
            regs: self.regs,
            idx: (n - 1) as usize,
        }
    }

    /// 存储器→存储器异步搬运（自动占用通道 3，把 1/2 留给实时外设——
    /// 阻塞版同款策略）
    ///
    /// `src` / `dst` 长度必须相等且非空、≤65535；元素类型决定位宽
    /// （u8/u16/u32 → 8/16/32bit）。
    pub async fn mem_to_mem<T: Copy>(&mut self, src: &[T], dst: &mut [T]) -> Result<(), DmaError> {
        let width = Width::for_size(core::mem::size_of::<T>()).ok_or(DmaError::UnsupportedWidth)?;
        if src.is_empty() || src.len() != dst.len() || src.len() > 0xFFFF {
            return Err(DmaError::BadLength);
        }
        let cfg = Config::mem_to_mem(
            src.as_ptr() as u32,
            dst.as_mut_ptr() as u32,
            src.len() as u16,
            width,
        );
        let mut ch = self.channel(3);
        ch.configure(&cfg);
        ch.start();
        ch.wait().await
    }

    /// 外设→存储器异步搬运（硬件触发，自动占用通道 1）
    ///
    /// `periph_addr` 为外设数据寄存器地址（如 ADC1 RESULT0 =
    /// `ADC1_BASE + 0x30`），`hs` 为表 8-8 的硬件触发源（[`pac::dma::hs`]
    /// 常量）。缓冲元素宽度必须与 `width` 一致。
    ///
    /// 配方 = 阻塞版 dma_demo P2（实机验证）：`configure`（EN=1）→
    /// 外设侧开 DMA 请求（如 ADC `IER.DMAEOC`）→ 外设启动，`wait` 等 TC。
    pub async fn periph_to_mem<T: Copy>(
        &mut self,
        periph_addr: u32,
        buf: &mut [T],
        width: Width,
        hs: u8,
    ) -> Result<(), DmaError> {
        if Width::for_size(core::mem::size_of::<T>()) != Some(width) {
            return Err(DmaError::UnsupportedWidth);
        }
        if buf.is_empty() || buf.len() > 0xFFFF {
            return Err(DmaError::BadLength);
        }
        let cfg = Config::periph_to_mem(
            periph_addr,
            buf.as_mut_ptr() as u32,
            buf.len() as u16,
            width,
            hs,
        );
        let mut ch = self.channel(1);
        ch.configure(&cfg);
        ch.wait().await
    }

    /// 存储器→外设异步搬运（硬件触发，自动占用通道 2）
    ///
    /// 与 [`Dma::periph_to_mem`] 对称：源自增、目的固定（外设数据寄存器）。
    pub async fn mem_to_periph<T: Copy>(
        &mut self,
        buf: &[T],
        periph_addr: u32,
        width: Width,
        hs: u8,
    ) -> Result<(), DmaError> {
        if Width::for_size(core::mem::size_of::<T>()) != Some(width) {
            return Err(DmaError::UnsupportedWidth);
        }
        if buf.is_empty() || buf.len() > 0xFFFF {
            return Err(DmaError::BadLength);
        }
        let cfg = Config::mem_to_periph(
            buf.as_ptr() as u32,
            periph_addr,
            buf.len() as u16,
            width,
            hs,
        );
        let mut ch = self.channel(2);
        ch.configure(&cfg);
        ch.wait().await
    }

    /// 寄存器块原始访问
    pub fn regs(&self) -> &'static pac::Dma {
        self.regs
    }
}

/// 单个 DMA 通道（§8.8：每通道 0x20 字节的 CSR/CNT/SRC/DST/TRIG/CCNT…）
pub struct Channel<'d> {
    regs: &'d pac::Dma,
    idx: usize,
}

impl<'d> Channel<'d> {
    #[inline]
    fn csr(&self) -> &'d pac::Rw<u32> {
        &self.regs.ch[self.idx].csr
    }

    /// 通道号（1..=4）
    #[inline]
    pub fn index(&self) -> u8 {
        (self.idx + 1) as u8
    }

    /// 按配置装载通道（自动 `EN=0 → 清标志 → 写寄存器 → EN=1`）
    ///
    /// **每轮传输前都要调用**：`CNT.REPEAT` 会被硬件清 0（§8.5.2）。
    pub fn configure(&self, cfg: &Config) {
        let ch = &self.regs.ch[self.idx];
        ch.csr.modify(|v| v & !dc::CSR_EN);
        self.clear_flags();
        ch.cnt.write(cfg.encode_cnt());
        ch.src.write(cfg.src);
        ch.dst.write(cfg.dst);
        ch.trig.write(cfg.encode_trig());
        ch.csr.write(cfg.encode_csr() | dc::CSR_EN);
    }

    /// 软件触发启动（硬件触发模式下无需调用）
    ///
    /// `SOFTSRC` 在传输完成后由硬件自动清零，重复传输时要重新写。
    pub fn start(&self) {
        self.regs.ch[self.idx].trig.modify(|v| v | dc::TRIG_SOFTSRC);
    }

    /// 只置 `EN`（用于"先配好、等外设就绪再开"的场景）
    pub fn enable(&self) {
        self.csr().modify(|v| v | dc::CSR_EN);
    }

    /// 清 `EN`（停止通道；正在进行的传输会被中止）
    pub fn stop(&self) {
        self.csr().modify(|v| v & !dc::CSR_EN);
    }

    /// 还剩多少个数据块没搬（`CCNT`，RO）
    pub fn remaining(&self) -> u16 {
        (self.regs.ch[self.idx].ccnt.read() & 0xFFFF) as u16
    }

    /// 通道状态（`CSR.STATUS`，表 8-2 编码：101=传输完成）
    pub fn status(&self) -> u8 {
        ((self.csr().read() & dc::CSR_STATUS_MASK) >> dc::CSR_STATUS_POS) as u8
    }

    /// 是否传输完成（`CSR.TC`）
    pub fn is_done(&self) -> bool {
        self.csr().read() & dc::CSR_TC != 0
    }

    /// 是否出错（`CSR.TE`）
    pub fn is_error(&self) -> bool {
        self.csr().read() & dc::CSR_TE != 0
    }

    /// 解析当前状态为 `Result`（表 8-2 的 STATUS 编码 → 错误种类）
    pub fn check(&self) -> Result<(), DmaError> {
        decode_csr(self.csr().read())
    }

    /// 清本通道的 TC/TE 标志（`ICR` 为 R1W0：写 0 清除）
    pub fn clear_flags(&self) {
        let tc = 1u32 << (2 * self.idx);
        let te = 1u32 << (2 * self.idx + 1);
        self.regs.icr.write(!(tc | te));
    }

    /// 异步等待传输完成/出错（TC/TE 中断驱动）
    ///
    /// 等待时才开本通道的 `CSR.TCIE|TEIE`，IRQ 唤醒即压掉（防风暴，纪律同
    /// [`crate::uart`]）；命中后从清除标志前的 CSR 解码结果（先清 `ICR`
    /// 再读 `TE` 会误报 Ok），最后经 `ICR`（R1W0）清掉本通道标志，避免
    /// 残留 TC 让下次 `wait` 秒回。
    ///
    /// 前置条件：对应 NVIC 线已绑定 [`Dma12Irq`]/[`Dma34Irq`]（见
    /// [`Dma::new`]）；NVIC 线由本方法幂等使能。无内置超时——需要限时
    /// 就用 `embassy_time::with_timeout` 包一层（超时后调用方应
    /// [`Channel::stop`] + [`Channel::clear_flags`] 收尾）。
    pub async fn wait(&mut self) -> Result<(), DmaError> {
        // NVIC 线（幂等）：通道 1/2 → IRQ9 DMA12，通道 3/4 → IRQ10 DMA34
        match self.idx {
            0 | 1 => crate::interrupt::nvic_enable::<interrupt::DMA12>(),
            _ => crate::interrupt::nvic_enable::<interrupt::DMA34>(),
        }
        poll_fn(|cx| {
            WAKERS[self.idx].register(cx.waker());
            let csr = self.csr().read();
            if csr & (dc::CSR_TC | dc::CSR_TE) != 0 {
                // 命中：先解码（TE 位还在），再压掉中断使能、清标志
                let r = decode_csr(csr);
                self.csr().modify(|v| v & !(dc::CSR_TCIE | dc::CSR_TEIE));
                self.clear_flags();
                Poll::Ready(r)
            } else {
                self.csr().modify(|v| v | dc::CSR_TCIE | dc::CSR_TEIE);
                Poll::Pending
            }
        })
        .await
    }

    /// 寄存器块原始访问
    pub fn regs(&self) -> &'d pac::DmaChannel {
        &self.regs.ch[self.idx]
    }
}

/// 从 CSR 值解码传输结果（表 8-2：TE 置位时按 STATUS 给错误种类）
fn decode_csr(csr: u32) -> Result<(), DmaError> {
    if csr & dc::CSR_TE != 0 {
        return Err(match (csr & dc::CSR_STATUS_MASK) >> dc::CSR_STATUS_POS {
            1 => DmaError::AddressOutOfRange,
            2 => DmaError::Aborted,
            3 => DmaError::SourceError,
            4 => DmaError::DestError,
            _ => DmaError::Aborted,
        });
    }
    Ok(())
}

// ================= 中断派发 =================

/// DMA 组中断公共处理（IRQ9=通道 1/2，IRQ10=通道 3/4）：读全局 ISR，
/// 有 TC/TE 的通道单发压掉 TCIE/TEIE 并唤醒等待者。ISR 标志不动——
/// poll 复查命中后经 ICR R1W0 清除（uart.rs 同款纪律）。
fn on_irq_group(base: usize, regs: &'static pac::Dma) {
    let isr = regs.isr.read();
    for i in 0..2 {
        let idx = base + i;
        if isr & (0b11 << (2 * idx)) != 0 {
            // 压掉该通道的中断使能（下次 poll 会重新打开）
            regs.ch[idx].csr.modify(|v| v & !(dc::CSR_TCIE | dc::CSR_TEIE));
            WAKERS[idx].wake();
        }
    }
}

/// DMA 通道 1/2 共享中断标记（IRQ9 `DMA12`，bind_interrupts! 绑定用）
pub struct Dma12Irq;
impl IrqHandler<interrupt::DMA12> for Dma12Irq {
    fn call() {
        on_irq_group(0, unsafe { &*(pac::DMA_BASE as *const pac::Dma) });
    }
}

/// DMA 通道 3/4 共享中断标记（IRQ10 `DMA34`）
pub struct Dma34Irq;
impl IrqHandler<interrupt::DMA34> for Dma34Irq {
    fn call() {
        on_irq_group(2, unsafe { &*(pac::DMA_BASE as *const pac::Dma) });
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // 与阻塞版 cw32l012-hal/src/dma.rs 的同名单元测试同源
    #[test]
    fn width_codes_and_bytes() {
        assert_eq!(Width::B8.code(), 0);
        assert_eq!(Width::B16.code(), 1);
        assert_eq!(Width::B32.code(), 2);
        assert_eq!(Width::B8.bytes(), 1);
        assert_eq!(Width::B16.bytes(), 2);
        assert_eq!(Width::B32.bytes(), 4);
        assert_eq!(Width::for_size(4), Some(Width::B32));
        assert_eq!(Width::for_size(3), None);
    }

    #[test]
    fn csr_encoding_matches_manual_bits() {
        let cfg = Config::mem_to_mem(0x2000_0000, 0x2000_0100, 32, Width::B32);
        let csr = cfg.encode_csr();
        assert_eq!(csr & dc::CSR_SIZE_MASK, 2 << dc::CSR_SIZE_POS); // 32bit
        assert_ne!(csr & dc::CSR_SRCINC, 0);
        assert_ne!(csr & dc::CSR_DSTINC, 0);
        assert_eq!(csr & dc::CSR_TRANS, 0); // BULK
        assert_eq!(csr & dc::CSR_EN, 0); // EN 由 configure 单独置位
    }

    #[test]
    fn cnt_and_trig_encoding() {
        let mut cfg = Config::mem_to_mem(0, 0, 1000, Width::B16);
        cfg.repeat = 3;
        assert_eq!(cfg.encode_cnt(), 1000 | (3 << 16));
        assert_eq!(cfg.encode_trig(), 0); // 软件触发

        let cfg = Config::periph_to_mem(
            0x4000_0030,
            0x2000_0000,
            8,
            Width::B16,
            dc::hs::ADC1_SINGLE,
        );
        assert_eq!(
            cfg.encode_trig(),
            dc::TRIG_TYPE | ((dc::hs::ADC1_SINGLE as u32) << dc::TRIG_HARDSRC_POS)
        );
    }

    #[test]
    fn repeat_is_clamped_to_valid_range() {
        let mut cfg = Config::mem_to_mem(0, 0, 1, Width::B8);
        cfg.repeat = 0; // 手册：REPEAT 不可为 0
        assert_eq!(cfg.encode_cnt() >> 16, 1);
        cfg.repeat = 0xFF;
        assert_eq!(cfg.encode_cnt() >> 16, 15);
    }
}
