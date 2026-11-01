//! 异步 DMA 控制器：软件/硬件触发、TC 中断驱动的异步等待。
//!
//! **两代 IP、两套类型**（家族 feature 互斥，任一构建只存在其一）：
//!
//! - **L012 代组**（`cw32l012`/`cw32l010`/`cw32l011` → [`Dma`]，仅 L012
//!   有 DMA 硬件）：4 通道、同一寄存器块内偏移、ISR/ICR 每通道 2bit、
//!   带 CCNT/CSRC/CDST 回读与 RESTART/地址重载位。寄存器配方与阻塞版
//!   HAL 同源（`cw32l012-hal/src/dma.rs`；其 `examples/dma_demo.rs` 已
//!   上板验证——mem2mem + ADC 硬件触发，2026-10-23 通过）。
//! - **经典组**（F020/F030/L031/L052/L083/R031/W031 → [`ClassicDma`]）：
//!   另一套 IP——每通道独立 0x20 步进寄存器块（f030/f020/l083 5 通道、
//!   l031 组 4 通道）、ISR/ICR 的 TCy/TEy 按每通道 4bit 打包、**无**
//!   CCNT/CSRC/CDST 回读（`remaining()` 不可移植，未实现）、CSR 无
//!   TC/TE/RESTART/重载位（完成/出错标志只在全局 ISR）。F003/L010/L011
//!   无 DMA 硬件，不接入。
//!
//! 两套 API 形状一致（`new`/`channel`/`mem_to_mem`/`periph_to_mem`/
//! `mem_to_periph`/`wait`），配置类型按代独立命名（[`ClassicConfig`] vs
//! [`Config`]），位面各引各代 PAC 常量。
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
#[cfg(feature = "cw32l012")]
pub enum Width {
    B8 = 0,
    B16 = 1,
    B32 = 2,
}

#[cfg(feature = "cw32l012")]
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
#[cfg(feature = "cw32l012")]
pub enum Mode {
    /// 块传输：每搬完一个数据块插入传输间隙，允许 CPU / 高优先级通道抢占
    Block,
    /// 批量传输：一口气搬完，期间不让出总线（数据量别太大）
    Bulk,
}

/// 触发方式（`TRIG.TYPE` + `TRIG.HARDSRC`，§8.8 表 8-8）
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg(feature = "cw32l012")]
pub enum Trigger {
    /// 软件触发：由 [`Channel::start`] 写 `SOFTSRC` 启动
    Software,
    /// 硬件触发：由外设事件启动，取值为 [`pac::dma::hs`] 中的常量
    Hardware(u8),
}

/// 通道配置（对应手册 §8.4.1 的步骤 1~12）
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg(feature = "cw32l012")]
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

#[cfg(feature = "cw32l012")]
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
#[cfg(feature = "cw32l012")]
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
#[cfg(feature = "cw32l012")]
static WAKERS: [AtomicWaker; 4] = [const { AtomicWaker::new() }; 4];

/// DMA 控制器（§8，DMA_BASE = 0x4002_0000）
#[cfg(feature = "cw32l012")]
pub struct Dma {
    regs: &'static pac::Dma,
}

#[cfg(feature = "cw32l012")]
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
#[cfg(feature = "cw32l012")]
pub struct Channel<'d> {
    regs: &'d pac::Dma,
    idx: usize,
}

#[cfg(feature = "cw32l012")]
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
#[cfg(feature = "cw32l012")]
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
#[cfg(feature = "cw32l012")]
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
#[cfg(feature = "cw32l012")]
pub struct Dma12Irq;
#[cfg(feature = "cw32l012")]
impl IrqHandler<interrupt::DMA12> for Dma12Irq {
    fn call() {
        on_irq_group(0, unsafe { &*(pac::DMA_BASE as *const pac::Dma) });
    }
}

/// DMA 通道 3/4 共享中断标记（IRQ10 `DMA34`）
#[cfg(feature = "cw32l012")]
pub struct Dma34Irq;
#[cfg(feature = "cw32l012")]
impl IrqHandler<interrupt::DMA34> for Dma34Irq {
    fn call() {
        on_irq_group(2, unsafe { &*(pac::DMA_BASE as *const pac::Dma) });
    }
}

// ================= 经典组（F020/F030/L031/L052/L083/R031/W031） =================
//
// 与 L012 的 DMA 是两代不同 IP，逐项差异（均已对 PAC/厂商头文件抽验）：
//
// - **通道布局**：全局块只有 ISR(RO)@+0x00 / ICR@+0x04；每通道独立寄存器
//   块 `DMACHANNELn` = DMA_BASE + 0x20×n（1 基），内含 CSR/CNT/SRCADDR/
//   DSTADDR/TRIG 五个寄存器（0x20 步进含保留区，块本身 20 字节）。通道数：
//   f030/f020/l083 = 5（PAC `DMACHANNEL1..5_BASE` = 0x40020020..0x400200A0，
//   f020 厂商头文件 `DMA_TypeDef Size = 180 (0xb4)`），l031/l052/r031/w031
//   = 4（`DMACHANNEL1..4_BASE`）；**f003/l010/l011 无 DMA**。
// - **无 CCNT/CSRC/CDST 回读**：`DMACHANNEL_TypeDef Size = 20`（cw32f020.h
//   /cw32l031.h），只有 5 个寄存器——`remaining()` 不可移植，不实现。
// - **ISR/ICR 每通道 4bit 打包**：TCy@bit 4(y-1)、TEy@bit 4(y-1)+1
//   （PAC `dma::isr`：tc1@0/te1@1/tc2@4/te2@5/tc3@8…tc5@16；厂商库
//   `DMA_IT_TC1..TE5`、`DMA_IT_ALL=0x33333` 同值）。ICR 是 **R1W0**：厂商
//   库 `DMA_ClearITPendingBit` = `CW_DMA->ICR &= (~DMA_IT)`——写 0 清标志。
// - **CSR 位面**（与 L012 低 11 位逐位一致，但**没有** L012 的
//   TC/TE/RESTART/SRCLOAD/DSTLOAD 位）：EN@0、TCIE@1、TEIE@2、TRANS@3、
//   SRCINC@4、DSTINC@5、SIZE[7:6]、STATUS[10:8]（厂商头文件
//   `DMACHx_CSR_*` 位定义 + f030 `CSR_CLEAR_Mask=0xFFFFFF07` 可写区；
//   f020 头文件 STATUS 为 `__IM` 只读）。完成/出错标志只在全局 ISR，
//   `wait` 改为轮询 ISR 而非 CSR。STATUS 编码同 L012 表 8-2（厂商
//   `DMA_CHANNEL_STATUS_*`：1=地址越界、2=停止请求、3=源错、4=目的错、
//   5=传输完成）。
// - **CNT**：CNT[15:0] + REPEAT[19:16]（同 L012；厂商 `DMA_Init` 固定写
//   `CNT = 0x1_0000 | TransferCnt`，即 REPEAT=1）。
// - **TRIG**：TYPE@0（0=软件/1=硬件）、SOFTSRC@1（写 1 启动，完成后硬件
//   自动清零）、HARDSRC[7:2]——位面同 L012，但**触发源编号表不同**（见
//   [`hs`]）。
// - **门控**：`SYSCTRL_AHBEN.DMA`（**bit0**，各家厂商头文件
//   `SYSCTRL_AHBEN_DMA_Pos = 0UL`；注意 APBEN1 的 bit0 是保留位 `__IM`，
//   并非 DMA 门控）。经典组无 KEY，读改写即可。
// - **中断分组**（各家 PAC interrupt 表 + 厂商 `IRQn_Type` 注释）：
//   f030/l083 = `DMACH1`(IRQ9,通道1) / `DMACH23`(IRQ10,通道2+3) /
//   `DMACH45`(IRQ11,通道4+5)；l031/l052/r031/w031 = `DMACH1`/`DMACH23`/
//   `DMACH4`(IRQ11,通道4)；f020 特殊 = `DMACH1`(IRQ9,通道1) /
//   `DMACH2`(IRQ10,通道2+3——厂商头文件注释 "DMA Channel 2/3
//   Interrupt")，IRQ11 保留 → **f020 通道 4/5 无 NVIC 线**，不支持
//   `wait`（轮询可用）。
//
// 经典组位面常量引用经典组自己的 PAC（`pac::dma::csr1..csr5` /
// `cnt1..cnt5` / `trig1..trig5` 逐通道同名同值，全家族抽验一致；统一引
// 用 `*1` 一组）。

/// 经典组 DMA 通道数（PAC `DMACHANNELn_BASE` 常量逐家核对）
#[cfg(any(feature = "cw32f030", feature = "cw32f020", feature = "cw32l083"))]
const CLASSIC_CHANNELS: usize = 5;
/// 经典组 DMA 通道数（l031 组：PAC 只有 DMACHANNEL1..4_BASE）
#[cfg(any(
    feature = "cw32l031",
    feature = "cw32l052",
    feature = "cw32r031",
    feature = "cw32w031"
))]
const CLASSIC_CHANNELS: usize = 4;

// 位段常量（编码见上节；值引自经典组 PAC 逐通道位域模块）
#[cfg(any(
    feature = "cw32f030",
    feature = "cw32f020",
    feature = "cw32l031",
    feature = "cw32l052",
    feature = "cw32l083",
    feature = "cw32r031",
    feature = "cw32w031"
))]
mod cbits {
    use super::dc;

    // ---- CSR（通道块 +0x00；csr1..csr5 逐通道同名同值）----
    pub const CSR_EN: u32 = dc::csr1::en_MASK;
    pub const CSR_TCIE: u32 = dc::csr1::tcie_MASK;
    pub const CSR_TEIE: u32 = dc::csr1::teie_MASK;
    /// 1=BLOCK（块传输）、0=BULK
    pub const CSR_TRANS: u32 = dc::csr1::trans_MASK;
    pub const CSR_SRCINC: u32 = dc::csr1::srcinc_MASK;
    pub const CSR_DSTINC: u32 = dc::csr1::dstinc_MASK;
    pub const CSR_SIZE_POS: u32 = dc::csr1::size_POS;
    pub const CSR_STATUS_POS: u32 = dc::csr1::status_POS;
    pub const CSR_STATUS_MASK: u32 = dc::csr1::status_MASK;

    // ---- CNT（+0x04）----
    pub const CNT_CNT_MASK: u32 = dc::cnt1::cnt_MASK;
    pub const CNT_REPEAT_POS: u32 = dc::cnt1::repeat_POS;

    // ---- TRIG（+0x10）----
    pub const TRIG_TYPE: u32 = dc::trig1::r#type_MASK;
    pub const TRIG_SOFTSRC: u32 = dc::trig1::softsrc_MASK;
    pub const TRIG_HARDSRC_POS: u32 = dc::trig1::hardsrc_POS;
}

/// 经典组 DMA 通道 n（1 基）的寄存器块（DMA_BASE + 0x20×n）
#[inline]
#[cfg(any(
    feature = "cw32f030",
    feature = "cw32f020",
    feature = "cw32l031",
    feature = "cw32l052",
    feature = "cw32l083",
    feature = "cw32r031",
    feature = "cw32w031"
))]
fn classic_ch(idx: usize) -> &'static pac::DMACHANNEL {
    unsafe { &*((pac::DMA_BASE + 0x20 * (idx + 1)) as *const pac::DMACHANNEL) }
}

/// 经典组 per-通道 waker（0..4 = 通道 1..5；8 槽覆盖经典组最大通道数 5，
/// 家族 cfg 决定有效槽数）
#[cfg(any(
    feature = "cw32f030",
    feature = "cw32f020",
    feature = "cw32l031",
    feature = "cw32l052",
    feature = "cw32l083",
    feature = "cw32r031",
    feature = "cw32w031"
))]
static CLASSIC_WAKERS: [AtomicWaker; 8] = [const { AtomicWaker::new() }; 8];

/// 经典组传输数据位宽（`CSR.SIZE[7:6]`；自增量随之 1/2/4 字节）
#[allow(missing_docs)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
#[cfg(any(feature = "cw32f030", feature = "cw32f020", feature = "cw32l031", feature = "cw32l052", feature = "cw32l083", feature = "cw32r031", feature = "cw32w031"))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ClassicWidth {
    B8 = 0,
    B16 = 1,
    B32 = 2,
}

#[cfg(any(feature = "cw32f030", feature = "cw32f020", feature = "cw32l031", feature = "cw32l052", feature = "cw32l083", feature = "cw32r031", feature = "cw32w031"))]
impl ClassicWidth {
    /// 寄存器编码
    #[inline]
    pub const fn code(self) -> u32 {
        self as u32
    }

    /// 单次搬运字节数
    #[inline]
    pub const fn bytes(self) -> usize {
        1 << (self as usize)
    }

    /// 按元素大小选位宽
    pub const fn for_size(size: usize) -> Option<Self> {
        match size {
            1 => Some(ClassicWidth::B8),
            2 => Some(ClassicWidth::B16),
            4 => Some(ClassicWidth::B32),
            _ => None,
        }
    }
}

/// 经典组传输模式（`CSR.TRANS`）
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg(any(feature = "cw32f030", feature = "cw32f020", feature = "cw32l031", feature = "cw32l052", feature = "cw32l083", feature = "cw32r031", feature = "cw32w031"))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ClassicMode {
    /// 块传输：每搬完一个数据块插入传输间隙，允许 CPU / 高优先级通道抢占
    Block,
    /// 批量传输：一口气搬完，期间不让出总线
    Bulk,
}

/// 经典组触发方式（`TRIG.TYPE` + `TRIG.HARDSRC[7:2]`）
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg(any(feature = "cw32f030", feature = "cw32f020", feature = "cw32l031", feature = "cw32l052", feature = "cw32l083", feature = "cw32r031", feature = "cw32w031"))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ClassicTrigger {
    /// 软件触发：由 [`Channel::start`] 写 `SOFTSRC` 启动
    Software,
    /// 硬件触发：由外设事件启动，取值为 [`hs`] 中的常量（或按自家手册
    /// §8.8.4 请求表填原始 6bit 编码）
    Hardware(u8),
}

/// 经典组硬件触发源编码（`TRIG.HARDSRC[7:2]` 的 6bit 值）
///
/// **出处**：各家用户手册 §8.8.4「DMA_TRIGy.HARDSRC[7:2]」请求表——
/// f030=CW32x030_UserManual_EN_V1.0 p147-148（与厂商库 `cw32f030_dma.h`
/// 的 `DMA_HardTrig_*` 枚举逐一对应）、f020=CN_V1.4 p137、l031=CN_V1.6
/// p133、l052=CN_V1.5 p138、l083=CN_V2.0 p148-149、r031=CN_V1.3 p135、
/// w031=CN_V1.4 p134。**各家族编号有差异**（如 ADC：f030/f020 是 10=转换
/// 完成，l031 组是 10=序列+50=单次；SPI2/LCD/UART4-6 只在部分家族存在），
/// 常量按家族 cfg 裁剪——只列出该家族手册表中存在的请求。
#[allow(missing_docs)]
#[cfg(any(feature = "cw32f030", feature = "cw32f020", feature = "cw32l031", feature = "cw32l052", feature = "cw32l083", feature = "cw32r031", feature = "cw32w031"))]
pub mod hs {
    // ---- 全经典组共有（各家 §8.8.4 请求表同段同值）----
    pub const UART1_RX: u8 = 0x00;
    pub const UART1_TX: u8 = 0x01;
    pub const UART2_RX: u8 = 0x02;
    pub const UART2_TX: u8 = 0x03;
    pub const UART3_RX: u8 = 0x04;
    pub const UART3_TX: u8 = 0x05;
    pub const SPI1_RX: u8 = 0x06;
    pub const SPI1_TX: u8 = 0x07;
    pub const BTIM1_UPD: u8 = 0x0B;
    pub const BTIM1_TRIG: u8 = 0x0C;
    pub const BTIM2_UPD: u8 = 0x0D;
    pub const BTIM2_TRIG: u8 = 0x0E;
    pub const BTIM3_UPD: u8 = 0x0F;
    pub const BTIM3_TRIG: u8 = 0x10;
    pub const GTIM1_UPD: u8 = 0x13;
    pub const GTIM1_TRIG: u8 = 0x14;
    pub const GTIM1_CC1: u8 = 0x15;
    pub const GTIM1_CC2: u8 = 0x16;
    pub const GTIM1_CC3: u8 = 0x17;
    pub const GTIM1_CC4: u8 = 0x18;
    pub const GTIM2_UPD: u8 = 0x19;
    pub const GTIM2_TRIG: u8 = 0x1A;
    pub const GTIM2_CC1: u8 = 0x1B;
    pub const GTIM2_CC2: u8 = 0x1C;
    pub const GTIM2_CC3: u8 = 0x1D;
    pub const GTIM2_CC4: u8 = 0x1E;

    // ---- SPI2（f030/f020/l052/l083 有 SPI2；l031/r031/w031 手册表无）----
    #[cfg(any(feature = "cw32f030", feature = "cw32f020", feature = "cw32l052", feature = "cw32l083"))]
    pub const SPI2_RX: u8 = 0x08;
    #[cfg(any(feature = "cw32f030", feature = "cw32f020", feature = "cw32l052", feature = "cw32l083"))]
    pub const SPI2_TX: u8 = 0x09;

    // ---- ADC：f030/f020 手册表只有一个"转换完成"请求(10)；l031 组拆
    // 序列(10)/单次(50)两个请求 ----
    #[cfg(any(feature = "cw32f030", feature = "cw32f020"))]
    pub const ADC_COMPLETE: u8 = 0x0A;
    #[cfg(any(
        feature = "cw32l031",
        feature = "cw32l052",
        feature = "cw32l083",
        feature = "cw32r031",
        feature = "cw32w031"
    ))]
    pub const ADC_SEQ: u8 = 0x0A;
    #[cfg(any(
        feature = "cw32l031",
        feature = "cw32l052",
        feature = "cw32l083",
        feature = "cw32r031",
        feature = "cw32w031"
    ))]
    pub const ADC_SINGLE: u8 = 0x32;

    // ---- ATIM（f020 手册表无 ATIM 请求，其余六家有）----
    #[cfg(any(
        feature = "cw32f030",
        feature = "cw32l031",
        feature = "cw32l052",
        feature = "cw32l083",
        feature = "cw32r031",
        feature = "cw32w031"
    ))]
    pub const ATIM_CH1A2A3A4: u8 = 0x11;
    #[cfg(any(
        feature = "cw32f030",
        feature = "cw32l031",
        feature = "cw32l052",
        feature = "cw32l083",
        feature = "cw32r031",
        feature = "cw32w031"
    ))]
    pub const ATIM_CH1B2B3B: u8 = 0x12;

    // ---- GTIM3/GTIM4（l031/r031/w031 手册表无；f030/f020/l052/l083 有）----
    #[cfg(any(feature = "cw32f030", feature = "cw32f020", feature = "cw32l052", feature = "cw32l083"))]
    pub const GTIM3_UPD: u8 = 0x1F;
    #[cfg(any(feature = "cw32f030", feature = "cw32f020", feature = "cw32l052", feature = "cw32l083"))]
    pub const GTIM3_TRIG: u8 = 0x20;
    #[cfg(any(feature = "cw32f030", feature = "cw32f020", feature = "cw32l052", feature = "cw32l083"))]
    pub const GTIM3_CC1: u8 = 0x21;
    #[cfg(any(feature = "cw32f030", feature = "cw32f020", feature = "cw32l052", feature = "cw32l083"))]
    pub const GTIM3_CC2: u8 = 0x22;
    #[cfg(any(feature = "cw32f030", feature = "cw32f020", feature = "cw32l052", feature = "cw32l083"))]
    pub const GTIM3_CC3: u8 = 0x23;
    #[cfg(any(feature = "cw32f030", feature = "cw32f020", feature = "cw32l052", feature = "cw32l083"))]
    pub const GTIM3_CC4: u8 = 0x24;
    #[cfg(any(feature = "cw32f030", feature = "cw32f020", feature = "cw32l052", feature = "cw32l083"))]
    pub const GTIM4_UPD: u8 = 0x25;
    #[cfg(any(feature = "cw32f030", feature = "cw32f020", feature = "cw32l052", feature = "cw32l083"))]
    pub const GTIM4_TRIG: u8 = 0x26;
    #[cfg(any(feature = "cw32f030", feature = "cw32f020", feature = "cw32l052", feature = "cw32l083"))]
    pub const GTIM4_CC1: u8 = 0x27;
    #[cfg(any(feature = "cw32f030", feature = "cw32f020", feature = "cw32l052", feature = "cw32l083"))]
    pub const GTIM4_CC2: u8 = 0x28;
    #[cfg(any(feature = "cw32f030", feature = "cw32f020", feature = "cw32l052", feature = "cw32l083"))]
    pub const GTIM4_CC3: u8 = 0x29;
    #[cfg(any(feature = "cw32f030", feature = "cw32f020", feature = "cw32l052", feature = "cw32l083"))]
    pub const GTIM4_CC4: u8 = 0x2A;

    // ---- LCD（l052/l083 手册表有）----
    #[cfg(any(feature = "cw32l052", feature = "cw32l083"))]
    pub const LCD_FRAME: u8 = 0x31;

    // ---- UART4-6（仅 l083 手册表有）----
    #[cfg(feature = "cw32l083")]
    pub const UART4_RX: u8 = 0x2B;
    #[cfg(feature = "cw32l083")]
    pub const UART4_TX: u8 = 0x2C;
    #[cfg(feature = "cw32l083")]
    pub const UART5_RX: u8 = 0x2D;
    #[cfg(feature = "cw32l083")]
    pub const UART5_TX: u8 = 0x2E;
    #[cfg(feature = "cw32l083")]
    pub const UART6_RX: u8 = 0x2F;
    #[cfg(feature = "cw32l083")]
    pub const UART6_TX: u8 = 0x30;
}

/// 经典组通道配置（对应手册 §8.4.1 配方中经典组支持的步骤）
///
/// 与 L012 的 [`Config`] 同形但**没有** `src_reload`/`dst_reload`/
/// `restart` 字段——经典组 CSR 无这些位（见模块文档）。经典组的
/// `CNT.REPEAT` 同样在每次传输后被硬件清 0（厂商 `DMA_Init` 每次都重写
/// `CNT=0x1_0000|CNT`），每轮传输前都要重新 [`Channel::configure`]。
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg(any(feature = "cw32f030", feature = "cw32f020", feature = "cw32l031", feature = "cw32l052", feature = "cw32l083", feature = "cw32r031", feature = "cw32w031"))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct ClassicConfig {
    /// 源地址
    pub src: u32,
    /// 目的地址
    pub dst: u32,
    /// 传输数据块数量（1..=65535）
    pub len: u16,
    /// 数据块大小（1..=15，建议 1；每次传输后硬件清 0）
    pub repeat: u8,
    /// 位宽
    pub width: ClassicWidth,
    /// BLOCK / BULK
    pub mode: ClassicMode,
    /// 触发方式
    pub trigger: ClassicTrigger,
    /// 源地址自增
    pub src_inc: bool,
    /// 目的地址自增
    pub dst_inc: bool,
}

#[cfg(any(feature = "cw32f030", feature = "cw32f020", feature = "cw32l031", feature = "cw32l052", feature = "cw32l083", feature = "cw32r031", feature = "cw32w031"))]
impl ClassicConfig {
    /// 存储器→存储器（地址自增、BULK、软件触发）
    pub const fn mem_to_mem(src: u32, dst: u32, len: u16, width: ClassicWidth) -> Self {
        Self {
            src,
            dst,
            len,
            repeat: 1,
            width,
            mode: ClassicMode::Bulk,
            trigger: ClassicTrigger::Software,
            src_inc: true,
            dst_inc: true,
        }
    }

    /// 外设→存储器（源固定、目的自增、BLOCK、硬件触发）
    pub const fn periph_to_mem(
        src: u32,
        dst: u32,
        len: u16,
        width: ClassicWidth,
        hs: u8,
    ) -> Self {
        Self {
            src,
            dst,
            len,
            repeat: 1,
            width,
            mode: ClassicMode::Block,
            trigger: ClassicTrigger::Hardware(hs),
            src_inc: false,
            dst_inc: true,
        }
    }

    /// 存储器→外设（源自增、目的固定、BLOCK、硬件触发）
    pub const fn mem_to_periph(
        src: u32,
        dst: u32,
        len: u16,
        width: ClassicWidth,
        hs: u8,
    ) -> Self {
        Self {
            src,
            dst,
            len,
            repeat: 1,
            width,
            mode: ClassicMode::Block,
            trigger: ClassicTrigger::Hardware(hs),
            src_inc: true,
            dst_inc: false,
        }
    }

    /// 编码 `CSR` 的可写位（不含 `EN`；经典组无 RESTART/重载位）
    pub const fn encode_csr(&self) -> u32 {
        let mut csr = self.width.code() << cbits::CSR_SIZE_POS;
        if self.src_inc {
            csr |= cbits::CSR_SRCINC;
        }
        if self.dst_inc {
            csr |= cbits::CSR_DSTINC;
        }
        if let ClassicMode::Block = self.mode {
            csr |= cbits::CSR_TRANS;
        }
        csr
    }

    /// 编码 `CNT`（CNT[15:0] + REPEAT[19:16]）
    pub const fn encode_cnt(&self) -> u32 {
        // 同 L012：REPEAT 不可为 0，合法范围 1..=15（const fn 手工夹紧）
        let r = if self.repeat == 0 {
            1
        } else if self.repeat > 15 {
            15
        } else {
            self.repeat
        };
        (self.len as u32 & cbits::CNT_CNT_MASK) | ((r as u32) << cbits::CNT_REPEAT_POS)
    }

    /// 编码 `TRIG`（TYPE + HARDSRC）
    pub const fn encode_trig(&self) -> u32 {
        match self.trigger {
            ClassicTrigger::Software => 0,
            ClassicTrigger::Hardware(hs) => {
                cbits::TRIG_TYPE | (((hs as u32) & 0x3F) << cbits::TRIG_HARDSRC_POS)
            }
        }
    }
}

/// 经典组 DMA 错误（STATUS 编码同 L012 表 8-2 / 厂商 `DMA_CHANNEL_STATUS_*`）
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg(any(feature = "cw32f030", feature = "cw32f020", feature = "cw32l031", feature = "cw32l052", feature = "cw32l083", feature = "cw32r031", feature = "cw32w031"))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum DmaErrorClassic {
    /// 通道号非法
    BadChannel,
    /// 元素宽度不是 1/2/4 字节（或与缓冲元素类型不匹配）
    UnsupportedWidth,
    /// 源/目的长度不一致、为 0 或超过 65535
    BadLength,
    /// `STATUS=001`：传输地址超出寻址范围
    AddressOutOfRange,
    /// `STATUS=010`：传输停止请求引起中止
    Aborted,
    /// `STATUS=011`：访问源地址出错
    SourceError,
    /// `STATUS=100`：访问目的地址出错
    DestError,
}

/// 经典组 DMA 控制器（DMA_BASE = 0x4002_0000，每通道独立 0x20 步进块）
///
/// 用法与 [`Dma`] 同形；中断绑定标记按家族分三组（见模块文档）：
///
/// ```ignore
/// embassy_cw32::bind_interrupts!(struct Irqs {
///     DMACH1 => embassy_cw32::dma::Dmach1Irq;
///     DMACH23 => embassy_cw32::dma::Dmach23Irq;
///     DMACH45 => embassy_cw32::dma::Dmach45Irq; // l031 组是 DMACH4 => Dmach4Irq
/// });
/// let mut dma = ClassicDma::new(p.dma, Irqs);
/// dma.mem_to_mem(&SRC, &mut dst).await?;
/// ```
#[cfg(any(feature = "cw32f030", feature = "cw32f020", feature = "cw32l031", feature = "cw32l052", feature = "cw32l083", feature = "cw32r031", feature = "cw32w031"))]
pub struct ClassicDma {
    regs: &'static pac::DMA,
}

#[cfg(any(feature = "cw32f030", feature = "cw32f020", feature = "cw32l031", feature = "cw32l052", feature = "cw32l083", feature = "cw32r031", feature = "cw32w031"))]
impl ClassicDma {
    /// 初始化（门控 AHBEN.DMA bit0，无 KEY；清残留标志）
    ///
    /// * `regs` — DMA 寄存器块（`p.dma` 单例）
    /// * `_irqs` — 中断绑定证明：本家族全部 DMA NVIC 线都要绑到本模块的
    ///   标记类型（f030/l083：DMACH1/DMACH23/DMACH45；l031 组：DMACH1/
    ///   DMACH23/DMACH4；f020：DMACH1/DMACH2），[`Channel::wait`] 依赖对应
    ///   向量。
    #[cfg(any(feature = "cw32f030", feature = "cw32l083"))]
    pub fn new(
        regs: &'static pac::DMA,
        _irqs: impl Binding<interrupt::DMACH1>
            + Binding<interrupt::DMACH23>
            + Binding<interrupt::DMACH45>,
    ) -> Self {
        Self::init(regs)
    }

    /// 初始化（l031/l052/r031/w031：三条 NVIC 线，通道 4 独占 IRQ11）
    #[cfg(any(
        feature = "cw32l031",
        feature = "cw32l052",
        feature = "cw32r031",
        feature = "cw32w031"
    ))]
    pub fn new(
        regs: &'static pac::DMA,
        _irqs: impl Binding<interrupt::DMACH1>
            + Binding<interrupt::DMACH23>
            + Binding<interrupt::DMACH4>,
    ) -> Self {
        Self::init(regs)
    }

    /// 初始化（f020：通道 1 与通道 2+3 两条线；通道 4/5 无 NVIC 线）
    #[cfg(feature = "cw32f020")]
    pub fn new(
        regs: &'static pac::DMA,
        _irqs: impl Binding<interrupt::DMACH1> + Binding<interrupt::DMACH2>,
    ) -> Self {
        Self::init(regs)
    }

    fn init(regs: &'static pac::DMA) -> Self {
        // AHBEN.DMA（bit0，SYSCTRL_AHBEN_DMA_Pos=0UL，各家厂商头文件）：
        // 经典组无 KEY，读改写保留其它外设使能位（lib.rs GPIO 门控同款）
        let sysctrl = unsafe { &*(pac::SYSCTRL_BASE as *const crate::Sysctrl) };
        sysctrl.ahben.modify(|v| v | (1 << 0));
        // ICR 是 R1W0（厂商 DMA_ClearITPendingBit 配方）：写 0 清掉全部残留标志
        regs.icr.write(0);
        ClassicDma { regs }
    }

    /// 本家族通道数（f030/f020/l083=5，l031 组=4）
    #[inline]
    pub fn channel_count() -> usize {
        CLASSIC_CHANNELS
    }

    /// 取通道（`n` = 1..=本家族通道数，数字越小优先级越高）
    ///
    /// 注意 f020 的通道 4/5 可取出做轮询传输（[`Channel::enable`]/
    /// [`Channel::status`]），但无 NVIC 线，[`Channel::wait`] 会 panic。
    pub fn channel(&mut self, n: u8) -> Channel<'_> {
        assert!(
            (1..=CLASSIC_CHANNELS as u8).contains(&n),
            "DMA 通道号必须为 1..={CLASSIC_CHANNELS}（Got {n}）"
        );
        Channel {
            regs: self.regs,
            idx: (n - 1) as usize,
        }
    }

    /// 存储器→存储器异步搬运（自动占用通道 3，把 1/2 留给实时外设——
    /// L012 版同款策略；f020 上通道 3 在 IRQ10 `DMACH2` 组内，同样可用）
    ///
    /// `src` / `dst` 长度必须相等且非空、≤65535；元素类型决定位宽
    /// （u8/u16/u32 → 8/16/32bit）。
    pub async fn mem_to_mem<T: Copy>(&mut self, src: &[T], dst: &mut [T]) -> Result<(), DmaErrorClassic> {
        let width =
            ClassicWidth::for_size(core::mem::size_of::<T>()).ok_or(DmaErrorClassic::UnsupportedWidth)?;
        if src.is_empty() || src.len() != dst.len() || src.len() > 0xFFFF {
            return Err(DmaErrorClassic::BadLength);
        }
        let cfg = ClassicConfig::mem_to_mem(
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
    /// `periph_addr` 为外设数据寄存器地址，`hs` 为各家手册 §8.8.4 请求表
    /// 的编码（[`hs`] 常量）。缓冲元素宽度必须与 `width` 一致。
    pub async fn periph_to_mem<T: Copy>(
        &mut self,
        periph_addr: u32,
        buf: &mut [T],
        width: ClassicWidth,
        hs: u8,
    ) -> Result<(), DmaErrorClassic> {
        if ClassicWidth::for_size(core::mem::size_of::<T>()) != Some(width) {
            return Err(DmaErrorClassic::UnsupportedWidth);
        }
        if buf.is_empty() || buf.len() > 0xFFFF {
            return Err(DmaErrorClassic::BadLength);
        }
        let cfg = ClassicConfig::periph_to_mem(
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
    /// 与 [`ClassicDma::periph_to_mem`] 对称：源自增、目的固定。
    pub async fn mem_to_periph<T: Copy>(
        &mut self,
        buf: &[T],
        periph_addr: u32,
        width: ClassicWidth,
        hs: u8,
    ) -> Result<(), DmaErrorClassic> {
        if ClassicWidth::for_size(core::mem::size_of::<T>()) != Some(width) {
            return Err(DmaErrorClassic::UnsupportedWidth);
        }
        if buf.is_empty() || buf.len() > 0xFFFF {
            return Err(DmaErrorClassic::BadLength);
        }
        let cfg = ClassicConfig::mem_to_periph(
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
    pub fn regs(&self) -> &'static pac::DMA {
        self.regs
    }
}

/// 经典组单个 DMA 通道（每通道独立块：CSR/CNT/SRCADDR/DSTADDR/TRIG）
#[cfg(any(feature = "cw32f030", feature = "cw32f020", feature = "cw32l031", feature = "cw32l052", feature = "cw32l083", feature = "cw32r031", feature = "cw32w031"))]
pub struct Channel<'d> {
    regs: &'d pac::DMA,
    idx: usize,
}

#[cfg(any(feature = "cw32f030", feature = "cw32f020", feature = "cw32l031", feature = "cw32l052", feature = "cw32l083", feature = "cw32r031", feature = "cw32w031"))]
impl Channel<'_> {
    #[inline]
    fn ch(&self) -> &'static pac::DMACHANNEL {
        classic_ch(self.idx)
    }

    /// 通道号（1..=本家族通道数）
    #[inline]
    pub fn index(&self) -> u8 {
        (self.idx + 1) as u8
    }

    /// 本通道在全局 ISR/ICR 里的 TC/TE 位掩码（每通道 4bit 打包）
    #[inline]
    fn isr_mask(&self) -> u32 {
        0b11 << (4 * self.idx)
    }

    /// 按配置装载通道（自动 `EN=0 → 清标志 → 写寄存器 → EN=1`）
    ///
    /// **每轮传输前都要调用**：经典组 `CNT.REPEAT` 同样被硬件清 0（厂商
    /// `DMA_Init` 每轮都重写 `CNT`）。
    pub fn configure(&self, cfg: &ClassicConfig) {
        let ch = self.ch();
        ch.csr.modify(|v| v & !cbits::CSR_EN);
        self.clear_flags();
        ch.cnt.write(cfg.encode_cnt());
        ch.srcaddr.write(cfg.src);
        ch.dstaddr.write(cfg.dst);
        ch.trig.write(cfg.encode_trig());
        ch.csr.write(cfg.encode_csr() | cbits::CSR_EN);
    }

    /// 软件触发启动（硬件触发模式下无需调用）
    ///
    /// `SOFTSRC` 在传输完成后由硬件自动清零，重复传输时要重新写。
    pub fn start(&self) {
        self.ch().trig.modify(|v| v | cbits::TRIG_SOFTSRC);
    }

    /// 只置 `EN`（用于"先配好、等外设就绪再开"的场景）
    pub fn enable(&self) {
        self.ch().csr.modify(|v| v | cbits::CSR_EN);
    }

    /// 清 `EN`（停止通道；正在进行的传输会被中止）
    pub fn stop(&self) {
        self.ch().csr.modify(|v| v & !cbits::CSR_EN);
    }

    /// 通道状态（`CSR.STATUS[10:8]`，编码同 L012 表 8-2：101=传输完成）
    pub fn status(&self) -> u8 {
        ((self.ch().csr.read() & cbits::CSR_STATUS_MASK) >> cbits::CSR_STATUS_POS) as u8
    }

    /// 解析当前状态为 `Result`（STATUS=1..4 为错误类别；经典组 CSR 无
    /// 独立 TE 位，直接按 STATUS 解码——厂商 `DMA_GetFlagStatus` 同源）
    pub fn check(&self) -> Result<(), DmaErrorClassic> {
        decode_status(self.status())
    }

    /// 清本通道的 TC/TE 标志（`ICR` 为 R1W0：写 0 清除，保留其余位）
    pub fn clear_flags(&self) {
        self.regs.icr.write(!self.isr_mask());
    }

    /// 异步等待传输完成/出错（TC/TE 中断驱动）
    ///
    /// 经典组 CSR 无 TC/TE 位，完成/出错以**全局 ISR** 的本通道 4bit 组
    /// 为准。等待时才开本通道的 `CSR.TCIE|TEIE`，IRQ 唤醒即压掉（防风暴）；
    /// ISR 标志命中后先按 CSR.STATUS 解码、再经 `ICR`（R1W0）清除。
    ///
    /// 前置条件：对应 NVIC 线已绑定（见 [`ClassicDma::new`]）；NVIC 线由
    /// 本方法幂等使能。无内置超时——需要限时就用
    /// `embassy_time::with_timeout` 包一层。**f020 通道 4/5 无 NVIC 线
    /// （IRQ11 保留），调用即 panic**。
    pub async fn wait(&mut self) -> Result<(), DmaErrorClassic> {
        // NVIC 线（幂等）：f030/l083 = ch1→DMACH1、ch2/3→DMACH23、
        // ch4/5→DMACH45；l031 组 = ch4→DMACH4；f020 = ch2/3→DMACH2、
        // ch4/5 无线（panic）
        #[cfg(any(feature = "cw32f030", feature = "cw32l083"))]
        match self.idx {
            0 => crate::interrupt::nvic_enable::<interrupt::DMACH1>(),
            1 | 2 => crate::interrupt::nvic_enable::<interrupt::DMACH23>(),
            _ => crate::interrupt::nvic_enable::<interrupt::DMACH45>(),
        }
        #[cfg(any(
            feature = "cw32l031",
            feature = "cw32l052",
            feature = "cw32r031",
            feature = "cw32w031"
        ))]
        match self.idx {
            0 => crate::interrupt::nvic_enable::<interrupt::DMACH1>(),
            1 | 2 => crate::interrupt::nvic_enable::<interrupt::DMACH23>(),
            _ => crate::interrupt::nvic_enable::<interrupt::DMACH4>(),
        }
        #[cfg(feature = "cw32f020")]
        {
            assert!(
                self.idx < 3,
                "f020 DMA 通道 4/5 无 NVIC 线（IRQ11 保留），不支持异步 wait"
            );
            match self.idx {
                0 => crate::interrupt::nvic_enable::<interrupt::DMACH1>(),
                _ => crate::interrupt::nvic_enable::<interrupt::DMACH2>(),
            }
        }
        poll_fn(|cx| {
            CLASSIC_WAKERS[self.idx].register(cx.waker());
            let isr = self.regs.isr.read();
            if isr & self.isr_mask() != 0 {
                // 命中：先解码（STATUS 还在），再压掉中断使能、清标志
                let te = isr & (1 << (4 * self.idx + 1)) != 0;
                let r = if te {
                    decode_status(self.status())
                } else {
                    Ok(())
                };
                self.ch().csr.modify(|v| v & !(cbits::CSR_TCIE | cbits::CSR_TEIE));
                self.clear_flags();
                Poll::Ready(r)
            } else {
                self.ch()
                    .csr
                    .modify(|v| v | cbits::CSR_TCIE | cbits::CSR_TEIE);
                Poll::Pending
            }
        })
        .await
    }

    /// 通道寄存器块原始访问
    pub fn regs(&self) -> &'static pac::DMACHANNEL {
        self.ch()
    }
}

/// 从经典组 CSR.STATUS 解码传输结果（厂商 `DMA_CHANNEL_STATUS_*` 编码）
#[cfg(any(feature = "cw32f030", feature = "cw32f020", feature = "cw32l031", feature = "cw32l052", feature = "cw32l083", feature = "cw32r031", feature = "cw32w031"))]
fn decode_status(status: u8) -> Result<(), DmaErrorClassic> {
    match status {
        1 => Err(DmaErrorClassic::AddressOutOfRange),
        2 => Err(DmaErrorClassic::Aborted),
        3 => Err(DmaErrorClassic::SourceError),
        4 => Err(DmaErrorClassic::DestError),
        _ => Ok(()),
    }
}

/// 经典组 DMA 组中断公共处理（读全局 ISR，对应通道有 TC/TE 即单发压掉
/// TCIE/TEIE 并唤醒等待者；ISR 标志不动——poll 复查命中后经 ICR R1W0
/// 清除，纪律同 L012 分支）
#[cfg(any(
    feature = "cw32f030",
    feature = "cw32f020",
    feature = "cw32l031",
    feature = "cw32l052",
    feature = "cw32l083",
    feature = "cw32r031",
    feature = "cw32w031"
))]
fn on_irq_channels(base: usize, count: usize, regs: &'static pac::DMA) {
    let isr = regs.isr.read();
    for i in 0..count {
        let idx = base + i;
        if isr & (0b11 << (4 * idx)) != 0 {
            // 压掉该通道的中断使能（下次 poll 会重新打开）
            classic_ch(idx).csr.modify(|v| v & !(cbits::CSR_TCIE | cbits::CSR_TEIE));
            CLASSIC_WAKERS[idx].wake();
        }
    }
}

/// 经典组 DMA 通道 1 独占中断标记（IRQ9 `DMACH1`，全经典组同名）
#[cfg(any(feature = "cw32f030", feature = "cw32f020", feature = "cw32l031", feature = "cw32l052", feature = "cw32l083", feature = "cw32r031", feature = "cw32w031"))]
pub struct Dmach1Irq;
#[cfg(any(feature = "cw32f030", feature = "cw32f020", feature = "cw32l031", feature = "cw32l052", feature = "cw32l083", feature = "cw32r031", feature = "cw32w031"))]
impl IrqHandler<interrupt::DMACH1> for Dmach1Irq {
    fn call() {
        on_irq_channels(0, 1, unsafe { &*(pac::DMA_BASE as *const pac::DMA) });
    }
}

/// 经典组 DMA 通道 2/3 共享中断标记（IRQ10 `DMACH23`；f020 向量符号为
/// `DMACH2`，厂商注释同为通道 2/3）
#[cfg(any(
    feature = "cw32f030",
    feature = "cw32l031",
    feature = "cw32l052",
    feature = "cw32l083",
    feature = "cw32r031",
    feature = "cw32w031"
))]
pub struct Dmach23Irq;
#[cfg(any(
    feature = "cw32f030",
    feature = "cw32l031",
    feature = "cw32l052",
    feature = "cw32l083",
    feature = "cw32r031",
    feature = "cw32w031"
))]
impl IrqHandler<interrupt::DMACH23> for Dmach23Irq {
    fn call() {
        on_irq_channels(1, 2, unsafe { &*(pac::DMA_BASE as *const pac::DMA) });
    }
}

/// f020 DMA 通道 2/3 共享中断标记（IRQ10 `DMACH2`）
#[cfg(feature = "cw32f020")]
pub struct Dmach2Irq;
#[cfg(feature = "cw32f020")]
impl IrqHandler<interrupt::DMACH2> for Dmach2Irq {
    fn call() {
        on_irq_channels(1, 2, unsafe { &*(pac::DMA_BASE as *const pac::DMA) });
    }
}

/// f030/l083 DMA 通道 4/5 共享中断标记（IRQ11 `DMACH45`）
#[cfg(any(feature = "cw32f030", feature = "cw32l083"))]
pub struct Dmach45Irq;
#[cfg(any(feature = "cw32f030", feature = "cw32l083"))]
impl IrqHandler<interrupt::DMACH45> for Dmach45Irq {
    fn call() {
        on_irq_channels(3, 2, unsafe { &*(pac::DMA_BASE as *const pac::DMA) });
    }
}

/// l031 组 DMA 通道 4 独占中断标记（IRQ11 `DMACH4`）
#[cfg(any(
    feature = "cw32l031",
    feature = "cw32l052",
    feature = "cw32r031",
    feature = "cw32w031"
))]
pub struct Dmach4Irq;
#[cfg(any(
    feature = "cw32l031",
    feature = "cw32l052",
    feature = "cw32r031",
    feature = "cw32w031"
))]
impl IrqHandler<interrupt::DMACH4> for Dmach4Irq {
    fn call() {
        on_irq_channels(3, 1, unsafe { &*(pac::DMA_BASE as *const pac::DMA) });
    }
}

// 经典组位面/编码单元测试（位常量来自经典组 PAC，宿主机即可跑）
#[cfg(all(test, any(feature = "cw32f030", feature = "cw32f020", feature = "cw32l031", feature = "cw32l052", feature = "cw32l083", feature = "cw32r031", feature = "cw32w031")))]
mod classic_tests {
    use super::*;

    #[test]
    fn classic_width_codes_and_bytes() {
        assert_eq!(ClassicWidth::B8.code(), 0);
        assert_eq!(ClassicWidth::B16.code(), 1);
        assert_eq!(ClassicWidth::B32.code(), 2);
        assert_eq!(ClassicWidth::B8.bytes(), 1);
        assert_eq!(ClassicWidth::B16.bytes(), 2);
        assert_eq!(ClassicWidth::B32.bytes(), 4);
        assert_eq!(ClassicWidth::for_size(4), Some(ClassicWidth::B32));
        assert_eq!(ClassicWidth::for_size(3), None);
    }

    // CSR 位面与厂商头文件/手工编码比对：SIZE[7:6]、TRANS@3、SRCINC@4、
    // DSTINC@5、无 RESTART/重载位（经典组 IP 没有）
    #[test]
    fn classic_csr_encoding() {
        let cfg = ClassicConfig::mem_to_mem(0x2000_0000, 0x2000_0100, 32, ClassicWidth::B32);
        let csr = cfg.encode_csr();
        assert_eq!(csr & cbits::CSR_STATUS_MASK, 0);
        assert_eq!(csr >> cbits::CSR_SIZE_POS, 2); // 32bit
        assert_ne!(csr & cbits::CSR_SRCINC, 0);
        assert_ne!(csr & cbits::CSR_DSTINC, 0);
        assert_eq!(csr & cbits::CSR_TRANS, 0); // BULK
        assert_eq!(csr & cbits::CSR_EN, 0); // EN 由 configure 单独置位
        assert_eq!(csr & !0xFF, 0); // 高位无任何编码位（经典组 CSR 只用到 bit7）
    }

    #[test]
    fn classic_cnt_and_trig_encoding() {
        let mut cfg = ClassicConfig::mem_to_mem(0, 0, 1000, ClassicWidth::B16);
        cfg.repeat = 3;
        assert_eq!(cfg.encode_cnt(), 1000 | (3 << 16));
        assert_eq!(cfg.encode_trig(), 0); // 软件触发

        let cfg = ClassicConfig::periph_to_mem(
            0x4000_0030,
            0x2000_0000,
            8,
            ClassicWidth::B16,
            hs::UART1_RX,
        );
        assert_eq!(
            cfg.encode_trig(),
            cbits::TRIG_TYPE | ((hs::UART1_RX as u32) << cbits::TRIG_HARDSRC_POS)
        );
    }

    #[test]
    fn classic_repeat_is_clamped_to_valid_range() {
        let mut cfg = ClassicConfig::mem_to_mem(0, 0, 1, ClassicWidth::B8);
        cfg.repeat = 0; // REPEAT 不可为 0
        assert_eq!(cfg.encode_cnt() >> 16, 1);
        cfg.repeat = 0xFF;
        assert_eq!(cfg.encode_cnt() >> 16, 15);
    }

    // 每通道 4bit 打包：idx=0→0b11<<0、idx=1→0b11<<4、idx=4→0b11<<16
    #[test]
    fn classic_isr_mask_packing() {
        let ch = |idx: usize| 0b11u32 << (4 * idx);
        assert_eq!(ch(0), 0x0000_0003);
        assert_eq!(ch(1), 0x0000_0030);
        assert_eq!(ch(2), 0x0000_0300);
        assert_eq!(ch(3), 0x0000_3000);
        assert_eq!(ch(4), 0x0003_0000);
        // 与厂商库 DMA_IT_TC1..TE5 / DMA_IT_ALL=0x33333 一致
        assert_eq!(ch(0) | ch(1) | ch(2) | ch(3) | ch(4), 0x0003_3333);
    }

    #[test]
    fn classic_status_decode() {
        assert_eq!(decode_status(0), Ok(()));
        assert_eq!(decode_status(5), Ok(())); // TRANSCOMPLETE
        assert_eq!(decode_status(1), Err(DmaErrorClassic::AddressOutOfRange));
        assert_eq!(decode_status(2), Err(DmaErrorClassic::Aborted));
        assert_eq!(decode_status(3), Err(DmaErrorClassic::SourceError));
        assert_eq!(decode_status(4), Err(DmaErrorClassic::DestError));
    }

    // PAC 位面抽验：编码值必须与生成 PAC 常量一致（防手抄错位）
    #[test]
    fn classic_bits_match_pac() {
        use super::dc;
        assert_eq!(cbits::CSR_EN, dc::csr1::en_MASK);
        assert_eq!(cbits::CSR_TCIE, dc::csr1::tcie_MASK);
        assert_eq!(cbits::CSR_TEIE, dc::csr1::teie_MASK);
        assert_eq!(cbits::CSR_TRANS, dc::csr1::trans_MASK);
        assert_eq!(cbits::CSR_SRCINC, dc::csr1::srcinc_MASK);
        assert_eq!(cbits::CSR_DSTINC, dc::csr1::dstinc_MASK);
        assert_eq!(cbits::CSR_SIZE_POS, dc::csr1::size_POS);
        assert_eq!(cbits::CSR_STATUS_POS, dc::csr1::status_POS);
        // 逐通道同值（csr5/trig5/cnt5 与 csr1 一致；有 5 通道的家族才编译）
        #[cfg(any(feature = "cw32f030", feature = "cw32f020", feature = "cw32l083"))]
        {
            assert_eq!(dc::csr5::en_MASK, dc::csr1::en_MASK);
            assert_eq!(dc::cnt5::repeat_POS, dc::cnt1::repeat_POS);
            assert_eq!(dc::trig5::hardsrc_POS, dc::trig1::hardsrc_POS);
            assert_eq!(dc::isr::tc5_MASK, 1 << 16);
        }
        // ISR 4bit 打包抽验（PAC 常量）
        assert_eq!(dc::isr::tc1_POS, 0);
        assert_eq!(dc::isr::te1_POS, 1);
        assert_eq!(dc::isr::tc2_POS, 4);
        assert_eq!(dc::isr::te2_POS, 5);
        assert_eq!(dc::isr::tc3_POS, 8);
        assert_eq!(dc::isr::tc4_POS, 12);
    }
}

// L012 位面专属（经典组分支在下方另有自己的测试模块）
#[cfg(all(test, feature = "cw32l012"))]
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
