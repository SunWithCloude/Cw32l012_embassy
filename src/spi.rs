//! 异步 SPI 主机（中断驱动逐字节，`embedded-hal-async` SpiBus）。
//!
//! 片选由用户 GPIO 管理（软件 CS）：传输前拉低、`flush` 后拉高。
//! 时序默认 Mode 0、MSB first、8-bit。
//!
//! 家族差异见 [`SpiFamily`]（L012:BR 7 位@CR1[24]、EN@CR2[0]、
//! SYSCTRL 带 KEY;F030 经典:BR 3 位@CR1[3..5]、EN@CR1[6]、无 KEY、
//! ICR 同为 R1W0——厂商库 `SPIx->ICR &= ~flag` 实锤）。

use core::future::poll_fn;
use core::task::Poll;

use embassy_sync::waitqueue::AtomicWaker;

use crate::gpio::{GpioPort, Pin};
use crate::interrupt::{Binding, IrqHandler, interrupt};
use crate::pac;

// ================= 家族位常量面 =================

/// 共用 ISR/IER 位（两代相同：TXE=0、RXNE=1、BUSY=8）
mod common {
    /// 发送缓冲空
    pub const ISR_TXE: u32 = 1 << 0;
    /// 接收缓冲非空
    pub const ISR_RXNE: u32 = 1 << 1;
    /// 总线忙
    pub const ISR_BUSY: u32 = 1 << 8;
}

#[cfg(feature = "cw32l012")]
mod fam {
    pub use crate::pac::spi as sb;
    /// SYSCTRL KEY 仅 L012
    pub const KEY: u32 = crate::SYSCTRL_KEY;
    /// 门控:全在 APBEN1(SPI1=2/SPI2=13/SPI3=14)
    pub fn gate(sysctrl: &crate::Sysctrl, idx: usize) {
        let g = match idx {
            0 => crate::pac::sysctrl::APBEN1_SPI1,
            1 => crate::pac::sysctrl::APBEN1_SPI2,
            _ => crate::pac::sysctrl::APBEN1_SPI3,
        };
        sysctrl.apben1.write(KEY | g);
    }
    /// EN 在 CR2[0];CR1 可写前提是 CR2.EN=0
    pub const CR2_EN: u32 = sb::CR2_EN;
    /// BR 位域:7 位,在 CR1[24..31]
    pub const BR_POS: u32 = sb::CR1_BR_POS;
    pub const BR_MASK: u32 = sb::CR1_BR_MASK; // 0x7F
    /// CR1 组合位
    pub const CR1_MSTR: u32 = sb::CR1_MSTR;
    pub const CR1_SSM: u32 = sb::CR1_SSM;
    pub const CR1_CPOL: u32 = sb::CR1_CPOL;
    pub const CR1_CPHA: u32 = sb::CR1_CPHA;
    pub const CR1_LSBF: u32 = sb::CR1_LSBF;
    /// 使能序列
    pub fn enable(regs: &crate::Spi, cr1: u32) {
        regs.cr2.write(0); // EN=0 时才能写 CR1
        regs.cr1.write(cr1);
        regs.ssi.write(1); // 内部 CS 拉高
        regs.icr.write(0); // R1W0:写 0 清全部标志
        regs.cr2.write(CR2_EN);
    }
}

#[cfg(feature = "cw32f030")]
mod fam {
    /// F030 SYSCTRL 的 AHBEN/APBEN 无 KEY 保护(厂商库 |= 直写)
    /// 门控:SPI1=APBEN2.8、SPI2=APBEN1.6
    pub fn gate(sysctrl: &crate::Sysctrl, idx: usize) {
        match idx {
            0 => sysctrl.apben2.modify(|v| v | (1 << 8)),
            1 => sysctrl.apben1.modify(|v| v | (1 << 6)),
            _ => unreachable!("F030 无 SPI3"),
        }
    }
    /// BR 位域:3 位,在 CR1[3..5]
    pub const BR_POS: u32 = 3;
    pub const BR_MASK: u32 = 0x7 << BR_POS;
    /// CR1 组合位(F030 头 SPIx_CR1_*)
    pub const CR1_MSTR: u32 = 1 << 2;
    pub const CR1_SSM: u32 = 1 << 9;
    pub const CR1_CPOL: u32 = 1 << 1;
    pub const CR1_CPHA: u32 = 1 << 0;
    pub const CR1_LSBF: u32 = 1 << 7;
    pub const CR1_EN: u32 = 1 << 6;
    /// 使能序列:CR1 一把写(EN 在 CR1 里,无 CR2.EN 前置约束)
    pub fn enable(regs: &crate::Spi, cr1: u32) {
        regs.icr.modify(|v| v); // R1W0:保持值写入即可清历史标志(全 1 无动作位)
        regs.cr1.write(cr1 | CR1_EN);
        regs.ssi.write(1);
    }
}

use common::*;
use fam as f;

/// SPI 配置
#[derive(Debug, Clone, Copy)]
pub struct Config {
    /// 目标 SCK 频率（自动取不超过该值的 2^n 分频）
    pub frequency: u32,
    /// SPI 模式（CPOL/CPHA 组合）
    pub mode: Mode,
    /// 高位先行（默认 true）
    pub msb_first: bool,
}

/// SPI 时序模式
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Mode {
    /// CPOL=0 CPHA=0
    Mode0,
    /// CPOL=0 CPHA=1
    Mode1,
    /// CPOL=1 CPHA=0
    Mode2,
    /// CPOL=1 CPHA=1
    Mode3,
}

impl Default for Config {
    fn default() -> Self {
        Config {
            frequency: 1_000_000,
            mode: Mode::Mode0,
            msb_first: true,
        }
    }
}

/// 实例索引（0/1/2 = SPI1/2/3）
fn instance_of(regs: &'static crate::Spi) -> usize {
    match regs as *const _ as usize {
        pac::SPI1_BASE => 0,
        pac::SPI2_BASE => 1,
        #[cfg(feature = "cw32l012")]
        pac::SPI3_BASE => 2,
        _ => unreachable!("非 SPI 寄存器块"),
    }
}

static WAKER: [AtomicWaker; 3] = [const { AtomicWaker::new() }; 3];

/// 异步 SPI 主机
pub struct Spi {
    regs: &'static crate::Spi,
    idx: usize,
}

impl Spi {
    /// 初始化 SPI 主机（全双工、软件 CS）并配置引脚复用。
    #[cfg(feature = "cw32l012")]
    pub fn new(
        regs: &'static crate::Spi,
        sck: Pin<impl GpioPort>,
        mosi: Pin<impl GpioPort>,
        miso: Pin<impl GpioPort>,
        af: u8,
        _irqs: impl Binding<interrupt::SPI1> + Binding<interrupt::SPI23>,
        config: Config,
    ) -> Self {
        Self::new_inner(regs, sck, mosi, miso, af, config)
    }

    #[cfg(feature = "cw32f030")]
    pub fn new(
        regs: &'static crate::Spi,
        sck: Pin<impl GpioPort>,
        mosi: Pin<impl GpioPort>,
        miso: Pin<impl GpioPort>,
        af: u8,
        _irqs: impl Binding<interrupt::SPI1> + Binding<interrupt::SPI2>,
        config: Config,
    ) -> Self {
        Self::new_inner(regs, sck, mosi, miso, af, config)
    }

    /// 初始化 SPI1（IRQ 绑定收窄到 SPI1）。
    pub fn new_spi1(
        regs: &'static crate::Spi,
        sck: Pin<impl GpioPort>,
        mosi: Pin<impl GpioPort>,
        miso: Pin<impl GpioPort>,
        af: u8,
        _irqs: impl Binding<interrupt::SPI1>,
        config: Config,
    ) -> Self {
        Self::new_inner(regs, sck, mosi, miso, af, config)
    }

    /// 初始化 SPI2（IRQ 绑定收窄到 SPI23）。
    #[cfg(feature = "cw32l012")]
    pub fn new_spi2(
        regs: &'static crate::Spi,
        sck: Pin<impl GpioPort>,
        mosi: Pin<impl GpioPort>,
        miso: Pin<impl GpioPort>,
        af: u8,
        _irqs: impl Binding<interrupt::SPI23>,
        config: Config,
    ) -> Self {
        Self::new_inner(regs, sck, mosi, miso, af, config)
    }

    /// 初始化 SPI2(F030,独立向量)。
    #[cfg(feature = "cw32f030")]
    pub fn new_spi2(
        regs: &'static crate::Spi,
        sck: Pin<impl GpioPort>,
        mosi: Pin<impl GpioPort>,
        miso: Pin<impl GpioPort>,
        af: u8,
        _irqs: impl Binding<interrupt::SPI2>,
        config: Config,
    ) -> Self {
        Self::new_inner(regs, sck, mosi, miso, af, config)
    }

    /// 初始化 SPI3（IRQ 绑定收窄到 SPI23;仅 L012）。
    #[cfg(feature = "cw32l012")]
    pub fn new_spi3(
        regs: &'static crate::Spi,
        sck: Pin<impl GpioPort>,
        mosi: Pin<impl GpioPort>,
        miso: Pin<impl GpioPort>,
        af: u8,
        _irqs: impl Binding<interrupt::SPI23>,
        config: Config,
    ) -> Self {
        Self::new_inner(regs, sck, mosi, miso, af, config)
    }

    fn new_inner(
        regs: &'static crate::Spi,
        sck: Pin<impl GpioPort>,
        mosi: Pin<impl GpioPort>,
        miso: Pin<impl GpioPort>,
        af: u8,
        config: Config,
    ) -> Self {
        let idx = instance_of(regs);
        let sysctrl = unsafe { &*(pac::SYSCTRL_BASE as *const crate::Sysctrl) };
        f::gate(sysctrl, idx);

        sck.set_af(af);
        mosi.set_af(af);
        miso.set_af(af);

        let pclk = crate::PCLK_HZ.load(core::sync::atomic::Ordering::Relaxed);
        // 波特率 = PCLK / (BR + 1)
        let div = pclk.div_ceil(config.frequency).max(1);
        let br = (div - 1).min(f::BR_MASK >> f::BR_POS);

        let (cpol, cpha) = match config.mode {
            Mode::Mode0 => (0, 0),
            Mode::Mode1 => (0, f::CR1_CPHA),
            Mode::Mode2 => (f::CR1_CPOL, 0),
            Mode::Mode3 => (f::CR1_CPOL, f::CR1_CPHA),
        };
        let lsbf = if config.msb_first { 0 } else { f::CR1_LSBF };

        // 使能 NVIC 中断线(typelevel 号两代同位)
        match idx {
            0 => crate::interrupt::nvic_enable::<interrupt::SPI1>(),
            #[cfg(feature = "cw32l012")]
            _ => crate::interrupt::nvic_enable::<interrupt::SPI23>(),
            #[cfg(feature = "cw32f030")]
            _ => crate::interrupt::nvic_enable::<interrupt::SPI2>(),
        }
        let cr1 = (br << f::BR_POS) | f::CR1_MSTR | f::CR1_SSM | cpol | cpha | lsbf;
        f::enable(regs, cr1);

        Self { regs, idx }
    }

    /// 等 TXE（可写）
    async fn wait_txe(&mut self) {
        poll_fn(|cx| {
            WAKER[self.idx].register(cx.waker());
            if self.regs.isr.read() & ISR_TXE != 0 {
                Poll::Ready(())
            } else {
                self.regs.ier.modify(|v| v | ISR_TXE);
                Poll::Pending
            }
        })
        .await;
    }

    /// 等 RXNE（可读）
    async fn wait_rxne(&mut self) {
        poll_fn(|cx| {
            WAKER[self.idx].register(cx.waker());
            if self.regs.isr.read() & ISR_RXNE != 0 {
                Poll::Ready(())
            } else {
                self.regs.ier.modify(|v| v | ISR_RXNE);
                Poll::Pending
            }
        })
        .await;
    }

    /// 等 BUSY 清零（传输结束）
    async fn wait_idle(&mut self) {
        poll_fn(|cx| {
            WAKER[self.idx].register(cx.waker());
            if self.regs.isr.read() & ISR_BUSY == 0 {
                Poll::Ready(())
            } else {
                // BUSY 无中断源：挂 RXNE（每字节产生）作为唤醒代理
                self.regs.ier.modify(|v| v | ISR_RXNE);
                Poll::Pending
            }
        })
        .await;
    }

    /// 读写单字节（全双工）
    async fn transfer_byte(&mut self, b: u8) -> u8 {
        self.wait_txe().await;
        self.regs.dr.write(b as u32);
        self.wait_rxne().await;
        self.regs.dr.read() as u8
    }
}

impl embedded_hal_async::spi::ErrorType for Spi {
    type Error = core::convert::Infallible;
}

impl embedded_hal_async::spi::SpiBus for Spi {
    async fn read(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
        for w in words.iter_mut() {
            *w = self.transfer_byte(0xFF).await;
        }
        Ok(())
    }

    async fn write(&mut self, words: &[u8]) -> Result<(), Self::Error> {
        for b in words {
            self.transfer_byte(*b).await;
        }
        Ok(())
    }

    async fn transfer(&mut self, read: &mut [u8], write: &[u8]) -> Result<(), Self::Error> {
        for i in 0..core::cmp::max(read.len(), write.len()) {
            let w = write.get(i).copied().unwrap_or(0xFF);
            let r = self.transfer_byte(w).await;
            if let Some(slot) = read.get_mut(i) {
                *slot = r;
            }
        }
        Ok(())
    }

    async fn transfer_in_place(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
        for w in words.iter_mut() {
            *w = self.transfer_byte(*w).await;
        }
        Ok(())
    }

    async fn flush(&mut self) -> Result<(), Self::Error> {
        self.wait_idle().await;
        Ok(())
    }
}

// ================= 中断派发 =================

/// 单实例中断处理：唤醒等待者并压掉无等待源的中断使能。
fn on_irq(idx: usize, regs: &'static crate::Spi) {
    let isr = regs.isr.read();
    if isr & (ISR_TXE | ISR_RXNE) != 0 {
        WAKER[idx].wake();
        if isr & ISR_TXE != 0 {
            regs.ier.modify(|v| v & !ISR_TXE);
        }
        if isr & ISR_RXNE != 0 {
            regs.ier.modify(|v| v & !ISR_RXNE);
        }
    }
}

/// SPI1 中断标记（IRQ25）
pub struct Spi1Irq;
impl IrqHandler<interrupt::SPI1> for Spi1Irq {
    fn call() {
        on_irq(0, unsafe { &*(pac::SPI1_BASE as *const crate::Spi) });
    }
}

#[cfg(feature = "cw32l012")]
/// SPI2 中断标记（IRQ26，与 SPI3 共享向量；L012）
pub struct Spi2Irq;
#[cfg(feature = "cw32l012")]
impl IrqHandler<interrupt::SPI23> for Spi2Irq {
    fn call() {
        on_irq(1, unsafe { &*(pac::SPI2_BASE as *const crate::Spi) });
        on_irq(2, unsafe { &*(pac::SPI3_BASE as *const crate::Spi) });
    }
}

#[cfg(feature = "cw32l012")]
/// SPI3 中断标记（IRQ26，与 SPI2 共享向量；L012）
pub struct Spi3Irq;
#[cfg(feature = "cw32l012")]
impl IrqHandler<interrupt::SPI23> for Spi3Irq {
    fn call() {
        on_irq(1, unsafe { &*(pac::SPI2_BASE as *const crate::Spi) });
        on_irq(2, unsafe { &*(pac::SPI3_BASE as *const crate::Spi) });
    }
}

/// SPI2 中断标记（IRQ26;F030 仅 SPI1/SPI2）
#[cfg(feature = "cw32f030")]
pub struct Spi2Irq;
#[cfg(feature = "cw32f030")]
impl IrqHandler<interrupt::SPI2> for Spi2Irq {
    fn call() {
        on_irq(1, unsafe { &*(pac::SPI2_BASE as *const crate::Spi) });
    }
}
