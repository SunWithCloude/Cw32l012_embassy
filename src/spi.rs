//! 异步 SPI 主机（中断驱动逐字节，`embedded-hal-async` SpiBus）。
//!
//! 片选由用户 GPIO 管理（软件 CS）：传输前拉低、`flush` 后拉高。
//! 时序默认 Mode 0、MSB first、8-bit。
//!
//! 家族差异见 [`SpiFamily`]（L012:BR 7 位@CR1[24]、EN@CR2[0]、
//! SYSCTRL 带 KEY;经典组 F030/F020/F003:BR 3 位@CR1[3..5]、EN@CR1[6]、
//! 无 KEY、ICR 同为 R1W0——厂商库 `SPIx->ICR &= ~flag` 实锤;F020/F003
//! 的 CR1 位面与 F030 逐 PAC 比对一致,单实例）。
//!
//! l083（经典组双实例）：SPI1/SPI2 寄存器面与 F030 同构（逐 PAC 比对），
//! 中断为独立向量（`SPI1` IRQ25 / `SPI2` IRQ26）。引脚组合摘要（数据
//! 手册表 5-3..5-8，全表见 cw32-hal2 数据仓库 `af/cw32l083-serial.yaml`，
//! TX/RX 同组同 AF）：
//!
//! | 实例 | 常用组合（SCK/MISO/MOSI） |
//! |------|---------------------------|
//! | SPI1 | PA10/PA11/PA12（AF5）、PB03/PB04/PB05（AF5）、PB13/PB14/PB15（AF5）、PC01/PC02/PC03（AF3）、PD13/PD14/PD15（AF4）、PE04/PE05/PE06（AF4）、PE09/PE10/PE11（AF4） |
//! | SPI2 | PA02/PA00/PA01（AF5）、PB08/PB07/PB06（AF5）、PB10/PC14/PC15（AF5）、PB13/PB14/PB15（AF4）、PD01/PD03/PD04（AF5）、PD09/PD10/PD11（AF4）、PE13/PE14/PE15（AF4） |
//!
//! l083 的 DMA 触发源映射 SPI1/SPI2 **全接线**——SPI2 实例仅 f030/l083
//! 有，触发源 SPI2_RX/SPI2_TX=0x08/0x09 两家手册请求表同值（§8.8.4）。
//!
//! # DMA 收发（L012 与经典组；语义级实现待实板验证）
//!
//! [`Spi::transfer_dma`]（全双工：RX/TX 双通道并行）/[`Spi::write_dma`]
//! （只发）用 [`crate::dma`] 的通道抽象搬运，硬件触发 SPIx_RX/SPIx_TX
//! （表 8-8）。**DMA 模式与逐字节 SpiBus 模式互斥**——同一实例同时只能
//! 用一种传输方式。
//!
//! 两代位面差异：L012 的 DMA 请求使能在 CR2[2]/CR2[1]；**经典组的在
//! CR1[17]/CR1[16]**（厂商库 `SPI_DMAReq_Tx=0x00020000`/`Rx=0x00010000`
//! 写 `SPIx->CR1`），且经典组 ICR 的 FLUSH 位写 0 会清空发送缓冲——清
//! 标志时必须保位。触发源取 [`crate::dma::hs::SPI1_TX`] 等（§8.8.4）。
//!
//! ⚠ 诚实标注：阻塞版 cw32l012-hal 没有 SPI 的 DMA 实机配方，本实现按
//! PAC `dma::hs` 触发表（表 8-8）+ PAC/厂商库的 CR2.DMATX/CR2.DMARX 位面
//! （§22.7；经典组 SPIx_CR1_DMATX/DMARX 位面）编写，**语义级实现待实板
//! 验证**。

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

#[cfg(any(feature = "cw32l010", feature = "cw32l011"))]
mod fam {
    use crate::pac::spi as sb;
    /// L012 代组:门控带 KEY 写保护,spi1 = APBEN1.2(仅此一实例)
    pub const KEY: u32 = crate::SYSCTRL_KEY;
    pub fn gate(sysctrl: &crate::Sysctrl, _idx: usize) {
        // spi1en = APBEN1.2(L012 代组布局,带 KEY 写保护)
        sysctrl.apben1.write(KEY | (1 << 2));
    }
    /// EN 在 CR2[0](同 L012 模式:CR1 可写前提是 EN=0)
    pub const CR2_EN: u32 = sb::cr2::en_MASK;
    /// BR 位域:3 位,在 CR1[6:4]
    pub const BR_POS: u32 = sb::cr1::br_POS;
    pub const BR_MASK: u32 = sb::cr1::br_MASK;
    /// CR1 组合位(生成 PAC 嵌套常量)
    pub const CR1_MSTR: u32 = sb::cr1::mstr_MASK;
    pub const CR1_SSM: u32 = sb::cr1::ssm_MASK;
    pub const CR1_CPOL: u32 = sb::cr1::cpol_MASK;
    pub const CR1_CPHA: u32 = sb::cr1::cpha_MASK;
    pub const CR1_LSBF: u32 = sb::cr1::lsbf_MASK;
    /// 使能序列(同 L012:EN=0 时写 CR1)
    pub fn enable(regs: &crate::Spi, cr1: u32) {
        regs.cr2.write(0);
        regs.cr1.write(cr1);
        regs.ssi.write(1);
        regs.icr.write(0); // R1W0:写 0 清全部标志
        regs.cr2.write(CR2_EN);
    }
}

// 经典组(F030/F020/F003/L031/L052/R031/W031 逐个接入):SPI1 位面与 F030
// 完全相同(逐 PAC 核对:EN@CR1[6]、BR 3 位@CR1[5:3]、MSTR2/CPOL1/CPHA0/
// LSBF7/SSM9),单实例 SPI1=APBEN2.8;F003 的中断/基址名无数字(SPI/SPI_BASE)
#[cfg(any(
    feature = "cw32f030",
    feature = "cw32l031",
    feature = "cw32l052",
    feature = "cw32r031",
    feature = "cw32w031",
    feature = "cw32f020",
    feature = "cw32f003",
    feature = "cw32l083"
))]
mod fam {
    /// 经典组 SYSCTRL 的 AHBEN/APBEN 无 KEY 保护(厂商库 |= 直写)。
    /// SPI1=APBEN2.8、SPI2=APBEN1.6(f030 与 l083 的 PAC 位域同位)
    pub fn gate(sysctrl: &crate::Sysctrl, idx: usize) {
        match idx {
            0 => sysctrl.apben2.modify(|v| v | (1 << 8)),
            1 => sysctrl.apben1.modify(|v| v | (1 << 6)),
            _ => unreachable!("经典组无 SPI3"),
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
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
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
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
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
    // 单实例家族:基址常量名无数字(SPI_BASE,f003/l010/l011;f020 为
    // SPI1_BASE 与多实例家族同名)
    #[cfg(any(feature = "cw32l010", feature = "cw32l011", feature = "cw32f003"))]
    {
        if regs as *const _ as usize == pac::SPI_BASE {
            return 0;
        }
        unreachable!("非 SPI 寄存器块");
    }
    #[cfg(not(any(feature = "cw32l010", feature = "cw32l011", feature = "cw32f003")))]
    match regs as *const _ as usize {
        pac::SPI1_BASE => 0,
        #[cfg(any(feature = "cw32l012", feature = "cw32f030", feature = "cw32l083"))]
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
    // 单实例家族:中断名 f003/l010/l011 为 SPI,f020 为 SPI1(PAC 无 SPI2 外设,
    // 仅绑定 SPI1 即可)
    #[cfg(any(feature = "cw32l010", feature = "cw32l011", feature = "cw32f003"))]
    pub fn new(
        regs: &'static crate::Spi,
        sck: Pin<impl GpioPort>,
        mosi: Pin<impl GpioPort>,
        miso: Pin<impl GpioPort>,
        af: u8,
        _irqs: impl Binding<interrupt::SPI>,
        config: Config,
    ) -> Self {
        Self::new_inner(regs, sck, mosi, miso, af, config)
    }

    #[cfg(feature = "cw32f020")]
    pub fn new(
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

    /// 经典代单 SPI 家族(l031 组)入口:中断名 SPI1,单实例
    #[cfg(any(
        feature = "cw32l031",
        feature = "cw32l052",
        feature = "cw32r031",
        feature = "cw32w031"
    ))]
    pub fn new(
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

    /// 初始化 SPI 主机（l083 入口：SPI1/SPI2 双实例、独立向量，
    /// `bind_interrupts!` 需同时绑定 `SPI1 => spi::Spi1Irq;` 与
    /// `SPI2 => spi::Spi2Irq;`）。
    #[cfg(feature = "cw32l083")]
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

    /// 初始化 SPI1（IRQ 绑定收窄到 SPI1；L010/F003 的单实例中断名为 SPI）。
    #[cfg(any(
        feature = "cw32l012",
        feature = "cw32f030",
        feature = "cw32l031",
        feature = "cw32l052",
        feature = "cw32l083",
        feature = "cw32r031",
        feature = "cw32w031",
        feature = "cw32f020"
    ))]
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

    /// 初始化 SPI2（l083,独立向量 SPI2/IRQ26;引脚组合见模块文档表）。
    #[cfg(feature = "cw32l083")]
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

        // 使能 NVIC 中断线(单实例家族的中断名:f003/l010/l011=SPI,
        // f020=SPI1;多实例家族按 idx 派发)
        #[cfg(any(feature = "cw32l010", feature = "cw32l011", feature = "cw32f003"))]
        crate::interrupt::nvic_enable::<interrupt::SPI>();
        #[cfg(any(
            feature = "cw32l031",
            feature = "cw32l052",
            feature = "cw32r031",
            feature = "cw32w031",
            feature = "cw32f020"
        ))]
        crate::interrupt::nvic_enable::<interrupt::SPI1>();
        // l083 双实例、独立向量:SPI1=IRQ25、SPI2=IRQ26
        #[cfg(feature = "cw32l083")]
        match idx {
            0 => crate::interrupt::nvic_enable::<interrupt::SPI1>(),
            _ => crate::interrupt::nvic_enable::<interrupt::SPI2>(),
        }
        #[cfg(any(feature = "cw32l012", feature = "cw32f030"))]
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

// ================= DMA 收发（L012；语义级实现待实板验证） =================
// SPI 的 DMA 请求使能位 PAC 已收编（§22.7）：CR2[2]=DMATX、CR2[1]=DMARX
// （厂商库 SPI_DMAReq_Tx=0x4 / SPI_DMAReq_Rx=0x2 同位面）。阻塞版 HAL 无
// SPI DMA 配方——触发源按 PAC dma::hs（表 8-8，与厂商库 cw32l012_dma.h
// 的 DMA_TRIGGER_SRC_UART/SPI 枚举逐项一致），位宽/增量按 UART DMA 同款
// 推导（B8 + DR 固定 + 对侧自增 + BLOCK）。

#[cfg(feature = "cw32l012")]
impl Spi {
    /// DMA 全双工传输：rx.len() 字节（占用 DMA 通道 1=RX、通道 2=TX）
    ///
    /// 两通道同时在途：TX 通道按 TXE"发送缓冲空"把 tx 逐字节搬进 DR，
    /// RX 通道按 RXNE"接收缓冲非空"把 DR 逐字节搬进 rx——主机时钟一起，
    /// 两方向全双工并行推进。`rx` 与 `tx` 长度必须相等、非空、≤65535。
    ///
    /// 次序：清残留接收态 → 两通道 `configure`（EN=1）→ 置
    /// `CR2.DMATX|CR2.DMARX` → 依次 `wait` 两通道 TC（先 RX 后 TX；TC
    /// 标志保持到 ICR 清除，先后不丢）→ 关两个请求位 → 等 BUSY=0
    /// （移位收尾，调用方随后才可拉高软件片选）。
    ///
    /// 两通道同时在途而 [`crate::dma::Channel`] 独占借用 `&mut Dma`——
    /// 配置阶段逐个配好即弃句柄、等待阶段按通道号重取：Channel 是无状态
    /// 寄存器视图（状态全在 CSR/ISR），重取不丢上下文。
    ///
    /// **与逐字节 SpiBus 模式互斥**（见模块文档）；长度不合法返回
    /// [`crate::dma::DmaError::BadLength`]。⚠ 语义级实现待实板验证。
    pub async fn transfer_dma(
        &mut self,
        dma: &mut crate::dma::Dma,
        rx: &mut [u8],
        tx: &[u8],
    ) -> Result<(), crate::dma::DmaError> {
        if rx.is_empty() || rx.len() != tx.len() || rx.len() > 0xFFFF {
            return Err(crate::dma::DmaError::BadLength);
        }
        // 清残留接收态：RXNE 挂着会让 RX 通道第一拍搬走旧字节（整帧错位
        // 一字节）；OV 残留污染错误语义（ICR R1W0：目标位写 0 清除）
        while self.regs.isr.read() & ISR_RXNE != 0 {
            let _ = self.regs.dr.read();
        }
        self.regs.icr.write(pac::spi::ICR_CLEAR_OV);

        let (hs_rx, hs_tx) = match self.idx {
            0 => (pac::dma::hs::SPI1_RX, pac::dma::hs::SPI1_TX),
            1 => (pac::dma::hs::SPI2_RX, pac::dma::hs::SPI2_TX),
            _ => (pac::dma::hs::SPI3_RX, pac::dma::hs::SPI3_TX),
        };
        let dr = core::ptr::addr_of!(self.regs.dr) as u32;
        {
            let ch = dma.channel(1);
            ch.configure(&crate::dma::Config::periph_to_mem(
                dr,
                rx.as_mut_ptr() as u32,
                rx.len() as u16,
                crate::dma::Width::B8,
                hs_rx,
            ));
        }
        {
            let ch = dma.channel(2);
            ch.configure(&crate::dma::Config::mem_to_periph(
                tx.as_ptr() as u32,
                dr,
                tx.len() as u16,
                crate::dma::Width::B8,
                hs_tx,
            ));
        }
        // 同时开两个方向的 DMA 请求（CR2.DMATX|DMARX，§22.7）——首个 TXE
        // 请求装填 DR、主机起钟，之后 RXNE/TXE 交替驱动两通道
        self.regs
            .cr2
            .modify(|v| v | pac::spi::CR2_DMATX | pac::spi::CR2_DMARX);
        let rx_res = {
            let mut ch = dma.channel(1);
            ch.wait().await
        };
        let tx_res = {
            let mut ch = dma.channel(2);
            ch.wait().await
        };
        self.regs
            .cr2
            .modify(|v| v & !(pac::spi::CR2_DMATX | pac::spi::CR2_DMARX));
        // 末字节移位收尾（BUSY=0）后再停通道，防残留请求
        self.wait_idle().await;
        dma.channel(1).stop();
        dma.channel(2).stop();
        rx_res?;
        tx_res?;
        Ok(())
    }

    /// DMA 只写：tx.len() 字节（占用 DMA 通道 2；TXE 触发）
    ///
    /// 发送伴随收到的字节无人读走：中间字节上溢（ISR.OV）、末字节挂在
    /// RXNE——返回前排空 RXNE 并经 ICR 清 OV（R1W0），恢复到与逐字节
    /// `write`（每字节读走即弃）相同的干净状态。⚠ 若实板验证发现本款
    /// SPI IP 上溢会停收发（TX 随之停摆），改用 [`Spi::transfer_dma`]
    /// 把接收侧一并 DMA 走。
    ///
    /// **与逐字节 SpiBus 模式互斥**（见模块文档）；空缓冲或 >65535 字节
    /// 返回 [`crate::dma::DmaError::BadLength`]。⚠ 语义级实现待实板验证。
    pub async fn write_dma(
        &mut self,
        dma: &mut crate::dma::Dma,
        tx: &[u8],
    ) -> Result<(), crate::dma::DmaError> {
        if tx.is_empty() || tx.len() > 0xFFFF {
            return Err(crate::dma::DmaError::BadLength);
        }
        // 清残留接收态（同 transfer_dma：防旧 RXNE 抢占/污染语义）
        while self.regs.isr.read() & ISR_RXNE != 0 {
            let _ = self.regs.dr.read();
        }
        self.regs.icr.write(pac::spi::ICR_CLEAR_OV);

        let hs = match self.idx {
            0 => pac::dma::hs::SPI1_TX,
            1 => pac::dma::hs::SPI2_TX,
            _ => pac::dma::hs::SPI3_TX,
        };
        let cfg = crate::dma::Config::mem_to_periph(
            tx.as_ptr() as u32,
            core::ptr::addr_of!(self.regs.dr) as u32,
            tx.len() as u16,
            crate::dma::Width::B8,
            hs,
        );
        let mut ch = dma.channel(2);
        ch.configure(&cfg); // EN=1，等 TXE 请求
        self.regs.cr2.modify(|v| v | pac::spi::CR2_DMATX);
        let r = ch.wait().await;
        self.regs.cr2.modify(|v| v & !pac::spi::CR2_DMATX);
        // 末字节移位收尾（BUSY=0）后再排空接收侧、停通道
        self.wait_idle().await;
        while self.regs.isr.read() & ISR_RXNE != 0 {
            let _ = self.regs.dr.read();
        }
        self.regs.icr.write(pac::spi::ICR_CLEAR_OV);
        ch.stop();
        r
    }
}

// ================= DMA 收发（经典组；语义级实现待实板验证） =================
// 关键位面差异：**DMA 请求使能位在 CR1 而非 CR2**——CR1[17]=DMATX/
// CR1[16]=DMARX（厂商库 cw32f030_spi.h:195-196 SPI_DMAReq_Tx=0x00020000/
// Rx=0x00010000，cw32f030_spi.c:326-337 SPI_DMACmd 写 SPIx->CR1；各家族
// 厂商头 SPIx_CR1_DMATX_Pos=17/DMARX_Pos=16 逐家抽验同值：cw32f020.h:
// 6535-6538、cw32l031.h:5556-5559、cw32l052.h:7556-7559、cw32l083.h:
// 8103-8106、cw32r031.h:5556-5559、cw32w031.h:5910-5913；生成 PAC
// spi::cr1::dmatx/dmarx_MASK 同值）。触发源 dma::hs::SPIx_RX/TX（§8.8.4：
// SPI1_RX=0x06/SPI1_TX=0x07 全经典组、SPI2_RX=0x08/SPI2_TX=0x09 有 SPI2
// 的家族；厂商库 cw32f030_dma.h:112-115 DMA_HardTrig_SPIx_RXBufferNE/
// TXBufferE）。ICR 同为 R1W0（厂商 SPI_ClearFlag = SPIx->ICR &= ~flag）：
// OV@bit5/RXNE@bit1 写 0 清除；**bit0=FLUSH 写 0 会清空发送缓冲/移位
// 寄存器**（cw32f030_spi.c SPI_FlushSendBuff），清标志掩码除目标位外全 1
// （FLUSH 位恒写 1 无动作）。f003 无 DMA 硬件，不接入。

#[cfg(any(
    feature = "cw32f030",
    feature = "cw32f020",
    feature = "cw32l031",
    feature = "cw32l052",
    feature = "cw32l083",
    feature = "cw32r031",
    feature = "cw32w031"
))]
impl Spi {
    /// DMA 发送请求使能（CR1[17]，厂商库 SPI_DMAReq_Tx=0x00020000；PAC 位常量）
    const CR1_DMATX: u32 = pac::spi::cr1::dmatx_MASK;
    /// DMA 接收请求使能（CR1[16]，厂商库 SPI_DMAReq_Rx=0x00010000）
    const CR1_DMARX: u32 = pac::spi::cr1::dmarx_MASK;

    /// 发送方向硬件触发源（§8.8.4 / 厂商库 DMA_HardTrig_SPIx_TXBufferE）。
    ///
    /// SPI1/SPI2 **全接线**——SPI2 实例仅 f030/l083 有（`instance_of` 同款
    /// cfg），SPI2_TX=0x09 两家手册请求表同值（f030=EN_V1.0 p147-148、
    /// l083=CN_V2.0 p148）。`_` 分支是结构性兜底——单实例家族（f020/f003/
    /// l031/l052/r031/w031）`instance_of` 只产 0。
    fn tx_hs(&self) -> u8 {
        match self.idx {
            0 => crate::dma::hs::SPI1_TX,
            #[cfg(any(feature = "cw32f030", feature = "cw32l083"))]
            1 => crate::dma::hs::SPI2_TX,
            _ => unreachable!("SPI 实例号超出本家族实例数（instance_of 只产 0..N-1）"),
        }
    }

    /// 接收方向硬件触发源（§8.8.4 / 厂商库 DMA_HardTrig_SPIx_RXBufferNE）
    ///
    /// 接线范围同 [`Spi::tx_hs`]：SPI1_RX=0x06 全经典组、SPI2_RX=0x08。
    fn rx_hs(&self) -> u8 {
        match self.idx {
            0 => crate::dma::hs::SPI1_RX,
            #[cfg(any(feature = "cw32f030", feature = "cw32l083"))]
            1 => crate::dma::hs::SPI2_RX,
            _ => unreachable!("SPI 实例号超出本家族实例数（instance_of 只产 0..N-1）"),
        }
    }

    /// 清残留接收态：排空 RXNE 并经 ICR 清 OV/RXNE（R1W0：目标位写 0）
    ///
    /// RXNE 挂着会让 RX 通道第一拍搬走旧字节（整帧错位一字节）；OV 残留
    /// 污染错误语义。FLUSH@ICR[0] 写 0 会清空发送缓冲，掩码除 OV/RXNE 外
    /// 全 1（写 1 无动作）。
    fn drain_rx(&mut self) {
        while self.regs.isr.read() & ISR_RXNE != 0 {
            let _ = self.regs.dr.read();
        }
        self.regs
            .icr
            .write(!(pac::spi::icr::ov_MASK | pac::spi::icr::rxne_MASK));
    }

    /// DMA 全双工传输（经典组）：rx.len() 字节（占用 DMA 通道 1=RX、通道 2=TX）
    ///
    /// 两通道同时在途：TX 通道按 TXE"发送缓冲空"把 tx 逐字节搬进 DR，
    /// RX 通道按 RXNE"接收缓冲非空"把 DR 逐字节搬进 rx——主机时钟一起，
    /// 两方向全双工并行推进。`rx` 与 `tx` 长度必须相等、非空、≤65535。
    ///
    /// 次序：清残留接收态 → 两通道 `configure`（EN=1）→ 置
    /// `CR1.DMATX|CR1.DMARX`（厂商库 `SPI_DMACmd` 同款位面）→ 依次 `wait`
    /// 两通道 TC（先 RX 后 TX；TC 标志保持到 ICR 清除，先后不丢）→ 关两个
    /// 请求位 → 等 BUSY=0（移位收尾，调用方随后才可拉高软件片选）。
    ///
    /// 两通道同时在途而 [`crate::dma::Channel`] 独占借用 `&mut ClassicDma`
    /// ——配置阶段逐个配好即弃句柄、等待阶段按通道号重取：Channel 是无状态
    /// 寄存器视图（状态全在 CSR/ISR），重取不丢上下文。通道 1/2 在 f020 上
    /// 各有 NVIC 线（DMACH1/DMACH2），`wait` 可用（避开无线的通道 4/5）。
    ///
    /// **与逐字节 SpiBus 模式互斥**（见模块文档）；长度不合法返回
    /// [`crate::dma::DmaErrorClassic::BadLength`]。⚠ 语义级实现待实板验证。
    pub async fn transfer_dma(
        &mut self,
        dma: &mut crate::dma::ClassicDma,
        rx: &mut [u8],
        tx: &[u8],
    ) -> Result<(), crate::dma::DmaErrorClassic> {
        if rx.is_empty() || rx.len() != tx.len() || rx.len() > 0xFFFF {
            return Err(crate::dma::DmaErrorClassic::BadLength);
        }
        self.drain_rx();

        let dr = core::ptr::addr_of!(self.regs.dr) as u32;
        {
            let ch = dma.channel(1);
            ch.configure(&crate::dma::ClassicConfig::periph_to_mem(
                dr,
                rx.as_mut_ptr() as u32,
                rx.len() as u16,
                crate::dma::ClassicWidth::B8,
                self.rx_hs(),
            ));
        }
        {
            let ch = dma.channel(2);
            ch.configure(&crate::dma::ClassicConfig::mem_to_periph(
                tx.as_ptr() as u32,
                dr,
                tx.len() as u16,
                crate::dma::ClassicWidth::B8,
                self.tx_hs(),
            ));
        }
        // 同时开两个方向的 DMA 请求（CR1.DMATX|CR1.DMARX）——首个 TXE 请求
        // 装填 DR、主机起钟，之后 RXNE/TXE 交替驱动两通道
        self.regs
            .cr1
            .modify(|v| v | Self::CR1_DMATX | Self::CR1_DMARX);
        let rx_res = {
            let mut ch = dma.channel(1);
            ch.wait().await
        };
        let tx_res = {
            let mut ch = dma.channel(2);
            ch.wait().await
        };
        self.regs
            .cr1
            .modify(|v| v & !(Self::CR1_DMATX | Self::CR1_DMARX));
        // 末字节移位收尾（BUSY=0）后再停通道，防残留请求
        self.wait_idle().await;
        dma.channel(1).stop();
        dma.channel(2).stop();
        rx_res?;
        tx_res?;
        Ok(())
    }

    /// DMA 只写（经典组）：tx.len() 字节（占用 DMA 通道 2；TXE 触发）
    ///
    /// 发送伴随收到的字节无人读走：中间字节上溢（ISR.OV）、末字节挂在
    /// RXNE——返回前排空 RXNE 并经 ICR 清 OV/RXNE（R1W0），恢复到与逐字节
    /// `write`（每字节读走即弃）相同的干净状态。⚠ 若实板验证发现本款
    /// SPI IP 上溢会停收发（TX 随之停摆），改用 [`Spi::transfer_dma`]
    /// 把接收侧一并 DMA 走。
    ///
    /// **与逐字节 SpiBus 模式互斥**（见模块文档）；空缓冲或 >65535 字节
    /// 返回 [`crate::dma::DmaErrorClassic::BadLength`]。⚠ 语义级实现待实板
    /// 验证。
    pub async fn write_dma(
        &mut self,
        dma: &mut crate::dma::ClassicDma,
        tx: &[u8],
    ) -> Result<(), crate::dma::DmaErrorClassic> {
        if tx.is_empty() || tx.len() > 0xFFFF {
            return Err(crate::dma::DmaErrorClassic::BadLength);
        }
        // 清残留接收态（同 transfer_dma：防旧 RXNE 抢占/污染语义）
        self.drain_rx();

        let cfg = crate::dma::ClassicConfig::mem_to_periph(
            tx.as_ptr() as u32,
            core::ptr::addr_of!(self.regs.dr) as u32,
            tx.len() as u16,
            crate::dma::ClassicWidth::B8,
            self.tx_hs(),
        );
        let mut ch = dma.channel(2);
        ch.configure(&cfg); // EN=1，等 TXE 请求
        self.regs.cr1.modify(|v| v | Self::CR1_DMATX);
        let r = ch.wait().await;
        self.regs.cr1.modify(|v| v & !Self::CR1_DMATX);
        // 末字节移位收尾（BUSY=0）后再排空接收侧、停通道
        self.wait_idle().await;
        while self.regs.isr.read() & ISR_RXNE != 0 {
            let _ = self.regs.dr.read();
        }
        self.regs
            .icr
            .write(!(pac::spi::icr::ov_MASK | pac::spi::icr::rxne_MASK));
        ch.stop();
        r
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

/// SPI1 中断标记（f003/l010/l011 的中断名为 SPI,IRQ25）
#[cfg(any(feature = "cw32l010", feature = "cw32l011", feature = "cw32f003"))]
pub struct Spi1Irq;
#[cfg(any(feature = "cw32l010", feature = "cw32l011", feature = "cw32f003"))]
impl IrqHandler<interrupt::SPI> for Spi1Irq {
    fn call() {
        on_irq(0, unsafe { &*(pac::SPI_BASE as *const crate::Spi) });
    }
}

#[cfg(any(
    feature = "cw32l012",
    feature = "cw32f030",
    feature = "cw32l031",
    feature = "cw32l052",
    feature = "cw32l083",
    feature = "cw32r031",
    feature = "cw32w031",
    feature = "cw32f020"
))]
pub struct Spi1Irq;
#[cfg(any(
    feature = "cw32l012",
    feature = "cw32f030",
    feature = "cw32l031",
    feature = "cw32l052",
    feature = "cw32l083",
    feature = "cw32r031",
    feature = "cw32w031",
    feature = "cw32f020"
))]
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

/// SPI2 中断标记（l083:独立向量 IRQ26,typelevel 名 SPI2;
/// `bind_interrupts!` 侧写作 `SPI2 => spi::Spi2Irq;`）
#[cfg(feature = "cw32l083")]
pub struct Spi2Irq;
#[cfg(feature = "cw32l083")]
impl IrqHandler<interrupt::SPI2> for Spi2Irq {
    fn call() {
        on_irq(1, unsafe { &*(pac::SPI2_BASE as *const crate::Spi) });
    }
}
