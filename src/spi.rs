//! 异步 SPI 主机（中断驱动逐字节，`embedded-hal-async` SpiBus）。
//!
//! 片选由用户 GPIO 管理（软件 CS）：传输前拉低、`flush` 后拉高。
//! 时序默认 Mode 0、MSB first、8-bit。
//!
//! 引脚（AF3，UM 表 9-2 摘要）：SPI1 = PA05(SCK)/PA06(MISO)/PA07(MOSI)。

use core::future::poll_fn;
use core::task::Poll;

use embassy_sync::waitqueue::AtomicWaker;

use crate::gpio::Pin;
use crate::interrupt::{Binding, IrqHandler, interrupt};
use crate::pac::{self, spi as sb};

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
fn instance_of(regs: &'static pac::Spi) -> usize {
    match regs as *const _ as usize {
        pac::SPI1_BASE => 0,
        pac::SPI2_BASE => 1,
        pac::SPI3_BASE => 2,
        _ => unreachable!("非 SPI 寄存器块"),
    }
}

static WAKER: [AtomicWaker; 3] = [const { AtomicWaker::new() }; 3];

/// 异步 SPI 主机
pub struct Spi {
    regs: &'static pac::Spi,
    idx: usize,
}

impl Spi {
    /// 初始化 SPI 主机（全双工、软件 CS）并配置引脚复用。
    pub fn new(
        regs: &'static pac::Spi,
        sck: Pin,
        mosi: Pin,
        miso: Pin,
        af: u8,
        _irqs: impl Binding<interrupt::SPI1> + Binding<interrupt::SPI23>,
        config: Config,
    ) -> Self {
        Self::new_inner(regs, sck, mosi, miso, af, config)
    }

    /// 初始化 SPI1（IRQ 绑定收窄到 SPI1）。
    pub fn new_spi1(
        regs: &'static pac::Spi,
        sck: Pin,
        mosi: Pin,
        miso: Pin,
        af: u8,
        _irqs: impl Binding<interrupt::SPI1>,
        config: Config,
    ) -> Self {
        Self::new_inner(regs, sck, mosi, miso, af, config)
    }

    /// 初始化 SPI2（IRQ 绑定收窄到 SPI23）。
    pub fn new_spi2(
        regs: &'static pac::Spi,
        sck: Pin,
        mosi: Pin,
        miso: Pin,
        af: u8,
        _irqs: impl Binding<interrupt::SPI23>,
        config: Config,
    ) -> Self {
        Self::new_inner(regs, sck, mosi, miso, af, config)
    }

    /// 初始化 SPI3（IRQ 绑定收窄到 SPI23）。
    pub fn new_spi3(
        regs: &'static pac::Spi,
        sck: Pin,
        mosi: Pin,
        miso: Pin,
        af: u8,
        _irqs: impl Binding<interrupt::SPI23>,
        config: Config,
    ) -> Self {
        Self::new_inner(regs, sck, mosi, miso, af, config)
    }

    fn new_inner(
        regs: &'static pac::Spi,
        sck: Pin,
        mosi: Pin,
        miso: Pin,
        af: u8,
        config: Config,
    ) -> Self {
        let idx = instance_of(regs);
        let sysctrl = unsafe { &*(pac::SYSCTRL_BASE as *const pac::Sysctrl) };
        let gate = match idx {
            0 => pac::sysctrl::APBEN1_SPI1,
            1 => pac::sysctrl::APBEN1_SPI2,
            _ => pac::sysctrl::APBEN1_SPI3,
        };
        sysctrl.apben1.write(pac::sysctrl::KEY | gate);

        sck.set_af(af);
        mosi.set_af(af);
        miso.set_af(af);

        let pclk = crate::PCLK_HZ.load(core::sync::atomic::Ordering::Relaxed);
        // 波特率 = PCLK / (BR + 1)，取不超过目标的 最大分频满足者
        let div = pclk.div_ceil(config.frequency).max(1);
        let br = (div - 1).min(0x7F);

        let (cpol, cpha) = match config.mode {
            Mode::Mode0 => (0, 0),
            Mode::Mode1 => (0, sb::CR1_CPHA),
            Mode::Mode2 => (sb::CR1_CPOL, 0),
            Mode::Mode3 => (sb::CR1_CPOL, sb::CR1_CPHA),
        };
        let lsbf = if config.msb_first { 0 } else { sb::CR1_LSBF };

        regs.cr2.write(0); // EN=0 时才能写 CR1
        regs.cr1
            .write((br << sb::CR1_BR_POS) | sb::CR1_MSTR | sb::CR1_SSM | cpol | cpha | lsbf);
        regs.ssi.write(1); // 内部 CS 拉高（未选中）
        regs.icr.write(0); // R1W0：写 0 清全部标志
        regs.cr2.write(sb::CR2_EN);

        Self { regs, idx }
    }

    /// 等 TXE（可写）
    async fn wait_txe(&mut self) {
        poll_fn(|cx| {
            WAKER[self.idx].register(cx.waker());
            if self.regs.isr.read() & sb::ISR_TXE != 0 {
                Poll::Ready(())
            } else {
                self.regs.ier.modify(|v| v | sb::ISR_TXE);
                Poll::Pending
            }
        })
        .await;
    }

    /// 等 RXNE（可读）
    async fn wait_rxne(&mut self) {
        poll_fn(|cx| {
            WAKER[self.idx].register(cx.waker());
            if self.regs.isr.read() & sb::ISR_RXNE != 0 {
                Poll::Ready(())
            } else {
                self.regs.ier.modify(|v| v | sb::ISR_RXNE);
                Poll::Pending
            }
        })
        .await;
    }

    /// 等 BUSY 清零（传输结束）
    async fn wait_idle(&mut self) {
        poll_fn(|cx| {
            WAKER[self.idx].register(cx.waker());
            if self.regs.isr.read() & sb::ISR_BUSY == 0 {
                Poll::Ready(())
            } else {
                // BUSY 无中断源：挂 RXNE（每字节产生）作为唤醒代理
                self.regs.ier.modify(|v| v | sb::ISR_RXNE);
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
fn on_irq(idx: usize, regs: &'static pac::Spi) {
    let isr = regs.isr.read();
    if isr & (sb::ISR_TXE | sb::ISR_RXNE) != 0 {
        WAKER[idx].wake();
        if isr & sb::ISR_TXE != 0 {
            regs.ier.modify(|v| v & !sb::ISR_TXE);
        }
        if isr & sb::ISR_RXNE != 0 {
            regs.ier.modify(|v| v & !sb::ISR_RXNE);
        }
    }
}

/// SPI1 中断标记（IRQ25）
pub struct Spi1Irq;
impl IrqHandler<interrupt::SPI1> for Spi1Irq {
    fn call() {
        on_irq(0, unsafe { &*(pac::SPI1_BASE as *const pac::Spi) });
    }
}

/// SPI2 中断标记（IRQ26，与 SPI3 共享向量）
pub struct Spi2Irq;
impl IrqHandler<interrupt::SPI23> for Spi2Irq {
    fn call() {
        on_irq(1, unsafe { &*(pac::SPI2_BASE as *const pac::Spi) });
        on_irq(2, unsafe { &*(pac::SPI3_BASE as *const pac::Spi) });
    }
}

/// SPI3 中断标记（IRQ26，与 SPI2 共享向量）
pub struct Spi3Irq;
impl IrqHandler<interrupt::SPI23> for Spi3Irq {
    fn call() {
        on_irq(1, unsafe { &*(pac::SPI2_BASE as *const pac::Spi) });
        on_irq(2, unsafe { &*(pac::SPI3_BASE as *const pac::Spi) });
    }
}
