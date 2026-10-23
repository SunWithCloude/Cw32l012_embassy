//! 异步 I2C 主机（`embedded-hal-async` 接口）。
//!
//! CW32L012 的 I2C 主机是命令 FIFO 式（MTDR 写命令）：START+地址、
//! TX、RXCOUNT、STOP。等待点通过 MIER 中断 + waker 异步化：
//! TXE（可写命令）、RXNE（可读数据）、PACKET（包阶段完成）、
//! NACK/ARBI/PINLOW（错误）。
//!
//! 时序按 I2C 规范推导（与 cw32l012-hal 同源公式，已对照手册表 23-4
//! 官方示例校准）。

use core::future::poll_fn;
use core::task::Poll;

use embassy_sync::waitqueue::AtomicWaker;

use crate::gpio::Pin;
use crate::interrupt::{Binding, IrqHandler, interrupt};
use crate::pac::{self, i2c as ic};

/// I2C 速率
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Speed {
    /// 标准模式 100 kHz
    Standard100k,
    /// 快速模式 400 kHz
    Fast400k,
}

/// I2C 配置
#[derive(Debug, Clone, Copy)]
pub struct Config {
    /// 速率档位
    pub speed: Speed,
}

impl Default for Config {
    fn default() -> Self {
        Config {
            speed: Speed::Standard100k,
        }
    }
}

/// I2C 错误
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum I2cError {
    /// 地址或数据被 NACK
    Nack,
    /// 仲裁失败
    Arbitration,
    /// FIFO 溢出/下溢
    Fifo,
    /// SDA/SCL 被拉低（总线卡死）
    PinLow,
}

impl embedded_hal::i2c::Error for I2cError {
    fn kind(&self) -> embedded_hal::i2c::ErrorKind {
        embedded_hal::i2c::ErrorKind::NoAcknowledge(match self {
            I2cError::Nack => embedded_hal::i2c::NoAcknowledgeSource::Address,
            _ => embedded_hal::i2c::NoAcknowledgeSource::Unknown,
        })
    }
}

/// I2C 时序参数（TCLKD 单位）
struct Timing {
    prescale: u8,
    sethold: u8,
    clkhi: u8,
    clklo: u8,
    datavd: u8,
}

/// 按规范推导 I2C 时序（同 cw32l012-hal calc_timing）。
fn calc_timing(pclk_hz: u32, speed: Speed) -> Option<Timing> {
    let (f, t_low_ns, t_hd_sta_ns): (u32, u64, u64) = match speed {
        Speed::Standard100k => (100_000, 4_700, 4_000),
        Speed::Fast400k => (400_000, 1_300, 600),
    };
    for p in 0u8..=7 {
        let tclkd = (pclk_hz >> p) as u64;
        if tclkd == 0 {
            break;
        }
        let period = tclkd / f as u64;
        if period < 10 {
            continue;
        }
        let clklo = (t_low_ns * tclkd).div_ceil(1_000_000_000);
        if clklo > 63 {
            continue;
        }
        const SYNC: u64 = 4;
        let clkhi = period.saturating_sub(clklo + SYNC);
        if !(2..=63).contains(&clkhi) {
            continue;
        }
        let sethold = (t_hd_sta_ns * tclkd).div_ceil(1_000_000_000);
        let datavd = (250 * tclkd / 1_000_000_000).clamp(1, 63);
        return Some(Timing {
            prescale: p,
            sethold: sethold.clamp(1, 63) as u8,
            clkhi: clkhi as u8,
            clklo: clklo as u8,
            datavd: datavd as u8,
        });
    }
    None
}

/// 实例索引（0/1 = I2C1/2）
fn instance_of(regs: &'static pac::I2c) -> usize {
    match regs as *const _ as usize {
        pac::I2C1_BASE => 0,
        pac::I2C2_BASE => 1,
        _ => unreachable!("非 I2C 寄存器块"),
    }
}

static WAKER: [AtomicWaker; 2] = [const { AtomicWaker::new() }; 2];

/// 异步 I2C 主机
pub struct I2c {
    regs: &'static pac::I2c,
    idx: usize,
}

impl I2c {
    /// 初始化 I2C 主机并配置引脚（开漏复用由 PINCFG=OD_IO 处理）。
    pub fn new(
        regs: &'static pac::I2c,
        scl: Pin,
        sda: Pin,
        af: u8,
        _irqs: impl Binding<interrupt::I2C1> + Binding<interrupt::I2C2>,
        config: Config,
    ) -> Self {
        Self::new_inner(regs, scl, sda, af, config)
    }

    /// 初始化 I2C1（IRQ 绑定收窄到 I2C1）。
    pub fn new_i2c1(
        regs: &'static pac::I2c,
        scl: Pin,
        sda: Pin,
        af: u8,
        _irqs: impl Binding<interrupt::I2C1>,
        config: Config,
    ) -> Self {
        Self::new_inner(regs, scl, sda, af, config)
    }

    /// 初始化 I2C2（IRQ 绑定收窄到 I2C2）。
    pub fn new_i2c2(
        regs: &'static pac::I2c,
        scl: Pin,
        sda: Pin,
        af: u8,
        _irqs: impl Binding<interrupt::I2C2>,
        config: Config,
    ) -> Self {
        Self::new_inner(regs, scl, sda, af, config)
    }

    fn new_inner(regs: &'static pac::I2c, scl: Pin, sda: Pin, af: u8, config: Config) -> Self {
        let idx = instance_of(regs);
        let sysctrl = unsafe { &*(pac::SYSCTRL_BASE as *const pac::Sysctrl) };
        let gate = if idx == 0 {
            pac::sysctrl::APBEN2_I2C1
        } else {
            pac::sysctrl::APBEN2_I2C2
        };
        sysctrl.apben2.write(pac::sysctrl::KEY | gate);

        scl.set_af(af);
        sda.set_af(af);

        let pclk = crate::PCLK_HZ.load(core::sync::atomic::Ordering::Relaxed);
        let t = calc_timing(pclk, config.speed).expect("PCLK 超出该 I2C 速率档支持范围");

        regs.mcr0.write(ic::MCR0_TXFIFORST | ic::MCR0_RXFIFORST);
        regs.mcr2.write(
            (t.prescale as u32 & ic::MCR2_PRESCALE_MASK) << ic::MCR2_PRESCALE_POS
                | ic::MCR2_PINCFG_OD_IO,
        );
        regs.mccr.write(
            (t.datavd as u32) << ic::MCCR_DATAVD_POS
                | (t.sethold as u32) << ic::MCCR_SETHOLD_POS
                | (t.clkhi as u32) << ic::MCCR_CLKHI_POS
                | (t.clklo as u32) << ic::MCCR_CLKLO_POS,
        );
        regs.mier.write(0);
        regs.mcr0.write(ic::MCR0_MEN | ic::MCR0_CLKSRC_PCLK);

        Self { regs, idx }
    }

    /// 等待 MISR 满足 `mask` 中任一位；返回命中的位。
    async fn wait_flags(&mut self, mask: u32) -> u32 {
        poll_fn(|cx| {
            WAKER[self.idx].register(cx.waker());
            let misr = self.regs.misr.read();
            if misr & mask != 0 {
                Poll::Ready(misr & mask)
            } else {
                self.regs.mier.modify(|v| v | mask);
                Poll::Pending
            }
        })
        .await
    }

    /// 检查并解码错误；有错时发 STOP 并返回 Err。
    async fn check_errors(&mut self, misr: u32) -> Result<(), I2cError> {
        let err_mask = ic::MISR_NACK | ic::MISR_ARBI | ic::MISR_FIFO | ic::MISR_PINLOW;
        if misr & err_mask != 0 {
            let e = if misr & ic::MISR_NACK != 0 {
                I2cError::Nack
            } else if misr & ic::MISR_ARBI != 0 {
                I2cError::Arbitration
            } else if misr & ic::MISR_FIFO != 0 {
                I2cError::Fifo
            } else {
                I2cError::PinLow
            };
            // 尽力发 STOP 复位总线
            if self.regs.misr.read() & ic::MISR_TXE != 0 {
                self.regs.mtdr.write(ic::CMD_STOP << ic::MTDR_CMD_POS);
            }
            Err(e)
        } else {
            Ok(())
        }
    }

    /// 发一条 MTDR 命令（等 TXE + 错误检查）
    async fn cmd(&mut self, cmd: u32, data: u8) -> Result<(), I2cError> {
        loop {
            let f = self
                .wait_flags(
                    ic::MISR_TXE | ic::MISR_NACK | ic::MISR_ARBI | ic::MISR_FIFO | ic::MISR_PINLOW,
                )
                .await;
            self.check_errors(f).await?;
            if f & ic::MISR_TXE != 0 {
                break;
            }
        }
        self.regs
            .mtdr
            .write((cmd << ic::MTDR_CMD_POS) | data as u32);
        Ok(())
    }

    /// 等总线空闲（MSTBUSY 清零）
    async fn wait_bus_idle(&mut self) {
        poll_fn(|cx| {
            WAKER[self.idx].register(cx.waker());
            if self.regs.misr.read() & ic::MISR_MSTBUSY == 0 {
                Poll::Ready(())
            } else {
                // MSTBUSY 无中断：用 PACKET 作唤醒代理
                self.regs.mier.modify(|v| v | ic::MISR_PACKET);
                Poll::Pending
            }
        })
        .await;
    }

    /// 执行一次完整事务：START(地址) → 写/读 → STOP。
    async fn transaction_inner(
        &mut self,
        addr: u8,
        write: Option<&[u8]>,
        read: Option<&mut [u8]>,
    ) -> Result<(), I2cError> {
        self.wait_bus_idle().await;

        // START + 地址（含 R/W 位）
        let start = if read.is_some() {
            ic::CMD_START_NACK
        } else {
            ic::CMD_START_ACK
        };
        self.cmd(start, addr << 1 | u8::from(read.is_some()))
            .await?;

        if let Some(w) = write {
            if !w.is_empty() {
                for b in w {
                    self.cmd(ic::CMD_TX, *b).await?;
                }
            }
        }
        if let Some(r) = read {
            if !r.is_empty() {
                // RXCOUNT = n-1
                self.cmd(ic::CMD_RXCOUNT, (r.len() - 1) as u8).await?;
                for slot in r.iter_mut() {
                    loop {
                        let f = self
                            .wait_flags(
                                ic::MISR_RXNE
                                    | ic::MISR_NACK
                                    | ic::MISR_ARBI
                                    | ic::MISR_FIFO
                                    | ic::MISR_PINLOW,
                            )
                            .await;
                        self.check_errors(f).await?;
                        if f & ic::MISR_RXNE != 0 {
                            break;
                        }
                    }
                    let v = self.regs.mrdr.read();
                    *slot = (v & ic::MRDR_DATA_MASK) as u8;
                }
            }
        }
        self.cmd(ic::CMD_STOP, 0).await?;
        // 等 STOP 完成
        loop {
            let f = self
                .wait_flags(ic::MISR_STOP | ic::MISR_NACK | ic::MISR_ARBI)
                .await;
            self.check_errors(f).await?;
            if f & ic::MISR_STOP != 0 {
                break;
            }
        }
        Ok(())
    }
}

impl embedded_hal_async::i2c::ErrorType for I2c {
    type Error = I2cError;
}

impl embedded_hal_async::i2c::I2c for I2c {
    /// v0.1：每个 Operation 独立执行完整事务（START…STOP）。
    /// `Write` 后紧跟 `Read` 不做重复起始（repeated-start），个别要求
    /// 重复起始的器件（如部分传感器保持寄存器指针）暂不支持。
    async fn transaction(
        &mut self,
        address: u8,
        operations: &mut [embedded_hal_async::i2c::Operation<'_>],
    ) -> Result<(), Self::Error> {
        for op in operations {
            match op {
                embedded_hal_async::i2c::Operation::Write(w) => {
                    self.transaction_inner(address, Some(w), None).await?
                }
                embedded_hal_async::i2c::Operation::Read(r) => {
                    self.transaction_inner(address, None, Some(r)).await?
                }
            }
        }
        Ok(())
    }
}

// ================= 中断派发 =================

fn on_irq(idx: usize, regs: &'static pac::I2c) {
    regs.mier.write(0); // 压掉使能，避免风暴（下次 poll 重开）
    WAKER[idx].wake();
}

/// I2C1 中断标记（IRQ23）
pub struct I2c1Irq;
impl IrqHandler<interrupt::I2C1> for I2c1Irq {
    fn call() {
        on_irq(0, unsafe { &*(pac::I2C1_BASE as *const pac::I2c) });
    }
}

/// I2C2 中断标记（IRQ24）
pub struct I2c2Irq;
impl IrqHandler<interrupt::I2C2> for I2c2Irq {
    fn call() {
        on_irq(1, unsafe { &*(pac::I2C2_BASE as *const pac::I2c) });
    }
}
