//! 异步 I2C 主机（`embedded-hal-async` 接口，双家族）。
//!
//! 家族差异（feature 二选一）：
//!
//! - **cw32l012**（命令 FIFO 式，保留 v0.1 驱动）：MTDR 写命令
//!   （START+地址、TX、RXCOUNT、STOP），等待点 TXE/RXNE/PACKET/错误经
//!   MIER 中断 + waker 异步化；时序按 I2C 规范推导（与 cw32l012-hal
//!   同源公式，已对照手册表 23-4 官方示例校准）。事务语义：每个
//!   Operation 独立执行 START…STOP，`Write` 后跟 `Read` 不做重复起始。
//! - **cw32f030**（新写）：经典 8051 风格状态机外设——CR 置 STA/STO、
//!   DR 装字节，SI 置位时 STAT 给出状态码（0x08/0x10/0x18/…/0x58）。
//!   等待点唯一（CR.SI），经 NVIC + AtomicWaker 异步化。事务语义
//!   **强于 L012 版**：整个 transaction 单次 START，操作间切换方向时
//!   发重复起始（repeated-START）——`[Write, Read]` 天然复用重复起始，
//!   适合"写寄存器指针 + 读数据"类器件（EEPROM 随机读、传感器保持
//!   寄存器等）；同向连续 Operation 直接续传不重发地址；读尾自动
//!   AA=0 NACK 末字节；错误态发 STOP 复位总线。
//!
//! F030 补充说明：
//!
//! - 波特率：BRR 公式**未经实板校准**，厂商库注释与任务基线两种口径
//!   厂商库注释口径 fPCLK/(8×fSCL)−1,见 `calc_brr`。
//! - 中断仅由 NVIC + CR.SI 门控（寄存器块无 IER，厂商库 `I2C_GetIrq`
//!   即读 CR bit3）；为防"等待者消失 + SI 常亮"中断风暴，IRQ 派发侧
//!   压掉 NVIC 线、等待侧重开——L012 版 mier 压制模式的 F030 等价物。
//! - 事务中途放弃（future 被 drop）会留下 SI 置位的半途状态；重新发起
//!   事务时的首个 CR 写会清 SI 复位状态机。若 SCL 被从机拉死，需复位
//!   外设恢复（两家族共同限制）。
//! - 引脚组合（厂商 cw32f030_gpio.h `PAxx_AFx_I2Cx` 宏，勿凭记忆猜）：
//!
//!   | 实例 | 常用组合 |
//!   |------|----------|
//!   | I2C1 | PA09=SCL / PA10=SDA（AF3）、PA14=SCL / PA13=SDA（AF2）、PB06/PB07（AF3）、PB08/PB09（AF1）、PB10/PB11（AF3）、PF01/PF00（AF3） |
//!   | I2C2 | PA11/PA12（AF3）、PA01/PA02（AF3）、PA04/PA05（AF3）、PB00/PB01（AF3）、PB10/PB11（AF4）、PB13/PB14（AF3） |

use embassy_sync::waitqueue::AtomicWaker;

// ================= 共用 =================

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

/// per-实例 waker（0/1 = I2C1/2；两代中断语义不同但实例布局相同）
static WAKER: [AtomicWaker; 2] = [const { AtomicWaker::new() }; 2];

// ================= L012 家族（命令 FIFO 式,保留 v0.1 驱动） =================

/// L012 家族驱动。命令 FIFO 式外设——保留 v0.1 代码，仅随双家族化
/// 调整模块归属与导入。
#[cfg(feature = "cw32l012")]
mod l012 {
    use core::future::poll_fn;
    use core::task::Poll;

    use super::{Config, Speed, WAKER};
    use crate::gpio::{GpioPort, Pin};
    use crate::interrupt::{Binding, IrqHandler, interrupt};
    use crate::pac::{self, i2c as ic};

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

    /// 异步 I2C 主机
    pub struct I2c {
        regs: &'static pac::I2c,
        idx: usize,
    }

    impl I2c {
        /// 初始化 I2C 主机并配置引脚（开漏复用由 PINCFG=OD_IO 处理）。
        pub fn new(
            regs: &'static pac::I2c,
            scl: Pin<impl GpioPort>,
            sda: Pin<impl GpioPort>,
            af: u8,
            _irqs: impl Binding<interrupt::I2C1> + Binding<interrupt::I2C2>,
            config: Config,
        ) -> Self {
            Self::new_inner(regs, scl, sda, af, config)
        }

        /// 初始化 I2C1（IRQ 绑定收窄到 I2C1）。
        pub fn new_i2c1(
            regs: &'static pac::I2c,
            scl: Pin<impl GpioPort>,
            sda: Pin<impl GpioPort>,
            af: u8,
            _irqs: impl Binding<interrupt::I2C1>,
            config: Config,
        ) -> Self {
            Self::new_inner(regs, scl, sda, af, config)
        }

        /// 初始化 I2C2（IRQ 绑定收窄到 I2C2）。
        pub fn new_i2c2(
            regs: &'static pac::I2c,
            scl: Pin<impl GpioPort>,
            sda: Pin<impl GpioPort>,
            af: u8,
            _irqs: impl Binding<interrupt::I2C2>,
            config: Config,
        ) -> Self {
            Self::new_inner(regs, scl, sda, af, config)
        }

        fn new_inner(
            regs: &'static pac::I2c,
            scl: Pin<impl GpioPort>,
            sda: Pin<impl GpioPort>,
            af: u8,
            config: Config,
        ) -> Self {
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
            // 使能 NVIC 中断线（v0.1 遗漏修复）
            if idx == 0 {
                crate::pac::Interrupt::I2c1.enable();
            } else {
                crate::pac::Interrupt::I2c2.enable();
            }
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
                        ic::MISR_TXE
                            | ic::MISR_NACK
                            | ic::MISR_ARBI
                            | ic::MISR_FIFO
                            | ic::MISR_PINLOW,
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
}

#[cfg(feature = "cw32l012")]
pub use l012::*;

// ================= F030 家族（经典 8051 状态机式外设） =================

/// F030 经典家族驱动。寄存器位面取自生成 PAC（`pac::i2c`，与厂商头
/// `cw32f030_i2c.h` 一致），状态机时序逐条对照厂商库 `cw32f030_i2c.c`
/// （注释中的 `c:NN` 即该文件行号）。
#[cfg(feature = "cw32f030")]
mod f030 {
    use core::future::poll_fn;
    use core::task::Poll;

    use embedded_hal_async::i2c::Operation;

    use super::{Config, Speed, WAKER};
    use crate::gpio::{GpioPort, Pin, Reg};
    use crate::interrupt::{Binding, IrqHandler, interrupt};
    use crate::pac::{self, i2c as raw};

    // ---- CR/BRREN 位面（PAC 生成常量 = 厂商头位段） ----
    /// 总线滤波器（CR bit0；主机 BRR<=9 时置 1，见 [`I2c::new_inner`]）
    const CR_FLT: u32 = raw::cr::flt_MASK;
    /// 应答使能（CR bit2；主机接收需要，读尾两字节由流程动态清零）
    const CR_AA: u32 = raw::cr::aa_MASK;
    /// 中断标志（CR bit3）——状态码就绪信号。清除 = 写 0（读改写语义，
    /// 厂商 I2C_ClearIrq 即 `CR_f.SI = 0`，c:293-298；**不是** R1W0）
    const CR_SI: u32 = raw::cr::si_MASK;
    /// 产生 STOP（CR bit4）
    const CR_STO: u32 = raw::cr::sto_MASK;
    /// 产生 START/重复 START（CR bit5）
    const CR_STA: u32 = raw::cr::sta_MASK;
    /// 外设使能（CR bit6）
    const CR_EN: u32 = raw::cr::en_MASK;
    /// 波特率发生器使能（BRREN bit0；c:50-65 I2C_BaudGeneratorEnable）
    const BRREN_EN: u32 = raw::brren::en_MASK;
    /// STAT 状态码掩码（低 8 位；PAC 中 STAT 为只读）
    const STAT_MASK: u32 = raw::stat::stat_MASK;

    // ---- 状态码（8051 语义；出处:厂商库 cw32f030_i2c.c 主发/主收 switch） ----
    /// START 已发（c:488/560/610/675）
    const STAT_START: u8 = 0x08;
    /// 重复 START 已发（c:498/611）
    const STAT_REP_START: u8 = 0x10;
    /// SLA+W 已发、ACK 已收（c:492/564/679）
    const STAT_SLAW_ACK: u8 = 0x18;
    /// SLA+W 已发、NACK（c:570/692）
    const STAT_SLAW_NACK: u8 = 0x20;
    /// 数据字节已发、ACK 已收（c:495/567/683）
    const STAT_TXD_ACK: u8 = 0x28;
    /// 数据字节已发、NACK（c:574/696）
    const STAT_TXD_NACK: u8 = 0x30;
    /// 仲裁丢失（c:519/571/632/693；硬件自动清 STA/STO 并释放总线）
    const STAT_ARB_LOST: u8 = 0x38;
    /// SLA+R 已发、ACK 已收（c:502/615）
    const STAT_SLAR_ACK: u8 = 0x40;
    /// SLA+R 已发、NACK（c:522/635）
    const STAT_SLAR_NACK: u8 = 0x48;
    /// 收到数据字节、ACK 已回（c:508/621）
    const STAT_RXD_ACK: u8 = 0x50;
    /// 收到数据字节、NACK 已回（c:515/628）——读 run 末字节
    const STAT_RXD_NACK: u8 = 0x58;
    /// 总线错误（误启/误停；8051 语义，恢复序列为发 STOP）
    const STAT_BUS_ERROR: u8 = 0x00;
    // （0xF8 = 无相关信息，仅在 SI=0 时出现；等待 SI 期间不会读到）

    /// I2C 错误（经典家族）
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub enum I2cError {
        /// 地址或数据被 NACK（STAT 0x20/0x30/0x48）
        Nack,
        /// 仲裁失败（STAT 0x38；硬件已自动释放总线）
        Arbitration,
        /// 总线错误（STAT 0x00）或状态机收到意外状态码
        Bus,
    }

    impl embedded_hal::i2c::Error for I2cError {
        fn kind(&self) -> embedded_hal::i2c::ErrorKind {
            match self {
                I2cError::Nack => embedded_hal::i2c::ErrorKind::NoAcknowledge(
                    embedded_hal::i2c::NoAcknowledgeSource::Address,
                ),
                I2cError::Arbitration => embedded_hal::i2c::ErrorKind::ArbitrationLoss,
                I2cError::Bus => embedded_hal::i2c::ErrorKind::Bus,
            }
        }
    }

    /// 计算 BRR 分频值。⚠ 公式出处为厂商库注释，未经实板测频。
    ///
    /// **BRR = fPCLK/(8×fSCL) − 1**(厂商库 `I2C_SetBaud` 的 doxygen 注释,
    /// cw32f030_i2c.c:71——这是目前唯一有出处的口径;100kHz@8MHz → BRR=9,
    /// 恰好触发厂商 Master_Init 的 BRR≤9→FLT=1 滤波规则,一致性自洽)。
    /// 曾评估过另一口径 SCL = PCLK/(4×(BRR+2))(8051 风格惯例),但无任何
    /// 文档佐证,弃用;若实板测频发现厂商注释有误,再按
    /// "BRR只会配慢安全"原则回退。
    ///
    /// 结果饱和到 8 位(255 → 速率低于发生器下限、0 → 高于目标,实际 SCL
    /// 会偏离设定值);向上取整保证真实 SCL 恒 ≤ 目标。
    fn calc_brr(pclk_hz: u32, speed: Speed) -> u32 {
        let scl = match speed {
            Speed::Standard100k => 100_000,
            Speed::Fast400k => 400_000,
        };
        // fPCLK/(8×fSCL) − 1,向上取整的减法保守化:先整除再减会放大,
        // 用 (p + 8s − 1) / (8s) − 1 = ceil(p/(8s)) − 1
        pclk_hz.div_ceil(8 * scl).saturating_sub(1).min(0xFF)
    }

    /// 实例索引（0/1 = I2C1/2；两代基地址相同）
    fn instance_of(regs: &'static pac::I2C) -> usize {
        match regs as *const _ as usize {
            pac::I2C1_BASE => 0,
            pac::I2C2_BASE => 1,
            _ => unreachable!("非 I2C 寄存器块"),
        }
    }

    /// 关 NVIC 中断线（ICER 写 1——[`crate::interrupt::nvic_enable`] 的
    /// 对偶）。F030 I2C 寄存器块无 IER，风暴防护只能动 NVIC。
    fn nvic_disable<I: interrupt::Interrupt>() {
        unsafe { core::ptr::write_volatile(0xE000_E180 as *mut u32, 1 << I::NUMBER) }
    }

    /// 事务状态机的阶段（SI 事件到来时该做什么）
    #[derive(Clone, Copy)]
    enum Phase {
        /// START/重复 START 已完成，待装地址字节
        Start,
        /// SLA+W 已 ACK（0x18），待装首数据字节
        AddrW,
        /// 数据发送中（每个 0x28 装下一字节）
        Tx,
        /// SLA+R 已 ACK（0x40），待收首字节
        AddrR,
        /// 数据接收中（0x50/0x58 存字节、调 AA）
        Rx,
    }

    /// 写方向一次装载后的走向
    #[derive(Clone, Copy)]
    enum TxFlow {
        /// 已装字节，继续发送
        Load,
        /// 写 run 耗尽且下一操作为读——已发重复 START（回 Start 阶段）
        Restart,
        /// 已发 STOP，事务完成
        Done,
    }

    /// 异步 I2C 主机（F030 经典外设）
    pub struct I2c {
        regs: &'static pac::I2C,
        idx: usize,
        /// CR 常驻位面（EN|FLT）——每次 CR 写都带上（AA/STA/STO/SI 按流程显式给值）
        cr_base: u32,
    }

    impl I2c {
        /// 初始化 I2C 主机并配置引脚。
        ///
        /// * `regs` — I2C1/2 寄存器块
        /// * `scl` / `sda` — 引脚（内部自动 `set_af(af)`；常用组合见模块文档）
        /// * `_irqs` — 中断绑定证明（`bind_interrupts!` 生成的结构体）
        pub fn new(
            regs: &'static pac::I2C,
            scl: Pin<impl GpioPort>,
            sda: Pin<impl GpioPort>,
            af: u8,
            _irqs: impl Binding<interrupt::I2C1> + Binding<interrupt::I2C2>,
            config: Config,
        ) -> Self {
            Self::new_inner(regs, scl, sda, af, config)
        }

        /// 初始化 I2C1（IRQ 绑定收窄到 I2C1）。
        pub fn new_i2c1(
            regs: &'static pac::I2C,
            scl: Pin<impl GpioPort>,
            sda: Pin<impl GpioPort>,
            af: u8,
            _irqs: impl Binding<interrupt::I2C1>,
            config: Config,
        ) -> Self {
            Self::new_inner(regs, scl, sda, af, config)
        }

        /// 初始化 I2C2（IRQ 绑定收窄到 I2C2）。
        pub fn new_i2c2(
            regs: &'static pac::I2C,
            scl: Pin<impl GpioPort>,
            sda: Pin<impl GpioPort>,
            af: u8,
            _irqs: impl Binding<interrupt::I2C2>,
            config: Config,
        ) -> Self {
            Self::new_inner(regs, scl, sda, af, config)
        }

        fn new_inner(
            regs: &'static pac::I2C,
            scl: Pin<impl GpioPort>,
            sda: Pin<impl GpioPort>,
            af: u8,
            config: Config,
        ) -> Self {
            let idx = instance_of(regs);
            // 时钟门控：F030 APBEN 无 KEY 保护，读改写防顶掉其它使能位
            // （uart/spi.rs F030 分支同款）；I2C1=APBEN1.11、I2C2=APBEN1.12
            // （PAC sysctrl::apben1::i2c1/i2c2 位域）
            let sysctrl = unsafe { &*(pac::SYSCTRL_BASE as *const crate::Sysctrl) };
            match idx {
                0 => sysctrl.apben1.modify(|v| v | (1 << 11)),
                _ => sysctrl.apben1.modify(|v| v | (1 << 12)),
            }

            scl.set_af(af);
            sda.set_af(af);

            // 引脚电气配置(⚠ 2026-10-24 上板实锤的三件事,缺一状态机
            // 就挂死:CR.STA 置位但 START 永不发出、STAT 停 0xF8、SI 不来):
            // 1. ANALOG 必须清零——F030 复位默认 GPIOA_ANALOG=0x9fff(仅
            //    SWD 脚 PA13/14 数字使能),不清零 AF 外设到不了引脚;
            // 2. DIR=输出(DIR=0)——保持复位默认输入态时状态机不运转;
            // 3. OPENDRAIN=1 + PUR=1(开漏+内部上拉,I2C 必需;等价厂商
            //    BSP 的 GPIO_MODE_OUTPUT_OD 配方)
            // 两脚类型参数不同,各自走一遍(掩码取自 pin.mask())
            let scl_m = scl.mask();
            scl.port().mod_(Reg::Analog, |v| v & !scl_m);
            scl.port().mod_(Reg::Dir, |v| v & !scl_m);
            scl.port().mod_(Reg::OpEndrain, |v| v | scl_m);
            scl.port().mod_(Reg::Pur, |v| v | scl_m);
            let sda_m = sda.mask();
            sda.port().mod_(Reg::Analog, |v| v & !sda_m);
            sda.port().mod_(Reg::Dir, |v| v & !sda_m);
            sda.port().mod_(Reg::OpEndrain, |v| v | sda_m);
            sda.port().mod_(Reg::Pur, |v| v | sda_m);

            // 波特率：8 位分频值（c:74-79 I2C_SetBaud）+ 使能波特率发生器
            // （c:50-65）。⚠ 公式未经实板校准，见 calc_brr。
            let pclk = crate::PCLK_HZ.load(core::sync::atomic::Ordering::Relaxed);
            let brr = calc_brr(pclk, config.speed);
            regs.brr.write(brr);
            regs.brren.write(BRREN_EN);

            // 主机滤波规则：BRR<=9 → FLT=1（c:165-169 注释、c:324-329 Master_Init）
            let flt = if brr <= 9 { CR_FLT } else { 0 };

            // 使能 NVIC 中断线（IRQ23/24，两代 typelevel 同名同号）。中断仅由
            // CR.SI 门控——寄存器块无 IER（厂商库 I2C_GetIrq 即读 CR bit3，
            // c:282-286），SI=0 期间线静默，派发侧再压线防风暴。
            match idx {
                0 => crate::interrupt::nvic_enable::<interrupt::I2C1>(),
                _ => crate::interrupt::nvic_enable::<interrupt::I2C2>(),
            }

            // FLT/AA 先行（EN=0，等效 Master_Init c:320-330），再使能外设
            // （等效 I2C_Cmd c:259-274）。AA=1 常态：主机接收需应答，读尾
            // 两字节由流程动态清零（c:143-158 AcknowledgeConfig）。
            regs.cr.write(flt | CR_AA);
            let cr_base = CR_EN | flt;
            regs.cr.write(cr_base | CR_AA);

            Self { regs, idx, cr_base }
        }

        /// 写 CR：SI=0（清中断标志——读改写语义，经全位面绝对写实现，与
        /// 厂商 I2C_ClearIrq 的 `CR_f.SI = 0` 等价，c:293-298）。
        /// EN/FLT 常驻；STA/STO/AA/SI 由 `flags` 显式给值。
        fn cr(&self, flags: u32) {
            self.regs.cr.write(self.cr_base | flags);
        }

        /// 等 CR.SI 置位（状态码就绪，厂商 GenerateSTART 即轮询 CR bit3，
        /// c:104），返回 STAT 低 8 位。
        ///
        /// 错误状态码就地解码：NACK/总线错误发 STOP 复位总线后转 Err；
        /// 仲裁丢失时硬件已自动释放总线，仅清 SI 归位。
        async fn wait_si(&mut self) -> Result<u8, I2cError> {
            poll_fn(|cx| {
                WAKER[self.idx].register(cx.waker());
                if self.regs.cr.read() & CR_SI != 0 {
                    Poll::Ready(())
                } else {
                    // 开 NVIC 线（IRQ 派发侧会再压掉，防无等待者风暴；
                    // 线关断期间 SI 置位会被 NVIC 挂起，重开即触发——不丢唤醒）
                    match self.idx {
                        0 => crate::interrupt::nvic_enable::<interrupt::I2C1>(),
                        _ => crate::interrupt::nvic_enable::<interrupt::I2C2>(),
                    }
                    Poll::Pending
                }
            })
            .await;
            let stat = (self.regs.stat.read() & STAT_MASK) as u8;
            match stat {
                STAT_SLAW_NACK | STAT_TXD_NACK | STAT_SLAR_NACK => {
                    // NACK：发 STOP 复位总线（c:120-135 GenerateSTOP；
                    // c:582"此顺序不能调换"——STO 先置、清 SI 后行，此处
                    // 一次写同时完成两者）
                    self.cr(CR_AA | CR_STO);
                    Err(I2cError::Nack)
                }
                STAT_ARB_LOST => {
                    // 仲裁丢失（c:519/571）：硬件自动清 STA/STO 并释放总线，
                    // 非主机态置 STO 无效——清 SI 归位后报错（厂商库选择
                    // 重试 START，本驱动选择上抛）
                    self.cr(CR_AA);
                    Err(I2cError::Arbitration)
                }
                STAT_BUS_ERROR => {
                    // 总线错误（0x00）：发 STOP 复位（8051 惯例恢复序列）
                    self.cr(CR_AA | CR_STO);
                    Err(I2cError::Bus)
                }
                _ => Ok(stat),
            }
        }

        /// 方向 run（连续同向操作）结束后的收尾：下一操作切向 → 发重复
        /// START（返回 false，事务回 Start 阶段）；无后续 → 发 STOP（返回
        /// true，事务完成）。STO 先置、清 SI 后行（c:582"此顺序不能调换"）。
        fn switch_or_stop(&mut self, operations: &[Operation<'_>], op_i: usize) -> bool {
            if matches!(operations.get(op_i), Some(Operation::Write(_))) {
                // 0x18/0x28/0x58 态置 STA 发重复起始（8051 状态机允许；
                // 厂商库在 0x28 态同款操作，c:496）
                self.cr(CR_AA | CR_STA);
                false
            } else {
                self.cr(CR_AA | CR_STO);
                true
            }
        }

        /// 装载下一发送字节（跨连续 `Write` op 合并游标，见 [`next_tx_byte`]）。
        fn tx_load(
            &mut self,
            operations: &[Operation<'_>],
            op_i: &mut usize,
            byte_i: &mut usize,
        ) -> TxFlow {
            match next_tx_byte(operations, op_i, byte_i) {
                Some(b) => {
                    // 次序照厂商：先写 DR、再清 SI（c:381 SendData →
                    // c:530/586 ClearIrq——SI 清除才释放状态机，DR 必须先行）
                    self.regs.dr.write(b as u32);
                    self.cr(CR_AA);
                    TxFlow::Load
                }
                None => {
                    if self.switch_or_stop(operations, *op_i) {
                        TxFlow::Done
                    } else {
                        TxFlow::Restart
                    }
                }
            }
        }

        /// 防御：阶段与状态码不匹配（协议异常/从机态侵入；wait_si 已过滤
        /// 错误状态码，此处只会收到意外正向状态码）→ 发 STOP 复位并报 Bus。
        fn unexpected(&self) -> I2cError {
            self.cr(CR_AA | CR_STO);
            I2cError::Bus
        }

        /// 执行事务：单 START + 切向重复起始 + 结尾 STOP。
        ///
        /// 事件序列（每个 SI 事件一个动作，与厂商库主发/主收 switch 同款
        /// 次序）：START(0x08/0x10) → 地址字节 → 0x18/0x40 → 数据字节 →
        /// 0x28/0x50/0x58 → … → STOP。写方向跨连续 `Write` op 续传、读方向
        /// 跨连续 `Read` op 续传；`Write`↔`Read` 切向时在当前状态置 STA 发
        /// 重复起始（repeated-START）。
        async fn transaction_inner(
            &mut self,
            address: u8,
            operations: &mut [Operation<'_>],
        ) -> Result<(), I2cError> {
            if operations.is_empty() {
                return Ok(()); // 空事务无总线动作（与 L012 版一致）
            }
            // 地址字节 = 7 位地址 <<1 | R/W 位（c:405-423 Send7bitAddress：
            // 读 |= 0x01、写 &= 0xFE——左移后 bit0 天然为 0）
            let addr_w = (address << 1) as u32;
            let addr_r = addr_w | 1;

            let mut phase = Phase::Start;
            let mut op_i = 0usize;
            let mut byte_i = 0usize;

            // 发 START（c:87-106 GenerateSTART：STA=1、SI=0——一次写同时
            // 清残留 STA/STO/SI，兼作异常恢复。总线忙时硬件等其空闲再发起，
            // 8051 语义，无需先行等 BUSY——寄存器块也无 BUSY 位）
            self.cr(CR_AA | CR_STA);

            loop {
                let stat = self.wait_si().await?;
                match phase {
                    Phase::Start => {
                        if stat != STAT_START && stat != STAT_REP_START {
                            return Err(self.unexpected());
                        }
                        // START/重复 START 完成 → 装地址（R/W 位按当前操作）。
                        // 次序照厂商：先写 DR、再清 STA/SI（c:490 + c:105）
                        match operations.get(op_i) {
                            Some(Operation::Write(_)) => {
                                phase = Phase::AddrW;
                                self.regs.dr.write(addr_w);
                                self.cr(CR_AA);
                            }
                            Some(Operation::Read(_)) => {
                                phase = Phase::AddrR;
                                self.regs.dr.write(addr_r);
                                self.cr(CR_AA);
                            }
                            None => unreachable!("Start 阶段游标越界"),
                        }
                    }
                    Phase::AddrW => {
                        // 期望 0x18（SLA+W 已 ACK，c:492/564）
                        if stat != STAT_SLAW_ACK {
                            return Err(self.unexpected());
                        }
                        phase = Phase::Tx;
                        match self.tx_load(operations, &mut op_i, &mut byte_i) {
                            TxFlow::Load => {}
                            TxFlow::Restart => phase = Phase::Start,
                            TxFlow::Done => return Ok(()),
                        }
                    }
                    Phase::Tx => {
                        // 期望 0x28（数据字节已 ACK，c:495/567/683）
                        if stat != STAT_TXD_ACK {
                            return Err(self.unexpected());
                        }
                        match self.tx_load(operations, &mut op_i, &mut byte_i) {
                            TxFlow::Load => {}
                            TxFlow::Restart => phase = Phase::Start,
                            TxFlow::Done => return Ok(()),
                        }
                    }
                    Phase::AddrR => {
                        // 期望 0x40（SLA+R 已 ACK，c:502/615）
                        if stat != STAT_SLAR_ACK {
                            return Err(self.unexpected());
                        }
                        phase = Phase::Rx;
                        match read_remain(operations, op_i, byte_i) {
                            0 => {
                                // 空读 run：仅寻址，直接收尾/切向
                                advance_reads(operations, &mut op_i, &mut byte_i);
                                if self.switch_or_stop(operations, op_i) {
                                    return Ok(());
                                }
                                phase = Phase::Start;
                            }
                            // 唯一字节直接 NACK（c:503-506/616-619：仅 len>1
                            // 才置 AA=1）
                            1 => self.cr(0),
                            // 多字节：AA=1 正常接收
                            _ => self.cr(CR_AA),
                        }
                    }
                    Phase::Rx => match stat {
                        // 0x50：收到字节、ACK 已回（c:508/621）
                        STAT_RXD_ACK => {
                            let b = self.regs.dr.read() as u8;
                            match operations.get_mut(op_i) {
                                Some(Operation::Read(r)) if byte_i < r.len() => r[byte_i] = b,
                                _ => return Err(self.unexpected()),
                            }
                            byte_i += 1;
                            if advance_reads(operations, &mut op_i, &mut byte_i) {
                                // 还有读字节：剩 1 个则清 AA，让末字节以
                                // 0x58（NACK）到达（c:510-513/623-626：
                                // 倒数第二字节收完后 AA=0）
                                if read_remain(operations, op_i, byte_i) == 1 {
                                    self.cr(0);
                                } else {
                                    self.cr(CR_AA);
                                }
                            } else {
                                // 读 run 完成。防御：末字节竟以 ACK 到达
                                // （AA 管理异常）——按完成收尾避免死等 0x58
                                if self.switch_or_stop(operations, op_i) {
                                    return Ok(());
                                }
                                phase = Phase::Start;
                            }
                        }
                        // 0x58：末字节已 NACK（c:515-518/628-631）——读 run 正常终点
                        STAT_RXD_NACK => {
                            let b = self.regs.dr.read() as u8;
                            match operations.get_mut(op_i) {
                                Some(Operation::Read(r)) if byte_i < r.len() => r[byte_i] = b,
                                _ => return Err(self.unexpected()),
                            }
                            byte_i += 1;
                            advance_reads(operations, &mut op_i, &mut byte_i);
                            if self.switch_or_stop(operations, op_i) {
                                return Ok(());
                            }
                            phase = Phase::Start;
                        }
                        _ => return Err(self.unexpected()),
                    },
                }
            }
        }
    }

    /// 取下一待发字节：跨连续 `Write` op 合并（耗尽/空写 op 直接推进
    /// 游标），遇 `Read` op 或游标到尾返回 `None`（方向切换交给调用方
    /// 重复起始）。
    fn next_tx_byte(
        operations: &[Operation<'_>],
        op_i: &mut usize,
        byte_i: &mut usize,
    ) -> Option<u8> {
        loop {
            match operations.get(*op_i)? {
                Operation::Write(w) => {
                    if *byte_i < w.len() {
                        let b = w[*byte_i];
                        *byte_i += 1;
                        return Some(b);
                    }
                    // 本写 op 耗尽 → 下一 op（空写 op 同样穿透）
                    *op_i += 1;
                    *byte_i = 0;
                }
                _ => return None,
            }
        }
    }

    /// 推进读游标：跳过耗尽/空的连续 `Read` op；返回是否仍在读 run 内。
    fn advance_reads(operations: &[Operation<'_>], op_i: &mut usize, byte_i: &mut usize) -> bool {
        loop {
            match operations.get(*op_i) {
                Some(Operation::Read(r)) if *byte_i < r.len() => return true,
                Some(Operation::Read(_)) => {
                    *op_i += 1;
                    *byte_i = 0;
                }
                _ => return false,
            }
        }
    }

    /// 合并读 run 的剩余未收字节数（当前 op 剩余 + 后续连续 `Read` op 全长；
    /// AA=0 的时机按整个 run 计，跨 op 不重发地址）。
    fn read_remain(operations: &[Operation<'_>], op_i: usize, byte_i: usize) -> usize {
        let mut n = 0;
        for (k, op) in operations.iter().enumerate().skip(op_i) {
            match op {
                Operation::Read(r) => {
                    n += if k == op_i {
                        r.len().saturating_sub(byte_i)
                    } else {
                        r.len()
                    };
                }
                _ => break,
            }
        }
        n
    }

    impl embedded_hal_async::i2c::ErrorType for I2c {
        type Error = I2cError;
    }

    impl embedded_hal_async::i2c::I2c for I2c {
        /// 单 START + 切向重复起始：`[Write, Read]` 复用 repeated-START
        /// （经典外设状态机天然支持，比 L012 命令 FIFO 版的"逐 Operation
        /// 独立事务"更强，适合随机读类器件）。
        async fn transaction(
            &mut self,
            address: u8,
            operations: &mut [Operation<'_>],
        ) -> Result<(), Self::Error> {
            self.transaction_inner(address, operations).await
        }
    }

    // ================= 中断派发 =================

    /// 单实例中断处理：读 STAT（纯状态无清副作用，c:307-312 I2C_GetState；
    /// **不清 SI——留给任务侧推进状态机**）并唤醒等待者。压掉 NVIC 线防
    /// "无等待者 + SI 常亮"中断风暴（下次 wait_si 重开——L012 版 mier
    /// 压制模式的等价物；F030 无 IER 只能动 NVIC）。
    fn on_irq(idx: usize, regs: &'static pac::I2C) {
        let _ = regs.stat.read();
        WAKER[idx].wake();
        match idx {
            0 => nvic_disable::<interrupt::I2C1>(),
            _ => nvic_disable::<interrupt::I2C2>(),
        }
    }

    /// I2C1 中断标记（IRQ23）
    pub struct I2c1Irq;
    impl IrqHandler<interrupt::I2C1> for I2c1Irq {
        fn call() {
            on_irq(0, unsafe { &*(pac::I2C1_BASE as *const pac::I2C) });
        }
    }

    /// I2C2 中断标记（IRQ24）
    pub struct I2c2Irq;
    impl IrqHandler<interrupt::I2C2> for I2c2Irq {
        fn call() {
            on_irq(1, unsafe { &*(pac::I2C2_BASE as *const pac::I2C) });
        }
    }
}

#[cfg(feature = "cw32f030")]
pub use f030::*;
