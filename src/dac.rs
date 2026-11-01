//! 异步 DAC（§26）：双通道 12 位直写 + DMA 波形搬运。
//!
//! **L012 家族先行**（`cw32l012` feature，同 [`crate::adc`]/[`crate::timer`]/
//! [`crate::dma`] 的文档措辞）；直写寄存器配方与阻塞版 HAL **同源**
//! （`cw32l012-hal/src/dac.rs`，实机验证见其 `examples/dac_adc_loop.rs`
//! ——DAC_OUT2→ADC IN12 片内回环 15 轮扫描 MAX_ERR=3 LSB，2026-10 上板通过）。
//!
//! # 通道与引脚（手册 §26 / 表 9-2）
//!
//! | 通道 | 引脚输出 | 内部互联 |
//! |------|----------|----------|
//! | CH1 | PA4（DAC_OUT1，需 [`Dac::enable_pin1`]） | ADC1_IN13（DAC_OUT1） |
//! | CH2 | PA5（DAC_OUT2，需 [`Dac::enable_pin2`]） | **ADC1_IN12（DAC_OUT2）** |
//!
//! ⚠ PA4/PA5 同时是 SPI1 的 SCK/MOSI（官方板屏幕占用中）——引脚输出会与
//! SPI 冲突；**内部互联（ADC 回环）不经过引脚，随时可用**。CH2→IN12 回环
//! 已实机验证；CH1→IN13 为同一互联机制，本仓未单独上板（待板验证）。
//!
//! # 直写与触发（§26.6）
//!
//! `CR0.TENx=0`（复位默认）：写 DHR 后数据经 1 个 dac_pclk 直接载入 DOR，
//! 输出立即改变——[`Dac::set_ch1`]/[`set_ch2`](Dac::set_ch2)/
//! [`set_both`](Dac::set_both) 走这条板验路径。
//! `TENx=1`：DHR 值在 TSELx 选择的触发源（表 26-1，见 [`DacTrig`]）来拍时
//! 才载入 DOR。改 TSEL 须先压 TENx（[`Dac::set_trigger`] 已处理）。
//!
//! # DMA 模式（§26.7——**语义为"DAC 请求搬运"，非"写 DHR 触发搬运"**）
//!
//! 手册 §26.7 原文要点：`CR0.DMAENx=1` + DMA 通道配**硬件触发**且触发源为
//! 相应 DAC 通道（表 8-8：`hs::DAC1=0x10` / `hs::DAC2=0x11`）时，**DAC
//! 完成 DOR 加载后自动请求 DMA 把下一个数据搬入 DHR**。即每个触发事件播放
//! 一个样点并预载下一样点，DMA 通道只需 EN=1 挂起等请求；触发快于 DMA 则
//! `DORx.DMAUDRx` 置下溢标志（RW0 写 0 清，§26.8 表 26-2），本驱动在
//! [`Dac::write_dma`]/[`Dac::start_dma_loop`] 收尾时检查并报
//! [`DacError::DmaUnderrun`]。
//!
//! 由此每样点播放时序：`触发 → DOR←DHR（播放当前样点）→ DAC 发请求 →
//! DMA 写 DHR（预载下一样点）`。所以 **N 个样点需要 N+1 次触发才完整播出**
//! （第 1 次触发播放的是 DHR 里的旧值）；[`Dac::write_dma`] 收尾时把末样点
//! 经直写路径补进 DOR，对调用方呈现"整段已播完"。
//!
//! 两个入口（DMA 通道均占用 **通道 2**，与 [`crate::dma::Dma::mem_to_periph`]
//! 一致；B16 位宽、源自增、目的固定 DHR12Rx）：
//!
//! - [`Dac::write_dma`]（one-shot）：前置条件是**触发源已在持续供拍**
//!   （如 GTIM1 TRGO，`examples/dac_wave.rs` 有完整配方）。等 DMA TC
//!   （len 次请求）后收尾：恢复直写模式（TENx/DMAENx=0）并补播末样点。
//!   ⚠ 无内置超时（纪律同 [`crate::dma::Channel::wait`]）——触发源不供拍
//!   将永久挂起，需要限时用 `embassy_time::with_timeout` 包一层。
//! - [`Dac::start_dma_loop`] / [`Dac::stop_dma`]（循环）：DMA 通道加
//!   `RESTART` 自动重传 + 源地址重载（§8 环形），触发源驱动下无限循环播放
//!   缓冲，调用后立即返回。⚠ **待板验证**：`RESTART`×环形在 DAC 请求源下
//!   的首块衔接时序本仓未上板核对（阻塞版无 DAC DMA 配方）；保守路径是
//!   周期性调用 [`Dac::write_dma`] 逐轮播放。
//!
//! ⚠ 软件触发（TSEL=0000）下 SWTRIG 写是否产生 DMA 请求：手册 §26.7 未
//! 排除（请求在"DOR 加载完成后"发出，未限定触发来源），但 STM32 F1/F0
//! 同构外设明确排除软件触发——本驱动不做 SWTRIG 泵（未实机验证的路径不进
//! API），write_dma 需要 GTIM/BTIM/ATIM/LPTIM 等真实触发源。
//!
//! DAC 中断（DMA 下溢，IRQ30 `ADC2_DAC`）本驱动不接：下溢经 DOR 标志轮询
//! 返回错误；要中断通知请经 [`Dac::regs`] 原始置 `CR0.DMAUDRIEx`（表 26-2）。
//!
//! # DAC 与 ADC 闭环自测（dac_adc_loop 先例）
//!
//! DAC 输出无需任何外部接线即可自检：CH2 的输出在片内直连 ADC1 IN12
//! （表 25-4），写 DAC 一个值立刻用 ADC 读回比对（典型误差 <15 LSB，本板
//! 实测 3 LSB）。embassy 版例程见 `examples/dac_adc_loop.rs`：
//!
//! ```ignore
//! let mut dac = Dac::new(p.dac);
//! let mut adc = Adc::new_adc1(p.adc1, AdcClk::Div2, Irqs);
//! dac.set_ch2(2048)?;
//! Timer::after_micros(50).await; // DHR→DOR 1 pclk + 模拟建立（µs 级）
//! let raw = adc.read(AdcChannel::In(12), SampleCycles::S24).await?; // ≈2048
//! ```
//!
//! # DMA 波形输出（GTIM1 TRGO 供拍，完整配方见 examples/dac_wave.rs）
//!
//! ```ignore
//! let mut dac = Dac::new(p.dac);
//! let mut dma = Dma::new(p.dma, Irqs); // Irqs 须绑 DMA12/DMA34
//! // 先把 GTIM1 配成 1kHz 更新 + TRGO（MMS=update），再：
//! dac.set_trigger(DacChannel::Ch1, DacTrig::Gtim1Trgo);
//! dac.write_dma(&mut dma, DacChannel::Ch1, &TRIANGLE).await?;
//! // 每 1ms 播一个样点，播完返回（输出停在末样点）
//! ```

use crate::dma::{Config, Dma, DmaError, Width};
use crate::pac;
use crate::pac::dac as dc;

// ================= CR0 位面（§26.10，PAC 未定义故此处补） =================
// PAC dac 模块已有 EN1/TEN1/EN2/TEN2/SWTRIGx/CR1_OUTx；TSEL/DMAEN/下溢位面
// 按手册 §26.10.1 CR0 补（厂商库 cw32l012.h DAC_CR0_* 同源）。

/// 通道 1 触发源选择（CR0[5:2]，表 26-1）
const CR0_TSEL1_POS: u32 = 2;
const CR0_TSEL1_MASK: u32 = 0xF << CR0_TSEL1_POS;
/// 通道 1 DMA 模式使能（CR0[12]，§26.10.1）
const CR0_DMAEN1: u32 = 1 << 12;
/// 通道 2 触发源选择（CR0[21:18]，表 26-1）
const CR0_TSEL2_POS: u32 = 18;
const CR0_TSEL2_MASK: u32 = 0xF << CR0_TSEL2_POS;
/// 通道 2 DMA 模式使能（CR0[28]，§26.10.1）
const CR0_DMAEN2: u32 = 1 << 28;
/// DOR1.DMAUDR1 DMA 下溢标志（DOR1[15]，RW0 写 0 清，§26.10.11）
const DOR_DMAUDR1: u32 = 1 << 15;
/// DOR2.DMAUDR2 DMA 下溢标志（DOR2[15]，RW0 写 0 清，§26.10.12）
const DOR_DMAUDR2: u32 = 1 << 15;

// ================= 类型 =================

/// DAC 通道
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum DacChannel {
    /// 通道 1（PA4/DAC_OUT1，内部互联 ADC1_IN13）
    Ch1,
    /// 通道 2（PA5/DAC_OUT2，内部互联 ADC1_IN12——板验回环通道）
    Ch2,
}

impl DacChannel {
    /// CR0.ENx 位
    const fn en_bit(self) -> u32 {
        match self {
            DacChannel::Ch1 => dc::CR0_EN1,
            DacChannel::Ch2 => dc::CR0_EN2,
        }
    }

    /// CR0.TENx 位
    const fn ten_bit(self) -> u32 {
        match self {
            DacChannel::Ch1 => dc::CR0_TEN1,
            DacChannel::Ch2 => dc::CR0_TEN2,
        }
    }

    /// CR0.DMAENx 位
    const fn dmaen_bit(self) -> u32 {
        match self {
            DacChannel::Ch1 => CR0_DMAEN1,
            DacChannel::Ch2 => CR0_DMAEN2,
        }
    }

    /// CR0.TSELx 位域（POS, MASK）
    const fn tsel_field(self) -> (u32, u32) {
        match self {
            DacChannel::Ch1 => (CR0_TSEL1_POS, CR0_TSEL1_MASK),
            DacChannel::Ch2 => (CR0_TSEL2_POS, CR0_TSEL2_MASK),
        }
    }

    /// DORx.DMAUDRx 标志位（两通道同在各自 DOR 的 bit15，§26.8）
    const fn dmaudr_bit(self) -> u32 {
        match self {
            DacChannel::Ch1 => DOR_DMAUDR1,
            DacChannel::Ch2 => DOR_DMAUDR2,
        }
    }

    /// DHR12Rx 寄存器地址（DMA 目的，固定不自增；§26.9 表 26-3 偏移）
    const fn dhr_addr(self, base: usize) -> u32 {
        match self {
            DacChannel::Ch1 => (base + 0x08) as u32, // DHR12R1
            DacChannel::Ch2 => (base + 0x14) as u32, // DHR12R2
        }
    }

    /// DORx 寄存器地址（下溢标志检查/清除用）
    const fn dor_addr(self, base: usize) -> usize {
        match self {
            DacChannel::Ch1 => base + 0x2C, // DOR1
            DacChannel::Ch2 => base + 0x30, // DOR2
        }
    }

    /// DMA 硬件触发源（表 8-8，PAC dma::hs）
    const fn hs(self) -> u8 {
        match self {
            DacChannel::Ch1 => pac::dma::hs::DAC1,
            DacChannel::Ch2 => pac::dma::hs::DAC2,
        }
    }
}

/// DAC 触发源（CR0.TSELx，§26.6 表 26-1；ETR 变体 1010..1111 经 [`Dac::regs`]
/// 原始配置，本枚举不含）
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum DacTrig {
    /// 软件触发（写 SWTRGR；⚠ 未实机验证 SWTRIG→DMA 请求路径，见模块文档）
    Software,
    /// BTIM1 TRGO
    Btim1Trgo,
    /// BTIM2 TRGO
    Btim2Trgo,
    /// BTIM3 TRGO
    Btim3Trgo,
    /// GTIM1 TRGO（dac_wave 例程的供拍源）
    Gtim1Trgo,
    /// GTIM2 TRGO
    Gtim2Trgo,
    /// GTIM3 TRGO
    Gtim3Trgo,
    /// GTIM4 TRGO
    Gtim4Trgo,
    /// ATIM TRGO
    AtimTrgo,
    /// LPTIM 溢出
    LptimOv,
}

impl DacTrig {
    /// 表 26-1 的 TSELx 位域编码
    pub const fn code(self) -> u32 {
        match self {
            DacTrig::Software => 0x0,
            DacTrig::Btim1Trgo => 0x1,
            DacTrig::Btim2Trgo => 0x2,
            DacTrig::Btim3Trgo => 0x3,
            DacTrig::Gtim1Trgo => 0x4,
            DacTrig::Gtim2Trgo => 0x5,
            DacTrig::Gtim3Trgo => 0x6,
            DacTrig::Gtim4Trgo => 0x7,
            DacTrig::AtimTrgo => 0x8,
            DacTrig::LptimOv => 0x9,
        }
    }
}

/// DAC 错误
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum DacError {
    /// 12 位值域越界（>0xFFF）
    ValueOutOfRange,
    /// DMA 缓冲为空或超过 65535 元素
    BadLength,
    /// 底层 DMA 通道错误（表 8-2 STATUS 解码，透传 [`DmaError`]）
    Dma(DmaError),
    /// DMA 下溢（`DORx.DMAUDRx=1`，§26.7：触发快于 DMA；标志已代为清除）
    DmaUnderrun,
}

/// 12 位值域检查
fn check12(v: u16) -> Result<(), DacError> {
    if v > 0xFFF {
        Err(DacError::ValueOutOfRange)
    } else {
        Ok(())
    }
}

// ================= 驱动本体 =================

/// 异步 DAC（双通道；直写模式 + DMA 波形搬运）
pub struct Dac {
    regs: &'static pac::Dac,
}

impl Dac {
    /// 获取 DAC 并使能时钟（**APBEN2.DAC bit10**，§4.7；阻塞版
    /// `rcc.enable_dac` 实机配方。KEY 读改写保留其它外设使能位，写法同
    /// [`crate::timer`]/[`crate::rtc`]）
    pub fn new(regs: &'static pac::Dac) -> Self {
        let sysctrl = unsafe { &*(pac::SYSCTRL_BASE as *const crate::Sysctrl) };
        sysctrl
            .apben2
            .modify(|v| pac::sysctrl::KEY | (v & 0xFFFF) | pac::sysctrl::APBEN2_DAC);
        Dac { regs }
    }

    /// 写通道 1（0..4095；未使能通道时自动使能，阻塞版板验配方：
    /// TEN=0 直写，1 个 dac_pclk 载入 DOR）
    pub fn set_ch1(&mut self, value: u16) -> Result<(), DacError> {
        check12(value)?;
        self.regs.cr0.modify(|v| v | dc::CR0_EN1);
        self.regs.dhr12r1.write(value as u32);
        Ok(())
    }

    /// 写通道 2（0..4095；未使能通道时自动使能）
    pub fn set_ch2(&mut self, value: u16) -> Result<(), DacError> {
        check12(value)?;
        self.regs.cr0.modify(|v| v | dc::CR0_EN2);
        self.regs.dhr12r2.write(value as u32);
        Ok(())
    }

    /// 同时写双通道（一次总线写 DHR12RD：低 16 位 CH1、高 16 位 CH2）
    pub fn set_both(&mut self, ch1: u16, ch2: u16) -> Result<(), DacError> {
        check12(ch1)?;
        check12(ch2)?;
        // 双通道写对未使能的通道同样自动使能（阻塞版配方：先并位再一次 modify）
        let en = (!self.regs.cr0.read()) & (dc::CR0_EN1 | dc::CR0_EN2);
        if en != 0 {
            self.regs.cr0.modify(|v| v | en);
        }
        self.regs.dhr12rd.write((ch2 as u32) << 16 | ch1 as u32);
        Ok(())
    }

    /// 读通道 1 实际输出值（DOR1 低 12 位；bit15 是 DMA 下溢标志，已掩掉
    /// ——下溢状态经 [`Dac::write_dma`] 的返回值或 [`Dac::dmaudr`] 查看）
    pub fn output1(&self) -> u16 {
        (self.regs.dor1.read() & dc::DHR12_MASK) as u16
    }

    /// 读通道 2 实际输出值（DOR2 低 12 位）
    pub fn output2(&self) -> u16 {
        (self.regs.dor2.read() & dc::DHR12_MASK) as u16
    }

    /// DMA 下溢标志（`DORx.DMAUDRx`，§26.8 表 26-2；true=发生过下溢）。
    /// 读后**代为清除**（RW0 写 0）——要保留现场请勿调用本方法。
    pub fn dmaudr(&mut self, ch: DacChannel) -> bool {
        let addr = ch.dor_addr(self.regs as *const _ as usize) as *mut u32;
        // PAC 把 DORx 建模为只读（数据位 RO），但 DMAUDR 位是 RW0（§26.8
        // "写 0 到 DORx.DMAUDRx"）——经裸指针清位；DATA 位写回被硬件忽略
        // （阻塞版/厂商库 DAC_ClearFlag 的 DOR1_f.DMAUDR=0 同款位域清法）
        let v = unsafe { core::ptr::read_volatile(addr) };
        let flag = v & ch.dmaudr_bit() != 0;
        if flag {
            unsafe { core::ptr::write_volatile(addr, v & !ch.dmaudr_bit()) };
        }
        flag
    }

    /// 通道 1 输出到 PA4 引脚（⚠ 与 SPI1_SCK/OPA 共享，内部互联不受影响）
    pub fn enable_pin1(&mut self) {
        self.regs.cr1.modify(|v| v | dc::CR1_OUT1);
    }

    /// 通道 2 输出到 PA5 引脚（⚠ 与 SPI1_MOSI/OPA 共享，内部互联不受影响）
    pub fn enable_pin2(&mut self) {
        self.regs.cr1.modify(|v| v | dc::CR1_OUT2);
    }

    /// 配置触发源（表 26-1 写 TSELx；须在触发未使能时调用——本方法先压
    /// TENx 再写位域，通道使能状态保持不变）
    pub fn set_trigger(&mut self, ch: DacChannel, trig: DacTrig) {
        let (pos, mask) = ch.tsel_field();
        self.regs.cr0.modify(|v| {
            (v & !ch.ten_bit() & !mask) | (trig.code() << pos)
        });
    }

    /// DMA 波形播放（one-shot）：整段缓冲经 DMA 自动搬运到 DHR，逐触发
    /// 播出，DMA TC（len 次请求）后返回
    ///
    /// * `dma` — DMA 控制器（内部占用**通道 2**，B16 位宽、源自增、目的
    ///   固定 DHR12Rx，硬件触发 `hs::DAC1/DAC2`，§26.7 语义见模块文档）
    /// * `ch` — 通道
    /// * `buf` — 波形样点（每个 0..4095，超界整体拒绝）
    ///
    /// # 前置条件
    ///
    /// **触发源必须在持续供拍**（`set_trigger` 选好 GTIM/BTIM/ATIM TRGO 等
    /// 并把该定时器跑起来，`examples/dac_wave.rs` 有 GTIM1 TRGO 完整配方）。
    /// 复位默认的软件触发（TSEL=0000）下无人写 SWTRGR，请求永不到来，
    /// 本方法将永久挂起——软件触发 + DMA 的组合未实机验证，不做泵。
    ///
    /// # 时序与收尾（§26.7）
    ///
    /// 每次触发：DOR←DHR（播放当前样点）→ DAC 发请求 → DMA 写 DHR（预载
    /// 下一样点）。故 N 样点需 N+1 次触发才完整播出（第 1 次触发播的是
    /// DHR 旧值）；TC（= N 次搬运完成）后本方法恢复直写模式（TENx/DMAENx=0，
    /// TSEL 保持）并把末样点经直写补进 DOR。全程检查 DMA 下溢（触发快于
    /// DMA），发生则返回 [`DacError::DmaUnderrun`]。
    ///
    /// ⚠ 无内置超时——触发源停拍将挂起；需要限时用
    /// `embassy_time::with_timeout` 包裹（超时后请 [`Dac::stop_dma`] 收尾）。
    pub async fn write_dma(
        &mut self,
        dma: &mut Dma,
        ch: DacChannel,
        buf: &[u16],
    ) -> Result<(), DacError> {
        if buf.is_empty() || buf.len() > 0xFFFF {
            return Err(DacError::BadLength);
        }
        for &v in buf {
            check12(v)?;
        }
        let base = self.regs as *const _ as usize;

        // CR0：使能通道 + 触发 + DMA 模式（TSEL 保持 set_trigger 配好的值）
        let en = ch.en_bit() | ch.ten_bit() | ch.dmaen_bit();
        self.regs.cr0.modify(|v| v | en);

        // DMA 通道 2：mem_to_periph（源自增、目的固定、BLOCK、硬件触发）。
        // CNT=len、REPEAT=1——§26.7 语义下一次请求搬一个样点，触发源供拍
        let mut dch = dma.channel(2);
        let cfg = Config::mem_to_periph(
            buf.as_ptr() as u32,
            ch.dhr_addr(base),
            buf.len() as u16,
            Width::B16,
            ch.hs(),
        );
        dch.configure(&cfg); // EN=1，挂起等 DAC 请求

        // 等 DMA TC（len 次请求完成后置位；无内置超时，见方法文档）
        dch.wait().await.map_err(DacError::Dma)?;

        // 收尾：停 DMA 通道 → 恢复直写模式（DMAENx/TENx=0，TSEL 保留）→
        // 末样点经直写补播（第 N+1 次触发的播放效果）
        dch.stop();
        self.regs.cr0.modify(|v| v & !(ch.dmaen_bit() | ch.ten_bit()));
        match ch {
            DacChannel::Ch1 => self.regs.dhr12r1.write(*buf.last().unwrap() as u32),
            DacChannel::Ch2 => self.regs.dhr12r2.write(*buf.last().unwrap() as u32),
        }

        // 下溢检查（有则代清标志并报错——样点有跳变，不静默）
        if self.dmaudr(ch) {
            return Err(DacError::DmaUnderrun);
        }
        Ok(())
    }

    /// DMA 波形循环播放（continuous）：缓冲经 DMA 环形搬运，触发源驱动下
    /// 无限循环，配置完成后**立即返回**（不等待）
    ///
    /// 与 [`Dac::write_dma`] 的差别：DMA 通道加 `RESTART` 自动重传 + 源地址
    /// 重载（§8 环形搬运；dma.rs「硬件触发 + RESTART 可做无限环形搬运」），
    /// 触发源供拍下每个样点 1 次触发、整段循环。TC 标志每块置位但不开
    /// TCIE（无中断风暴），[`Dac::stop_dma`] 停止并清理。
    ///
    /// ⚠ **待板验证**：阻塞版无 DAC DMA 实机配方，`RESTART`×环形在 DAC
    /// 请求源下的首块衔接时序未上板核对；保守路径是周期性调用
    /// [`Dac::write_dma`]。首块第 1 次触发会播放 DHR 里的旧值（一次，
    /// 之后衔接循环）。
    pub fn start_dma_loop(
        &mut self,
        dma: &mut Dma,
        ch: DacChannel,
        buf: &[u16],
    ) -> Result<(), DacError> {
        if buf.is_empty() || buf.len() > 0xFFFF {
            return Err(DacError::BadLength);
        }
        for &v in buf {
            check12(v)?;
        }
        let base = self.regs as *const _ as usize;

        // CR0：使能通道 + 触发 + DMA 模式（TSEL 保持已配置值）
        let en = ch.en_bit() | ch.ten_bit() | ch.dmaen_bit();
        self.regs.cr0.modify(|v| v | en);

        // DMA 通道 2：环形版 mem_to_periph（SRCLOAD 重载 + RESTART 自动重传）
        let dch = dma.channel(2);
        let mut cfg = Config::mem_to_periph(
            buf.as_ptr() as u32,
            ch.dhr_addr(base),
            buf.len() as u16,
            Width::B16,
            ch.hs(),
        );
        cfg.src_reload = true;
        cfg.restart = true;
        dch.configure(&cfg); // EN=1，挂起等 DAC 请求，循环不等 TC
        Ok(())
    }

    /// 停止 [`Dac::start_dma_loop`] 的循环播放（停 DMA 通道、清残留标志、
    /// 恢复直写模式；输出停在停止时 DOR 里的值）
    pub fn stop_dma(&mut self, dma: &mut Dma, ch: DacChannel) {
        let dch = dma.channel(2);
        dch.stop();
        dch.clear_flags();
        self.regs.cr0.modify(|v| v & !(ch.dmaen_bit() | ch.ten_bit()));
        self.dmaudr(ch); // 代清可能残留的下溢标志
    }

    /// 寄存器块原始访问（WAVE 波形发生器 / MAMP / ETR 触发源 / DMAUDRIE
    /// 中断逃生口，§26.10）
    pub fn regs(&self) -> &'static pac::Dac {
        self.regs
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn value_range() {
        assert!(check12(0).is_ok());
        assert!(check12(0xFFF).is_ok());
        assert_eq!(check12(0x1000), Err(DacError::ValueOutOfRange));
        assert_eq!(check12(0xFFFF), Err(DacError::ValueOutOfRange));
    }

    #[test]
    fn trig_codes_match_table_26_1() {
        // 手册 §26.6 表 26-1：0000 软件、0001-0011 BTIM1-3、0100-0111
        // GTIM1-4、1000 ATIM、1001 LPTIM_OV
        assert_eq!(DacTrig::Software.code(), 0);
        assert_eq!(DacTrig::Btim1Trgo.code(), 1);
        assert_eq!(DacTrig::Btim3Trgo.code(), 3);
        assert_eq!(DacTrig::Gtim1Trgo.code(), 4);
        assert_eq!(DacTrig::Gtim4Trgo.code(), 7);
        assert_eq!(DacTrig::AtimTrgo.code(), 8);
        assert_eq!(DacTrig::LptimOv.code(), 9);
    }

    #[test]
    fn channel_bit_planes_match_manual() {
        // 手册 §26.10.1 CR0：EN1[0]/TEN1[1]/TSEL1[5:2]/DMAEN1[12]；
        // 通道 2 镜像 +16（EN2[16]/TEN2[17]/TSEL2[21:18]/DMAEN2[28]）
        assert_eq!(DacChannel::Ch1.en_bit(), 1 << 0);
        assert_eq!(DacChannel::Ch1.ten_bit(), 1 << 1);
        assert_eq!(DacChannel::Ch1.tsel_field(), (2, 0xF << 2));
        assert_eq!(DacChannel::Ch1.dmaen_bit(), 1 << 12);
        assert_eq!(DacChannel::Ch2.en_bit(), 1 << 16);
        assert_eq!(DacChannel::Ch2.ten_bit(), 1 << 17);
        assert_eq!(DacChannel::Ch2.tsel_field(), (18, 0xF << 18));
        assert_eq!(DacChannel::Ch2.dmaen_bit(), 1 << 28);
        // §26.10.11/12：下溢标志同在各自 DOR 的 bit15
        assert_eq!(DacChannel::Ch1.dmaudr_bit(), 1 << 15);
        assert_eq!(DacChannel::Ch2.dmaudr_bit(), 1 << 15);
    }

    #[test]
    fn dhr_dor_offsets_match_register_map() {
        // 手册 §26.9 表 26-3：DHR12R1@+0x08、DHR12R2@+0x14、DOR1@+0x2C、
        // DOR2@+0x30（PAC 结构体同布局）
        let base = 0x4000_00C0usize; // DAC_BASE
        assert_eq!(DacChannel::Ch1.dhr_addr(base), 0x4000_00C8);
        assert_eq!(DacChannel::Ch2.dhr_addr(base), 0x4000_00D4);
        assert_eq!(DacChannel::Ch1.dor_addr(base), base + 0x2C);
        assert_eq!(DacChannel::Ch2.dor_addr(base), base + 0x30);
    }
}
