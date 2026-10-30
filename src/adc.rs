//! 异步 ADC（§25）：单次与序列转换，EOC/EOS 中断驱动。
//!
//! **L012 家族先行**（`cw32l012` feature，ADC1/ADC2 均支持）；F030
//! 经典家族的 ADC 寄存器面待实板回归后补齐（届时解除模块级 cfg）。
//!
//! 通道映射（表 25-4，ADC1 视角）：IN0..7=PA0..PA7、IN8=PB0、IN9=PB1、
//! IN10=PB10、IN11=PB2、IN12/13=DAC 输出（内部）、IN14=温度传感器
//! （内部，仅 ADC1）、IN15=1.2V 基准（内部，仅 ADC1）。ADC2 的外部
//! 通道映射不同（IN0=PA05 …，表 25-4）且无内部通道。
//!
//! 外部引脚须先 [`crate::gpio::Pin::into_analog`] 转模拟。
//!
//! ```ignore
//! embassy_cw32::bind_interrupts!(struct Irqs {
//!     ADC1 => adc::Adc1Irq;
//! });
//! let mut adc = Adc::new_adc1(p.adc1, AdcClk::Div2, Irqs);
//! let raw = adc.read(AdcChannel::In(0), SampleCycles::S24).await?; // PA0
//! let mv  = adc.vdda_mv().await?;                                  // VDDA 实测
//! ```
//!
//! 等待转换用 EOC/EOS 中断（等待时才开 IER 位，完毕/超时即压掉），
//! 10ms 软超时（最长转换 ≈550 ADCCLK，即使 Div8@1MHz 也 <1ms）。

use core::future::poll_fn;
use core::sync::atomic::Ordering;
use core::task::Poll;

use embassy_sync::waitqueue::AtomicWaker;
use embassy_time::{Duration, Timer, with_timeout};

use crate::interrupt::{Binding, IrqHandler, interrupt};
use crate::pac;

// ================= 家族位常量面 =================
// L012 手写 PAC 的 adc 常量与手册 §25 一致。IER 位面与 ISR 同布局
// （§25.7.19 IER：bit0 EOCIE / bit1 EOSIE，PAC 未定义故此处补）。
#[cfg(feature = "cw32l012")]
pub(crate) mod ab {
    pub use crate::pac::adc as raw;
    /// EOC 中断使能（IER bit0）
    pub const IER_EOC: u32 = 1 << 0;
    /// EOS 中断使能（IER bit1）
    pub const IER_EOS: u32 = 1 << 1;
    /// BGR 控制器 BGREN（复位默认已开，BGR_CR=0x105）
    pub const BGREN: u32 = crate::pac::bgr::CR_BGREN;
    /// BGR 控制器 TSEN
    pub const TSEN: u32 = crate::pac::bgr::CR_TSEN;
}

use ab::raw;

/// ADC 时钟分频（ADCCLK = PCLK / N）
#[allow(missing_docs)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u32)]
pub enum AdcClk {
    Div1 = 0,
    Div2 = 1,
    Div4 = 2,
    Div8 = 3,
}

/// 采样周期档位（表 25-2，单位 ADCCLK）
#[allow(missing_docs)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum SampleCycles {
    S6 = 0,
    S7 = 1,
    S9 = 2,
    S12 = 3,
    S18 = 4,
    S24 = 5,
    S30 = 6,
    S42 = 7,
    S54 = 8,
    S70 = 9,
    S102 = 10,
    S134 = 11,
    S166 = 12,
    S198 = 13,
    S262 = 14,
    S518 = 15,
}

/// 转换通道（表 25-4）
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AdcChannel {
    /// 外部引脚 / 内部信号（0..=13，映射见模块文档）
    In(u8),
    /// 内置温度传感器（IN14，仅 ADC1）
    TempSensor,
    /// 1.2V 内核基准（IN15，仅 ADC1）
    Bgr1v2,
}

impl AdcChannel {
    fn code(self) -> u32 {
        match self {
            AdcChannel::In(n) => n as u32 & 0xF,
            AdcChannel::TempSensor => ab::raw::CH_TS,
            AdcChannel::Bgr1v2 => ab::raw::CH_BGR,
        }
    }
}

/// ADC 错误
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AdcError {
    /// 转换未在 10ms 内完成（时钟门控没开 / 序列没启动？）
    Timeout,
    /// 校准值异常（0/0xFFFF，出厂未编程）
    BadCalibration,
    /// 内部通道在 ADC2 上不可用
    UnsupportedChannel,
    /// 温度传感器输出异常（原始值 <100 counts ≈ <0.08V，正常 ~0.78V）。
    ///
    /// 某板实测（2026-10-21 终审，`ts_probe` 判定矩阵 + 受控 POR）：
    /// 该芯片个体 TS 模拟块输出级无输出（硬件状态），非驱动问题。
    SensorDead,
}

/// per-实例 waker（0=ADC1，1=ADC2）
static ADC_WAKER: [AtomicWaker; 2] = [const { AtomicWaker::new() }; 2];

/// 异步 ADC
pub struct Adc {
    regs: &'static crate::Adc,
    idx: usize,
}

impl Adc {
    /// 初始化 ADC1（表 25-4 全通道 + 内部通道可用）。
    pub fn new_adc1(
        regs: &'static crate::Adc,
        clk: AdcClk,
        _irqs: impl Binding<interrupt::ADC1>,
    ) -> Self {
        Self::new_inner(regs, 0, clk)
    }

    /// 初始化 ADC2（外部通道映射不同，无内部通道）。
    pub fn new_adc2(
        regs: &'static crate::Adc,
        clk: AdcClk,
        _irqs: impl Binding<interrupt::ADC2_DAC>,
    ) -> Self {
        Self::new_inner(regs, 1, clk)
    }

    fn new_inner(regs: &'static crate::Adc, idx: usize, clk: AdcClk) -> Self {
        // APBEN1.ADC 时钟（L012 带写保护键）
        let sysctrl = unsafe { &*(pac::SYSCTRL_BASE as *const crate::Sysctrl) };
        sysctrl
            .apben1
            .write(pac::sysctrl::KEY | pac::sysctrl::APBEN1_ADC);

        // 模拟块复位 + 分频 + 上电（CR bit8 复位为 1 须保留，经 CR_RESET 置回）
        regs.cr
            .modify(|v| (v & !raw::CR_CLK_MASK) | raw::CR_RESET | ((clk as u32) << raw::CR_CLK_POS));
        regs.cr.modify(|v| v | raw::CR_EN);
        // 模拟电路初始化约 1µs（手册流程步骤 3）：构造函数里短阻塞，
        // 圈数按每圈 ~4 HCLK 周期估，各频率下约 4µs
        let n = crate::HCLK_HZ.load(Ordering::Relaxed) / 1_000_000;
        for _ in 0..n {
            core::hint::spin_loop();
        }

        match idx {
            0 => crate::interrupt::nvic_enable::<interrupt::ADC1>(),
            _ => crate::interrupt::nvic_enable::<interrupt::ADC2_DAC>(),
        }

        Adc { regs, idx }
    }

    /// 内部通道前置条件：确保 BGREN（及需要时的 TSEN）已使能并度过
    /// 稳定期。BGREN 复位默认已开（BGR_CR=0x105），常见路径零等待；
    /// 稳定期按保守 40µs（手册 §25.9 30µs + 裕量）异步等待。
    async fn ensure_internal_ready(&self, want_ts: bool) {
        let bgr = unsafe { &*(pac::BGR_BASE as *const pac::Bgr) };
        let cr = bgr.cr.read();
        let missing_bgr = cr & ab::BGREN == 0;
        let missing_ts = want_ts && cr & ab::TSEN == 0;
        if missing_bgr || missing_ts {
            let mask = ab::BGREN | if want_ts { ab::TSEN } else { 0 };
            bgr.cr.modify(|v| v | mask);
            Timer::after_micros(40).await;
        }
    }

    /// 单次转换一个通道（等 EOC，返回 0..4095）
    pub async fn read(&mut self, ch: AdcChannel, sample: SampleCycles) -> Result<u16, AdcError> {
        if self.idx == 1 && matches!(ch, AdcChannel::TempSensor | AdcChannel::Bgr1v2) {
            return Err(AdcError::UnsupportedChannel);
        }
        self.ensure_internal_ready(matches!(ch, AdcChannel::TempSensor)).await;
        let regs = self.regs;
        regs.start.write(0); // 停止并复位序列
        regs.sqrcfr.write(ch.code() << raw::SQRCH0_POS);
        regs.sample.write((sample as u32) << raw::SQRCH0_POS);
        regs.icr.write(raw::ICR_CLEAR_EOC_EOS);
        regs.cr.modify(|v| v & !raw::CR_ENS_MASK); // 单通道模式（ENS=0）
        regs.start.write(raw::START);
        match with_timeout(Duration::from_millis(10), wait_flag(regs, raw::ISR_EOC, ab::IER_EOC)).await
        {
            Ok(()) => {}
            Err(_) => {
                regs.start.write(0);
                return Err(AdcError::Timeout);
            }
        }
        let r = (regs.result0.read() & 0xFFF) as u16;
        regs.icr.write(raw::ICR_CLEAR_EOC_EOS);
        Ok(r)
    }

    /// 序列扫描转换（ENS 模式，2..=8 通道，一次启动按序全部转换）
    ///
    /// * `chs` / `samples` — 通道与采样周期（等长）；结果按通道顺序写入
    ///   `out`（长度 ≥ chs.len()）。等待 EOS（整序列完成）。
    /// * 内部通道前置条件自动处理；ADC2 上不可含内部通道。
    pub async fn read_sequence(
        &mut self,
        chs: &[AdcChannel],
        samples: &[SampleCycles],
        out: &mut [u16],
    ) -> Result<(), AdcError> {
        let n = chs.len();
        if n == 0 || n > 8 || samples.len() != n || out.len() < n {
            return Err(AdcError::UnsupportedChannel);
        }
        let any_ts = chs.iter().any(|c| matches!(c, AdcChannel::TempSensor));
        if self.idx == 1 && any_ts {
            return Err(AdcError::UnsupportedChannel);
        }
        if self.idx == 1
            && chs.iter().any(|c| matches!(c, AdcChannel::Bgr1v2))
        {
            return Err(AdcError::UnsupportedChannel);
        }
        self.ensure_internal_ready(any_ts).await;
        let regs = self.regs;
        let mut sqrcfr = 0u32;
        let mut sample = 0u32;
        for (i, ch) in chs.iter().enumerate() {
            sqrcfr |= ch.code() << (4 * i);
            sample |= (samples[i] as u32) << (4 * i);
        }
        regs.start.write(0); // 复位序列
        regs.sqrcfr.write(sqrcfr);
        regs.sample.write(sample);
        regs.icr.write(raw::ICR_CLEAR_EOC_EOS);
        regs.cr
            .modify(|v| (v & !raw::CR_ENS_MASK) | ((n as u32 - 1) << raw::CR_ENS_POS));
        regs.start.write(raw::START);
        match with_timeout(Duration::from_millis(10), wait_flag(regs, raw::ISR_EOS, ab::IER_EOS)).await
        {
            Ok(()) => {}
            Err(_) => {
                regs.start.write(0);
                regs.cr.modify(|v| v & !raw::CR_ENS_MASK);
                return Err(AdcError::Timeout);
            }
        }
        let results = [
            regs.result0.read(),
            regs.result1.read(),
            regs.result2.read(),
            regs.result3.read(),
            regs.result4.read(),
            regs.result5.read(),
            regs.result6.read(),
            regs.result7.read(),
        ];
        for (slot, r) in out.iter_mut().zip(results.iter()) {
            *slot = (r & 0xFFF) as u16;
        }
        regs.icr.write(raw::ICR_CLEAR_EOC_EOS);
        regs.cr.modify(|v| v & !raw::CR_ENS_MASK); // 恢复单通道模式
        Ok(())
    }

    /// 测量 VDDA（经 1.2V 基准通道 + 出厂校准，返回 mV）
    ///
    /// 公式（手册 §25.8 步骤 13）：`VDD = cal × 4095 / RESULT0`，
    /// cal 为 0x0010_07D2 处的 16-bit 工厂校准值（单位 mV）。
    pub async fn vdda_mv(&mut self) -> Result<u32, AdcError> {
        let cal = unsafe { (pac::calib::VDDA_ADDR as *const u16).read_volatile() };
        if cal == 0 || cal == 0xFFFF {
            return Err(AdcError::BadCalibration);
        }
        let raw = self.read(AdcChannel::Bgr1v2, SampleCycles::S166).await?;
        if raw == 0 {
            return Err(AdcError::BadCalibration);
        }
        Ok(cal as u32 * 4095 / raw as u32)
    }

    /// 测量环境温度（返回毫摄氏度 m℃，经 VDDA 自校准）
    ///
    /// 公式（手册 §25.9）：
    /// `T = T0×0.5 + 0.095765×(Vref×AdcValue − 4.095×Trim)`
    /// 其中 T0@0x0010_07CD（0.5℃ 单位）、Trim@0x0010_07CE（16bit）。
    pub async fn temperature_mc(&mut self) -> Result<i32, AdcError> {
        let vdda = self.vdda_mv().await?;
        self.temperature_mc_with_vref(vdda).await
    }

    /// 指定 Vref（mV）测量温度（避免重复测 VDDA）
    pub async fn temperature_mc_with_vref(&mut self, vref_mv: u32) -> Result<i32, AdcError> {
        let t0 = unsafe { (pac::calib::TS_T0_ADDR as *const u8).read_volatile() } as i64;
        let trim = unsafe { (pac::calib::TS_TRIM_ADDR as *const u16).read_volatile() } as i64;
        if trim == 0 || trim == 0xFFFF {
            return Err(AdcError::BadCalibration);
        }
        let adc = self.read(AdcChannel::TempSensor, SampleCycles::S262).await? as i64;
        // 传感器失效保护：正常室温下 TS 输出 ≈0.6~0.9V（raw 750~1100），
        // 读到 <100 counts 说明传感器无输出——拒绝计算，避免输出
        // "-271℃（绝对零度）"式假数据（见 [`AdcError::SensorDead`]）
        if adc < 100 {
            return Err(AdcError::SensorDead);
        }
        let diff = vref_mv as i64 * adc - 4095 * trim;
        let t_mc = t0 * 500 + 95_765 * diff / 1_000_000;
        Ok(t_mc as i32)
    }
}

/// 轮询等待 ISR 标志置位：等待时才开对应 IER 位，IRQ 唤醒后由本
/// poll 复查（标志清除由调用方负责——读结果后经 ICR R1W0 清）。
async fn wait_flag(regs: &'static crate::Adc, flag: u32, ier_bit: u32) {
    poll_fn(|cx| {
        ADC_WAKER[instance_idx(regs)].register(cx.waker());
        if regs.isr.read() & flag != 0 {
            Poll::Ready(())
        } else {
            regs.ier.modify(|v| v | ier_bit);
            Poll::Pending
        }
    })
    .await;
}

fn instance_idx(regs: &'static crate::Adc) -> usize {
    if regs as *const _ as usize == pac::ADC1_BASE {
        0
    } else {
        1
    }
}

// ================= 中断派发 =================

fn on_irq(idx: usize, regs: &'static crate::Adc) {
    let isr = regs.isr.read();
    // 唤醒等待者并单发压掉 IER 位（下次 poll 会重新打开）；ISR 标志
    // 由驱动读结果后经 ICR 清除，此处不动
    if isr & raw::ISR_EOC != 0 {
        regs.ier.modify(|v| v & !ab::IER_EOC);
        ADC_WAKER[idx].wake();
    }
    if isr & raw::ISR_EOS != 0 {
        regs.ier.modify(|v| v & !ab::IER_EOS);
        ADC_WAKER[idx].wake();
    }
}

/// ADC1 中断标记（IRQ12）
pub struct Adc1Irq;
impl IrqHandler<interrupt::ADC1> for Adc1Irq {
    fn call() {
        on_irq(0, unsafe { &*(pac::ADC1_BASE as *const crate::Adc) });
    }
}

/// ADC2/DAC 中断标记（IRQ30）
pub struct Adc2Irq;
impl IrqHandler<interrupt::ADC2_DAC> for Adc2Irq {
    fn call() {
        on_irq(1, unsafe { &*(pac::ADC2_BASE as *const crate::Adc) });
    }
}
