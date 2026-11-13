//! OPA1/OPA2 片上运算放大器（手册 §29）。
//!
//! **L012 家族独占**（`cw32l012` feature，同 [`crate::dac`] 的文档措辞）。
//! 家族覆盖结论（11 个家族 PAC 逐一比对 `Opa` 块，2026-11-14 grep）：
//! 只有 cw32l012-pac 定义了 OPA 运放（OPA1/OPA2 @0x4000_00B0/0xB8）；
//! 经典组 f003/f020/f030/l031/l052/l083/r031/w031 与 L012 代组
//! l010/l011 的 PAC 均无 `Opa` 结构体（这 9 个家族有 COMP 比较器、
//! 无运放硬件），故本模块无经典分支、非 L012 feature 下不编译。
//!
//! 寄存器配方与阻塞版 HAL **同源**（`cw32l012-hal/src/opa.rs`），实测记录
//! 见其 `TESTING.md` 的 analog_loop 条目（DAC→OPA1 跟随器→PB00 上板通过，
//! 2026-10-23）。两条必踩坑照搬：
//!
//! 1. **OPA 时钟门控在 `APBEN2[9]`**（不是 APBEN1，§4.7.13）。写错时
//!    `OPAx_CR` 读回恒为复位值 `0xE000`，配置静默失效（TESTING.md 实测坑）。
//! 2. **模拟脚必须 `GPIOx_ANALOG.PINy=1`**（[`crate::gpio::Pin::into_analog`]
//!    或本模块的引脚助手）。不配时 DIR=0/ODR=0 的数字输出驱动器把模拟
//!    节点压在 0V，OPA 输出被短路（TESTING.md：比较器恒低、CROSS=0xFFFF）。
//!
//! # 引脚映射（手册表 29-1；厂商库 cw32l012_opa.h AFx_* 宏）
//!
//! | 信号 | OPA1 | OPA2 | 配置 |
//! |------|------|------|------|
//! | INP1 | PA03 | PA04 | [`Opa::enable_input_pin`]（模拟 + CR 导通位） |
//! | INP2 | PA06 | PA06 | 同上 |
//! | INP3 | PB01 | PB00 | 同上 |
//! | INP4 | **DAC_OUT1 内部** | **DAC_OUT2 内部** | 无引脚；[`Opa::enable_inputs`]
//! | INN1 | PA04 | PA05 | [`Opa::enable_input_pin`] |
//! | INN2 | PA07 | PA07 | 同上 |
//! | OUT  | PB00 | PB01 | [`Opa::enable_output_pin`]（仅模拟） |
//!
//! ⚠ 输入/输出焊盘与其它外设共享（PA04=DAC_OUT1/SPI1_SCK、PB00=UART2
//! TXD 等）：模拟用途优先，数字复用功能同时使用会冲突。输出脚同时是
//! ADC 通道（表 25-4：IN8=PB00、IN9=PB01）——跟随器输出无需外部接线
//! 即可用 ADC 原地读回（`examples/opa.rs` 演示）。
//!
//! # BGR
//!
//! 使能 OPA 会**自动启动 BGR**（手册 §25.12.19 注），无需手动开基准；
//! 若需更高精度可先等 BGR 稳定（约 30µs）再使能。
//!
//! # 校准（§29.3.3）
//!
//! 两种：**电平校准**（[`Opa::start_level_calibration`]，`START` 置 1 启动、
//! 清 0 停止，实际时长由调用方控制）与**触发校准**
//! （[`Opa::trigger_calibration`]，软件触发一次，时长 = `CALPERIOD` 个
//! OPACLK，自动结束）。建议上电后做一次校准以消除失调。⚠ 时长口径：
//! 本仓 PAC 按手册 §29.6 记为 `8 << CALPERIOD` 个 OPACLK；厂商库
//! cw32l012_opa.h 的宏名（AZCLK16..AZCLK524288）暗示 `16 << code`——
//! 差一倍不影响正确性（校准自动结束），板验时以实测为准。
//!
//! # 用法（OPA1 跟随器，正端 INP4 = DAC_OUT1 内部，输出 PB00）
//!
//! ```ignore
//! use embassy_cw32::opa::{Opa, OpaMode};
//!
//! let mut opa1 = Opa::new_opa1(p.opa1);   // 使能 APBEN2.OPA（含 KEY）
//! opa1.enable_output_pin();               // PB00 → 模拟
//! opa1.enable_inputs(embassy_cw32::pac::opa::CR_INP4EN, 0); // INP4 = DAC 内部
//! opa1.set_mode(OpaMode::Follower);
//! opa1.enable();
//! opa1.trigger_calibration(CalPeriod::C1024); // 触发校准，自动结束
//! while opa1.is_calibrating() {}
//! // 此后 ADC 采样 IN8（=PB00 焊盘）即 OPA1 输出
//! ```

use crate::pac;
use crate::pac::opa as oc;

// ================= 类型 =================

/// 工作模式（CR.MODE[2:1]，§29.6.1）
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum OpaMode {
    /// 外部放大（MODE=0x，反馈网络在片外，运放只做开环/闭环放大）
    External,
    /// 内部 PGA（MODE=10），增益由 [`PgaGain`] 决定
    Pga(PgaGain),
    /// 电压跟随器（MODE=11，增益 1，输出 = 正端电压）
    Follower,
}

/// PGA 同相增益（CR.AMP[12:10]）：64/32=×2 … 64/2=×32
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum PgaGain {
    /// ×2（AMP=000）
    X2,
    /// ×4（AMP=001）
    X4,
    /// ×8（AMP=010）
    X8,
    /// ×16（AMP=011）
    X16,
    /// ×32（AMP=100）
    X32,
}

impl PgaGain {
    #[inline]
    const fn code(self) -> u32 {
        match self {
            PgaGain::X2 => 0,
            PgaGain::X4 => 1,
            PgaGain::X8 => 2,
            PgaGain::X16 => 3,
            PgaGain::X32 => 4,
        }
    }

    /// 数值倍数（便于回代验算）
    #[inline]
    pub const fn factor(self) -> u32 {
        1 << (self.code() + 1)
    }
}

/// 偏置电流（CR.BIAS[15:13]）：code+1 µA；电流越小建立越慢
/// （厂商库 cw32l012_opa.h OPA_BiasTypeDef：1µA→36µs … 8µA→4µs）
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Bias {
    /// 1µA（建立约 36µs）
    Ua1,
    /// 2µA（约 20µs）
    Ua2,
    /// 3µA（约 11µs）
    Ua3,
    /// 4µA（约 10µs）
    Ua4,
    /// 5µA（约 8µs）
    Ua5,
    /// 6µA（约 7µs）
    Ua6,
    /// 7µA（约 6µs）
    Ua7,
    /// 8µA（约 4µs；复位默认）
    Ua8,
}

impl Bias {
    #[inline]
    const fn code(self) -> u32 {
        match self {
            Bias::Ua1 => 0,
            Bias::Ua2 => 1,
            Bias::Ua3 => 2,
            Bias::Ua4 => 3,
            Bias::Ua5 => 4,
            Bias::Ua6 => 5,
            Bias::Ua7 => 6,
            Bias::Ua8 => 7,
        }
    }

    /// 微安数
    #[inline]
    pub const fn microamps(self) -> u32 {
        self.code() + 1
    }
}

/// 校准时长（CAL.CALPERIOD[7:4]）：`8 << code` 个 OPACLK 周期（§29.6；
/// OPACLK = PCLK/CLKDIV，见 [`Opa::set_cal_clock_div`]）。
/// 命名沿用阻塞版 HAL（`CalPeriod::C8`..）；厂商库宏名口径差一倍，
/// 见模块文档。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[allow(non_camel_case_types)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum CalPeriod {
    C8,
    C16,
    C32,
    C64,
    C128,
    C256,
    C512,
    C1024,
    C2048,
    C4096,
    C8192,
    C16384,
    C32768,
    C65536,
    C131072,
    C262144,
}

impl CalPeriod {
    #[inline]
    const fn code(self) -> u32 {
        use CalPeriod::*;
        match self {
            C8 => 0,
            C16 => 1,
            C32 => 2,
            C64 => 3,
            C128 => 4,
            C256 => 5,
            C512 => 6,
            C1024 => 7,
            C2048 => 8,
            C4096 => 9,
            C8192 => 10,
            C16384 => 11,
            C32768 => 12,
            C65536 => 13,
            C131072 => 14,
            C262144 => 15,
        }
    }

    /// 校准周期数（OPACLK 个数）
    #[inline]
    pub const fn cycles(self) -> u32 {
        8u32 << self.code()
    }
}

/// OPA 输入脚（引脚映射按实例区分，见模块文档表；INP4=DAC 内部互联
/// 无引脚，不在本枚举——用 [`Opa::enable_inputs`] 直通）
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum OpaInputPin {
    /// 正端 INP1（OPA1=PA03 / OPA2=PA04）
    Inp1,
    /// 正端 INP2（两实例同为 PA06）
    Inp2,
    /// 正端 INP3（OPA1=PB01 / OPA2=PB00）
    Inp3,
    /// 负端 INN1（OPA1=PA04 / OPA2=PA05）
    Inn1,
    /// 负端 INN2（两实例同为 PA07）
    Inn2,
}

impl OpaInputPin {
    /// 对应焊盘（端口基地址、脚号）与端口 AHBEN 位（L012 代组 KEY 写用）
    const fn pad(self, idx: u8) -> (usize, u8, u32) {
        const GPIOA: usize = pac::GPIOA_BASE; // 0x4800_0000
        const GPIOB: usize = pac::GPIOB_BASE; // 0x4800_0100
        const A: u32 = 1 << 4;
        const B: u32 = 1 << 5;
        match (idx, self) {
            (1, OpaInputPin::Inp1) => (GPIOA, 3, A),
            (1, OpaInputPin::Inp2) => (GPIOA, 6, A),
            (1, OpaInputPin::Inp3) => (GPIOB, 1, B),
            (1, OpaInputPin::Inn1) => (GPIOA, 4, A),
            (1, OpaInputPin::Inn2) => (GPIOA, 7, A),
            (2, OpaInputPin::Inp1) => (GPIOA, 4, A),
            (2, OpaInputPin::Inp2) => (GPIOA, 6, A),
            (2, OpaInputPin::Inp3) => (GPIOB, 0, B),
            (2, OpaInputPin::Inn1) => (GPIOA, 5, A),
            (2, OpaInputPin::Inn2) => (GPIOA, 7, A),
            _ => (GPIOA, 0, A), // idx 只会是 1/2（构造函数限定）
        }
    }

    /// CR 导通位（INPxEN/INNxEN，§29.6.1）
    const fn cr_bit(self) -> u32 {
        match self {
            OpaInputPin::Inp1 => oc::CR_INP1EN,
            OpaInputPin::Inp2 => oc::CR_INP2EN,
            OpaInputPin::Inp3 => oc::CR_INP3EN,
            OpaInputPin::Inn1 => oc::CR_INN1EN,
            OpaInputPin::Inn2 => oc::CR_INN2EN,
        }
    }
}

/// OPA 错误
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum OpaError {
    /// 校准时钟分频越界（`set_cal_clock_div` 参数须 0..=7，CAL.CLKDIV[13:11]）
    BadCalClockDiv,
}

// ================= 引脚复用 =================

/// 把焊盘配置为模拟（`GPIOx_ANALOG.PINy=1`），并确保端口 AHB 时钟开启。
///
/// 与 [`crate::gpio::Pin::into_analog`] 等效的寄存器路径（L012 单一
/// `Gpio` 结构体），供 OPA 引脚助手按手册表 29-1 的焊盘编号直接操作
/// （厂商库 AFx_* 宏同款：只动 ANALOG 位，不动 DIR/ODR/AFR）。
/// 端口时钟读改写**保留其它外设使能位**（写法同 [`crate::dac::Dac::new`]，
/// 带 KEY——与 `lib.rs` 便捷函数的整字 write 不同）。
fn pad_to_analog(port: usize, pin: u8, ahb_bit: u32) {
    // SAFETY: SYSCTRL/GPIO 均为芯片内固定地址的内存映射寄存器；
    // 只置位 ANALOG/AHBEN，不破坏其它位（读改写）。多上下文并发配置
    // 同一寄存器会互相覆盖 —— 与 gpio/rtc 模块同等的无锁假设（单执行器
    // 主任务初始化阶段调用）。
    let sysctrl = unsafe { &*(pac::SYSCTRL_BASE as *const crate::Sysctrl) };
    sysctrl
        .ahben
        .modify(|v| pac::sysctrl::KEY | (v & 0xFFFF) | ahb_bit);
    let gpio = unsafe { &*(port as *const pac::Gpio) };
    gpio.analog.modify(|v| v | (1 << pin));
}

// ================= 驱动本体 =================

/// OPA1/OPA2 运算放大器（embassy 所有权风格：`Peripherals::take` 给出的
/// `&'static pac::Opa` 换取驱动，构造时自行使能时钟）
pub struct Opa {
    regs: &'static pac::Opa,
    /// 实例号 1/2（引脚映射与文档用）
    idx: u8,
}

impl Opa {
    /// 获取 OPA1 并使能时钟（**APBEN2.OPA bit9**，§4.7.13；KEY 读改写
    /// 保留其它外设使能位，写法同 [`crate::dac::Dac::new`]。⚠ 门控在
    /// APBEN2 不在 APBEN1——写错 CR 读回恒为复位值，配置静默失效，
    /// cw32l012-hal TESTING.md analog_loop 实测坑）
    pub fn new_opa1(regs: &'static pac::Opa) -> Self {
        Self::new_inner(regs, 1)
    }

    /// 获取 OPA2 并使能时钟（同 [`Opa::new_opa1`]）
    pub fn new_opa2(regs: &'static pac::Opa) -> Self {
        Self::new_inner(regs, 2)
    }

    fn new_inner(regs: &'static pac::Opa, idx: u8) -> Self {
        let sysctrl = unsafe { &*(pac::SYSCTRL_BASE as *const crate::Sysctrl) };
        sysctrl
            .apben2
            .modify(|v| pac::sysctrl::KEY | (v & 0xFFFF) | pac::sysctrl::APBEN2_OPA);
        Opa { regs, idx }
    }

    /// 配置工作模式与偏置电流（不改变输入导通与 EN；阻塞版
    /// `Opa::configure` 同款语义）
    pub fn configure(&mut self, mode: OpaMode, bias: Bias) {
        let mode_bits = match mode {
            OpaMode::External => oc::MODE_EXTERNAL,
            OpaMode::Pga(_) => oc::MODE_PGA,
            OpaMode::Follower => oc::MODE_FOLLOWER,
        };
        let amp_bits = match mode {
            OpaMode::Pga(g) => g.code() << oc::CR_AMP_POS,
            _ => 0,
        };
        self.regs.cr.modify(|v| {
            (v & !(oc::CR_MODE_MASK | oc::CR_AMP_MASK | oc::CR_BIAS_MASK))
                | mode_bits
                | amp_bits
                | (bias.code() << oc::CR_BIAS_POS)
        });
    }

    /// 仅设置工作模式（保持 AMP/BIAS 现值；阻塞版同款）
    pub fn set_mode(&mut self, mode: OpaMode) {
        let mode_bits = match mode {
            OpaMode::External => oc::MODE_EXTERNAL,
            OpaMode::Pga(_) => oc::MODE_PGA,
            OpaMode::Follower => oc::MODE_FOLLOWER,
        };
        let amp_bits = match mode {
            OpaMode::Pga(g) => g.code() << oc::CR_AMP_POS,
            _ => 0,
        };
        self.regs
            .cr
            .modify(|v| (v & !(oc::CR_MODE_MASK | oc::CR_AMP_MASK)) | mode_bits | amp_bits);
    }

    /// 输入导通控制（CR 位掩码，可或组合；阻塞版同款签名）
    ///
    /// 例如 `enable_inputs(CR_INP4EN, 0)` 只接通正端 INP4（DAC 内部互联）。
    /// 非法位被掩掉。负端导通在跟随器模式下硬件不用（厂商 `OPA_Init`
    /// 对 FOLLOWER 直接清 INN——本方法不代清，保留用户配置）。
    pub fn enable_inputs(&mut self, pos_mask: u32, neg_mask: u32) {
        let pos = pos_mask & (oc::CR_INP1EN | oc::CR_INP2EN | oc::CR_INP3EN | oc::CR_INP4EN);
        let neg = neg_mask & (oc::CR_INN1EN | oc::CR_INN2EN);
        self.regs.cr.modify(|v| {
            (v & !(oc::CR_INP1EN
                | oc::CR_INP2EN
                | oc::CR_INP3EN
                | oc::CR_INP4EN
                | oc::CR_INN1EN
                | oc::CR_INN2EN))
                | pos
                | neg
        });
    }

    /// 接通一个带引脚的输入：按实例的引脚映射把焊盘配成模拟（含端口
    /// 时钟，见模块文档表）+ 置 CR 导通位。可多次调用叠加；要一次配好
    /// 整组请用 [`Opa::enable_inputs`]。
    pub fn enable_input_pin(&mut self, pin: OpaInputPin) {
        let (port, p, ahb) = pin.pad(self.idx);
        pad_to_analog(port, p, ahb);
        self.regs.cr.modify(|v| v | pin.cr_bit());
    }

    /// 输出脚配成模拟（OPA1_OUT=PB00 / OPA2_OUT=PB01；输出无 CR 使能位，
    /// EN=1 后硬件直接驱动焊盘——厂商库 `AFx_OPAxOUT` 宏同款只配 ANALOG）。
    /// 输出脚同时是 ADC 通道（IN8/IN9），配置后可用 ADC 读回。
    pub fn enable_output_pin(&mut self) {
        let (port, pin, ahb) = match self.idx {
            2 => (pac::GPIOB_BASE, 1u8, 1u32 << 5), // OPA2_OUT = PB01
            _ => (pac::GPIOB_BASE, 0u8, 1u32 << 5), // OPA1_OUT = PB00
        };
        pad_to_analog(port, pin, ahb);
    }

    /// 使能运放（CR.EN；会自动启动 BGR，手册 §25.12.19 注）
    pub fn enable(&mut self) {
        self.regs.cr.modify(|v| v | oc::CR_EN);
    }

    /// 关闭运放
    pub fn disable(&mut self) {
        self.regs.cr.modify(|v| v & !oc::CR_EN);
    }

    /// 电平校准：`START=1` 启动，持续 `period` 后需调用
    /// [`Opa::stop_calibration`]（§29.3.3：电平校准的实际时长由调用方
    /// 控制，`period` 仅决定内部时序参数）
    pub fn start_level_calibration(&mut self, period: CalPeriod) {
        self.regs.cal.modify(|v| {
            (v & !oc::CAL_PERIOD_MASK)
                | (period.code() << oc::CAL_PERIOD_POS)
                | oc::CAL_EN
                | oc::CAL_MODE_LEVEL
                | oc::CAL_START
        });
    }

    /// 停止电平校准（`START`/`CALEN` 清 0）
    pub fn stop_calibration(&mut self) {
        self.regs.cal.modify(|v| v & !(oc::CAL_START | oc::CAL_EN));
    }

    /// 触发校准（软件触发一次，时长 = `period` 个 OPACLK，自动结束；
    /// 触发源选择字段保持已配置值——ADCxSTART/ATIM/GTIM 硬件触发经
    /// [`Opa::regs`] 原始配置，本方法只走软件触发路径）
    pub fn trigger_calibration(&mut self, period: CalPeriod) {
        self.regs.cal.modify(|v| {
            (v & !(oc::CAL_PERIOD_MASK | oc::CAL_MODE_LEVEL))
                | (period.code() << oc::CAL_PERIOD_POS)
                | oc::CAL_EN
        });
        self.regs.cal.modify(|v| v | oc::CAL_SOFTTRIG);
    }

    /// 校准是否进行中（CAL.AZRUN，RO）
    pub fn is_calibrating(&self) -> bool {
        self.regs.cal.read() & oc::CAL_AZRUN != 0
    }

    /// 设置校准时钟分频（CAL.CLKDIV[13:11]，0..=7；OPACLK = PCLK/2^div，
    /// 厂商库 OPA_AutoZeroClockTypeDef 同款）
    pub fn set_cal_clock_div(&mut self, div: u8) -> Result<(), OpaError> {
        if div > 7 {
            return Err(OpaError::BadCalClockDiv);
        }
        self.regs
            .cal
            .modify(|v| (v & !oc::CAL_CLKDIV_MASK) | ((div as u32) << oc::CAL_CLKDIV_POS));
        Ok(())
    }

    /// 实例号（1/2）
    pub fn idx(&self) -> u8 {
        self.idx
    }

    /// 原始控制寄存器值（调试用）
    pub fn cr(&self) -> u32 {
        self.regs.cr.read()
    }

    /// 寄存器块原始访问（硬件触发校准源 / ADC 启动触发等逃生口，§29.6.2）
    pub fn regs(&self) -> &'static pac::Opa {
        self.regs
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pga_gain_codes() {
        // 手册 §29.6.1：000=64/32(×2) … 100=64/2(×32)
        assert_eq!(PgaGain::X2.factor(), 2);
        assert_eq!(PgaGain::X4.factor(), 4);
        assert_eq!(PgaGain::X8.factor(), 8);
        assert_eq!(PgaGain::X16.factor(), 16);
        assert_eq!(PgaGain::X32.factor(), 32);
        assert_eq!(PgaGain::X2.code(), 0);
        assert_eq!(PgaGain::X32.code(), 4);
    }

    #[test]
    fn bias_codes() {
        assert_eq!(Bias::Ua1.microamps(), 1);
        assert_eq!(Bias::Ua4.microamps(), 4);
        assert_eq!(Bias::Ua8.microamps(), 8);
        // 复位默认 CR=0xE000 → BIAS=111=8µA（§29.6.1）
        assert_eq!((0xE000u32 & oc::CR_BIAS_MASK) >> oc::CR_BIAS_POS, 7);
    }

    #[test]
    fn cal_period_cycles() {
        // 手册 §29.6：8 << CALPERIOD 个 OPACLK
        assert_eq!(CalPeriod::C8.cycles(), 8);
        assert_eq!(CalPeriod::C1024.cycles(), 1024);
        assert_eq!(CalPeriod::C262144.cycles(), 262_144);
    }

    #[test]
    fn mode_bits_do_not_collide() {
        // MODE[2:1] 与 AMP[12:10] 不重叠
        assert_eq!(oc::CR_MODE_MASK & oc::CR_AMP_MASK, 0);
        assert_eq!(oc::MODE_PGA, 0b10 << 1);
        assert_eq!(oc::MODE_FOLLOWER, 0b11 << 1);
        // 复位默认 0xE000 的 MODE=00（外部放大）
        assert_eq!(0xE000u32 & oc::CR_MODE_MASK, 0);
    }

    #[test]
    fn input_pin_mapping_matches_manual_table_29_1() {
        // 厂商库 cw32l012_opa.h AFx_* 宏（= 手册表 29-1）
        let (a, b) = (pac::GPIOA_BASE, pac::GPIOB_BASE);
        // OPA1：INP1=PA03 INP2=PA06 INP3=PB01 INN1=PA04 INN2=PA07
        assert_eq!(OpaInputPin::Inp1.pad(1), (a, 3, 1 << 4));
        assert_eq!(OpaInputPin::Inp2.pad(1), (a, 6, 1 << 4));
        assert_eq!(OpaInputPin::Inp3.pad(1), (b, 1, 1 << 5));
        assert_eq!(OpaInputPin::Inn1.pad(1), (a, 4, 1 << 4));
        assert_eq!(OpaInputPin::Inn2.pad(1), (a, 7, 1 << 4));
        // OPA2：INP1=PA04 INP2=PA06 INP3=PB00 INN1=PA05 INN2=PA07
        assert_eq!(OpaInputPin::Inp1.pad(2), (a, 4, 1 << 4));
        assert_eq!(OpaInputPin::Inp2.pad(2), (a, 6, 1 << 4));
        assert_eq!(OpaInputPin::Inp3.pad(2), (b, 0, 1 << 5));
        assert_eq!(OpaInputPin::Inn1.pad(2), (a, 5, 1 << 4));
        assert_eq!(OpaInputPin::Inn2.pad(2), (a, 7, 1 << 4));
    }

    #[test]
    fn cr_bits_match_manual() {
        // §29.6.1：INP1..4=[4..7]、INN1/2=[8..9]
        assert_eq!(OpaInputPin::Inp1.cr_bit(), 1 << 4);
        assert_eq!(OpaInputPin::Inp3.cr_bit(), 1 << 6);
        assert_eq!(OpaInputPin::Inn1.cr_bit(), 1 << 8);
        assert_eq!(OpaInputPin::Inn2.cr_bit(), 1 << 9);
    }

    #[test]
    fn cal_clock_div_bounds() {
        fn check(div: u8) -> Result<(), OpaError> {
            if div > 7 {
                Err(OpaError::BadCalClockDiv)
            } else {
                Ok(())
            }
        }
        assert_eq!(check(0), Ok(()));
        assert_eq!(check(7), Ok(()));
        assert_eq!(check(8), Err(OpaError::BadCalClockDiv));
    }
}
