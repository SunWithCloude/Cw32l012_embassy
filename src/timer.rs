//! 异步定时器 PWM：两代定时器 IP，按家族 feature 二选一编译。
//!
//! - **L012 代组**（`cw32l012`/`cw32l010`/`cw32l011`，手册 §16）：GTIM
//!   单通道 PWM 输出 + 更新事件等待，见 [`l012gen`] 分支（`GtimPwm`）。
//!   STM32 风格寄存器面（PSC/ARR/CCR/CCMR/CCER/IER/EGR，影子寄存器
//!   预装载）。实例数随家族：L012 = GTIM1..4、l011 = GTIM1/GTIM2、
//!   l010 = GTIM1 单实例。三家族一套代码，字段命名差异（dier/ier、
//!   ccmr1/ccmr1cap）经薄适配收敛，见 [`l012gen`] 分支头注差异表。
//! - **经典家族**（`cw32f030`/`cw32f020`/`cw32l031`/`cw32l052`/`cw32l083`/
//!   `cw32r031`/`cw32w031`）：经典代 GTIM 的单通道 PWM + 溢出节拍等待，
//!   见 [`classic`] 分支（`ClassicPwm`）。与 L012 **无一处同构**：
//!   CR0（EN/MODE/PRS）+ CMMR（CCxM 4bit）+ IER/ISR/ICR（OV/TI/UD/CCx），
//!   无 PSC 独立寄存器的 f030/f020（分频走 CR0.PRS 2^n）与带 PSC 线性
//!   分频寄存器的 l031 组/l052/l083 两个子面；无 EGR、无影子寄存器
//!   （ARR/CCR 立即生效）、无 ARPE/OCxPE 预装载位。
//!
//! 选型说明：经典家族的 ATIM（高级 PWM，带死区/互补/刹车）只有
//! f030/f003/l031/l052/l083/r031/w031 有——**f020 无 ATIM**，故选
//! GTIM 做全家族覆盖（7/7 家族均有 GTIM）；厂商库的 GTIM 输出比较
//! API（`GTIM_OCInit` + `GTIM_SetCompareN`）也是简单 PWM 的官方配方。
//!
//! # 频率与占空比（两代共用约定）
//!
//! `f_pwm = PCLK / (分频 × (ARR+1))`，`new` 自动选最小分频使 ARR 尽量大
//! （量化精度最高，四舍五入同阻塞版配方）。占空比以**万分比**
//! （0..=10000）为满量程（同阻塞版 `PwmPin::MAX_DUTY`，喂给
//! `embedded_hal::pwm::SetDutyCycle` 时上层按 `max_duty_cycle()` 归一化）。
//!
//! # 中断模型
//!
//! PWM 本身是纯寄存器写（`set_duty_cycle` 无需异步等待），构造**不要求**
//! IRQ 绑定、不开 NVIC。`wait_update` 用周期边界标志做异步节拍
//! （等待时才开中断使能，IRQ 唤醒即压掉，防风暴）：
//!
//! ```ignore
//! // L012：GTIM1 更新中断（IRQ16）
//! embassy_cw32::bind_interrupts!(struct Irqs {
//!     GTIM1 => embassy_cw32::timer::Gtim1Irq;
//! });
//! // 经典组：同为 IRQ16，typelevel 中断名同 L012（GTIM1）
//! embassy_cw32::bind_interrupts!(struct Irqs {
//!     GTIM1 => embassy_cw32::timer::Gtim1Irq;
//! });
//! ```
//!
//! ⚠ `wait_update` 依赖对应定时器中断已 `bind_interrupts!` 绑定到本模块
//! 的标记类型；未绑定则 NVIC 线落到弱默认向量，首个周期事件即挂死——
//! 用 PWM 不用 `wait_update` 则完全无需绑定。

// ================= PWM 配置（两代共用） =================

/// PWM 通道
#[allow(missing_docs)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum PwmChannel {
    Ch1,
    Ch2,
    Ch3,
    Ch4,
}

/// PWM 配置（构造参数）
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct PwmConfig {
    /// 输出通道（须与引脚的 AF 复用通道一致）
    pub channel: PwmChannel,
    /// PWM 频率（Hz；上界 PCLK/2，下界受最大分频限制）
    pub freq_hz: u32,
    /// 初始占空比（万分比 0..=10000）
    pub duty_bp: u16,
}

/// PWM 配置错误（语义同阻塞版 gtim::PwmError）
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum PwmError {
    /// 频率超出所选定时器的可实现范围（上界 PCLK/2，下界受最大分频限制）
    OutOfRange,
    /// 占空比 > 10000（万分比）
    BadDuty,
}

impl embedded_hal::pwm::Error for PwmError {
    fn kind(&self) -> embedded_hal::pwm::ErrorKind {
        // embedded-hal 1.0 的 pwm::ErrorKind 目前只有 Other
        embedded_hal::pwm::ErrorKind::Other
    }
}

// ================= PSC/ARR 计算（两代共用） =================

/// 线性分频（L012 PSC / 经典组 l031/l052/l083/r031/w031 的 PSC 寄存器）：
/// 计算 PWM 的 (PSC, ARR)，PSC 取最小值使 ARR ≤ 0xFFFF。
///
/// 与阻塞版 `cw32l012-hal/src/gtim.rs::calc_pwm_psc_arr` 同源同实现
/// （含宿主机单元测试覆盖的量化策略）。
pub fn calc_pwm_psc_arr(pclk_hz: u32, freq_hz: u32) -> Option<(u16, u16)> {
    if freq_hz == 0 || pclk_hz == 0 {
        return None;
    }
    let mut psc = ((pclk_hz as u64 + 65_535 * freq_hz as u64 - 1) / (65_536 * freq_hz as u64))
        .saturating_sub(1);
    while psc <= 0xFFFF {
        let ck_cnt = pclk_hz / (psc as u32 + 1);
        let ticks = (ck_cnt as u64 + freq_hz as u64 / 2) / freq_hz as u64;
        if (2..=65_536).contains(&ticks) {
            return Some((psc as u16, (ticks - 1) as u16));
        }
        psc += 1;
    }
    None
}

/// 2^n 分频（经典组 f030/f020 的 CR0.PRS 变体）：计算 (分频, ARR)，
/// 分频取最小 2 的幂（1..=32768，PRS 4bit → 2^0..2^15）使 ARR ≤ 0xFFFF。
///
/// 量化四舍五入同 [`calc_pwm_psc_arr`]；`f > PCLK/2`（任何分频下
/// 周期 < 2 tick）或 `f < PCLK/32768/65536`（任何分频下周期爆表）返回 None。
pub fn calc_pwm_pow2_div(pclk_hz: u32, freq_hz: u32) -> Option<(u32, u16)> {
    if freq_hz == 0 || pclk_hz == 0 {
        return None;
    }
    for k in 0..=15u32 {
        let div = 1u32 << k;
        let ck_cnt = pclk_hz / div;
        let ticks = (ck_cnt as u64 + freq_hz as u64 / 2) / freq_hz as u64;
        if (2..=65_536).contains(&ticks) {
            return Some((div, (ticks - 1) as u16));
        }
    }
    None
}

// ================= L012 代组分支 =================
#[cfg(any(feature = "cw32l012", feature = "cw32l010", feature = "cw32l011"))]
mod l012gen {
    //! L012 代组（`cw32l012`/`cw32l010`/`cw32l011`，L012 手册 §16）：GTIM
    //! 的 `GtimPwm`——STM32 风格寄存器面（PSC/ARR/CCR/CCMR/CCER/IER/EGR，
    //! 影子寄存器预装载）。实例数随家族：L012 = GTIM1..4（GTIM3/4 共享
    //! IRQ18）；l011 = GTIM1/GTIM2（IRQ16/17 独立）；l010 = GTIM1 单实例。
    //!
    //! # 三家族差异表（一套代码，cfg + 薄适配收敛）
    //!
    //! | 差异点 | L012 | l010/l011 |
    //! |--------|------|-----------|
    //! | 中断使能寄存器 | PAC 字段名 `dier` | 生成名 `ier`（同址 0x0C） |
    //! | 输出比较 CCMR | PAC 字段名 `ccmr1`/`ccmr2` | 生成名 `ccmr1cap`/`ccmr2cap`（同址 0x18/0x1C，比较位面在 PAC `gtim::ccmr1cmp`/`ccmr2cmp`） |
    //! | ISR/ICR/EGR | isr RO、icr R1W0、egr WO | 同位序同语义（l010 的 isr PAC 类型标 RW、egr/icr 标 RW，读/写用法一致） |
    //! | 实例与门控 | GTIM1/2/3/4 = APBEN1 bit6/7/11/12 | GTIM1/2 = bit6/7（l011），l010 仅 GTIM1 |
    //! | NVIC 线 | GTIM1=16、GTIM2=17、GTIM3/4 共享 18 | l010/l011 GTIM1=16、l011 GTIM2=17，独立不共享 |
    //!
    //! 位面三家族逐位一致：cw32l012-pac-um 0.3.0 `gtim` 模块整字常量 ↔
    //! cw32l011-pac 0.1.2 / cw32l010-pac-um 0.1.2 的小写字段模块
    //! （`gtim::{cr1,egr,isr,icr,ccer,ccmr1cmp,ccmr2cmp}`）逐位比对一致
    //! （同一套 IP；门控位序另有 l011 SDK 头 `cw32l011.h` 的
    //! `SYSCTRL_APBEN1_GTIM1/2_Pos`=6/7 双源背书）。位常量按 L012 PAC
    //! 同名风格本地定义（见下），l010/l011 对应字段模块在注释注明。
    //!
    //! 寄存器配方与阻塞版 HAL **同源**（`cw32l012-hal/src/gtim.rs` 的
    //! `start_pwm`/`set_duty`，实机点灯验证见其 `examples/gtim_pwm.rs`）：
    //! 边沿对齐 PWM 模式 1（CNT<CCR 输出有效电平，高有效）、PSC/ARR 十六位
    //! 线性分频、影子寄存器预装载。⚠ l010/l011 分支**无阻塞版同源背书**
    //! （阻塞版未接这两家族），寄存器位面按三家族 PAC 逐字段比对 +
    //! hal2 证据链（`docs/buffered-pwm-route-evidence.json` 的引脚路线/
    //! 手册页码）核对，compiles-not-run。第一版不做 ATIM 互补/死区（见
    //! [`crate::atim`]）。
    //!
    //! # 中断模型（IER 单发纪律）
    //!
    //! [`GtimPwm::wait_update`] 用更新中断 UIF 做异步周期节拍（等待时才开
    //! IER.UIE，IRQ 唤醒即压掉，防风暴）。GTIMx_ICR 三家族均建模进 PAC
    //! （@ 0x70，R1W0）；UIE 位常量按手册 §16.10.4 本地补。
    //!
    //! # 引脚（须配为对应 AF 复用输出；SDK 宏 + 数据手册表 5-3/5-4 双源）
    //!
    //! L012（手册表 9-2，阻塞版例程实机验证）：
    //!
    //! | 通道 | 常用引脚                          |
    //! |------|-----------------------------------|
    //! | GTIM1_CH1 | PB04（AF6）、PA00（AF10）等  |
    //! | GTIM1_CH3 | PA03（AF6）等                |
    //!
    //! l010（GTIM1 单实例，AF6；`PA1` 的 CH3 路线被振荡器占用排除）：
    //!
    //! | 通道 | 引脚（AF6）      |
    //! |------|------------------|
    //! | GTIM1_CH1 | PA6 / PB6   |
    //! | GTIM1_CH2 | PA5 / PB5   |
    //! | GTIM1_CH3 | PA4 / PB4   |
    //! | GTIM1_CH4 | PA3 / PB3   |
    //!
    //! l011（GTIM1 AF6 / GTIM2 AF5 或 AF6；`PB7` 的 CH4 路线被振荡器占用
    //! 排除——hal2 证据链 safety_excluded）：
    //!
    //! | 通道 | GTIM1（AF6）       | GTIM2                |
    //! |------|--------------------|----------------------|
    //! | CH1  | PA6 / PB4          | PA0（AF6）/ PA15（AF5） |
    //! | CH2  | PA7 / PB5          | PA1/PA11（AF6）、PB3（AF5） |
    //! | CH3  | PA5 / PB0 / PB6    | PA2/PA12（AF6）、PB6（AF5） |
    //! | CH4  | PB1                | PA3（AF6）           |
    //!
    //! 引脚/AF 必须与所选通道对应；完整映射见各家数据手册表 5-3/5-4
    //! （hal2 证据链 `buffered-pwm-route-evidence.json` 双源核对）。

    use core::future::poll_fn;
    use core::sync::atomic::Ordering;
    use core::task::Poll;

    use embassy_sync::waitqueue::AtomicWaker;

    use super::{PwmChannel, PwmConfig, PwmError, calc_pwm_psc_arr};
    use crate::gpio::{GpioPort, Pin, Reg};
    use crate::interrupt::{IrqHandler, interrupt};
    use crate::pac;

    // ================= 位常量面 =================
    // 三家族一套整字常量：按 L012 PAC 同名风格本地定义（原 `use
    // crate::pac::gtim as gt` 只对 L012 成立），位序经三家族 PAC 逐位比对
    // 一致（见模块头注差异表），注释注明 l010/l011 生成 PAC 的对应字段。

    /// 计数使能（CR1 bit0；l010/l011 PAC `gtim::cr1::cen`）
    const CR1_EN: u32 = 1 << 0;
    /// ARR 预装载（CR1 bit7；`cr1::arpe`）
    const CR1_ARPE: u32 = 1 << 7;
    /// 更新事件产生（EGR bit0；`egr::ug`）
    const EGR_UG: u32 = 1 << 0;
    /// 更新标志（ISR bit0；`isr::uif`）
    const ISR_UIF: u32 = 1 << 0;
    /// ICR 更新标志位（ICR bit0；`icr::uif`——R1W0，写 0 清除）
    const ICR_UIF: u32 = 1 << 0;
    /// 更新中断使能（IER bit0；`ier::uie`——L012 PAC 字段名 dier）
    const IER_UIE: u32 = 1 << 0;
    /// OC1M 低位段掩码（CCMR1 [6:4]；`ccmr1cmp::oc1m`）
    const OC1M_MASK: u32 = 0x7 << 4;
    /// OC1 = PWM 模式 1（CNT<CCR 输出有效电平）
    const OC1M_PWM1: u32 = 6 << 4;
    /// OC1 比较预装载使能（`ccmr1cmp::oc1pe`）
    const OC1PE: u32 = 1 << 3;
    /// OC2M 低位段掩码（CCMR1 [14:12]；`ccmr1cmp::oc2m`）
    const OC2M_MASK: u32 = 0x7 << 12;
    /// OC2 = PWM 模式 1
    const OC2M_PWM1: u32 = 6 << 12;
    /// OC2 比较预装载使能（`ccmr1cmp::oc2pe`）
    const OC2PE: u32 = 1 << 11;
    /// OC3M 低位段掩码（CCMR2 [6:4]，布局同 CCMR1 的 OC1M；`ccmr2cmp::oc3m`）
    const OC3M_MASK: u32 = 0x7 << 4;
    /// OC3 = PWM 模式 1
    const OC3M_PWM1: u32 = 6 << 4;
    /// OC3 比较预装载使能（`ccmr2cmp::oc3pe`）
    const OC3PE: u32 = 1 << 3;
    /// OC4M 低位段掩码（CCMR2 [14:12]；`ccmr2cmp::oc4m`）
    const OC4M_MASK: u32 = 0x7 << 12;
    /// OC4 = PWM 模式 1
    const OC4M_PWM1: u32 = 6 << 12;
    /// OC4 比较预装载使能（`ccmr2cmp::oc4pe`）
    const OC4PE: u32 = 1 << 11;
    /// CC1 输出使能（CCER bit0；`ccer::cc1e`）
    const CC1E: u32 = 1 << 0;
    /// CC1 极性（CCER bit1；`ccer::cc1p`）
    const CC1P: u32 = 1 << 1;
    /// CC2 输出使能（CCER bit4；`ccer::cc2e`）
    const CC2E: u32 = 1 << 4;
    /// CC2 极性（CCER bit5；`ccer::cc2p`）
    const CC2P: u32 = 1 << 5;
    /// CC3 输出使能（CCER bit8；`ccer::cc3e`）
    const CC3E: u32 = 1 << 8;
    /// CC3 极性（CCER bit9；`ccer::cc3p`）
    const CC3P: u32 = 1 << 9;
    /// CC4 输出使能（CCER bit12；`ccer::cc4e`）
    const CC4E: u32 = 1 << 12;
    /// CC4 极性（CCER bit13；`ccer::cc4p`）
    const CC4P: u32 = 1 << 13;

    // ---- APBEN1 门控位（KEY 内嵌本寄存器 [31:16]，与 crate::SYSCTRL_KEY
    // 同协议；位序 l011 SDK 头 cw32l011.h:6022-6026 实锤）----
    /// GTIM1 = bit6（三家族同位）
    const APBEN1_GTIM1: u32 = 1 << 6;
    /// GTIM2 = bit7（L012/l011；l010 无 GTIM2 实例）
    #[cfg(any(feature = "cw32l012", feature = "cw32l011"))]
    const APBEN1_GTIM2: u32 = 1 << 7;
    /// GTIM3 = bit11（仅 L012；PAC `apben1::APBEN1_GTIM3`）
    #[cfg(feature = "cw32l012")]
    const APBEN1_GTIM3: u32 = 1 << 11;
    /// GTIM4 = bit12（仅 L012）
    #[cfg(feature = "cw32l012")]
    const APBEN1_GTIM4: u32 = 1 << 12;

    // ================= 家族差异适配 =================

    /// 寄存器块类型：L012 手写 PAC 名 `Gtim`，l010/l011 生成 PAC 名 `GTIM`
    #[cfg(feature = "cw32l012")]
    type GtimRegs = pac::Gtim;
    #[cfg(any(feature = "cw32l010", feature = "cw32l011"))]
    type GtimRegs = pac::GTIM;

    /// 中断使能寄存器（L012 PAC 字段名 `dier`，l010/l011 生成名 `ier`；
    /// 同址 0x0C）
    #[inline]
    fn ien(regs: &GtimRegs) -> &pac::Rw<u32> {
        #[cfg(feature = "cw32l012")]
        {
            &regs.dier
        }
        #[cfg(any(feature = "cw32l010", feature = "cw32l011"))]
        {
            &regs.ier
        }
    }

    /// 输出比较 CCMR1（L012 PAC 字段名 `ccmr1`，l010/l011 生成名
    /// `ccmr1cap`——同址 0x18，比较位面见 PAC `gtim::ccmr1cmp`）
    #[inline]
    fn ccmr1(regs: &GtimRegs) -> &pac::Rw<u32> {
        #[cfg(feature = "cw32l012")]
        {
            &regs.ccmr1
        }
        #[cfg(any(feature = "cw32l010", feature = "cw32l011"))]
        {
            &regs.ccmr1cap
        }
    }

    /// 输出比较 CCMR2（同上，0x1C；`ccmr2cap`，位面 `gtim::ccmr2cmp`）
    #[inline]
    fn ccmr2(regs: &GtimRegs) -> &pac::Rw<u32> {
        #[cfg(feature = "cw32l012")]
        {
            &regs.ccmr2
        }
        #[cfg(any(feature = "cw32l010", feature = "cw32l011"))]
        {
            &regs.ccmr2cap
        }
    }

    // ================= 实例识别与 waker =================

    /// 实例索引（0..3 = GTIM1..4，按基地址判定；l010 只有 0、l011 只有 0/1）
    fn instance_of(regs: &GtimRegs) -> usize {
        match regs as *const _ as usize {
            pac::GTIM1_BASE => 0,
            #[cfg(any(feature = "cw32l012", feature = "cw32l011"))]
            pac::GTIM2_BASE => 1,
            #[cfg(feature = "cw32l012")]
            pac::GTIM3_BASE => 2,
            // L012 的 GTIM4_BASE；l010/l011 无此实例、该档不可达
            _ => 3,
        }
    }

    /// per-实例 waker（0..3 = GTIM1..4；wait_update 用）
    static GTIM_WAKER: [AtomicWaker; 4] = [const { AtomicWaker::new() }; 4];

    // ================= 驱动本体 =================

    /// GTIM 单通道 PWM（边沿对齐模式 1，高有效）
    ///
    /// 构造即按阻塞版配方完成门控/引脚复用/PSC/ARR/CCR/输出使能并启动计数；
    /// 运行中改占空比经 [`SetDutyCycle`](embedded_hal::pwm::SetDutyCycle)
    /// 或 [`GtimPwm::set_duty_bp`]（OCyPE 预装载 → 下一更新周期生效）。
    pub struct GtimPwm {
        regs: &'static GtimRegs,
        idx: usize,
        ch: PwmChannel,
    }

    impl GtimPwm {
        /// 初始化并启动一路 PWM（阻塞版 `Gtim::start_pwm` 配方）
        ///
        /// * `regs` — GTIM 寄存器块（`p.gtim1`/`p.gtim2` 等；实例数随家族
        ///   L012=4、l011=2、l010=1）
        /// * `pin` — PWM 输出引脚（内部配为 AF 复用输出：清模拟/开漏/上下拉，
        ///   DIR=0 输出，`set_af(af)`——同阻塞版 `into_af_output`）
        /// * `af` — 复用功能编号（各家数据手册 AF 表，如 L012 的
        ///   GTIM1_CH1@PB04 = AF6、l010 的 GTIM1_CH1@PB6 = AF6）
        /// * `config` — 通道/频率/初始占空比
        pub fn new(
            regs: &'static GtimRegs,
            pin: Pin<impl GpioPort>,
            af: u8,
            config: PwmConfig,
        ) -> Result<Self, PwmError> {
            if config.duty_bp > 10_000 {
                return Err(PwmError::BadDuty);
            }
            let pclk_hz = crate::PCLK_HZ.load(Ordering::Relaxed);
            let (psc, arr) =
                calc_pwm_psc_arr(pclk_hz, config.freq_hz).ok_or(PwmError::OutOfRange)?;
            // CCR = (ARR+1) × 万分比 / 10000（100% → CCR=ARR+1，CNT 永不大于 CCR → 恒有效）
            let ccr = ((arr as u32 + 1) * config.duty_bp as u32 / 10_000) as u16;

            // APBEN1.GTIMx 时钟（KEY 内嵌本寄存器 [31:16]；读改写 + KEY
            // 保留其它外设的使能位——与 ATIM/UART 门控同寄存器共存，
            // 整字写会互相顶掉）。
            let sysctrl = unsafe { &*(pac::SYSCTRL_BASE as *const crate::Sysctrl) };
            let idx = instance_of(regs);
            let gate = match idx {
                0 => APBEN1_GTIM1,
                #[cfg(any(feature = "cw32l012", feature = "cw32l011"))]
                1 => APBEN1_GTIM2,
                // GTIM3/4 仅 L012（l010/l011 的 instance_of 不会给出该档）
                #[cfg(feature = "cw32l012")]
                2 => APBEN1_GTIM3,
                #[cfg(feature = "cw32l012")]
                _ => APBEN1_GTIM4,
                #[cfg(not(feature = "cw32l012"))]
                _ => unreachable!("l010/l011 无 GTIM3/4 实例"),
            };
            sysctrl
                .apben1
                .modify(|v| crate::SYSCTRL_KEY | (v & 0xFFFF) | gate);

            // 引脚复用（阻塞版 into_af_output 配方：清附加态 → AF → 输出方向；
            // l010/l011 的 GPIO 无 PDR 下拉寄存器（gpio.rs impl_port_no_pdr!），
            // 清下拉那步仅 L012 需要）
            let m = pin.mask();
            pin.port().mod_(Reg::OpEndrain, |v| v & !m);
            pin.port().mod_(Reg::Pur, |v| v & !m);
            #[cfg(feature = "cw32l012")]
            pin.port().mod_(Reg::Pdr, |v| v & !m);
            pin.port().mod_(Reg::Analog, |v| v & !m); // 数字模式
            pin.set_af(af);
            pin.port().mod_(Reg::Dir, |v| v & !m); // 0 = 输出

            let pwm = GtimPwm {
                regs,
                idx,
                ch: config.channel,
            };
            pwm.start_pwm(psc, arr, ccr);
            Ok(pwm)
        }

        /// PWM 寄存器配方（照搬阻塞版 gtim.rs `start_pwm`）
        fn start_pwm(&self, psc: u16, arr: u16, ccr: u16) {
            let regs = self.regs;
            regs.cr1.write(0); // 停止计数以便配置
            regs.psc.write(psc as u32);
            regs.arr.write(arr as u32);
            self.set_ccr(ccr);

            // 通道输出比较模式：PWM1 + 预装载（CCxS[1:0]=00 保持输出态）；
            // CCER 使能输出（CCxP=0 高有效）
            match self.ch {
                PwmChannel::Ch1 => {
                    ccmr1(regs).modify(|v| (v & !(OC1M_MASK | OC1PE | 0x3)) | OC1M_PWM1 | OC1PE);
                    regs.ccer.modify(|v| (v & !CC1P) | CC1E);
                }
                PwmChannel::Ch2 => {
                    ccmr1(regs)
                        .modify(|v| (v & !(OC2M_MASK | OC2PE | (0x3 << 8))) | OC2M_PWM1 | OC2PE);
                    regs.ccer.modify(|v| (v & !CC2P) | CC2E);
                }
                PwmChannel::Ch3 => {
                    ccmr2(regs).modify(|v| (v & !(OC3M_MASK | OC3PE | 0x3)) | OC3M_PWM1 | OC3PE);
                    regs.ccer.modify(|v| (v & !CC3P) | CC3E);
                }
                PwmChannel::Ch4 => {
                    ccmr2(regs)
                        .modify(|v| (v & !(OC4M_MASK | OC4PE | (0x3 << 8))) | OC4M_PWM1 | OC4PE);
                    regs.ccer.modify(|v| (v & !CC4P) | CC4E);
                }
            }

            // UG 重装影子寄存器 → ARPE + EN 启动（§16.3.1.4）
            regs.egr.write(EGR_UG);
            regs.cr1.write(CR1_ARPE | CR1_EN);
        }

        /// 直接写 CCR（原始占空比；OCyPE 预装载 → 下一更新周期生效）
        pub fn set_ccr(&self, ccr: u16) {
            let regs = self.regs;
            match self.ch {
                PwmChannel::Ch1 => regs.ccr1.write(ccr as u32),
                PwmChannel::Ch2 => regs.ccr2.write(ccr as u32),
                PwmChannel::Ch3 => regs.ccr3.write(ccr as u32),
                PwmChannel::Ch4 => regs.ccr4.write(ccr as u32),
            }
        }

        /// 满量程（万分比；`SetDutyCycle` 语义下 100% = 10000）
        ///
        /// 固定万分比满量程同阻塞版 PwmPin——ARR+1 最大 65536 放不进
        /// `SetDutyCycle::max_duty_cycle` 的 u16 返回值。
        pub fn max_duty(&self) -> u16 {
            10_000
        }

        /// 运行中修改占空比（万分比 0..=10000；ARPE 已开 → 下一更新周期生效）
        pub fn set_duty_bp(&mut self, duty_bp: u16) -> Result<(), PwmError> {
            if duty_bp > 10_000 {
                return Err(PwmError::BadDuty);
            }
            let arr = self.regs.arr.read() & 0xFFFF;
            let ccr = ((arr + 1) * duty_bp as u32 / 10_000) as u16;
            self.set_ccr(ccr);
            Ok(())
        }

        /// 读回当前占空比（万分比，由 CCR/ARR 反算）
        pub fn duty_bp(&self) -> u16 {
            let arr = self.regs.arr.read() & 0xFFFF;
            let ccr = match self.ch {
                PwmChannel::Ch1 => self.regs.ccr1.read(),
                PwmChannel::Ch2 => self.regs.ccr2.read(),
                PwmChannel::Ch3 => self.regs.ccr3.read(),
                PwmChannel::Ch4 => self.regs.ccr4.read(),
            } & 0xFFFF;
            ((ccr * 10_000) / (arr + 1)) as u16
        }

        /// 当前计数值（观测 CNT 在 0..ARR 间滚动 = PWM 在跑）
        pub fn count(&self) -> u16 {
            self.regs.cnt.read() as u16
        }

        /// 读 ARR/PSC 配置（回读校验用）
        pub fn period_config(&self) -> (u16, u16) {
            (
                (self.regs.arr.read() & 0xFFFF) as u16,
                (self.regs.psc.read() & 0xFFFF) as u16,
            )
        }

        /// 停止计数（保留 PSC/ARR/CCR 配置）
        pub fn stop(&mut self) {
            self.regs.cr1.modify(|v| v & !CR1_EN);
        }

        /// 恢复计数（保留已配置的 PSC/ARR/CCR）
        pub fn start(&mut self) {
            self.regs.cr1.modify(|v| v | CR1_EN);
        }

        /// 异步等待一次更新事件（PWM 周期节拍；IER 使能 + waker 模式）
        ///
        /// 每次调用先清掉残留 UIF（`start` 配方的 EGR_UG 在 URS=0 时会置
        /// UIF，§16.10.5），再等**下一个**真实更新事件——返回时刻即一个
        /// PWM 周期边界。等待期间才开 IER.UIE，IRQ 唤醒即压掉（防风暴）。
        ///
        /// 前置条件（模块文档详述）：对应 GTIM 中断已 `bind_interrupts!`
        /// 绑定到 [`Gtim1Irq`]/[`Gtim2Irq`]/[`Gtim34Irq`]（GTIM3/4 共享
        /// IRQ18 一条线，仅 L012；l010/l011 的 GTIM1/2 线独立）；NVIC 线
        /// 由本方法幂等使能。
        pub async fn wait_update(&mut self) {
            let regs = self.regs;
            let idx = self.idx;
            // NVIC 线使能（幂等；L012 的 GTIM3/4 共用 IRQ18，l010/l011 无
            // GTIM3/4 实例）
            match idx {
                0 => crate::interrupt::nvic_enable::<interrupt::GTIM1>(),
                #[cfg(any(feature = "cw32l012", feature = "cw32l011"))]
                1 => crate::interrupt::nvic_enable::<interrupt::GTIM2>(),
                #[cfg(feature = "cw32l012")]
                _ => crate::interrupt::nvic_enable::<interrupt::GTIM34>(),
                #[cfg(not(feature = "cw32l012"))]
                _ => unreachable!("l010/l011 无 GTIM3/4 实例"),
            }
            // 前置清残留 UIF：GTIMx_ICR（@ 0x70）为 R1W0——写 0 清除对应
            // 标志、写 1 无功能，写 `!ICR_UIF` 即仅 bit0=0 清更新标志、
            // 其余位写 1 无副作用
            regs.icr.write(!ICR_UIF);
            poll_fn(|cx| {
                GTIM_WAKER[idx].register(cx.waker());
                if regs.isr.read() & ISR_UIF != 0 {
                    ien(regs).modify(|v| v & !IER_UIE); // 就绪即压掉
                    Poll::Ready(())
                } else {
                    ien(regs).modify(|v| v | IER_UIE); // 等待期间才开
                    Poll::Pending
                }
            })
            .await;
        }
    }

    // ================= embedded-hal PWM trait =================

    impl embedded_hal::pwm::ErrorType for GtimPwm {
        type Error = PwmError;
    }

    impl embedded_hal::pwm::SetDutyCycle for GtimPwm {
        #[inline]
        fn max_duty_cycle(&self) -> u16 {
            GtimPwm::max_duty(self)
        }

        #[inline]
        fn set_duty_cycle(&mut self, duty: u16) -> Result<(), Self::Error> {
            // 满量程固定万分比 10000，超界夹到 100%（同阻塞版 ehal.rs 语义）
            GtimPwm::set_duty_bp(self, duty.min(10_000))
        }
    }

    // ================= 中断派发 =================

    /// GTIM 更新中断公共处理：单发压掉 IER.UIE 并唤醒等待者（ISR 的 UIF
    /// 标志不动——poll 复查后由下次 wait_update 的前置 ICR 清除）。
    fn on_irq(idx: usize, regs: &'static GtimRegs) {
        if regs.isr.read() & ISR_UIF != 0 {
            ien(regs).modify(|v| v & !IER_UIE);
            GTIM_WAKER[idx].wake();
        }
    }

    /// GTIM1 中断标记（IRQ16，bind_interrupts! 绑定用；三家族同号线）
    pub struct Gtim1Irq;
    impl IrqHandler<interrupt::GTIM1> for Gtim1Irq {
        fn call() {
            on_irq(0, unsafe { &*(pac::GTIM1_BASE as *const GtimRegs) });
        }
    }

    /// GTIM2 中断标记（IRQ17；L012/l011——l010 无 GTIM2 实例）
    #[cfg(any(feature = "cw32l012", feature = "cw32l011"))]
    pub struct Gtim2Irq;
    #[cfg(any(feature = "cw32l012", feature = "cw32l011"))]
    impl IrqHandler<interrupt::GTIM2> for Gtim2Irq {
        fn call() {
            on_irq(1, unsafe { &*(pac::GTIM2_BASE as *const GtimRegs) });
        }
    }

    /// GTIM3/4 共享中断标记（IRQ18 `GTIM34`——仅 L012 有 GTIM3/4 实例；
    /// 两个实例各自查各自 UIF，未挂起者空跑无害；时钟门控未开的实例
    /// ISR 读回 0）
    #[cfg(feature = "cw32l012")]
    pub struct Gtim34Irq;
    #[cfg(feature = "cw32l012")]
    impl IrqHandler<interrupt::GTIM34> for Gtim34Irq {
        fn call() {
            on_irq(2, unsafe { &*(pac::GTIM3_BASE as *const GtimRegs) });
            on_irq(3, unsafe { &*(pac::GTIM4_BASE as *const GtimRegs) });
        }
    }

    #[cfg(test)]
    mod tests {
        use super::*;

        /// 本地整字常量 ↔ L012 手写 PAC 整字常量 / l010-l011 生成 PAC 的
        /// 小写字段模块，逐位比对（三家族一套位面的锚点测试；位常量为本
        /// 模块私有，测试挂在这里）
        #[test]
        fn gtim_bitplane_matches_family_pacs() {
            #[cfg(feature = "cw32l012")]
            {
                use crate::pac::gtim as gt;
                assert_eq!(CR1_EN, gt::CR1_EN);
                assert_eq!(CR1_ARPE, gt::CR1_ARPE);
                assert_eq!(EGR_UG, gt::EGR_UG);
                assert_eq!(ISR_UIF, gt::ISR_UIF);
                assert_eq!(ICR_UIF, gt::ICR_UIF);
                assert_eq!(OC1M_MASK, gt::OC1M_MASK);
                assert_eq!(OC1M_PWM1, gt::OC1M_PWM1);
                assert_eq!(OC1PE, gt::OC1PE);
                assert_eq!(OC3M_MASK, gt::OC3M_MASK);
                assert_eq!(OC3M_PWM1, gt::OC3M_PWM1);
                assert_eq!(OC3PE, gt::OC3PE);
                assert_eq!(OC4M_MASK, gt::OC4M_MASK);
                assert_eq!(OC4M_PWM1, gt::OC4M_PWM1);
                assert_eq!(OC4PE, gt::OC4PE);
                assert_eq!(CC1E, gt::CC1E);
                assert_eq!(CC3P, gt::CC3P);
                assert_eq!(CC4E, gt::CC4E);
            }
            #[cfg(feature = "cw32l010")]
            {
                use crate::pac::gtim as gt;
                assert_eq!(CR1_EN, 1 << gt::cr1::cen_POS);
                assert_eq!(CR1_ARPE, gt::cr1::arpe_MASK);
                assert_eq!(EGR_UG, gt::egr::ug_MASK);
                assert_eq!(ISR_UIF, gt::isr::uif_MASK);
                assert_eq!(ICR_UIF, gt::icr::uif_MASK);
                assert_eq!(OC1M_MASK, gt::ccmr1cmp::oc1m_MASK);
                assert_eq!(OC1M_PWM1, 6 << gt::ccmr1cmp::oc1m_POS);
                assert_eq!(OC1PE, gt::ccmr1cmp::oc1pe_MASK);
                assert_eq!(OC3M_MASK, gt::ccmr2cmp::oc3m_MASK);
                assert_eq!(OC3PE, gt::ccmr2cmp::oc3pe_MASK);
                assert_eq!(OC4M_MASK, gt::ccmr2cmp::oc4m_MASK);
                assert_eq!(OC4PE, gt::ccmr2cmp::oc4pe_MASK);
                assert_eq!(CC1E, gt::ccer::cc1e_MASK);
                assert_eq!(CC2P, gt::ccer::cc2p_MASK);
                assert_eq!(CC3E, gt::ccer::cc3e_MASK);
                assert_eq!(CC4P, gt::ccer::cc4p_MASK);
                // PSC/ARR 位面（l010/l011 有独立 PSC 寄存器，与 L012 同配方）
                assert_eq!(gt::psc::psc_MASK, 0xFFFF);
                assert_eq!(gt::arr::arr_MASK, 0xFFFF);
                assert_eq!(IER_UIE, gt::ier::uie_MASK);
            }
            #[cfg(feature = "cw32l011")]
            {
                use crate::pac::gtim as gt;
                assert_eq!(CR1_EN, 1 << gt::cr1::cen_POS);
                assert_eq!(CR1_ARPE, gt::cr1::arpe_MASK);
                assert_eq!(ISR_UIF, gt::isr::uif_MASK);
                assert_eq!(ICR_UIF, gt::icr::uif_MASK);
                assert_eq!(OC1M_MASK, gt::ccmr1cmp::oc1m_MASK);
                assert_eq!(OC1PE, gt::ccmr1cmp::oc1pe_MASK);
                assert_eq!(OC3M_MASK, gt::ccmr2cmp::oc3m_MASK);
                assert_eq!(OC4M_PWM1, 6 << gt::ccmr2cmp::oc4m_POS);
                assert_eq!(CC1E, gt::ccer::cc1e_MASK);
                assert_eq!(CC2P, gt::ccer::cc2p_MASK);
                assert_eq!(CC4P, gt::ccer::cc4p_MASK);
                assert_eq!(IER_UIE, gt::ier::uie_MASK);
            }
        }
    }
}

#[cfg(any(feature = "cw32l012", feature = "cw32l010", feature = "cw32l011"))]
pub use l012gen::*;

// ================= 经典家族分支 =================
#[cfg(any(
    feature = "cw32f030",
    feature = "cw32f020",
    feature = "cw32l031",
    feature = "cw32l052",
    feature = "cw32l083",
    feature = "cw32r031",
    feature = "cw32w031"
))]
mod classic {
    //! 经典家族（f030/f020/l031/l052/l083/r031/w031）：经典代 GTIM 的
    //! `ClassicPwm`——边沿对齐向上计数、单通道 PWM + 溢出（OV）节拍。
    //!
    //! # 寄存器面（与 L012 GTIM 无一处同构）
    //!
    //! 布局：`ARR@+0x300 / CNT / CMMR / ETR / CR0 / IER / ISR / ICR /
    //! CCR1..4 / CR1 / (PSC) / (DMA)`（l031/r031/w031 的 PAC 结构体以块
    //! 首址 GTIM1@0x400_0700 起排、ARR@+0x00——绝对地址与 f030 的
    //! 0x400_0400+0x300 相同，逐寄存器比对过位面一致）。
    //!
    //! - `CR0`：EN（bit0 计数使能）、MODE[2:1]=0（TIME，PCLK 驱动）、
    //!   ONESHOT/TOGEN/TRS/POL 不用（保持 0）。
    //! - **分频两子面**：f030/f020 无独立 PSC 寄存器，CR0.PRS[10:7] 编码
    //!   2^n（n=0..15，即 1..=32768；厂商库 `GTIM_PRESCALER_DIV1..DIV32768`）；
    //!   l031/l052/l083/r031/w031 有 PSC 寄存器（@+0x34/+0x334）线性
    //!   `divisor-1`（16bit，1..=65536）。厂商库与各家手册逐家核对：
    //!   `cw32f030_rcc.h`/`cw32f030.h` GTIMx_CR0_PRS_Msk(0x780)、
    //!   `cw32l031.h` SYSCTRL 之外 GTIMx_PSC_PSC_Msk(0xFFFF)。
    //! - `CMMR`：CCxM 每 4bit 一通道——8=强制低、9=强制高、
    //!   **15=CNT<CCR 输出高**（本驱动的 PWM 模式）、14=CNT≥CCR 输出高
    //!   （各家手册 CMMR 表 + 厂商库 `GTIM_OC_OUTPUT_*`）。10/11 是
    //!   匹配翻转（非 PWM），本驱动不用。
    //! - `IER/ISR/ICR`：OV（bit0，计数溢出=周期边界）、TI/UD/CC1..4、
    //!   DIRCHANGE（bit9）。ICR 为"写 0 清除"（R1W0，bit7/8 复位值为 1
    //!   须写 1 保全），写 `!OV` 清溢出标志其余无副作用。
    //! - **无 EGR/无影子寄存器**：ARR/CCR 写入立即生效（改占空比可能截断
    //!   当前脉冲）；分频在溢出或 EN 上升沿锁存——本驱动停计数后写分频再
    //!   置 EN=1，随启动沿生效。无 ARPE/OCxPE 预装载概念。
    //! - 0%/100% 端点用强制电平模式（8/9）而非 CCR=周期（CCR 只有 16bit，
    //!   ARR=0xFFFF 时周期 65536 装不下）。
    //!
    //! # 门控与中断（全家族无 KEY，读改写即可）
    //!
    //! GTIM1=APBEN1 bit1、GTIM2=APBEN1 bit2（7 家族同位）；GTIM3=APBEN2
    //! bit10（f030/f020/l052/l083）、GTIM4=APBEN2 bit11（f030/f020/l083）。
    //! NVIC 线独立不共享：Gtim1=IRQ16、Gtim2=17、Gtim3=18、Gtim4=19
    //! （l031/r031/w031 只有 GTIM1/2 两实例）。实例数：f030/f020=4、
    //! l052=3、l083=4、l031/r031/w031=2。
    //!
    //! # 选 ATIM 还是 GTIM
    //!
    //! 厂商库的 ATIM（`ATIM_PWMOutputConfig`/死区/互补）PWM 能力最全，但
    //! **f020 无 ATIM**（各家手册外设表；cw32f020 SDK 亦无 atim 文件），
    //! 且 ATIM CH4 不引出、CH1A-3A 才是主输出——做全家族统一 PWM 选 GTIM
    //! （7/7 有，且厂商库 `GTIM_OCInit` 即简单 PWM 官方配方）。ATIM 互补/
    //! 死区留待后续。
    //!
    //! # 用法（f030：GTIM1_CH1 @ PB04，AF6）
    //!
    //! ```ignore
    //! embassy_cw32::bind_interrupts!(struct Irqs {
    //!     GTIM1 => embassy_cw32::timer::Gtim1Irq;
    //! });
    //! let mut pwm = ClassicPwm::new(p.gtim1, Pin::new(p.gpiob, 4), 6, PwmConfig {
    //!     channel: PwmChannel::Ch1,
    //!     freq_hz: 1_000,
    //!     duty_bp: 1_000,
    //! })?;
    //! loop {
    //!     pwm.set_duty_bp(duty)?;
    //!     pwm.wait_update().await; // 下一 PWM 周期节拍（需绑定 Gtim1 中断）
    //! }
    //! ```

    use core::future::poll_fn;
    use core::sync::atomic::Ordering;
    use core::task::Poll;

    use embassy_sync::waitqueue::AtomicWaker;

    use super::{PwmChannel, PwmConfig, PwmError};
    // 分频子面按家族二选一：f030/f020 用 2^n 档计算，其余复用 L012 的
    // 线性分频算法
    #[cfg(any(feature = "cw32f030", feature = "cw32f020"))]
    use super::calc_pwm_pow2_div;
    #[cfg(not(any(feature = "cw32f030", feature = "cw32f020")))]
    use super::calc_pwm_psc_arr;
    use crate::gpio::{GpioPort, Pin, Reg};
    use crate::interrupt::{IrqHandler, interrupt};
    use crate::pac;

    // ================= 位常量面 =================
    // 经典代 GTIM 位序（f030 cw32f030.h GTIMx_* 位段 + l031 cw32l031.h
    // 同名位段逐位比对一致；PAC 无位常量故此处按手册补）。

    /// CR0.EN（bit0）：计数使能
    const CR0_EN: u32 = 1 << 0;
    /// CR0.PRS[10:7]：2^n 分频档（仅 f030/f020 变体使用）
    #[cfg(any(feature = "cw32f030", feature = "cw32f020"))]
    const CR0_PRS_SHIFT: u32 = 7;
    #[cfg(any(feature = "cw32f030", feature = "cw32f020"))]
    const CR0_PRS_MASK: u32 = 0xF << CR0_PRS_SHIFT;
    /// CMMR.CC1M 的 4bit 通道步进（CC1M@0 / CC2M@4 / CC3M@8 / CC4M@12）
    const CMMR_CC1M_SHIFT: u32 = 0;
    /// 强制输出低（0% 端点，高有效语义）
    const CCXM_FORCE_LOW: u32 = 8;
    /// 强制输出高（100% 端点）
    const CCXM_FORCE_HIGH: u32 = 9;
    /// PWM：CNT<CCR 输出高（边缘对齐向上计数的"PWM1"）
    const CCXM_PWM_BELOW: u32 = 15;
    /// IER.OV（bit0）：溢出中断使能
    const IER_OV: u32 = 1 << 0;
    /// ISR.OV（bit0）：溢出标志（只读语义；l031 组 PAC 类型标 RW 不影响读用）
    const ISR_OV: u32 = 1 << 0;
    /// ICR.OV（bit0）：写 0 清溢出标志（R1W0）
    const ICR_OV: u32 = 1 << 0;

    // ================= 实例识别与 waker =================

    /// 实例索引（0..3 = GTIM1..4，按基地址判定；l031 组只有 0/1 两档）
    fn instance_of(regs: &pac::GTIM) -> usize {
        match regs as *const _ as usize {
            pac::GTIM1_BASE => 0,
            pac::GTIM2_BASE => 1,
            // l031/r031/w031 的 PAC 无 GTIM3/4（块内常量不存在），cfg 掉
            #[cfg(any(
                feature = "cw32f030",
                feature = "cw32f020",
                feature = "cw32l052",
                feature = "cw32l083"
            ))]
            pac::GTIM3_BASE => 2,
            #[cfg(any(feature = "cw32f030", feature = "cw32f020", feature = "cw32l083"))]
            pac::GTIM4_BASE => 3,
            _ => 3,
        }
    }

    /// per-实例 waker（0..3 = GTIM1..4；wait_update 用。经典组四条 NVIC
    /// 线独立，无共享向量派发）
    static CLASSIC_WAKER: [AtomicWaker; 4] = [const { AtomicWaker::new() }; 4];

    // ================= 分频子面适配 =================

    /// 写分频（f030/f020 → CR0.PRS；其余 → PSC 寄存器）。
    /// 经典组分频在溢出或 EN 上升沿锁存：构造路径停计数后写入、随启动沿生效。
    fn write_prescaler(regs: &pac::GTIM, divisor: u32) {
        #[cfg(any(feature = "cw32f030", feature = "cw32f020"))]
        {
            // PRS = log2(divisor)（divisor ∈ {2^0..2^15}）
            let prs = divisor.trailing_zeros();
            regs.cr0
                .modify(|v| (v & !CR0_PRS_MASK) | (prs << CR0_PRS_SHIFT));
        }
        #[cfg(not(any(feature = "cw32f030", feature = "cw32f020")))]
        {
            regs.psc.write(divisor - 1); // 线性：PSC = divisor-1（16bit）
        }
    }

    /// (分频, ARR) 计算：按家族分频子面路由
    fn calc_div_arr(pclk_hz: u32, freq_hz: u32) -> Option<(u32, u16)> {
        #[cfg(any(feature = "cw32f030", feature = "cw32f020"))]
        {
            calc_pwm_pow2_div(pclk_hz, freq_hz)
        }
        #[cfg(not(any(feature = "cw32f030", feature = "cw32f020")))]
        {
            // l031 组/l052/l083 的 PSC 为线性 divisor-1，与 L012 的 PSC
            // 同为 1..=65536 全量线性档——直接复用同源算法
            calc_pwm_psc_arr(pclk_hz, freq_hz).map(|(psc, arr)| (psc as u32 + 1, arr))
        }
    }

    // ================= 驱动本体 =================

    /// 经典代 GTIM 单通道 PWM（边沿对齐向上计数，高有效）
    ///
    /// 构造即完成门控/引脚复用/分频/ARR/CCR/输出模式并启动计数；运行中改
    /// 占空比经 [`SetDutyCycle`](embedded_hal::pwm::SetDutyCycle) 或
    /// [`ClassicPwm::set_duty_bp`]——**经典 GTIM 无预装载，CCR 立即生效**
    /// （与 L012 的"下一更新周期生效"不同），可能截断当前脉冲。
    pub struct ClassicPwm {
        regs: &'static pac::GTIM,
        idx: usize,
        ch: PwmChannel,
    }

    impl ClassicPwm {
        /// 初始化并启动一路 PWM（厂商库 `GTIM_TimeBaseInit`+`GTIM_OCInit`
        /// +`GTIM_Cmd` 配方）
        ///
        /// * `regs` — GTIM 寄存器块（`p.gtim1` 等；实例数随家族 2..4 个）
        /// * `pin` — PWM 输出引脚（内部配为 AF 复用输出：清模拟/开漏/上下拉，
        ///   DIR=0 输出，`set_af(af)`）
        /// * `af` — 复用功能编号（各家数据手册 AF 表，如 f030 的
        ///   GTIM1_CH1@PB04 = AF6，厂商库宏 `PB04_AFx_GTIM1CH1`）
        /// * `config` — 通道/频率/初始占空比
        pub fn new(
            regs: &'static pac::GTIM,
            pin: Pin<impl GpioPort>,
            af: u8,
            config: PwmConfig,
        ) -> Result<Self, PwmError> {
            if config.duty_bp > 10_000 {
                return Err(PwmError::BadDuty);
            }
            let pclk_hz = crate::PCLK_HZ.load(Ordering::Relaxed);
            let (divisor, arr) =
                calc_div_arr(pclk_hz, config.freq_hz).ok_or(PwmError::OutOfRange)?;

            // 时钟门控：经典组 AHBEN/APBEN 无 KEY（见 lib.rs 说明），读改写。
            // GTIM1=APBEN1 bit1、GTIM2=APBEN1 bit2（7 家族同位，厂商库
            // `__RCC_GTIM1_CLK_ENABLE` 等宏）。
            let sysctrl = unsafe { &*(pac::SYSCTRL_BASE as *const crate::Sysctrl) };
            let idx = instance_of(regs);
            if idx <= 1 {
                sysctrl.apben1.modify(|v| v | (1 << (1 + idx as u32)));
            }
            // GTIM3=APBEN2 bit10（f030/f020/l052/l083）
            #[cfg(any(
                feature = "cw32f030",
                feature = "cw32f020",
                feature = "cw32l052",
                feature = "cw32l083"
            ))]
            if idx == 2 {
                sysctrl.apben2.modify(|v| v | (1 << 10));
            }
            // GTIM4=APBEN2 bit11（f030/f020/l083）
            #[cfg(any(feature = "cw32f030", feature = "cw32f020", feature = "cw32l083"))]
            if idx == 3 {
                sysctrl.apben2.modify(|v| v | (1 << 11));
            }

            // 引脚复用（同 GtimPwm 的 into_af_output 配方）
            let m = pin.mask();
            pin.port().mod_(Reg::OpEndrain, |v| v & !m);
            pin.port().mod_(Reg::Pur, |v| v & !m);
            pin.port().mod_(Reg::Pdr, |v| v & !m);
            pin.port().mod_(Reg::Analog, |v| v & !m); // 数字模式
            pin.set_af(af);
            pin.port().mod_(Reg::Dir, |v| v & !m); // 0 = 输出

            let pwm = ClassicPwm {
                regs,
                idx,
                ch: config.channel,
            };
            pwm.start_pwm(divisor, arr, config.duty_bp);
            Ok(pwm)
        }

        /// PWM 寄存器配方（厂商库 GTIM_TimeBaseInit/GTIM_OCInit/GTIM_Cmd）
        ///
        /// 停计数 → 写 ARR/CNT/分频/占空比 → EN=1（分频随 EN 上升沿锁存）。
        fn start_pwm(&self, divisor: u32, arr: u16, duty_bp: u16) {
            let regs = self.regs;
            // EN=0 + MODE=TIME(00) + ONESHOT/TOGEN=0（PRS 变体连分频档一起清，
            // write_prescaler 随后重写）
            regs.cr0.write(0);
            regs.arr.write(arr as u32);
            regs.cnt.write(0);
            write_prescaler(regs, divisor);
            // 通道输出比较：终态模式（0%/100% 强制电平、中间值 CNT<CCR 高）
            self.apply_duty(duty_bp);
            // 启动（EN 上升沿锁存分频；连续计数）
            regs.cr0.modify(|v| v | CR0_EN);
        }

        /// CCxM 模式位写入（仅本通道的 4bit，其余通道保持）
        fn set_ccxm(&self, mode: u32) {
            let shift = CMMR_CC1M_SHIFT + 4 * channel_index(self.ch);
            self.regs
                .cmmr
                .modify(|v| (v & !(0xF << shift)) | (mode << shift));
        }

        /// 写本通道 CCR（原始比较值；经典 GTIM 立即生效，无预装载）
        pub fn set_ccr(&self, ccr: u16) {
            let regs = self.regs;
            match self.ch {
                PwmChannel::Ch1 => regs.ccr1.write(ccr as u32),
                PwmChannel::Ch2 => regs.ccr2.write(ccr as u32),
                PwmChannel::Ch3 => regs.ccr3.write(ccr as u32),
                PwmChannel::Ch4 => regs.ccr4.write(ccr as u32),
            }
        }

        /// 读本通道 CCR
        fn ccr(&self) -> u32 {
            let regs = self.regs;
            (match self.ch {
                PwmChannel::Ch1 => regs.ccr1.read(),
                PwmChannel::Ch2 => regs.ccr2.read(),
                PwmChannel::Ch3 => regs.ccr3.read(),
                PwmChannel::Ch4 => regs.ccr4.read(),
            }) & 0xFFFF
        }

        /// 占空比 → CCR + 终态 CCxM（0%/100% 走强制电平：CCR 只有 16bit，
        /// ARR=0xFFFF 时周期 65536 无法用 CCR 表达满占空）
        fn apply_duty(&self, duty_bp: u16) {
            let arr = self.regs.arr.read() & 0xFFFF;
            let period = arr + 1;
            let ticks = period * duty_bp as u32 / 10_000; // 0..=period
            let mode = if duty_bp == 0 {
                CCXM_FORCE_LOW
            } else if duty_bp >= 10_000 {
                CCXM_FORCE_HIGH
            } else {
                CCXM_PWM_BELOW
            };
            // CCR 先行（强制态下改写无副作用），模式随后生效
            self.set_ccr(ticks.min(0xFFFF) as u16);
            self.set_ccxm(mode);
        }

        /// 满量程（万分比；`SetDutyCycle` 语义下 100% = 10000）
        ///
        /// 固定万分比满量程同 [`super::l012gen::GtimPwm`]——周期最大 65536
        /// 放不进 `SetDutyCycle::max_duty_cycle` 的 u16 返回值。
        pub fn max_duty(&self) -> u16 {
            10_000
        }

        /// 运行中修改占空比（万分比 0..=10000；**立即生效**——经典 GTIM
        /// 无预装载，可能截断当前脉冲，与 L012 语义不同）
        pub fn set_duty_bp(&mut self, duty_bp: u16) -> Result<(), PwmError> {
            if duty_bp > 10_000 {
                return Err(PwmError::BadDuty);
            }
            self.apply_duty(duty_bp);
            Ok(())
        }

        /// 读回当前占空比（万分比；端点强制模式下直接返回 0/10000）
        pub fn duty_bp(&self) -> u16 {
            let shift = CMMR_CC1M_SHIFT + 4 * channel_index(self.ch);
            let mode = (self.regs.cmmr.read() >> shift) & 0xF;
            if mode == CCXM_FORCE_LOW {
                return 0;
            }
            if mode == CCXM_FORCE_HIGH {
                return 10_000;
            }
            let arr = self.regs.arr.read() & 0xFFFF;
            ((self.ccr() * 10_000) / (arr + 1)) as u16
        }

        /// 当前计数值（观测 CNT 在 0..ARR 间滚动 = PWM 在跑）
        pub fn count(&self) -> u16 {
            self.regs.cnt.read() as u16
        }

        /// 读 (ARR, 分频档原始值)（回读校验用）
        ///
        /// 分频档：f030/f020 = CR0.PRS 场值（log2 分频，0..15）；
        /// l031 组/l052/l083 = PSC 寄存器值（分频-1，0..65535）。
        pub fn period_config(&self) -> (u16, u16) {
            #[cfg(any(feature = "cw32f030", feature = "cw32f020"))]
            let div_raw = ((self.regs.cr0.read() & CR0_PRS_MASK) >> CR0_PRS_SHIFT) as u16;
            #[cfg(not(any(feature = "cw32f030", feature = "cw32f020")))]
            let div_raw = (self.regs.psc.read() & 0xFFFF) as u16;
            ((self.regs.arr.read() & 0xFFFF) as u16, div_raw)
        }

        /// 停止计数（保留分频/ARR/CCR 配置）
        pub fn stop(&mut self) {
            self.regs.cr0.modify(|v| v & !CR0_EN);
        }

        /// 恢复计数（EN 上升沿锁存分频档）
        pub fn start(&mut self) {
            self.regs.cr0.modify(|v| v | CR0_EN);
        }

        /// 异步等待一次计数溢出（PWM 周期节拍；IER 单发纪律）
        ///
        /// 每次调用先清掉残留 OV 标志（ICR 写 0 清除，R1W0），再等**下一个**
        /// 真实溢出事件——返回时刻即一个 PWM 周期边界。等待期间才开
        /// IER.OV，IRQ 唤醒即压掉（防风暴）。
        ///
        /// 前置条件（模块文档详述）：对应 GTIM 中断已 `bind_interrupts!`
        /// 绑定到 [`Gtim1Irq`]/[`Gtim2Irq`]/[`Gtim3Irq`]/[`Gtim4Irq`]；
        /// 经典组四条 NVIC 线独立（16/17/18/19），由本方法幂等使能。
        pub async fn wait_update(&mut self) {
            let regs = self.regs;
            let idx = self.idx;
            // NVIC 线使能（幂等；四实例独立线，l031 组只有 0/1 两档）
            if idx == 0 {
                crate::interrupt::nvic_enable::<interrupt::GTIM1>();
            }
            if idx == 1 {
                crate::interrupt::nvic_enable::<interrupt::GTIM2>();
            }
            #[cfg(any(
                feature = "cw32f030",
                feature = "cw32f020",
                feature = "cw32l052",
                feature = "cw32l083"
            ))]
            if idx == 2 {
                crate::interrupt::nvic_enable::<interrupt::GTIM3>();
            }
            #[cfg(any(feature = "cw32f030", feature = "cw32f020", feature = "cw32l083"))]
            if idx == 3 {
                crate::interrupt::nvic_enable::<interrupt::GTIM4>();
            }
            // 前置清残留 OV：ICR 为 R1W0（写 0 清除、写 1 无功能、bit7/8
            // 复位值 1 须保全）——写 `!ICR_OV` 仅 bit0=0 清溢出、其余位写 1
            // 无副作用（同 L012 分支约定）
            regs.icr.write(!ICR_OV);
            poll_fn(|cx| {
                CLASSIC_WAKER[idx].register(cx.waker());
                if regs.isr.read() & ISR_OV != 0 {
                    regs.ier.modify(|v| v & !IER_OV); // 就绪即压掉
                    Poll::Ready(())
                } else {
                    regs.ier.modify(|v| v | IER_OV); // 等待期间才开
                    Poll::Pending
                }
            })
            .await;
        }
    }

    /// 通道 → CMMR 4bit 步进序（Ch1=0 .. Ch4=3）
    fn channel_index(ch: PwmChannel) -> u32 {
        match ch {
            PwmChannel::Ch1 => 0,
            PwmChannel::Ch2 => 1,
            PwmChannel::Ch3 => 2,
            PwmChannel::Ch4 => 3,
        }
    }

    // ================= embedded-hal PWM trait =================

    impl embedded_hal::pwm::ErrorType for ClassicPwm {
        type Error = PwmError;
    }

    impl embedded_hal::pwm::SetDutyCycle for ClassicPwm {
        #[inline]
        fn max_duty_cycle(&self) -> u16 {
            ClassicPwm::max_duty(self)
        }

        #[inline]
        fn set_duty_cycle(&mut self, duty: u16) -> Result<(), Self::Error> {
            // 满量程固定万分比 10000，超界夹到 100%
            ClassicPwm::set_duty_bp(self, duty.min(10_000))
        }
    }

    // ================= 中断派发 =================

    /// GTIM 溢出中断公共处理：单发压掉 IER.OV 并唤醒等待者（ISR 的 OV
    /// 标志不动——poll 复查后由下次 wait_update 的前置 ICR 清除）。
    fn on_irq(idx: usize, regs: &'static pac::GTIM) {
        if regs.isr.read() & ISR_OV != 0 {
            regs.ier.modify(|v| v & !IER_OV);
            CLASSIC_WAKER[idx].wake();
        }
    }

    /// GTIM1 溢出中断标记（IRQ16，bind_interrupts! 绑定用）
    pub struct Gtim1Irq;
    impl IrqHandler<interrupt::GTIM1> for Gtim1Irq {
        fn call() {
            on_irq(0, unsafe { &*(pac::GTIM1_BASE as *const pac::GTIM) });
        }
    }

    /// GTIM2 溢出中断标记（IRQ17）
    pub struct Gtim2Irq;
    impl IrqHandler<interrupt::GTIM2> for Gtim2Irq {
        fn call() {
            on_irq(1, unsafe { &*(pac::GTIM2_BASE as *const pac::GTIM) });
        }
    }

    /// GTIM3 溢出中断标记（IRQ18；仅 f030/f020/l052/l083 有此实例）
    #[cfg(any(
        feature = "cw32f030",
        feature = "cw32f020",
        feature = "cw32l052",
        feature = "cw32l083"
    ))]
    pub struct Gtim3Irq;
    #[cfg(any(
        feature = "cw32f030",
        feature = "cw32f020",
        feature = "cw32l052",
        feature = "cw32l083"
    ))]
    impl IrqHandler<interrupt::GTIM3> for Gtim3Irq {
        fn call() {
            on_irq(2, unsafe { &*(pac::GTIM3_BASE as *const pac::GTIM) });
        }
    }

    /// GTIM4 溢出中断标记（IRQ19；仅 f030/f020/l083 有此实例）
    #[cfg(any(feature = "cw32f030", feature = "cw32f020", feature = "cw32l083"))]
    pub struct Gtim4Irq;
    #[cfg(any(feature = "cw32f030", feature = "cw32f020", feature = "cw32l083"))]
    impl IrqHandler<interrupt::GTIM4> for Gtim4Irq {
        fn call() {
            on_irq(3, unsafe { &*(pac::GTIM4_BASE as *const pac::GTIM) });
        }
    }
}

#[cfg(any(
    feature = "cw32f030",
    feature = "cw32f020",
    feature = "cw32l031",
    feature = "cw32l052",
    feature = "cw32l083",
    feature = "cw32r031",
    feature = "cw32w031"
))]
pub use classic::*;

#[cfg(test)]
mod tests {
    use super::*;

    // ===== GTIM 位常量 ↔ 三家族 PAC 位面锚定（见 l012gen 内同名测试，
    // 常量为 l012gen 私有故测试挂在 l012gen::tests 下）=====

    // 与阻塞版 cw32l012-hal/src/gtim.rs 的同名单元测试同源
    #[test]
    fn pwm_1khz_at_4mhz() {
        // 4M/1k = 4000 ticks ≤ 65536 → PSC=0 ARR=3999
        let (psc, arr) = calc_pwm_psc_arr(4_000_000, 1_000).unwrap();
        assert_eq!(psc, 0);
        assert_eq!(arr, 3999);
    }

    #[test]
    fn pwm_20khz_at_96mhz() {
        // 96M/20k = 4800 ticks → PSC=0 ARR=4799
        let (psc, arr) = calc_pwm_psc_arr(96_000_000, 20_000).unwrap();
        assert_eq!(psc, 0);
        assert_eq!(arr, 4799);
    }

    #[test]
    fn pwm_slow_needs_prescaler() {
        // 2Hz@96MHz：PSC=733 → ck_cnt=96M/733≈131k，ARR 大分频量化
        let (psc, arr) = calc_pwm_psc_arr(96_000_000, 2).unwrap();
        let ck = 96_000_000u64 / (psc as u64 + 1);
        let f = ck / (arr as u64 + 1);
        assert!((f as i64 - 2).abs() <= 1, "实际 {f}Hz 偏离 2Hz");
    }

    #[test]
    fn pwm_out_of_range() {
        // f > PCLK/2 不可实现（ticks<2）
        assert!(calc_pwm_psc_arr(4_000_000, 3_000_000).is_none());
        assert!(calc_pwm_psc_arr(4_000_000, 0).is_none());
    }

    #[test]
    fn duty_mapping() {
        // ARR=3999：50% → CCR = 4000×5000/10000 = 2000
        let arr: u32 = 3999;
        let ccr = ((arr + 1) * 5000 / 10_000) as u16;
        assert_eq!(ccr, 2000);
        // 100% → CCR = 4000（=ARR+1，CNT 永不大于 CCR → 恒有效电平）
        let ccr = ((arr + 1) * 10_000 / 10_000) as u16;
        assert_eq!(ccr, 4000);
    }

    // ===== 经典组 f030/f020 的 2^n 分频变体 =====

    #[test]
    fn pow2_div_1khz_at_8mhz() {
        // 8M/1k = 8000 ticks ≤ 65536 → 最小分频 1，ARR=7999（f030 复位默认 8MHz 档）
        let (div, arr) = calc_pwm_pow2_div(8_000_000, 1_000).unwrap();
        assert_eq!(div, 1);
        assert_eq!(arr, 7999);
    }

    #[test]
    fn pow2_div_needs_prescaler() {
        // 100Hz@8MHz：8M/100 = 80000 > 65536 → 分频 2 → 40000 ticks
        let (div, arr) = calc_pwm_pow2_div(8_000_000, 100).unwrap();
        assert_eq!(div, 2);
        assert_eq!(arr, 39999);
    }

    #[test]
    fn pow2_div_minimal_divisor_prefers_fine_quantization() {
        // 2Hz@8MHz：div=1 → 4M ticks 爆表；div=32 → 125k 爆表；
        // div=64 → 8M/64/2 = 62500 ticks ≤ 65536 ✓（最早可行的 2^n 档）
        let (div, arr) = calc_pwm_pow2_div(8_000_000, 2).unwrap();
        assert_eq!(div, 64);
        let f = 8_000_000u64 / div as u64 / (arr as u64 + 1);
        assert!((f as i64 - 2).abs() <= 1, "实际 {f}Hz 偏离 2Hz");
    }

    #[test]
    fn pow2_div_out_of_range() {
        // f > PCLK/2：任何 2^n 档都 ticks<2
        assert!(calc_pwm_pow2_div(4_000_000, 3_000_000).is_none());
        assert!(calc_pwm_pow2_div(4_000_000, 0).is_none());
    }

    #[test]
    fn pow2_div_extremes() {
        // 极低频：f030 PRS 最大 2^15=32768 × 周期 65536 → 下界 PCLK/2147483648
        let (div, arr) = calc_pwm_pow2_div(8_000_000, 8_000_000 / 2_147_483_648).unwrap_or((0, 0));
        assert_eq!(div, 0); // 0.0037Hz@8MHz 低于下界 → None（哨兵 0）
        let _ = arr;
        // 上界附近：PCLK/2 恰好 2 tick
        let (div, arr) = calc_pwm_pow2_div(8_000_000, 4_000_000).unwrap();
        assert_eq!(div, 1);
        assert_eq!(arr, 1);
    }
}
