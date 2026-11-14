//! ATIM 高级定时器互补 PWM（L012 家族独占，手册 §16）——三相电机 PWM 路线。
//!
//! CH1-3 主/互补输出对（六路，死区插入）+ CH4 单端 + BK 刹车输入。寄存器
//! 面为 STM32 风格（CR1/CR2/CCMR/CCER/BDTR/DIER/EGR，影子寄存器预装载），
//! PSC/ARR 计算与 [`crate::timer`] 的 GTIM 完全同源（直接复用
//! [`calc_pwm_psc_arr`]）。经典家族的 ATIM 是另一套 IP（CR0/CMMR 面，见
//! `src/timer.rs` classic 分支头注），不在本模块范围。
//!
//! # 状态：compiles-not-run（诚实标注）
//!
//! 编译通过 + 宿主机单元测试（DTG 四段编码全覆盖 / ICR 清位字构造）；
//! **实机验证待板**。寄存器语义三源核对：本仓 PAC `pac::atim` 位常量
//! （手册逐位转录，其注释记作 §17.10——与 §16 为同一章）、手册 §16、
//! fan2nd/cw32-hal2 证据链 `docs/atim-complementary-evidence.json`
//! （MIT）——死区时钟不走 PSC、DTG 四段编码、刹车硬件异步清 MOE 等
//! 关键口径均出自该证据链。**上板验证前不要驱动任何真电机。**
//!
//! # 死区（手册 §16，BDTR.DTG[7:0]）
//!
//! 死区时钟 = f_DTS = PCLK：本驱动把 CKD 固定在 00（=/1），死区不随
//! 计数分频 PSC 变化。`dead_time_ns` 向上取整到最近可编码值（宁多勿少，
//! 绝不取不足值）。DTG 四段编码（同 STM32 约定）：
//!
//! | 请求 tick     | 编码位段        | 实际 tick    | 步进 |
//! |---------------|-----------------|--------------|------|
//! | 0..=127       | 原值            | =DTG         | 1    |
//! | 128..=254     | 0x80\|(t/2-64)  | (64+d)×2     | 2    |
//! | 256..=504     | 0xC0\|(t/8-32)  | (32+d)×8     | 8    |
//! | 512..=1008    | 0xE0\|(t/16-32) | (32+d)×16    | 16   |
//!
//! 上限 1008 个 f_DTS 周期；换算后超出即 [`AtimError::DeadTimeTooLong`]。
//! 编解码助手 [`encode_dtg`] / [`decode_dtg`][`decode_dtg`]（宿主机测试
//! 全量覆盖：四段边界、向上取整、上限）。
//!
//! # 刹车（手册 §16，BDTR）
//!
//! 给了 `bk` 引脚才使能 BKE；BKP 极性由 `bk_polarity_high` 选、BKF 数字
//! 滤波 0..=15（f_DTS 周期口径）。刹车事件硬件**异步**清 MOE——不经
//! 软件、不经中断，六路输出立即进入空闲态（OISx=0、OSSI=1 → 上/下管
//! 全关）。软件侧 ISR.BIF 置位（[`AtimPwm::is_braked`]）；**刹车源活跃
//! 期间 BIF 清不掉**（[`AtimPwm::clear_break`] 返回 false）。`aoe=false`
//! （默认建议）时恢复由 `clear_break()` + `enable()` 显式完成；
//! `aoe=true` 时刹车解除后的下一更新事件硬件自动重置 MOE。
//!
//! # 推荐保守面（构造即写入）
//!
//! OSSI=1、OSSR=1（空闲/关闭态输出仍被驱动到 OIS 电平而非高阻）、
//! OISx/OISxN=0（空闲全低）、CCxP/CCxNP=0（高有效）、LOCK=0（不加写
//! 保护）、BK2 全关、DTR2 不碰（DTPE/DTAE=0）。
//!
//! # 占空比与端点语义
//!
//! 万分比 0..=10000（同 GTIM 约定），`ccr = (ARR+1) × 万分比 / 10000`；
//! OCxM=PWM1 + OCxPE 比较预装载 → 运行中改占空比**下一更新周期生效**；
//! ARPE=1（ARR 预装载）。端点：0% = CHx 常低、CHxN 常高（三相全下管
//! 导通 = 短路制动态）；100% = CHx 常高、CHxN 常低（全上管导通）。
//! 互补对 CHxN = CHx 经死区取反（CCER 每通道 4bit：CCxE/CCxP/CCxNE/
//! CCxNP；CC5/CC6 无互补、本驱动不建模）。CR2.CCDS 保持复位值 0（DMA
//! 请求跟随捕获/比较事件的口径——STM32 同名位约定；本驱动不用 DMA，
//! 需更新事件触发 DMA 的场景由调用方自行置 1）。
//!
//! # 引脚复用（数据手册 V1.0 表 5-3，SDK 双源验证）
//!
//! | 信号 | 引脚（AF）                                                       |
//! |------|------------------------------------------------------------------|
//! | CH1  | PA5 / PA8 / PB2 / PB5（均 AF7）                                  |
//! | CH1N | PA15 / PA7 / PB13（均 AF7）                                      |
//! | CH2  | PA9 / PB10 / PB6（均 AF7）                                       |
//! | CH2N | PA4 / PB0 / PB14 / PB3（均 AF7）                                 |
//! | CH3  | PA10 / PB11 / PB7（均 AF7）                                      |
//! | CH3N | PB1 / PB15 / PB4（均 AF7）                                       |
//! | CH4  | PA11 / PA3（AF7）、PB9（AF5）                                    |
//! | CH4N | PA2（AF7）、PB8（AF5）                                           |
//! | BK   | PA0 / PA6 / PB12 / PB9（AF7）、PA12 / PB5（AF5）、PC13（AF6）    |
//!
//! 输出脚配方同 [`crate::timer`]（清开漏/上下拉/模拟 → AF → DIR=0 输出）；
//! **刹车输入脚不同**：DIR=1 输入、保留调用方指定的上/下拉（外部上拉是
//! 低有效刹车的常规接法，驱动不得清掉）、不清开漏。GPIO 端口 AHBEN
//! 时钟由调用方先开（同 GTIM 示例配方）。
//!
//! # 中断（IRQ13 `ATIM`）
//!
//! ```ignore
//! embassy_cw32::bind_interrupts!(struct Irqs {
//!     ATIM => embassy_cw32::atim::AtimIrq;
//! });
//! ```
//!
//! [`AtimPwm::wait_update`] 用更新标志 UIF 做周期节拍（等待时才开
//! DIER.UIE，IRQ 唤醒即压掉，同 GTIM 的"单发纪律"）；[`AtimIrq`] 同时
//! 防御性处理 COMIF/BIF（压掉对应使能位并唤醒预留 waker——本驱动自身
//! 不开这两个中断，但防止调用方手工开 DIER.COMIE/BIE 后的风暴）。
//!
//! # 用法（三相互补，20kHz / 1µs 死区）
//!
//! ```ignore
//! embassy_cw32::bind_interrupts!(struct Irqs {
//!     ATIM => embassy_cw32::atim::AtimIrq;
//! });
//! // 不用的通道传 None：APIT 端口类型无法从 None 推断，需显式标注
//! let no_pin: Option<(Pin<embassy_cw32::Gpio>, u8)> = None;
//! let mut pwm = AtimPwm::new(
//!     p.atim,
//!     AtimPwmConfig { freq_hz: 20_000, dead_time_ns: 1_000, aoe: false },
//!     Some((Pin::new(p.gpioa, 8), 7)),  // CH1  = PA8  AF7
//!     Some((Pin::new(p.gpioa, 7), 7)),  // CH1N = PA7  AF7
//!     Some((Pin::new(p.gpioa, 9), 7)),  // CH2  = PA9  AF7
//!     Some((Pin::new(p.gpiob, 0), 7)),  // CH2N = PB0  AF7
//!     Some((Pin::new(p.gpioa, 10), 7)), // CH3  = PA10 AF7
//!     Some((Pin::new(p.gpiob, 1), 7)),  // CH3N = PB1  AF7
//!     no_pin,                           // CH4 单端，本例不用
//!     Some((Pin::new(p.gpioa, 6), 7)),  // BK = PA6 AF7
//!     false, 0,                         // BKP=0 低有效 / BKF=0 无滤波
//!     Irqs,
//! )?;
//! if !pwm.enable() {
//!     // MOE 置不上 = 刹车压着（构造时 BK 已活跃），先排除刹车源
//! }
//! pwm.set_duty_bp(1, 5_000)?; // 通道 1 半占空，下一更新周期生效
//! pwm.wait_update().await;    // 周期节拍（IRQ13）
//! ```
//!
//! # 安全边界（诚实声明）
//!
//! 本驱动只保证寄存器面正确：**不做电机安全，不做故障闩锁承诺**。
//! 堵转/过流/过温保护必须由外部刹车源（比较器 → BK 脚）+ 上层监控
//! 实现，本驱动不承诺任何故障响应时间；"刹车后何时可以恢复"的判断
//! 完全在调用方；死区按请求值向上取整插入，但**不构成防直通保证**
//! （驱动电路传播延迟差异需硬件侧核对）。

use core::future::poll_fn;
use core::sync::atomic::Ordering;
use core::task::Poll;

use embassy_sync::waitqueue::AtomicWaker;

use crate::gpio::{GpioPort, Pin, Reg};
use crate::interrupt::{Binding, IrqHandler, interrupt};
use crate::pac;
use crate::timer::{PwmError, calc_pwm_psc_arr};

// ================= 位常量面 =================
// PAC（cw32l012-pac-um =0.3.0，生成器产物不可改动）只转录了部分 ATIM 位
// 常量；下表为 PAC 缺失部分的本地补充，位序按手册 §16 逐位补齐。PAC 已有
// 的常量经 `use crate::pac::atim as at;` 引用，不在此重复定义。
use crate::pac::atim as at;

// ---- CR2 (0x04)：输出控制（本驱动保持复位值 0，常量供调用方按需使用）----
/// 互补输出预装载（CCER 写入经预装载寄存器、更新事件生效）
pub const CR2_CCPC: u32 = 1 << 0;
/// 互补输出预装载读/写选择（读改写预装载寄存器还是活寄存器）
pub const CR2_CCUS: u32 = 1 << 2;
/// 捕获/比较 DMA 选择：0=CCx 事件请求 DMA（本驱动保持 0），1=更新事件
pub const CR2_CCDS: u32 = 1 << 3;
/// 主模式选择 MMS[6:4]（TRGO 输出源）
pub const CR2_MMS_POS: u32 = 4;
/// MMS 掩码
pub const CR2_MMS_MASK: u32 = 0x7 << CR2_MMS_POS;
/// 空闲态输出 1（MOE=0、OSSI=1 时 CH1 的驱动电平）
pub const CR2_OIS1: u32 = 1 << 8;
/// 空闲态输出 1N（CH1N）
pub const CR2_OIS1N: u32 = 1 << 9;
/// 空闲态输出 2
pub const CR2_OIS2: u32 = 1 << 10;
/// 空闲态输出 2N
pub const CR2_OIS2N: u32 = 1 << 11;
/// 空闲态输出 3
pub const CR2_OIS3: u32 = 1 << 12;
/// 空闲态输出 3N
pub const CR2_OIS3N: u32 = 1 << 13;
/// 空闲态输出 4
pub const CR2_OIS4: u32 = 1 << 14;
/// 空闲态输出 4N
pub const CR2_OIS4N: u32 = 1 << 15;

// ---- CCMR2 (0x1C)：OC3/OC4 输出比较（布局同 CCMR1 的 OC1/OC2）----
/// OC3M 低位段位置
pub const OC3M_POS: u32 = 4;
/// OC3M 低位段掩码（高 3 位；第 4 位见 [`OC3MH`]）
pub const OC3M_MASK: u32 = 0x7 << OC3M_POS;
/// OC3M 第 4 位（PWM1=0b0110 → M=110 且 MH=0）
pub const OC3MH: u32 = 1 << 16;
/// OC3 = PWM 模式 1（CNT<CCR 输出有效电平）
pub const OC3M_PWM1: u32 = 6 << OC3M_POS;
/// OC3 比较预装载使能
pub const OC3PE: u32 = 1 << 3;
/// OC4M 低位段位置
pub const OC4M_POS: u32 = 12;
/// OC4M 低位段掩码
pub const OC4M_MASK: u32 = 0x7 << OC4M_POS;
/// OC4M 第 4 位
pub const OC4MH: u32 = 1 << 24;
/// OC4 = PWM 模式 1
pub const OC4M_PWM1: u32 = 6 << OC4M_POS;
/// OC4 比较预装载使能
pub const OC4PE: u32 = 1 << 11;

// ---- DIER (0x0C)：中断使能 ----
/// 更新中断使能
pub const DIER_UIE: u32 = 1 << 0;
/// CH1 比较/捕获中断使能
pub const DIER_CC1IE: u32 = 1 << 1;
/// CH2 比较/捕获中断使能
pub const DIER_CC2IE: u32 = 1 << 2;
/// CH3 比较/捕获中断使能
pub const DIER_CC3IE: u32 = 1 << 3;
/// CH4 比较/捕获中断使能
pub const DIER_CC4IE: u32 = 1 << 4;
/// 换相（COM）中断使能
pub const DIER_COMIE: u32 = 1 << 5;
/// 触发中断使能
pub const DIER_TIE: u32 = 1 << 6;
/// 刹车（BK）中断使能
pub const DIER_BIE: u32 = 1 << 7;
/// 刹车 2（BK2）中断使能
pub const DIER_B2IE: u32 = 1 << 8;

// ---- ISR (0x10，只读) / ICR (0x70，R1W0) ----
/// CH1 比较/捕获标志
pub const ISR_CC1IF: u32 = 1 << 1;
/// CH2 比较/捕获标志
pub const ISR_CC2IF: u32 = 1 << 2;
/// CH3 比较/捕获标志
pub const ISR_CC3IF: u32 = 1 << 3;
/// CH4 比较/捕获标志
pub const ISR_CC4IF: u32 = 1 << 4;
/// 换相标志
pub const ISR_COMIF: u32 = 1 << 5;
/// 触发标志
pub const ISR_TIF: u32 = 1 << 6;
/// 刹车标志（刹车源活跃期间硬件保持置位、清不掉）
pub const ISR_BIF: u32 = 1 << 7;
/// 刹车 2 标志
pub const ISR_B2IF: u32 = 1 << 8;
/// CH1 过捕获标志
pub const ISR_CC1OF: u32 = 1 << 9;
/// CH2 过捕获标志
pub const ISR_CC2OF: u32 = 1 << 10;
/// CH3 过捕获标志
pub const ISR_CC3OF: u32 = 1 << 11;
/// CH4 过捕获标志
pub const ISR_CC4OF: u32 = 1 << 12;

// ---- BDTR (0x44)：PAC 只有 BKF_POS，掩码在此补 ----
/// 刹车 BRK 数字滤波 BKF[19:16] 掩码
pub const BDTR_BKF_MASK: u32 = 0xF << 16;
/// 刹车 2 滤波 BK2F[23:20] 位置（本驱动不用 BK2，位面补全供调用方）
pub const BDTR_BK2F_POS: u32 = 20;
/// 刹车 2 滤波掩码
pub const BDTR_BK2F_MASK: u32 = 0xF << BDTR_BK2F_POS;

// ================= DTG 死区编码（手册 §16，BDTR.DTG[7:0]） =================

/// 死区时间（ns）→ f_DTS 周期数（向上取整：宁多勿少，绝不取不足值）
fn dead_time_ns_to_ticks(pclk_hz: u32, dead_time_ns: u32) -> u32 {
    (dead_time_ns as u64 * pclk_hz as u64).div_ceil(1_000_000_000) as u32
}

/// 请求 tick 数 → DTG 位段编码（BDTR.DTG[7:0]）
///
/// 四段编码（§16）：段内向上取整到最近格点；所有段都装不下（> 1008
/// tick）返回 `None`（调用方报 [`AtimError::DeadTimeTooLong`]）。
fn dtg_bits(ticks: u32) -> Option<u32> {
    // 段 1：DTG[7:0] 原值，步进 1（0..=127）
    if ticks <= 127 {
        return Some(ticks);
    }
    // 段 2：DTG[7:6]=10，DT=(64+DTG[5:0])×2，步进 2（128..=254）
    if ticks <= 254 {
        let d = ticks.div_ceil(2) - 64;
        if d <= 63 {
            return Some(0x80 | d);
        }
    }
    // 段 3：DTG[7:5]=110，DT=(32+DTG[4:0])×8，步进 8（256..=504）
    if ticks <= 504 {
        let d = ticks.div_ceil(8) - 32;
        if d <= 31 {
            return Some(0xC0 | d);
        }
    }
    // 段 4：DTG[7:5]=111，DT=(32+DTG[4:0])×16，步进 16（512..=1008）
    if ticks <= 1008 {
        let d = ticks.div_ceil(16) - 32;
        if d <= 31 {
            return Some(0xE0 | d);
        }
    }
    None
}

/// DTG 位段 → 实际死区 tick 数（f_DTS 周期；[`dtg_bits`] 的逆函数）
///
/// 四段解码：0xx → 原值；10x → (64+DTG[5:0])×2；110 → (32+DTG[4:0])×8；
/// 111 → (32+DTG[4:0])×16。最大可编码 1008 tick。
pub fn decode_dtg(bits: u32) -> u32 {
    match bits >> 5 {
        0b000..=0b011 => bits & 0x7F,              // 0xx：原值
        0b100..=0b101 => (64 + (bits & 0x3F)) * 2, // 10x
        0b110 => (32 + (bits & 0x1F)) * 8,         // 110
        _ => (32 + (bits & 0x1F)) * 16,            // 111
    }
}

/// 死区时间（ns）→ DTG 位段编码值（写入 BDTR.DTG[7:0]）
///
/// 死区时钟 f_DTS = PCLK（CKD 固定 /1，死区不走 PSC）；换算向上取整，
/// 超过可编码上限（1008 × f_DTS 周期，如 4MHz 下 252µs、96MHz 下
/// 10.5µs）返回 [`AtimError::DeadTimeTooLong`]。
pub fn encode_dtg(pclk_hz: u32, dead_time_ns: u32) -> Result<u32, AtimError> {
    dtg_bits(dead_time_ns_to_ticks(pclk_hz, dead_time_ns)).ok_or(AtimError::DeadTimeTooLong)
}

// ================= ICR 清标志写入字（R1W0，手册 §16） =================

/// 构造 ICR 写入字：清 `flag` 对应标志、保持其余可清标志。
///
/// ATIM 的 ICR 是 R1W0（写 0 清对应标志、写 1 保持），复位值
/// 0x00FF_3FFF——可清标志位（bit0..13）复位即 1。清某标志须写
/// "全 1 除该位"，**不能整字写 0**（会把其余可清标志一并清掉）。
/// 与 PAC `atim::ICR_CLEAR_UIF`（= !1）同套路。
const fn icr_clear(flag: u32) -> u32 {
    !flag
}

// ================= 配置与错误 =================

/// ATIM PWM 配置（构造参数）
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct AtimPwmConfig {
    /// PWM 频率（Hz；上界 PCLK/2，下界受最大分频限制——同 GTIM 约定）
    pub freq_hz: u32,
    /// 死区时间（ns；向上取整到最近可编码 DTG 值，上限见 [`encode_dtg`]）
    pub dead_time_ns: u32,
    /// 刹车自动恢复（BDTR.AOE）：true = 刹车解除后的下一更新事件硬件
    /// 自动重置 MOE；false（默认建议）= 恢复由 [`AtimPwm::clear_break`]
    /// + [`AtimPwm::enable`] 显式完成，决策权留给上层
    pub aoe: bool,
}

/// ATIM 配置错误
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum AtimError {
    /// 频率超出可实现范围（上界 PCLK/2，下界受最大分频限制）
    FreqOutOfRange,
    /// 占空比 > 10000（万分比）
    BadDuty,
    /// 死区超出 DTG 可编码上限（1008 × f_DTS 周期）
    DeadTimeTooLong,
    /// 刹车配置非法（BKF > 15 等）
    BadBreakConfig,
}

impl embedded_hal::pwm::Error for AtimError {
    fn kind(&self) -> embedded_hal::pwm::ErrorKind {
        // embedded-hal 1.0 的 pwm::ErrorKind 目前只有 Other
        embedded_hal::pwm::ErrorKind::Other
    }
}

// ================= 驱动本体 =================

/// ATIM 互补 PWM（三相互补对 + CH4 单端 + 刹车）
///
/// 构造即完成门控/引脚复用/时基/输出模式/死区/刹车配置并启动计数；
/// **MOE 默认关**——输出保持空闲安全态（OISx=0 全低），显式
/// [`AtimPwm::enable`] 后才有波形。运行中改占空比经
/// [`AtimPwm::set_duty_bp`]（OCxPE 预装载 → 下一更新周期生效）。
pub struct AtimPwm {
    regs: &'static pac::Atim,
}

impl AtimPwm {
    /// 初始化 ATIM（手册 §16 保守面 + GTIM 同源时基配方）
    ///
    /// * `regs` — ATIM 寄存器块（`p.atim`，`ATIM_BASE = 0x4000_1400`）
    /// * `config` — 频率/死区/AOE
    /// * `ch1`/`ch1n`..`ch3`/`ch3n` — 主/互补引脚（`(Pin, AF)`；`None` =
    ///   该路不用）。哪路的任一引脚给出就配哪路的输出比较模式与 CCER
    ///   使能位（主脚给 → CCxE、互补脚给 → CCxNE，极性恒高有效）
    /// * `ch4` — CH4 单端引脚（无互补，CH4N 本驱动不建模）
    /// * `bk` — 刹车输入引脚（None = 不用硬件刹车，BKE 保持 0）
    /// * `bk_polarity_high` — 刹车有效极性：false = 低电平有效（BKP=0，
    ///   开漏比较器 + 上拉的常规接法），true = 高电平有效
    /// * `bk_filter` — 刹车数字滤波 BKF（0..=15，越界报
    ///   [`AtimError::BadBreakConfig`]）
    /// * `_irqs` — `bind_interrupts!` 绑定证明（编译期校验绑错中断；
    ///   [`AtimPwm::wait_update`] 需要对应 IRQ 已绑定）
    ///
    /// 全部纯计算先行（频率/死区/刹车参数校验），通过后才碰硬件。
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        regs: &'static pac::Atim,
        config: AtimPwmConfig,
        ch1: Option<(Pin<impl GpioPort>, u8)>,
        ch1n: Option<(Pin<impl GpioPort>, u8)>,
        ch2: Option<(Pin<impl GpioPort>, u8)>,
        ch2n: Option<(Pin<impl GpioPort>, u8)>,
        ch3: Option<(Pin<impl GpioPort>, u8)>,
        ch3n: Option<(Pin<impl GpioPort>, u8)>,
        ch4: Option<(Pin<impl GpioPort>, u8)>,
        bk: Option<(Pin<impl GpioPort>, u8)>,
        bk_polarity_high: bool,
        bk_filter: u8,
        _irqs: impl Binding<interrupt::ATIM>,
    ) -> Result<Self, AtimError> {
        debug_assert_eq!(
            regs as *const _ as usize,
            pac::ATIM_BASE,
            "regs 须为 ATIM 基址（p.atim）"
        );
        // ---- 纯计算先行（全部通过后才产生硬件副作用）----
        if bk_filter > 15 {
            return Err(AtimError::BadBreakConfig);
        }
        let pclk_hz = crate::PCLK_HZ.load(Ordering::Relaxed);
        let (psc, arr) =
            calc_pwm_psc_arr(pclk_hz, config.freq_hz).ok_or(AtimError::FreqOutOfRange)?;
        let dtg = encode_dtg(pclk_hz, config.dead_time_ns)?;

        // ---- 时钟门控：APBEN1.ATIM = bit5（KEY 写保护，读改写保留其它
        // 外设使能位——同 timer.rs 配方）----
        let sysctrl = unsafe { &*(pac::SYSCTRL_BASE as *const crate::Sysctrl) };
        sysctrl
            .apben1
            .modify(|v| pac::sysctrl::KEY | (v & 0xFFFF) | pac::sysctrl::APBEN1_ATIM);

        // ---- 引脚复用 ----
        if let Some((pin, af)) = ch1.as_ref() {
            config_output_pin(pin, *af);
        }
        if let Some((pin, af)) = ch1n.as_ref() {
            config_output_pin(pin, *af);
        }
        if let Some((pin, af)) = ch2.as_ref() {
            config_output_pin(pin, *af);
        }
        if let Some((pin, af)) = ch2n.as_ref() {
            config_output_pin(pin, *af);
        }
        if let Some((pin, af)) = ch3.as_ref() {
            config_output_pin(pin, *af);
        }
        if let Some((pin, af)) = ch3n.as_ref() {
            config_output_pin(pin, *af);
        }
        if let Some((pin, af)) = ch4.as_ref() {
            config_output_pin(pin, *af);
        }
        if let Some((pin, af)) = bk.as_ref() {
            config_break_pin(pin, *af);
        }

        // ---- 时基（GTIM 同源：停计数写影子寄存器）----
        regs.cr1.write(0); // 停计数；CKD 保持 00（f_DTS=PCLK，死区时钟不走 PSC）
        regs.psc.write(psc as u32);
        regs.arr.write(arr as u32);
        regs.ccr1.write(0); // 初始占空比 0（CHx 常低、CHxN 常高 = 下管导通态）
        regs.ccr2.write(0);
        regs.ccr3.write(0);
        regs.ccr4.write(0);
        regs.rcr.write(0); // 每周期一个更新事件（REP 复位值即 0，显式写明）

        // ---- 输出比较模式：PWM1 + 比较预装载（哪路引脚给了就配哪路；
        // CCxS=00 输出态、极性恒高有效 CCxP/CCxNP=0）----
        if ch1.is_some() || ch1n.is_some() {
            regs.ccmr1.modify(|v| {
                (v & !(at::OC1M_MASK | at::OC1MH | at::OC1PE | 0x3)) | at::OC1M_PWM1 | at::OC1PE
            });
            let cc = match ch1n.is_some() {
                true => at::CC1E | at::CC1NE,
                false => at::CC1E,
            };
            regs.ccer
                .modify(|v| (v & !(at::CC1E | at::CC1P | at::CC1NE | at::CC1NP)) | cc);
        }
        if ch2.is_some() || ch2n.is_some() {
            regs.ccmr1.modify(|v| {
                (v & !(at::OC2M_MASK | at::OC2MH | at::OC2PE | (0x3 << 8)))
                    | at::OC2M_PWM1
                    | at::OC2PE
            });
            let cc = match ch2n.is_some() {
                true => at::CC2E | at::CC2NE,
                false => at::CC2E,
            };
            regs.ccer
                .modify(|v| (v & !(at::CC2E | at::CC2P | at::CC2NE | at::CC2NP)) | cc);
        }
        if ch3.is_some() || ch3n.is_some() {
            regs.ccmr2
                .modify(|v| (v & !(OC3M_MASK | OC3MH | OC3PE | 0x3)) | OC3M_PWM1 | OC3PE);
            let cc = match ch3n.is_some() {
                true => at::CC3E | at::CC3NE,
                false => at::CC3E,
            };
            regs.ccer
                .modify(|v| (v & !(at::CC3E | at::CC3P | at::CC3NE | at::CC3NP)) | cc);
        }
        if ch4.is_some() {
            regs.ccmr2
                .modify(|v| (v & !(OC4M_MASK | OC4MH | OC4PE | (0x3 << 8))) | OC4M_PWM1 | OC4PE);
            regs.ccer
                .modify(|v| (v & !(at::CC4E | at::CC4P)) | at::CC4E);
        }

        // ---- BDTR：DTG 死区 + 推荐保守面（OSSI/OSSR=1：空闲与关闭态
        // 输出被驱动到 OIS 电平；OISx=0 全低 = 上/下管全关。LOCK=0 不加
        // 写保护；BK2 全关；DTR2 不碰——DTPE/DTAE 保持 0）----
        let mut bdtr = at::BDTR_OSSI | at::BDTR_OSSR | dtg;
        if bk.is_some() {
            bdtr |= at::BDTR_BKE;
        }
        if bk_polarity_high {
            bdtr |= at::BDTR_BKP;
        }
        bdtr |= (bk_filter as u32 & 0xF) << at::BDTR_BKF_POS;
        if config.aoe {
            bdtr |= at::BDTR_AOE;
        }
        // MOE 不在此置位——司机显式 enable()（刹车活跃时置不上）
        regs.bdtr
            .modify(|v| (v & !(at::BDTR_DTG_MASK | BDTR_BKF_MASK)) | bdtr);

        // UG 重装影子寄存器（URS=0 会置 UIF，wait_update 前置清除处理）
        regs.egr.write(at::EGR_UG);
        // ARPE=1（ARR 预装载）+ 启动计数（MOE=0 → 输出保持空闲安全态）
        regs.cr1.write(at::CR1_ARPE | at::CR1_CEN);
        Ok(AtimPwm { regs })
    }

    /// 满量程（万分比；固定 10000 同 GTIM 约定——周期最大 65536 放不进
    /// `SetDutyCycle::max_duty_cycle` 的 u16）
    pub fn max_duty(&self) -> u16 {
        10_000
    }

    /// 运行中修改某通道占空比（万分比 0..=10000；`ch` = 1..=4）
    ///
    /// OCxPE 比较预装载开启 → **下一更新周期生效**。通道号越界返回
    /// [`PwmError::OutOfRange`]、占空比越界返回 [`PwmError::BadDuty`]。
    pub fn set_duty_bp(&mut self, ch: usize, duty_bp: u16) -> Result<(), PwmError> {
        if !(1..=4).contains(&ch) {
            return Err(PwmError::OutOfRange);
        }
        if duty_bp > 10_000 {
            return Err(PwmError::BadDuty);
        }
        let arr = self.regs.arr.read() & 0xFFFF;
        // ccr = (ARR+1) × 万分比 / 10000（100% → CCR=ARR+1，恒有效电平）
        let ccr = ((arr + 1) * duty_bp as u32 / 10_000) as u16;
        match ch {
            1 => self.regs.ccr1.write(ccr as u32),
            2 => self.regs.ccr2.write(ccr as u32),
            3 => self.regs.ccr3.write(ccr as u32),
            _ => self.regs.ccr4.write(ccr as u32),
        }
        Ok(())
    }

    /// 置 MOE 主输出使能（六路输出的总开关）
    ///
    /// 构造后 MOE=0：计数在跑（`wait_update` 可用）但输出保持 OIS 空闲
    /// 电平（OISx=0 → 全低）。**刹车活跃时硬件异步清 MOE、写入无效**——
    /// 返回写入后的 MOE 实态（false = 刹车压着，先排除刹车源）。
    pub fn enable(&mut self) -> bool {
        self.regs.bdtr.modify(|v| v | at::BDTR_MOE);
        self.regs.bdtr.read() & at::BDTR_MOE != 0
    }

    /// 清 MOE（封波：六路输出进 OIS 空闲态全低，计数继续跑）
    pub fn disable(&mut self) {
        self.regs.bdtr.modify(|v| v & !at::BDTR_MOE);
    }

    /// 是否处于刹车态（ISR.BIF；刹车源活跃期间恒 true）
    pub fn is_braked(&self) -> bool {
        self.regs.isr.read() & ISR_BIF != 0
    }

    /// 清刹车标志 BIF；返回 false = 刹车源仍活跃（硬件把 BIF 拉回置位，
    /// R1W0 清不掉）
    ///
    /// ICR 写"全 1 除 bit7"：仅清 BIF、其余可清标志写 1 保持（见
    /// [`icr_clear`]——不能整字写 0）。恢复波形还须 [`AtimPwm::enable`]
    /// 置回 MOE（`aoe=false` 时硬件不会自动恢复）。
    pub fn clear_break(&self) -> bool {
        self.regs.icr.write(icr_clear(ISR_BIF));
        self.regs.isr.read() & ISR_BIF == 0
    }

    /// 读回 DTG 死区编码（BDTR.DTG[7:0] 原始位段；含义见
    /// [`decode_dtg`]/[`encode_dtg`]，回读校验用）
    pub fn dtg(&self) -> u8 {
        (self.regs.bdtr.read() & at::BDTR_DTG_MASK) as u8
    }

    /// 读 (PSC, ARR) 配置（回读校验用；注意与 [`crate::timer`] 的
    /// `period_config` 返回序 (ARR, PSC) 相反——本驱动按手册约定返回
    /// 分频在前）
    pub fn period_config(&self) -> (u16, u16) {
        (
            (self.regs.psc.read() & 0xFFFF) as u16,
            (self.regs.arr.read() & 0xFFFF) as u16,
        )
    }

    /// 当前计数值（观测 CNT 在 0..ARR 间滚动 = PWM 在跑）
    pub fn count(&self) -> u16 {
        self.regs.cnt.read() as u16
    }

    /// 停止计数（保留 PSC/ARR/CCR/BDTR 配置；停了 `wait_update` 不再
    /// 返回——同 GTIM）
    pub fn stop(&mut self) {
        self.regs.cr1.modify(|v| v & !at::CR1_CEN);
    }

    /// 恢复计数（保留已配置的时基与输出参数）
    pub fn start(&mut self) {
        self.regs.cr1.modify(|v| v | at::CR1_CEN);
    }

    /// 异步等待一次更新事件（PWM 周期节拍；DIER.UIE 单发纪律）
    ///
    /// 每次调用先清掉残留 UIF（URS=0 下构造路径的 EGR_UG 会置 UIF），
    /// 再等**下一个**真实更新事件——返回时刻即一个 PWM 周期边界。等待
    /// 期间才开 DIER.UIE，IRQ 唤醒即压掉（防风暴）。
    ///
    /// 前置条件（模块文档详述）：ATIM 中断已 `bind_interrupts!` 绑定到
    /// [`AtimIrq`]；NVIC 线由本方法幂等使能。
    pub async fn wait_update(&mut self) {
        let regs = self.regs;
        crate::interrupt::nvic_enable::<interrupt::ATIM>();
        // 前置清残留 UIF：R1W0 写"全 1 除 bit0"（其余可清标志写 1 保持）
        regs.icr.write(icr_clear(at::ISR_UIF));
        poll_fn(|cx| {
            WAKER_UPDATE.register(cx.waker());
            if regs.isr.read() & at::ISR_UIF != 0 {
                regs.dier.modify(|v| v & !DIER_UIE); // 就绪即压掉
                Poll::Ready(())
            } else {
                regs.dier.modify(|v| v | DIER_UIE); // 等待期间才开
                Poll::Pending
            }
        })
        .await;
    }
}

// ================= 引脚复用 =================

/// PWM 输出引脚（同 `GtimPwm::new` 的 into_af_output 配方：清开漏/
/// 上下拉/模拟 → AF → DIR=0 输出）
fn config_output_pin(pin: &Pin<impl GpioPort>, af: u8) {
    let m = pin.mask();
    pin.port().mod_(Reg::OpEndrain, |v| v & !m);
    pin.port().mod_(Reg::Pur, |v| v & !m);
    pin.port().mod_(Reg::Pdr, |v| v & !m);
    pin.port().mod_(Reg::Analog, |v| v & !m); // 数字模式
    pin.set_af(af);
    pin.port().mod_(Reg::Dir, |v| v & !m); // 0 = 输出
}

/// 刹车输入引脚（BK 是**输入**，配方与输出脚不同，见模块文档）：
/// DIR=1 输入、保留调用方指定的上/下拉（外部上拉是低有效刹车的常规
/// 接法，不得清掉）、不清开漏（输入态无意义）
fn config_break_pin(pin: &Pin<impl GpioPort>, af: u8) {
    let m = pin.mask();
    pin.port().mod_(Reg::Analog, |v| v & !m); // 数字模式
    pin.set_af(af);
    pin.port().mod_(Reg::Dir, |v| v | m); // 1 = 输入（gpio.rs Reg::Dir 语义）
}

// ================= 中断派发 =================

/// per-事件 waker：UPDATE 供 [`AtimPwm::wait_update`]；COMMUT/BREAK 为
/// 预留（本驱动自身不开 COMIE/BIE，但 IRQ 里防御性压掉并唤醒，防止
/// 调用方手工开中断后的风暴）
static WAKER_UPDATE: AtomicWaker = AtomicWaker::new();
static WAKER_COMMUT: AtomicWaker = AtomicWaker::new();
static WAKER_BREAK: AtomicWaker = AtomicWaker::new();

/// ATIM 中断公共处理：查 UIF/COMIF/BIF，各自压掉对应 DIER 使能位并唤醒
/// （ISR 标志不动——poll 复查后由 wait_update 的前置 ICR 清除，同
/// timer.rs 纪律；BIF 由 `clear_break`/`is_braked` 软件侧处置）
fn on_irq(regs: &pac::Atim) {
    let isr = regs.isr.read();
    if isr & at::ISR_UIF != 0 {
        regs.dier.modify(|v| v & !DIER_UIE);
        WAKER_UPDATE.wake();
    }
    if isr & ISR_COMIF != 0 {
        regs.dier.modify(|v| v & !DIER_COMIE);
        WAKER_COMMUT.wake();
    }
    if isr & ISR_BIF != 0 {
        regs.dier.modify(|v| v & !DIER_BIE);
        WAKER_BREAK.wake();
    }
}

/// ATIM 中断标记（IRQ13，bind_interrupts! 绑定用）
pub struct AtimIrq;
impl IrqHandler<interrupt::ATIM> for AtimIrq {
    fn call() {
        on_irq(unsafe { &*(pac::ATIM_BASE as *const pac::Atim) });
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // ===== DTG 四段编码（手册 §16 BDTR.DTG[7:0]）=====

    #[test]
    fn dtg_segment1_direct_encoding() {
        // 段 1：0..=127 → 原值直编（步进 1）
        assert_eq!(dtg_bits(0), Some(0));
        assert_eq!(dtg_bits(1), Some(1));
        assert_eq!(dtg_bits(63), Some(63));
        assert_eq!(dtg_bits(127), Some(127));
        assert_eq!(decode_dtg(127), 127);
    }

    #[test]
    fn dtg_encode_segment_boundaries() {
        // 四段边界：127/128 | 191/192 | 223/224 | 255/256
        // （向上取整：奇数/非格点值进到下一格，绝不取不足值）
        let cases = [
            (127u32, 127u32), // 段 1 末格
            (128, 128),       // 段 2 首格：(64+0)×2
            (191, 192),       // 191 非格点 → (64+32)×2=192
            (192, 192),       // 192 恰为段 2 格点
            (223, 224),       // 223 → (64+48)×2=224
            (224, 224),       // 224 恰为段 2 格点
            (255, 256),       // 段 2 上限 254 → 255 落段 3 首格 (32+0)×8
            (256, 256),       // 段 3 首格
        ];
        for (ticks, expect) in cases {
            let bits = dtg_bits(ticks).unwrap();
            assert_eq!(decode_dtg(bits), expect, "ticks={ticks}");
            assert!(decode_dtg(bits) >= ticks, "编码不得取不足值 ticks={ticks}");
        }
    }

    #[test]
    fn dtg_encode_exhaustive_minimal_ceil() {
        // 全量 256 个 DTG 编码的可实现值：单调递增、互不重复、最大 1008
        let mut values = [0u32; 256];
        for (i, v) in values.iter_mut().enumerate() {
            *v = decode_dtg(i as u32);
        }
        assert_eq!(values[0], 0);
        assert_eq!(values[128], 128);
        assert_eq!(values[192], 256);
        assert_eq!(values[224], 512);
        assert_eq!(values[255], 1008);
        for w in values.windows(2) {
            assert!(w[0] < w[1], "可实现值应单调不重复");
        }
        // 对 0..=1100 的每个请求：编码结果 = 可实现值中 ≥ 请求的最小者
        // （向上取整且最小；不可编码区间 None）
        for t in 0..=1100u32 {
            let expect = values.iter().copied().find(|&v| v >= t);
            assert_eq!(dtg_bits(t).map(decode_dtg), expect, "ticks={t}");
        }
    }

    #[test]
    fn dtg_round_trip_all_256() {
        // 编码取最小规范形式：decode(b) 再 encode 回同一 b（256 个全覆盖）
        for b in 0..256u32 {
            assert_eq!(dtg_bits(decode_dtg(b)), Some(b), "bits={b}");
        }
    }

    #[test]
    fn dtg_over_limit_rejected() {
        // 上限 1008 tick；超过即拒绝（绝不饱和到不足值）
        assert_eq!(dtg_bits(504), Some(0xDF)); // (32+31)×8
        assert_eq!(dtg_bits(505), Some(0xE0)); // → (32+0)×16=512
        assert_eq!(dtg_bits(1008), Some(0xFF)); // (32+31)×16
        assert_eq!(dtg_bits(1009), None);
        assert_eq!(dtg_bits(u32::MAX), None);
    }

    #[test]
    fn dead_time_ns_to_ticks_ceil() {
        // 4MHz：1 tick = 250ns → 500ns=2、249ns=1、251ns=2（向上取整）
        assert_eq!(dead_time_ns_to_ticks(4_000_000, 0), 0);
        assert_eq!(dead_time_ns_to_ticks(4_000_000, 249), 1);
        assert_eq!(dead_time_ns_to_ticks(4_000_000, 250), 1);
        assert_eq!(dead_time_ns_to_ticks(4_000_000, 251), 2);
        assert_eq!(dead_time_ns_to_ticks(4_000_000, 500), 2);
        // 96MHz：1µs = 96 ticks
        assert_eq!(dead_time_ns_to_ticks(96_000_000, 1_000), 96);
    }

    #[test]
    fn dead_time_limit_is_freq_dependent() {
        // 死区上限按 f_DTS 折算：96MHz 下 1008 tick = 10.5µs
        assert_eq!(encode_dtg(96_000_000, 10_500).unwrap(), 0xFF);
        assert_eq!(
            encode_dtg(96_000_000, 10_600),
            Err(AtimError::DeadTimeTooLong)
        );
        // 4MHz 下 252µs = 1008 tick 恰可编码；零死区直编 0
        assert_eq!(encode_dtg(4_000_000, 252_000).unwrap(), 0xFF);
        assert_eq!(encode_dtg(4_000_000, 0).unwrap(), 0);
        assert_eq!(encode_dtg(4_000_000, 500).unwrap(), 2);
    }

    #[test]
    fn icr_clear_word_keeps_other_flags() {
        // ICR 为 R1W0（写 0 清、写 1 保持）：清某标志 = 全 1 除该位，
        // 与 PAC ICR_CLEAR_UIF 同套路；整字写 0 会误清其它标志
        let w = icr_clear(at::ISR_UIF);
        assert_eq!(w, at::ICR_CLEAR_UIF); // 与 PAC 已有常量逐位一致
        assert_eq!(w & at::ISR_UIF, 0); // 目标位写 0 → 清
        assert_eq!(w & ISR_COMIF, ISR_COMIF); // 其余标志写 1 → 保持
        assert_eq!(w & ISR_BIF, ISR_BIF);
        assert_eq!(w & ISR_CC1OF, ISR_CC1OF);

        let w = icr_clear(ISR_BIF);
        assert_eq!(w & ISR_BIF, 0); // 清 BIF
        assert_eq!(w & at::ISR_UIF, at::ISR_UIF); // UIF 保持
        assert_eq!(w & ISR_CC1IF, ISR_CC1IF);
        assert_eq!(w & ISR_CC4OF, ISR_CC4OF);
    }
}
