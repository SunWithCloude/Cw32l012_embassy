//! 经典家族（f030/f003/l031/r031/w031/l052/l083）ATIM 高级定时器：三路
//! 互补 PWM + 硬件死区（`AtimPwm`，`#[cfg(any(cw32f030, cw32f003,
//! cw32l031, cw32r031, cw32w031, cw32l052, cw32l083))]`）。
//!
//! 经典 ATIM 与 L012 的 ATIM 是**完全不同的 IP**（寄存器面无一处同构）。
//! 本模块覆盖经典代七家族（f020 无 ATIM）：f030（A22 先行落地）、
//! f003/l031/r031/w031（A26 扩展）、l052/l083（A27 扩展——每家族先过
//! **手册级语义核对**才进 cfg 名单，结论见下方家族差异表）。
//!
//! # 七家族同构性（PAC 逐家复核，crates.io 钉版 0.1.2）
//!
//! 各家族 PAC 与 f030 基线的 `pac::ATIM` 结构体逐字段同构（20 字段
//! arr/cnt/cr/isr/icr/mscr/fltr/trig/ch1cr..ch4cr + ch1ccra..ch3ccrb +
//! ch4ccr，基址同 0x4001_2C00，dtr@0x30）；ATIM 全部位常量子模块
//! （cr/dtr/icr/isr/fltr/ch1cr..ch4cr/mscr/trig/arr/cnt）l031/r031/w031
//! 与 f030 逐常量相同，f003 仅缺三处驱动不触碰的位常量（ch1cr.CDEB、
//! ch4cr.CDE、mscr.CCDS——位面仍在，本驱动对 mscr/ch4cr 整字写 0、
//! ch1cr 只用 BUFEA，不受影响）。APBEN2.ATIM 门控位七家族同在 bit7
//! （`pac::sysctrl::apben2::atim_MASK`=0x80，APBRST2 同位互证）；
//! 中断七家族同为 `ATIM = 13`。唯一 PAC 级差异：l052 给 MSCR 添
//! `msm@bit4`（l052 手册把该位标 RFU）——本驱动 MSCR 恒写 0，两种
//! 解释下行为相同。
//!
//! # 家族差异表（每格独立核对各自手册；页码 = 印刷页/PDF 页）
//!
//! | 维度 | f030 | f003 | l031 | r031 | w031 | l052 | l083 |
//! |---|---|---|---|---|---|---|---|
//! | ATIM 门控（APBEN2.ATIM） | bit7 | 同 | 同 | 同 | 同 | 同（cw32l052.h:7851） | 同（cw32l083.h:8425） |
//! | IRQ（`AtimIrq` 绑定名） | `ATIM`=13 | 同 | 同 | 同 | 同 | 同 | 同 |
//! | 引脚 AF 编号 | 全 AF7 | **全 AF4** | 全 AF7 | 全 AF7 | 全 AF7 | 全 AF7 | 全 AF7 |
//! | DTR 死区四段公式 + 范围表 | UM V2.5 p278/305 | UM V2.3 p212/238 | UM V1.6 p269/296 | UM V1.3 p272/299 | UM V1.4 p272/299 | UM V1.5 p306/333（表 16-9） | UM V2.0 p318/345（表 16-9） |
//! | ICR 写 0 清（复位 0x7_FFFF，保留 bit1） | UM V2.5 p300 | UM V2.3 p233 | UM V1.6 p291 | UM V1.3 p294 | UM V1.4 p294 | UM V1.5 p328 | UM V2.0 p340 |
//! | MODE=10 边沿对齐 / PRS"111：256" / PWM2S=1 单点 | UM V2.5 p296 | UM V2.3 p229 | UM V1.6 p287 | UM V1.3 p290 | UM V1.4 p290 | UM V1.5 p323-324 | UM V2.0 p335-336 |
//! | 互补配方（COMP=1、OCM=6、MOE/DTEN） | UM V2.5 p277 | UM V2.3 p211 | UM V1.6 p268 | UM V1.3 p271 | UM V1.4 p271 | UM V1.5 p305-306 | UM V2.0 p317-318 |
//! | CHxCR.BUFEA（bit6，CCRA 缓存） | PAC 位面同（A22 已核） | UM V2.3 p240 | UM V1.6 p298 | UM V1.3 p301 | UM V1.4 p301 | UM V1.5 p333 | UM V2.0 p345 |
//! | hal2 寄存器语义变体名 | atim_v1 | atim_cw32f003_v1 | atim_v1 | atim_v1 | atim_v1 | atim_cw32l052_v1 | atim_v1 |
//! | PAC 唯一差异 | — | 缺 3 个驱动不触碰的位常量 | — | — | — | MSCR 添 msm@bit4（手册标 RFU，本驱动恒写 0） | — |
//!
//! 结论：七家族 DTR 公式、ICR 语义、MODE/PWM2S 编码、PRS 档位、
//! MOE/DTEN 位号（DTR 位 12/9）、BUFEA 位号**逐字一致**——共用一套
//! 驱动本体，无家族分支；家族差异只剩引脚 AF 编号与可用路线（代码不
//! 分叉，AF 由调用方按引脚表传）。f003 的 `atim_cw32f003_v1` 变体名
//! 只是 hal2 证据链的转录口径（f003 手册独立成册），语义与 f030 相同。
//!
//! # 两代 ATIM IP 差异表
//!
//! | 维度 | L012 ATIM（STM32 风格） | 经典 ATIM（本模块，七家族） |
//! |---|---|---|
//! | 寄存器面 | CR1/SMCR/DIER/EGR/CCMR1-3/CCER/PSC/ARR/CCR1-6/BDTR/ICR | **单 CR**/MSCR/FLTR/TRIG/CHxCR/DTR/RCR/CHxCCRA+CCRB（PAC `pac::ATIM`，基址 0x4001_2C00） |
//! | 分频 | PSC[15:0] 线性 1..=65536 | CR.PRS[6:4] 3bit 档位 **1/2/4/8/16/32/64/256**（无 128 档！七家族手册 PRS 表均"111：256"） |
//! | 比较寄存器 | CCR1..6 | 每通道 CHxCCRA + CHxCCRB 双缓冲（本驱动只用 CCRA） |
//! | 互补输出 | CCER.CCxNE + BDTR.DTG | **COMP=1 + PWM2S=1**（CR 位 1/3），B 输出由硬件取 A 反相 + 死区 |
//! | 死区 | BDTR.DTG 四段编码 | DTR.DTR[7:0] 四段编码（2..1010 TCLK tick，见下） |
//! | 主输出使能 | BDTR.MOE | **DTR.MOE（位 12）**——在 DTR 里，不在 MSCR（MSCR 只有 MMS/CCDS/TS/SMS/IA1S/IB1S） |
//! | 更新事件 | EGR.UG（Wo 命令寄存器） | CR.UG（命令位）+ CR.URS（溢出才产生 UEV） |
//! | 中断标志清除 | ICR（Wo，R1W0） | ICR（RW，**写 0 清除**、写 1 无效、复位 0x7_FFFF） |
//!
//! # DTR 死区编码（经典四段，与 L012 BDTR.DTG 不同；七家族手册逐字同锚点）
//!
//! TCLK = PCLK / PRS 分频。f030 手册《CW32x030 UM V2.5》p278 公式 +
//! p305 范围表；f003/l031/r031/w031 各自手册同款锚点（页码见家族差异
//! 表），四段公式与范围逐字相同：
//!
//! | 码段（DTR[7:0]） | 死区（TCLK tick） | 范围 |
//! |---|---|---|
//! | 0..=127（0b0xxxxxxx） | `DTR[6:0] + 2` | 2 ~ 129 |
//! | 128..=191（0b10xxxxxx） | `(DTR[5:0] + 64) × 2 (+2)` | 130 ~ 256 |
//! | 192..=223（0b110xxxxx） | `(DTR[4:0] + 32) × 8 (+2)` | 258 ~ 506 |
//! | 224..=255（0b111xxxxx） | `(DTR[4:0] + 32) × 16 (+2)` | 514 ~ 1010 |
//!
//! 段 2..4 的基数 +2 由手册范围表反推（130 = 2×64+2、258 = 8×32+2、
//! 514 = 16×32+2，段边界逐值核对吻合）；各手册死区举例"63.25us"系
//! 同一排版误的四处翻版——按公式 514 tick @ 125ns TCLK = 64.25us（与
//! 各家范围表一致）。
//! ⚠ 上限 1010 tick：死区请求超出时返回 `AtimError::DeadTimeTooLong`，
//! 这与 L012 的 DTG 上限不同，用书时注意。
//!
//! # 双缓冲（影子/预装载）
//!
//! CR.BUFPEN=1 使 **ARR 预装载**、CHxCR.BUFEA=1 使 **CHxCCRA 预装载**：
//! 写入落在影子寄存器，更新事件（UEV，本驱动 URS=1 → 仅计数溢出）时装载
//! 进活动寄存器——与 L012 的 ARPE/OCxPE 语义对齐，改占空比"下一周期生效"，
//! 不截断当前脉冲。CHxCCRB 不参与互补比较（COMP=1 时 B 输出由硬件取 A 的
//! 反相 + 死区，七家族手册互补配方节同款原文），初始化写 0 后不再动；
//! 每通道的 B 侧预装载位 BUFEB 保持 0。
//!
//! # 引脚表（逐家族一行；SDK GPIO 宏 + 数据手册 AF 表双源一致才列出）
//!
//! | 家族 | AF | CH1A（主） | CH1B（互补） | CH2A（主） | CH2B（互补） | CH3A（主） | CH3B（互补） | 未列出路线 |
//! |---|---|---|---|---|---|---|---|---|
//! | f030 | 7 | PA5、PA8、PB2、PB5 | PA7、PA15、PB13 | PA4、PA9、PB6、PB10 | PB0、PB3、PB14 | PA3、PA10、PB7、PB11 | PB1、PB4、PB15 | — |
//! | f003 | 4 | PB6、PC0 | PA7、PB0、PB5 | PB3、PC1 | PA6、PB2、PC4 | PA0、PC2 | PA1、PA4、PC3 | —（PC3/PC4 仅 TSSOP24 封装引出，DS 表 5-2） |
//! | l031 | 7 | PA5、PA8、PB2、PB5 | PA7、PA15、PB13 | PA4、PA9、PB6、PB10 | PB0、PB3、PB14 | PA3、PA10、PB7、PB11 | PB1、PB4、PB15 | —（PB12..15 仅 LQFP48/QFN48；PB0/PB3/PB4/PB5 不在 QFN20/TSSOP20，DS 表 5-2） |
//! | r031 | 7 | PA5、PA8、PB2、PB5 | PA7、PA15、PB13 | PA4、PA9、PB6、PB10 | PB0、PB3、PB14 | PA10、PB7、PB11 | PB1、PB4、PB15 | CH3A@PA3：SDK 宏在而 QFN48 封装无 PA03 引脚（DS 表 5-2/5-3 反证）——**未获双源确认，不列** |
//! | w031 | 7 | PA5、PA8、PB2 | PA7 | PA4、PA9、PB10 | PB0、PB14 | PA3、PA10、PB7、PB11 | PB1、PB15 | CH1B@PA15/PB13、CH2A@PB6、CH2B@PB3、CH3B@PB4：SDK 宏在而 QFN64 封装无这些引脚（DS 表 5-2/5-4 反证）——**未获双源确认，不列** |
//! | l052 | 7 | PA5、PA8、PB2、PB5 | PA7、PA15、PB13 | PA4、PA9、PB6、PB10 | PB0、PB3、PB14 | PA3、PA10、PB7、PB11 | PB1、PB4、PB15 | —（DS V1.3 表 5-3/5-4/5-5 印刷页 30/31/32，29/29 双源全收） |
//! | l083 | 7 | PA5、PA8、PB2、PB5 | PA7、PA15、PB13 | PA4、PA9、PB6、PB10 | PB0、PB3、PB14 | PA3、PA10、PB7、PB11 | PB1、PB4、PB15 | CH2A@PE9：SDK 宏在而 DS V1.9 表 5-7（印刷页 37）功能 6=ATIM_BK、功能 7 空无 CH2A——**未获双源确认，不列**（44/45 双源收） |
//!
//! 双源出处：SDK GPIO 宏（`cw32f003_gpio.h`/`cw32l031_gpio.h`/
//! `cw32r031_gpio.h`/`cw32w031_gpio.h`/`cw32l052_gpio.h`/`cw32l083_gpio.h`，
//! 行号与 zip sha256 见 A26/A27 报告；
//! zip hash 与 hal2 证据链 `cw32-data/af/<家族>-atim-pwm.yaml` 逐字节
//! 对齐）+ 数据手册 AF 表（f003 DS V1.9 表 5-3/5-4/5-5 印刷页 24、PDF
//! 25；l031 DS V1.9 表 5-3/5-4 印刷页 28/29、PDF 29/30；r031 DS V1.2
//! 表 5-3/5-4 印刷页 31/32、PDF 32/33；w031 DS V1.3 表 5-3/5-4 印刷页
//! 30/31、PDF 31/32——"功能 N"列号即 AF 编号）。f030 沿用 A22 的
//! DS V1.9 表 5-3/5-4 + SDK 宏双源。封装可用性按各家 DS 表 5-2 逐脚
//! 核对。**宁缺毋滥**：仅 SDK 宏有而封装无引脚的路线一律不列（上表
//! "未列出路线"列）。
//!
//! # 安全边界（诚实标注，全部照实）
//!
//! - 构造完成后 **DTR.MOE=0**（主输出关），须显式 `AtimPwm::enable` 才
//!   有输出；MOE=0 时引脚电平状态无源可依（手册未定义"禁用"电平）。
//! - 只接受**内部比较值**（0 < compare < 周期）：0%/100% 端点在互补+死区
//!   下的波形无源可依，`AtimPwm::set_duty_bp` 拒绝（折算比较值为 0 或
//!   ≥ 周期即 `AtimError::BadDuty`，万分比 0/10000 必然落在端点）。
//! - 固定非反相极性（FLTR.CCPxA/B=0），不提供输出反相。
//! - **不使能任何刹车源**（DTR.BKE=0、CHxCR.BKSx=0、无刹车输入滤波）：
//!   `AtimPwm::is_braked` 读 ISR.BIF、`AtimPwm::clear_break` 写 ICR 清
//!   BIF，只是标志观测/清除原语——**不承诺**刹车后的 MOE 恢复序列、
//!   自动重启或重装（AOE 保持 0），故障重启语义留待后续有源可依时再做。
//! - 死区按**标称 PCLK** 向上取整（实际死区随真实 PCLK/HSI 容差浮动，
//!   本仓库 RCC 无 ClockBounds 包络数据，不做 ±容差保守包络）。
//! - 无逐对输出门（三对共享 MOE）、无逐对关断、无波形/板级/硅片验证。
//!
//! # 状态
//!
//! **compiles-not-run**：七家族 thumbv6m 全 feature 矩阵（clippy/build/
//! test）全绿；三相互补波形、死区宽度、刹车标志路径均**未上板验证**。
//! 硬件语义（CR/FLTR/CHxCR/DTR 配方、UG 装载序列、ICR 写 0 清）对照
//! 各家族厂商手册逐家核对（f030：《CW32x030 UM V2.5》p277/278/296/
//! 300/305；f003/l031/r031/w031 页码见家族差异表——只取手册硬件语义，
//! 实现为本仓 whole-register 风格）。
//!
//! # 用法（f030，CH1A@PA8 + CH1B@PB13，全部 AF7）
//!
//! ```ignore
//! embassy_cw32::bind_interrupts!(struct Irqs {
//!     ATIM => embassy_cw32::atim_classic::AtimIrq;
//! });
//! let mut pwm = AtimPwm::new(
//!     p.atim,
//!     Some(AtimPair::new(Pin::new(p.gpioa, 8), 7, Pin::new(p.gpiob, 13), 7)),
//!     None,
//!     None,
//!     AtimPwmConfig { freq_hz: 20_000, dead_time_ns: 500, duty_bp: 1_000 },
//! )?;
//! pwm.enable(); // DTR.MOE=1，三对输出一起开
//! pwm.set_duty_bp(AtimChannel::Ch1, 5_000)?; // 影子写入，下一 UEV 生效
//! pwm.wait_update().await;                    // 周期节拍（IRQ13）
//! ```
//!
//! l031 三相互补完整示例见 `examples/atim_motor_l031.rs`（引脚取上表
//! l031 行，LQFP48/QFN48 封装）。

// ================= 纯数学核心（家族无关：默认 cargo test 直接覆盖） =================
//
// 经典 ATIM 的分频档表 / DTR 死区四段编码 / 占空比→内部比较值映射是手册
// 硬件语义，独立于 PAC（家族 feature 关闭时也编译，host 测试在此直接跑）。
// 同 timer.rs 的 calc_pwm_pow2_div 先例：pub 函数 + 宿主机单元测试。

/// 经典 ATIM DTR 死区编码 → 死区 TCLK tick 数（四段）。
///
/// 手册《CW32x030 UM V2.5》p278 公式 + p305 范围表（段边界逐值核对）：
/// 段 1 `DTR[6:0]+2`（2..129）；段 2 `(DTR[5:0]+64)×2+2`（130..256）；
/// 段 3 `(DTR[4:0]+32)×8+2`（258..506）；段 4 `(DTR[4:0]+32)×16+2`
/// （514..1010）。编码 0 即 2 tick（DTEN 使能时的最小死区）。
/// f003/l031/r031/w031 手册同款四段公式/范围（页码见模块头家族差异
/// 表），七家族共用本函数。
pub fn dead_time_ticks(code: u8) -> u32 {
    2 + match code {
        0..=127 => u32::from(code),
        128..=191 => (64 + u32::from(code & 63)) * 2,
        192..=223 => (32 + u32::from(code & 31)) * 8,
        _ => (32 + u32::from(code & 31)) * 16,
    }
}

/// 死区编码下限搜索：满足 `dead_time_ticks(code) >= ticks_min` 的**最小**
/// 编码（0..=255）；超过上限 1010 tick 返回 None。
pub fn dead_time_code_min(ticks_min: u32) -> Option<u8> {
    (0..=u8::MAX).find(|&code| dead_time_ticks(code) >= ticks_min)
}

/// 请求死区（ns）→ 编码，按给定 TCLK 频率**向上取整**（实际死区 ≥ 请求）。
///
/// `tclk_hz` = PCLK / PRS 分频。返回 None 表示超出经典编码上限
/// （1010 TCLK tick）。`dead_time_ns == 0` 纯数学上返回 `Some(0)`，
/// 调用方（`AtimPwm::new`）应把 0 解释为"关死区"（DTEN=0）而非编码 0。
pub fn dead_time_code_for_ns(dead_time_ns: u32, tclk_hz: u32) -> Option<u8> {
    // ceil(ns × tclk / 1e9)；u64 乘法上界 4.3e9 × 1e8 << u64::MAX
    let ticks_min = (u64::from(dead_time_ns) * u64::from(tclk_hz)).div_ceil(1_000_000_000);
    dead_time_code_min(ticks_min as u32)
}

/// 经典 ATIM CR.PRS 档位 → 分频系数。
///
/// 档位表**不是** 2^n 全序列：000..110 = 1/2/4/8/16/32/64，**111 = 256**
/// （手册 PRS 表锚点"111：256"）。没有 128 档——从 64 直接跳 256。
/// （任务草稿"0..7→1..128"有误，按手册锚点修正。）
pub fn prs_divisor(code: u32) -> u32 {
    match code {
        0..=6 => 1 << code,
        _ => 256,
    }
}

/// 经典 ATIM 的 (PRS 档位, ARR) 计算：档位取**最小可行档**使周期
/// `PCLK/(分频×(ARR+1))` 尽量量化细（四舍五入同 [`crate::timer::calc_pwm_pow2_div`]）。
///
/// `f > PCLK/2`（任何档位下周期 < 2 tick）或 `f < PCLK/(256×65536)`
/// （任何档位下周期爆表）返回 None。ARR 上限 0xFFFF（16bit 计数器）。
pub fn calc_prs_arr(pclk_hz: u32, freq_hz: u32) -> Option<(u32, u16)> {
    if freq_hz == 0 || pclk_hz == 0 {
        return None;
    }
    for code in 0..=7u32 {
        let divisor = prs_divisor(code);
        let ck_cnt = pclk_hz / divisor;
        let ticks = (ck_cnt as u64 + freq_hz as u64 / 2) / freq_hz as u64;
        if (2..=65_536).contains(&ticks) {
            return Some((code, (ticks - 1) as u16));
        }
    }
    None
}

/// 万分比占空比 → 比较值。**仅内部值**：折算结果须满足
/// `0 < compare < period`（互补 + 死区下端点波形无源可依，拒绝端点），
/// 否则 None。
pub fn duty_compare(period: u32, duty_bp: u16) -> Option<u16> {
    if duty_bp > 10_000 {
        return None;
    }
    let compare = period * u32::from(duty_bp) / 10_000;
    if compare == 0 || compare >= period {
        return None;
    }
    // compare < period ≤ 65536 → 必然落入 u16
    Some(compare as u16)
}

#[cfg(test)]
mod calc_tests {
    use super::*;

    // ===== DTR 四段边界（手册 p305 范围表逐段端点） =====

    #[test]
    fn dtr_segment1_2_to_129() {
        assert_eq!(dead_time_ticks(0), 2); // 编码 0 = 2 tick（DTEN 使能最小值）
        assert_eq!(dead_time_ticks(1), 3);
        assert_eq!(dead_time_ticks(127), 129); // 段 1 上端：DTR[6:0]+2 = 127+2
    }

    #[test]
    fn dtr_segment2_130_to_256() {
        assert_eq!(dead_time_ticks(128), 130); // 段 2 下端：(0+64)×2+2
        assert_eq!(dead_time_ticks(129), 132);
        assert_eq!(dead_time_ticks(191), 256); // 段 2 上端：(63+64)×2+2
    }

    #[test]
    fn dtr_segment3_258_to_506() {
        assert_eq!(dead_time_ticks(192), 258); // 段 3 下端：(0+32)×8+2
        assert_eq!(dead_time_ticks(223), 506); // 段 3 上端：(31+32)×8+2
    }

    #[test]
    fn dtr_segment4_514_to_1010() {
        assert_eq!(dead_time_ticks(224), 514); // 段 4 下端：(0+32)×16+2
        assert_eq!(dead_time_ticks(255), 1010); // 段 4 上端（经典编码极限）
    }

    #[test]
    fn dtr_strictly_monotonic() {
        // 四段交接处 129<130、256<258、506<514，全程严格递增
        let mut prev = dead_time_ticks(0);
        for code in 1..=255u8 {
            let t = dead_time_ticks(code);
            assert!(t > prev, "code {code} 死区 {t} 未递增于 {prev}");
            prev = t;
        }
    }

    // ===== 编码下限搜索（≥ ticks_min 的最小编码） =====

    #[test]
    fn code_min_segment_edges() {
        assert_eq!(dead_time_code_min(0), Some(0));
        assert_eq!(dead_time_code_min(1), Some(0)); // 编码 0 即 2 tick
        assert_eq!(dead_time_code_min(2), Some(0));
        assert_eq!(dead_time_code_min(3), Some(1));
        assert_eq!(dead_time_code_min(129), Some(127));
        assert_eq!(dead_time_code_min(130), Some(128)); // 段 2 交接
        assert_eq!(dead_time_code_min(131), Some(129)); // 130 不够、132 够
        assert_eq!(dead_time_code_min(256), Some(191));
        assert_eq!(dead_time_code_min(257), Some(192)); // 段 3 交接
        assert_eq!(dead_time_code_min(506), Some(223));
        assert_eq!(dead_time_code_min(507), Some(224)); // 段 4 交接
        assert_eq!(dead_time_code_min(1010), Some(255));
        assert_eq!(dead_time_code_min(1011), None); // 超上限
    }

    #[test]
    fn ns_round_up_at_8mhz() {
        // f030 复位默认 PCLK=8MHz：1 tick = 125ns
        assert_eq!(dead_time_code_for_ns(1, 8_000_000), Some(0)); // 不足 1 tick → 编码 0（2 tick）
        assert_eq!(dead_time_code_for_ns(161, 8_000_000), Some(0)); // ceil(1.288)=2 → 编码 0
        assert_eq!(dead_time_code_for_ns(250, 8_000_000), Some(0)); // 恰好 2 tick
        assert_eq!(dead_time_code_for_ns(251, 8_000_000), Some(1)); // ceil(2.008)=3
        assert_eq!(dead_time_code_for_ns(500, 8_000_000), Some(2)); // 4 tick
        assert_eq!(dead_time_code_for_ns(1_000, 8_000_000), Some(6)); // 8 tick
        assert_eq!(dead_time_code_for_ns(1_001, 8_000_000), Some(7)); // ceil(8.008)=9
        assert_eq!(dead_time_code_for_ns(9_999, 8_000_000), Some(78)); // ceil(79.99)=80
        // 8MHz 下 1010 tick = 126.25us；再往上无编码
        assert_eq!(dead_time_code_for_ns(126_250, 8_000_000), Some(255));
        assert_eq!(dead_time_code_for_ns(126_251, 8_000_000), None);
    }

    #[test]
    fn evidence_json_example() {
        // hal2 证据 JSON 样例：PCLK 上界 24.48MHz、PRS=1、请求 1000ns
        // → ceil(24.48)=25 tick → 编码 23（保守包络按最大时钟算；本驱动
        // 只有标称 PCLK，24.00MHz 下会取编码 22，差异已在模块文档注明）
        assert_eq!(dead_time_code_for_ns(1_000, 24_480_000), Some(23));
        assert_eq!(dead_time_ticks(23), 25);
    }

    // ===== PRS 档位表（111=256，无 128 档） =====

    #[test]
    fn prs_table_no_128_step() {
        assert_eq!(prs_divisor(0), 1);
        assert_eq!(prs_divisor(6), 64);
        assert_eq!(prs_divisor(7), 256); // 手册"111：256"
    }

    #[test]
    fn calc_1khz_at_8mhz() {
        // 8M/1k = 8000 ticks ≤ 65536 → 档位 0（分频 1），ARR=7999
        assert_eq!(calc_prs_arr(8_000_000, 1_000), Some((0, 7999)));
    }

    #[test]
    fn calc_slow_needs_256_step() {
        // 1Hz@8MHz：档位 6（分频 64）→ 125000 ticks 爆表；下一档即 256
        //（无 128 档！）→ 8M/256/1 = 31250 ticks，ARR=31249
        let (code, arr) = calc_prs_arr(8_000_000, 1).unwrap();
        assert_eq!(code, 7);
        assert_eq!(arr, 31_249);
        let f = 8_000_000u64 / 256 / (arr as u64 + 1);
        assert_eq!(f, 1);
    }

    #[test]
    fn calc_out_of_range() {
        // 上界：四舍五入后周期 < 2 tick 才拒绝——6MHz@8M 折算 1 tick
        assert!(calc_prs_arr(8_000_000, 6_000_000).is_none());
        // 恰好 PCLK/2 → 2 tick；比 PCLK/2 高 1Hz 的频率四舍五入仍落
        // 2 tick（量化后恰为 PCLK/2，误差 <1Hz——约定同 calc_pwm_pow2_div）
        assert_eq!(calc_prs_arr(8_000_000, 4_000_000), Some((0, 1)));
        assert_eq!(calc_prs_arr(8_000_000, 4_000_001), Some((0, 1)));
        // f=0 / pclk=0 直接拒绝
        assert!(calc_prs_arr(8_000_000, 0).is_none());
        assert!(calc_prs_arr(0, 1_000).is_none());
        // 下界：f < PCLK/(256×65536) = 5.72Hz@96MHz——96MHz 下 1/5Hz 连
        // 256 分频档都爆表（375000/75000 ticks > 65536），6Hz 恰可行
        //（62500 ticks，ARR=62499）
        assert!(calc_prs_arr(96_000_000, 1).is_none());
        assert!(calc_prs_arr(96_000_000, 5).is_none());
        assert_eq!(calc_prs_arr(96_000_000, 6), Some((7, 62_499)));
    }

    // ===== 占空比 → 内部比较值 =====

    #[test]
    fn duty_interior_only() {
        // 周期 400（20kHz@8MHz）：50% → 200
        assert_eq!(duty_compare(400, 5_000), Some(200));
        assert_eq!(duty_compare(400, 1_000), Some(40));
        // 端点拒绝：0% 折算 0、100% 折算 400 = 周期
        assert_eq!(duty_compare(400, 0), None);
        assert_eq!(duty_compare(400, 10_000), None);
        assert_eq!(duty_compare(400, 10_001), None);
        // 极小万分比四舍五入到 0 也拒绝（内部值约束）
        assert_eq!(duty_compare(400, 2), None); // 400×2/10000 = 0
        assert_eq!(duty_compare(400, 25), Some(1)); // 400×25/10000 = 1
        // 极大万分比折算到周期也拒绝
        assert_eq!(duty_compare(400, 9_999), Some(399)); // 399 < 400 ✓
        // 最小周期 2：唯一内部比较值是 1（万分比 5000..=9999 都折算到 1）
        assert_eq!(duty_compare(2, 5_000), Some(1));
        assert_eq!(duty_compare(2, 9_999), Some(1));
        assert_eq!(duty_compare(2, 4_999), None); // 折算 0 → 端点
        assert_eq!(duty_compare(2, 10_000), None); // 折算 2 = 周期 → 端点
    }
}

// ================= 驱动本体（七家族共用：位面/门控/语义逐家族核对一致） =================

#[cfg(any(
    feature = "cw32f030",
    feature = "cw32f003",
    feature = "cw32l031",
    feature = "cw32r031",
    feature = "cw32w031",
    feature = "cw32l052",
    feature = "cw32l083",
))]
mod classic {
    use core::future::poll_fn;
    use core::sync::atomic::Ordering;
    use core::task::Poll;

    use embassy_sync::waitqueue::AtomicWaker;

    use super::{calc_prs_arr, dead_time_code_for_ns, dead_time_ticks, duty_compare, prs_divisor};
    use crate::gpio::{GpioPort, Pin, Reg};
    use crate::interrupt::{IrqHandler, interrupt};
    use crate::pac;
    // 经典 PAC 是生成器产物：Rw<u32> 整字寄存器 + 小写字段常量模块
    //（pac::atim::cr::prs_POS 等），全字写、字段常量取位面
    use crate::pac::atim::{ch1cr, cr, dtr, fltr, icr, isr};

    // ================= 位面补遗（PAC 缺常量，本地定义） =================

    /// ICR 全 1 无操作值：ISR 全部 18 个标志位（bit0..18）+ 保留 bit1。
    /// 经典 ATIM ICR 为 RW"写 0 清除、写 1 无效"，复位 0x7_FFFF——只清
    /// 目标标志时其余位写 1 保全（含复位值 1 的保留 bit1）。
    const ICR_NOOP: u32 = 0x0007_FFFF;
    /// ICR 保留 bit1（复位值 1，写 1 保全；"全清"配方即整字写 0b10）
    const ICR_RESERVED1: u32 = 0b10;

    /// CR 读回保留掩码：仅 defined 配置位。CR 是单寄存器且 TG/UG/BG 是
    /// 写触发的命令位（读 0 但绝不可经读回复写复放）、保留位不可预期——
    /// 运行期对 CR 的一切改动都走"读回 → 本掩码过滤 → 整字写回"。
    /// 位集合与手册 CR 位面（EN/COMP/CT/PWM2S/PRS/BUFPEN/UIE/MODE/
    /// ONESHOT/OCCS/URS/TIE/BIE/CISA/OCCE/DIR/OVE/UNDE）一致。
    const CR_KEEP_MASK: u32 = cr::en_MASK
        | cr::comp_MASK
        | cr::ct_MASK
        | cr::pwm2s_MASK
        | cr::prs_MASK
        | cr::bufpen_MASK
        | cr::uie_MASK
        | cr::mode_MASK
        | cr::oneshot_MASK
        | cr::occs_MASK
        | cr::urs_MASK
        | cr::tie_MASK
        | cr::bie_MASK
        | cr::cisa_MASK
        | cr::occe_MASK
        | cr::dir_MASK
        | cr::ove_MASK
        | cr::unde_MASK;

    /// CR 运行期安全改写（命令位/保留位绝不复放）：读回 → 过滤 → 或入 wanted
    fn cr_keep_or(regs: &pac::ATIM, wanted: u32) {
        regs.cr.write((regs.cr.read() & CR_KEEP_MASK) | wanted);
    }

    /// CR 运行期安全改写（清 cleared 位）
    fn cr_keep_and(regs: &pac::ATIM, cleared: u32) {
        regs.cr.write(regs.cr.read() & CR_KEEP_MASK & !cleared);
    }

    // ================= 寄存器字构造（全字写） =================

    /// 互补 PWM 的 CR 控制字（EN/UIE 由运行期安全改写叠加）。
    ///
    /// 配方（手册 p277"三路互补PWM"+ p295-297）：COMP=1（互补输出）、
    /// PWM2S=1、MODE=10（边沿对齐，手册锚点"10：边沿对齐模式"）、
    /// BUFPEN=1（ARR 预装载）、URS=1（UEV 仅溢出产生）、PRS 档位。
    fn control_word(prs_code: u32) -> u32 {
        cr::comp_MASK
            | cr::pwm2s_MASK
            | ((prs_code << cr::prs_POS) & cr::prs_MASK)
            | cr::bufpen_MASK
            | (2 << cr::mode_POS)
            | cr::urs_MASK
    }

    /// DTR 字：死区码 + DTEN；BKE/AOE/MOE/SAFEEN/VCE 保持复位 0
    ///（主输出关、无刹车源、无自动恢复——安全边界见模块文档）。
    fn dtr_word(dead_time: Option<u8>) -> u32 {
        match dead_time {
            Some(code) => (u32::from(code) & dtr::dtr_MASK) | dtr::dten_MASK,
            None => 0,
        }
    }

    /// FLTR 字：三路 A 通道 OCM=110（PWM 模式 1：CNT<CCR 输出有效；
    /// 手册 FLTR 表与 STM32 CCMR.OCxM 同编码）、CCPxA/CCPxB=0（非反相）、
    /// 无输出滤波、无刹车滤波/极性配置。
    fn fltr_word() -> u32 {
        (6 << fltr::ocm1aflt1a_POS) | (6 << fltr::ocm2aflt2a_POS) | (6 << fltr::ocm3aflt3a_POS)
    }

    /// CHxCR 字（CH1/2/3 同布局）：仅 BUFEA=1——CCRA 预装载（影子在
    /// UEV 装载）；CSA/CSB=0 输出比较、CIEA/CIEB/CDEA/CDEB 中断/DMA 关、
    /// BKSA/BKSB 刹车源 0（不使能刹车）、BUFEB=0（B 侧不参与比较）。
    fn chxcr_word() -> u32 {
        ch1cr::bufea_MASK
    }

    // ================= API 类型 =================

    /// ATIM 互补 PWM 通道（CH1..CH3；CH4 为内部通道无引出、无互补，不暴露）
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub enum AtimChannel {
        /// 通道 1（CH1A/CH1B 对）
        Ch1,
        /// 通道 2（CH2A/CH2B 对）
        Ch2,
        /// 通道 3（CH3A/CH3B 对）
        Ch3,
    }

    impl AtimChannel {
        /// 通道序（内部用：0..2 = CH1..CH3）
        fn index(self) -> usize {
            match self {
                AtimChannel::Ch1 => 0,
                AtimChannel::Ch2 => 1,
                AtimChannel::Ch3 => 2,
            }
        }
    }

    /// 构造参数（频率/死区/初始占空比构造期固定，见 [`AtimPwmConfig`] 各字段）
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub struct AtimPwmConfig {
        /// PWM 频率（Hz）。上界 PCLK/2，下界 PCLK/(256×65536)
        /// （PRS 档位无 128、直接跳 256，见 [`prs_divisor`]）
        pub freq_hz: u32,
        /// 对称死区下限（ns，按标称 PCLK 向上取整到经典四段编码，
        /// 实际死区 ≥ 请求）。0 = 关死区（DTEN=0）。上限 1010 TCLK tick
        /// （8MHz 时约 126us），超出返回 [`AtimError::DeadTimeTooLong`]
        pub dead_time_ns: u32,
        /// 初始占空比（万分比 0..=10000，三路同值）。仅接受内部值
        /// （0%/100% 端点拒绝，见模块文档安全边界）
        pub duty_bp: u16,
    }

    /// 配置/运行错误
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub enum AtimError {
        /// 频率超出可实现范围（上界 PCLK/2，下界 PCLK/(256×65536)）
        OutOfRange,
        /// 占空比非法（>10000，或折算比较值不是内部值 0<compare<period）
        BadDuty,
        /// 死区超过经典四段编码上限（1010 TCLK tick）
        DeadTimeTooLong,
        /// 三对互补引脚一对都没提供（至少要一对）
        NoPairs,
        /// 请求的通道未接管（该对引脚未提供）
        ChannelUnavailable,
    }

    impl embedded_hal::pwm::Error for AtimError {
        fn kind(&self) -> embedded_hal::pwm::ErrorKind {
            embedded_hal::pwm::ErrorKind::Other
        }
    }

    /// 一对同通道互补引脚（CHxA 主 + CHxB 互补；AF 按家族引脚表：
    /// f030/l031/r031/w031 全 AF7、f003 全 AF4，见模块文档引脚表）。
    ///
    /// 构造即接管引脚：`AtimPwm::new` 内部配为 AF 复用输出（清模拟/开漏/
    /// 上下拉、DIR=输出、set_af）。引脚接错通道编译期不查（本仓 gpio 是
    /// 运行期通用 Pin），接错只会上不了波形——对照模块文档引脚表。
    pub struct AtimPair<P: GpioPort, Q: GpioPort> {
        /// 主输出引脚（CHxA）
        pub main: Pin<P>,
        /// 主输出 AF 编号（按家族引脚表：f003=4，其余家族=7）
        pub main_af: u8,
        /// 互补输出引脚（CHxB）
        pub comp: Pin<Q>,
        /// 互补输出 AF 编号（按家族引脚表：f003=4，其余家族=7）
        pub comp_af: u8,
    }

    impl<P: GpioPort, Q: GpioPort> AtimPair<P, Q> {
        /// 组一对同通道的 A/B 引脚
        pub fn new(main: Pin<P>, main_af: u8, comp: Pin<Q>, comp_af: u8) -> Self {
            Self {
                main,
                main_af,
                comp,
                comp_af,
            }
        }
    }

    /// 把一个引脚配成 AF 复用输出（同 ClassicPwm 的引脚配方：
    /// 清开漏/上下拉/模拟，set_af，DIR=0 输出）
    fn connect_af<P: GpioPort>(pin: Pin<P>, af: u8) {
        let m = pin.mask();
        pin.port().mod_(Reg::OpEndrain, |v| v & !m);
        pin.port().mod_(Reg::Pur, |v| v & !m);
        pin.port().mod_(Reg::Pdr, |v| v & !m);
        pin.port().mod_(Reg::Analog, |v| v & !m); // 数字模式
        pin.set_af(af);
        pin.port().mod_(Reg::Dir, |v| v & !m); // 0 = 输出
    }

    // ================= 驱动本体 =================

    /// 经典 ATIM 三相互补 PWM（whole-ATIM 独占所有者，七家族每家单实例）
    ///
    /// 构造即完成门控/分频/ARR/死区/输出模式并**启动计数**，但
    /// **DTR.MOE=0（无输出）**——三对引脚已接 AF，输出使能须显式
    /// [`AtimPwm::enable`]（安全边界见模块文档）。运行期 CR/DTR 改动
    /// 全部走"读回 → defined 位过滤 → 整字写回"（命令位 TG/UG/BG 与
    /// 保留位绝不复放）。
    pub struct AtimPwm {
        regs: &'static pac::ATIM,
        pairs: [bool; 3],
        period_ticks: u32,
        dead_time: Option<u8>,
    }

    impl AtimPwm {
        /// 初始化 ATIM 并启动计数（输出默认关，MOE=0）
        ///
        /// * `regs` — ATIM 寄存器块（`p.atim`；七家族均单实例）
        /// * `ch1`/`ch2`/`ch3` — 三对互补引脚，缺哪对传 `None`
        ///   （封装没有对应引脚或用不到；至少要一对）
        /// * `config` — 频率/死区/初始占空比
        ///
        /// 全部校验先于寄存器/门控改动（失败时 ATIM 与时钟门控原样）。
        pub fn new<
            P1: GpioPort,
            Q1: GpioPort,
            P2: GpioPort,
            Q2: GpioPort,
            P3: GpioPort,
            Q3: GpioPort,
        >(
            regs: &'static pac::ATIM,
            ch1: Option<AtimPair<P1, Q1>>,
            ch2: Option<AtimPair<P2, Q2>>,
            ch3: Option<AtimPair<P3, Q3>>,
            config: AtimPwmConfig,
        ) -> Result<Self, AtimError> {
            let pairs = [ch1.is_some(), ch2.is_some(), ch3.is_some()];
            if !pairs.iter().any(|&p| p) {
                return Err(AtimError::NoPairs);
            }
            let pclk_hz = crate::PCLK_HZ.load(Ordering::Relaxed);
            let (prs_code, arr) =
                calc_prs_arr(pclk_hz, config.freq_hz).ok_or(AtimError::OutOfRange)?;
            let period_ticks = u32::from(arr) + 1;
            let compare = duty_compare(period_ticks, config.duty_bp).ok_or(AtimError::BadDuty)?;
            // 死区：TCLK = PCLK/PRS 分频，ns 向上取整到最小编码（0 = 关 DTEN）
            let dead_time = match config.dead_time_ns {
                0 => None,
                ns => {
                    let tclk_hz = pclk_hz / prs_divisor(prs_code);
                    Some(dead_time_code_for_ns(ns, tclk_hz).ok_or(AtimError::DeadTimeTooLong)?)
                }
            };

            // 时钟门控：经典代无 KEY，读改写。ATIM 门位 = APBEN2 bit7
            //（七家族 PAC 同位：pac::sysctrl::apben2::atim_MASK=0x80，
            // APBRST2 同位互证——七家族 PAC apbrst2::atim 亦 bit7）
            let sysctrl = unsafe { &*(pac::SYSCTRL_BASE as *const crate::Sysctrl) };
            sysctrl
                .apben2
                .modify(|v| v | pac::sysctrl::apben2::atim_MASK);

            // 寄存器配方（顺序对照厂商库 ATIM_PWMOutputConfig + 手册
            // p277/295-297/300/305；全字写，不读改写）
            let control = control_word(prs_code);
            regs.dtr.write(dtr_word(dead_time)); // MOE=0：输出先关
            regs.mscr.write(0); // 无主/从/级联请求
            regs.trig.write(0); // 无 ADC 触发
            regs.rcr.write(0); // 重复计数 0：每次溢出都产生 UEV
            regs.fltr.write(fltr_word()); // 三路 A：OCM=110(PWM1)、非反相
            regs.ch1cr.write(chxcr_word()); // BUFEA=1：CCRA 预装载
            regs.ch2cr.write(chxcr_word());
            regs.ch3cr.write(chxcr_word());
            regs.ch4cr.write(0); // CH4 内部通道，不暴露
            regs.ch4ccr.write(0);
            regs.ch1ccrb.write(0); // 互补模式 B 由硬件取 A 反相+死区，CCRB 不用
            regs.ch2ccrb.write(0);
            regs.ch3ccrb.write(0);
            regs.ch1ccra.write(u32::from(compare)); // 初始比较值进影子
            regs.ch2ccra.write(u32::from(compare));
            regs.ch3ccra.write(u32::from(compare));
            regs.arr.write(u32::from(arr));
            regs.cnt.write(0);
            // 软件更新事件：URS 暂时清 0 + UG=1 触发一次 UEV，把 ARR/CCRA
            // 影子装载进活动寄存器；命令位写完回读一次再重写干净值（命令位
            // 不复放；URS=1 时软件 UG 不产生 UEV，故须临时清）
            regs.cr.write((control & !cr::urs_MASK) | cr::ug_MASK);
            let _ = regs.cr.read();
            regs.cr.write(control);
            // ICR 写 0 清除全部事件标志（含 UG 产生的 UIF；保留 bit1 写 1 保全）
            regs.icr.write(ICR_RESERVED1);

            // 引脚复用（全部寄存器配好、MOE=0 之后再接 pad）
            if let Some(pair) = ch1 {
                connect_af(pair.main, pair.main_af);
                connect_af(pair.comp, pair.comp_af);
            }
            if let Some(pair) = ch2 {
                connect_af(pair.main, pair.main_af);
                connect_af(pair.comp, pair.comp_af);
            }
            if let Some(pair) = ch3 {
                connect_af(pair.main, pair.main_af);
                connect_af(pair.comp, pair.comp_af);
            }

            // 启动计数（EN=1；输出仍关，须显式 enable()）
            regs.cr.write(control | cr::en_MASK);
            Ok(Self {
                regs,
                pairs,
                period_ticks,
                dead_time,
            })
        }

        /// 三对主输出使能（DTR.MOE=1）
        ///
        /// MOE=0 时输出状态无源可依（手册未定义"禁用"电平）——电机类
        /// 负载请在确认功率级安全后再调用。
        pub fn enable(&mut self) {
            // DTR 全部是普通配置位（无命令位），读改写安全
            self.regs.dtr.modify(|v| v | dtr::moe_MASK);
        }

        /// 三对主输出关断（DTR.MOE=0）。不承诺引脚电平；也不承诺刹车后
        /// 的恢复序列——故障重启语义见模块文档安全边界。
        pub fn disable(&mut self) {
            self.regs.dtr.modify(|v| v & !dtr::moe_MASK);
        }

        /// 停止计数（CR.EN=0，保留分频/ARR/CCRA/死区配置；输出若使能则
        /// 冻结在当前比较态）。安全起见电机类应用先 [`AtimPwm::disable`]
        /// 再 stop。
        pub fn stop(&mut self) {
            cr_keep_and(self.regs, cr::en_MASK);
        }

        /// 恢复计数（CR.EN=1）
        pub fn start(&mut self) {
            cr_keep_or(self.regs, cr::en_MASK);
        }

        /// 刹车标志观测（ISR.BIF）
        ///
        /// ⚠ 本驱动构造不使能任何刹车源（BKE=0），BIF 不会由硬件置位；
        /// 本方法只是标志原语（供自行配置刹车源后观测），**不承诺**刹车
        /// 后的 MOE 恢复/自动重启语义（见模块文档）。
        pub fn is_braked(&self) -> bool {
            self.regs.isr.read() & isr::bif_MASK != 0
        }

        /// 清除刹车标志（ICR.BIF 写 0 清除，其余位写 1 保全）
        ///
        /// 只清标志，不动 MOE、不重启输出——重开输出须显式
        /// [`AtimPwm::enable`]（无源可依的自动恢复不做）。
        pub fn clear_break(&mut self) {
            self.regs.icr.write(ICR_NOOP & !icr::bif_MASK);
        }

        /// 读回 (ARR, PRS 档位原始值)（回读校验用）
        ///
        /// PRS 档位原始值 0..7，分频 = [`prs_divisor`]（0..6 = 2^n、
        /// 7 = 256，无 128 档）
        pub fn period_config(&self) -> (u16, u16) {
            let prs_raw = ((self.regs.cr.read() & cr::prs_MASK) >> cr::prs_POS) as u16;
            ((self.regs.arr.read() & 0xFFFF) as u16, prs_raw)
        }

        /// 周期 tick 数（ARR+1；也是占空比满量程的硬件刻度）
        pub fn period_ticks(&self) -> u32 {
            self.period_ticks
        }

        /// 满量程（万分比，同 [`crate::timer::ClassicPwm`] 约定）
        pub fn max_duty(&self) -> u16 {
            10_000
        }

        /// 该通道的互补对是否被本实例接管
        pub fn has_channel(&self, ch: AtimChannel) -> bool {
            self.pairs[ch.index()]
        }

        /// 修改某路占空比（万分比 0..=10000）
        ///
        /// 写 CHxCCRA **影子寄存器**（BUFEA 预装载），下一 UEV（周期边界）
        /// 生效——不截断当前脉冲（与经典 GTIM 立即生效不同，与 L012 一致）。
        /// 仅接受内部值：折算比较值为 0 或 ≥ 周期即
        /// [`AtimError::BadDuty`]（0%/100% 端点拒绝，安全边界见模块文档）。
        pub fn set_duty_bp(&mut self, ch: AtimChannel, duty_bp: u16) -> Result<(), AtimError> {
            if !self.has_channel(ch) {
                return Err(AtimError::ChannelUnavailable);
            }
            let compare = duty_compare(self.period_ticks, duty_bp).ok_or(AtimError::BadDuty)?;
            match ch {
                AtimChannel::Ch1 => self.regs.ch1ccra.write(u32::from(compare)),
                AtimChannel::Ch2 => self.regs.ch2ccra.write(u32::from(compare)),
                AtimChannel::Ch3 => self.regs.ch3ccra.write(u32::from(compare)),
            }
            Ok(())
        }

        /// 读回某路占空比设定（万分比；读的是 CCRA 影子值——若上次写入
        /// 还没等到 UEV，反映的是"已请求"而非"正在输出"的值）
        pub fn duty_bp(&self, ch: AtimChannel) -> Result<u16, AtimError> {
            if !self.has_channel(ch) {
                return Err(AtimError::ChannelUnavailable);
            }
            let ccr = (match ch {
                AtimChannel::Ch1 => self.regs.ch1ccra.read(),
                AtimChannel::Ch2 => self.regs.ch2ccra.read(),
                AtimChannel::Ch3 => self.regs.ch3ccra.read(),
            }) & 0xFFFF;
            Ok(((ccr * 10_000) / self.period_ticks) as u16)
        }

        /// 当前计数值（观测 CNT 在 0..ARR 间滚动 = PWM 在跑）
        pub fn count(&self) -> u16 {
            self.regs.cnt.read() as u16
        }

        /// 实际死区编码（None = DTEN 关闭）
        pub fn dead_time_code(&self) -> Option<u8> {
            self.dead_time
        }

        /// 实际死区 TCLK tick 数（0 = DTEN 关闭）
        pub fn dead_time_ticks(&self) -> u32 {
            self.dead_time.map(dead_time_ticks).unwrap_or(0)
        }

        /// 异步等待一次更新事件（PWM 周期节拍；UIE 单发纪律）
        ///
        /// 每次调用先清掉残留 UIF（ICR 写 0 清除），再等**下一个**真实
        /// UEV（URS=1 → 仅计数溢出）。等待期间才开 CR.UIE，IRQ 唤醒即压掉
        /// （防风暴）。前置条件：ATIM 中断已 `bind_interrupts!` 绑定到
        /// [`AtimIrq`]（IRQ13），计数在跑（stop 状态下永无溢出会挂等）。
        pub async fn wait_update(&mut self) {
            crate::interrupt::nvic_enable::<interrupt::ATIM>();
            // 前置清残留 UIF：写 0 清 UIF、其余位写 1 保全（含保留 bit1）
            self.regs.icr.write(ICR_NOOP & !icr::uif_MASK);
            poll_fn(|cx| {
                ATIM_WAKER.register(cx.waker());
                if self.regs.isr.read() & isr::uif_MASK != 0 {
                    cr_keep_and(self.regs, cr::uie_MASK); // 就绪即压掉
                    Poll::Ready(())
                } else {
                    cr_keep_or(self.regs, cr::uie_MASK); // 等待期间才开
                    Poll::Pending
                }
            })
            .await;
        }
    }

    impl Drop for AtimPwm {
        fn drop(&mut self) {
            // 安全网：先关主输出再停计数，UIE 一并压掉（引脚保持 AF 复用
            // 态、时钟门控不动——同仓 ClassicPwm 约定）
            self.disable();
            cr_keep_and(self.regs, cr::uie_MASK | cr::en_MASK);
        }
    }

    // ================= 中断派发 =================

    /// UEV waker（七家族每家单实例）
    static ATIM_WAKER: AtomicWaker = AtomicWaker::new();

    /// ATIM 更新中断标记（IRQ13，bind_interrupts! 绑定用）
    ///
    /// ```ignore
    /// embassy_cw32::bind_interrupts!(struct Irqs {
    ///     ATIM => embassy_cw32::atim_classic::AtimIrq;
    /// });
    /// ```
    pub struct AtimIrq;
    impl IrqHandler<interrupt::ATIM> for AtimIrq {
        fn call() {
            let regs = unsafe { &*(pac::ATIM_BASE as *const pac::ATIM) };
            if regs.isr.read() & isr::uif_MASK != 0 {
                // 单发压掉 UIE（CR 读回过滤后整字写回，命令位不复放；
                // ISR 的 UIF 不动——poll 复查后由下次 wait_update 前置清除）
                cr_keep_and(regs, cr::uie_MASK);
                ATIM_WAKER.wake();
            }
        }
    }

    // ================= 寄存器字构造 host 测试（任一经典家族 feature 下跑） =================

    #[cfg(test)]
    mod word_tests {
        use super::*;
        use crate::pac::atim::{ch2cr, ch3cr};

        #[test]
        fn control_word_edge_aligned_complementary() {
            // COMP|PWM2S|BUFPEN|MODE=10|URS，档位 0
            assert_eq!(control_word(0), 0x0002_208A);
            // 档位 7 = 256 分频（prs_MASK=0x70 满档）
            assert_eq!(control_word(7), 0x0002_20FA);
            // 档位位段落位正确
            assert_eq!(control_word(5) & cr::prs_MASK, 5 << cr::prs_POS);
        }

        #[test]
        fn dtr_word_encodes_code_and_dten_only() {
            assert_eq!(dtr_word(None), 0); // DTEN 关 + MOE/BKE/AOE 复位 0
            assert_eq!(dtr_word(Some(0)), dtr::dten_MASK); // 编码 0 = 2 tick
            assert_eq!(dtr_word(Some(23)), 0x0000_0217); // 码 23 + DTEN
            assert_eq!(dtr_word(Some(255)), 0x0000_02FF); // 上限编码 1010 tick
        }

        #[test]
        fn fltr_word_three_pwm1_noninverted() {
            assert_eq!(fltr_word(), 0x0006_0606); // 三路 OCM=110(PWM1)，CCP=0
        }

        #[test]
        fn chxcr_word_preloads_ccra_only() {
            // 双缓冲字：仅 BUFEA（bit6）——CCRA 影子使能，B 侧/捕获/IRQ/DMA 全 0
            assert_eq!(chxcr_word(), 0x40);
            assert_eq!(chxcr_word(), ch1cr::bufea_MASK);
            assert_eq!(ch2cr::bufea_MASK, ch1cr::bufea_MASK);
            assert_eq!(ch3cr::bufea_MASK, ch1cr::bufea_MASK);
        }

        #[test]
        fn cr_keep_mask_excludes_command_bits() {
            // TG/UG/BG 命令位（bit24/25/26）与保留位绝不落入读回复写
            assert_eq!(CR_KEEP_MASK & cr::ug_MASK, 0);
            assert_eq!(CR_KEEP_MASK & cr::tg_MASK, 0);
            assert_eq!(CR_KEEP_MASK & cr::bg_MASK, 0);
            // 配置位全在
            assert_ne!(CR_KEEP_MASK & cr::en_MASK, 0);
            assert_ne!(CR_KEEP_MASK & cr::uie_MASK, 0);
            assert_ne!(CR_KEEP_MASK & cr::mode_MASK, 0);
        }
    }
}

#[cfg(any(
    feature = "cw32f030",
    feature = "cw32f003",
    feature = "cw32l031",
    feature = "cw32r031",
    feature = "cw32w031",
    feature = "cw32l052",
    feature = "cw32l083",
))]
pub use classic::*;
