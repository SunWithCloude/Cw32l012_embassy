//! 看门狗：IWDT 独立看门狗（§19）+ WWDGT 窗口看门狗（§20）。
//!
//! **L012 家族先行**（`cw32l012` feature，同 [`crate::adc`]/[`crate::timer`]/
//! [`crate::dma`] 的文档措辞）；寄存器配方与阻塞版 HAL **同源**
//! （`cw32l012-hal/src/wdt.rs` + `src/wwdt.rs`，实机验证见其
//! `examples/wdt_demo.rs` / `wwdt_demo.rs`——复位循环与窗口拦截均已上板）。
//!
//! 两者都是**非异步薄封装**：喂狗是一次键值写（IWDT `KR=0xAAAA` /
//! WWDGT `CR0.WCNT` 重载），没有可等待的事件，异步化没有收益；溢出
//! "事件"本身就是要复位，不存在等它。
//!
//! # 选哪种（阻塞版注释结论的归纳）
//!
//! | 场景 | 选择 | 依据 |
//! |---|---|---|
//! | 防主时钟跑飞 / 系统级死机兜底 | [`Iwdt`] | 计数时钟来自独立 RC10K
//!   （≈10kHz，与主时钟域完全独立），主时钟崩溃时仍能复位芯片 |
//! | 检测主循环时序异常（喂太快 = 失控重入） | [`Wwdgt`] | 窗口内才允许
//!   喂狗，过早/过晚都复位 |
//! | 低功耗（DeepSleep 期间也要盯着） | [`Iwdt`] | WWDGT 在 DeepSleep
//!   期间停止计数，唤醒后恢复——睡久了会被误咬；IWDT 的 RC10K 域
//!   DeepSleep 下继续计数（CR.PAUSE=0 默认） |
//!
//! 两者可并用（IWDT 兜底 + WWDGT 管节奏）。操作面差异：[`Wwdgt`] 开启
//! 后**不可关闭**（CR0.EN 为 RW1，仅系统复位可停）；[`Iwdt`] 调试期可停
//! （0x5A5A+0xA5A5 双键，产线代码不应使用）。超时范围：IWDT 0.4ms ~
//! ≈209.7s（RC10K ±% 级漂移）；WWDGT 7-bit 计数器，4MHz 下最长 ≈8.4s、
//! 96MHz 下最长 ≈349ms。
//!
//! # 实机验证要点（阻塞版 wdt_demo 注释）
//!
//! 停止喂狗后芯片会在超时点复位——PC13 闪烁计数清零、探针会看到 SWD
//! 短暂掉线再恢复，即为看门狗复位生效的硬证据。
//!
//! # 用法
//!
//! ```ignore
//! // IWDT：≈1.5s 超时，忘喂即复位
//! let mut wdt = Iwdt::new(p.iwdt);
//! wdt.start(TimeoutMs(1500))?;
//! loop {
//!     do_work().await;
//!     wdt.feed(); // 忘了喂 → 芯片复位
//! }
//!
//! // WWDGT：喂狗窗口 [T_min, T_max]，过早/过晚都复位
//! let mut wwdt = Wwdgt::new(p.wwdt);
//! let cfg = WindowConfig::for_feed_interval(4_000_000, 200).unwrap();
//! wwdt.start(cfg); // 开启后不可关闭
//! wwdt.feed()?;    // 过早喂狗返回 Err 而不是让芯片复位
//! ```
//!
//! 两条看门狗共享 IRQ0（`WDT`，溢出中断/预溢出中断用）——本模块复位
//! 模式不需要中断，未提供异步接口。

use crate::pac;
use crate::pac::iwdt as wd;
use crate::pac::wwdt as ww;

// ================= IWDT 独立看门狗（§19） =================

/// 看门狗超时（毫秒；同阻塞版 `wdt::TimeoutMs`）
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TimeoutMs(pub u32);

/// 看门狗配置错误（超时不可实现）
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TimeoutError;

/// 计算能覆盖 `TimeoutMs` 的最小分频组合（PRS 分频、ARR）
///
/// 返回 `(prs_code, arr)`；不可实现（超时 < 0.4ms 或 > ≈209.7s）时 Err。
/// 公式（§19）：`T = (PRS 分频) × (ARR+1) / RC10K`，PRS ∈ {4,8,…,512}，
/// ARR ≤ 0xFFF。
pub fn calc_timeout(t: TimeoutMs) -> Result<(u32, u16), TimeoutError> {
    let ms = t.0 as u64;
    if ms == 0 {
        return Err(TimeoutError);
    }
    // 每档分频下的最小超时（ARR=1）与最大超时（ARR=0xFFF）
    for (code, div) in [4u64, 8, 16, 32, 64, 128, 256, 512].iter().enumerate() {
        // ticks = ceil(ms × RC10K / 1000 / div)（整体向上取整，保证不短于请求值）
        let ticks = (ms * wd::RC10K_HZ as u64 + 1000 * div - 1) / (1000 * div);
        if ticks >= 2 && ticks <= wd::ARR_MAX as u64 {
            return Ok((code as u32, (ticks - 1) as u16));
        }
    }
    Err(TimeoutError)
}

/// IWDT 独立看门狗（键值机：`0xCCCC` 启动 → `0xAAAA` 喂狗 → `0x5555`
/// 解锁改配置）
pub struct Iwdt {
    regs: &'static pac::Iwdt,
}

impl Iwdt {
    /// 获取 IWDT（配置/计数时钟为独立 RC10K，APBEN2[4] 仅门控寄存器
    /// 访问；KEY 写保护）
    pub fn new(regs: &'static pac::Iwdt) -> Self {
        let sysctrl = unsafe { &*(pac::SYSCTRL_BASE as *const crate::Sysctrl) };
        // 读改写 + KEY（阻塞版 rcc.rs set_bits 配方，保留其它外设使能位）
        sysctrl
            .apben2
            .modify(|v| pac::sysctrl::KEY | (v & 0xFFFF) | pac::sysctrl::APBEN2_IWDT);
        Iwdt { regs }
    }

    /// 启动看门狗（溢出动作 = 复位），超时 ≈ `t`（RC10K 有 ±% 级漂移，
    /// 喂狗间隔留足裕量）
    ///
    /// 已启动时再次调用会先解锁重配（不停止计数）。
    pub fn start(&mut self, t: TimeoutMs) -> Result<(), TimeoutError> {
        let (prs, arr) = calc_timeout(t)?;
        self.regs.kr.write(wd::KR_UNLOCK); // 解锁 CR/ARR/WINR
        self.regs
            .cr
            .write(prs & wd::CR_PRS_MASK | wd::CR_ACTION_RESET);
        self.regs.arr.write(arr as u32);
        self.regs.kr.write(wd::KR_FEED); // 装载新 ARR 并从零计起
        self.regs.kr.write(wd::KR_START); // 启动（未启动时）/ 重新上锁
        Ok(())
    }

    /// 喂狗（重载计数值，`KR=0xAAAA`）
    #[inline]
    pub fn feed(&mut self) {
        self.regs.kr.write(wd::KR_FEED);
    }

    /// 当前计数值（0..ARR，向下计数语义以手册 §19.6.4 为准）
    #[inline]
    pub fn count(&self) -> u16 {
        (self.regs.cnt.read() & 0xFFF) as u16
    }

    /// 停止看门狗（仅调试；产线代码不应使用——0x5A5A+0xA5A5 双键）
    pub fn stop(&mut self) {
        self.regs.kr.write(wd::KR_STOP1);
        self.regs.kr.write(wd::KR_STOP2);
    }

    /// 寄存器块原始访问
    pub fn regs(&self) -> &'static pac::Iwdt {
        self.regs
    }
}

// ================= WWDGT 窗口看门狗（§20） =================

/// 窗口配置（由 [`WindowConfig::for_feed_interval`] 生成或手工构造；
/// 同阻塞版 `wwdt::WindowConfig`）
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct WindowConfig {
    /// 预分频 PRS[0..7]：计数时钟 = PCLK / 4096 / 2^PRS
    pub prs: u8,
    /// 计数器初值 WCNT（0x40..=0x7F；≤0x3F 立即复位）
    pub wcnt: u8,
    /// 窗口值 WINR（0x40..=wcnt-1；喂狗下限）
    pub winr: u8,
}

/// 喂狗错误（同阻塞版 `wwdt::FeedError`）
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FeedError {
    /// 当前计数值仍在窗口值之上（喂得太早，真写会触发复位）
    TooEarly,
}

impl WindowConfig {
    /// 按喂狗周期自动求解 (PRS, WCNT, WINR)
    ///
    /// 目标：`T_max ≈ interval`（ceil 保证不短于请求）、`T_min ≈ interval/2`
    /// （软件每 interval 喂一次，落在窗口正中，两侧各有 ~25% 裕量）。
    /// 超出可实现范围（96MHz 下最长约 349ms、4MHz 下最长约 8.4s）返回 None。
    pub fn for_feed_interval(pclk_hz: u32, interval_ms: u32) -> Option<WindowConfig> {
        if pclk_hz == 0 || interval_ms == 0 {
            return None;
        }
        let interval_ns = interval_ms as u64 * 1_000_000;
        // 选最小 PRS 使 64 tick ≥ interval（64 = 0x7F-0x3F，最大计数跨度）
        let mut prs = None;
        for p in 0u8..8 {
            let tick = tick_ns(pclk_hz, p);
            if tick != 0 && tick * 64 >= interval_ns {
                prs = Some(p);
                break;
            }
        }
        let prs = prs?;
        let tick = tick_ns(pclk_hz, prs);
        // WCNT：ceil(interval / tick) 个 tick 后到 0x3F → T_max ≥ interval；
        // 至少 2 tick（保证 WINR < WCNT 有解）
        let mut wc_ticks = ((interval_ns + tick - 1) / tick) as u32;
        if wc_ticks < 2 {
            wc_ticks = 2;
        }
        let wcnt = 0x3F + wc_ticks; // ≤ 0x7F（前面已保证）
        if wcnt > 0x7F {
            return None;
        }
        // WINR：窗口 ≈ interval/2（但至少留 1 tick、至多留 wc_ticks-1 tick）
        let mut win_ticks = wc_ticks / 2;
        if win_ticks == 0 {
            win_ticks = 1;
        }
        if win_ticks >= wc_ticks {
            win_ticks = wc_ticks - 1;
        }
        let winr = (wcnt - win_ticks) as u8;
        Some(WindowConfig {
            prs,
            wcnt: wcnt as u8,
            winr,
        })
    }
}

/// 单个计数值的时长（ns）= 4096 × 2^PRS / PCLK（u64 全程，防整数除法丢精度）
fn tick_ns(pclk_hz: u32, prs: u8) -> u64 {
    (4096u64 << prs) * 1_000_000_000 / pclk_hz as u64
}

/// WWDGT 窗口看门狗——窗口内喂狗，过早/过晚都复位（§20）
///
/// 7-bit 递减计数器（PCLK/4096/2^PRS 驱动），三条件触发复位：
/// 1. 计数值递减到 0x3F（过晚喂狗）；
/// 2. 当前计数值 > 窗口值 WINR 时执行喂狗（过早喂狗）；
/// 3. 向 WCNT 写入 ≤ 0x3F 的值。
///
/// 喂狗时间窗（手册 §20.3.4）：
///
/// ```text
/// T_min = T_PCLK × 4096 × 2^PRS × (WCNT - WINR)
/// T_max = T_PCLK × 4096 × 2^PRS × (WCNT - 0x3F)
/// ```
pub struct Wwdgt {
    regs: &'static pac::Wwdt,
    initial: u8,
    winr: u8,
}

impl Wwdgt {
    /// 获取 WWDT 实例并使能寄存器时钟（APBEN2.WWDT，bit5，KEY 写保护）
    pub fn new(regs: &'static pac::Wwdt) -> Self {
        let sysctrl = unsafe { &*(pac::SYSCTRL_BASE as *const crate::Sysctrl) };
        sysctrl
            .apben2
            .modify(|v| pac::sysctrl::KEY | (v & 0xFFFF) | pac::sysctrl::APBEN2_WWDT);
        Wwdgt {
            regs,
            initial: 0x7F,
            winr: 0x7F,
        }
    }

    /// 启动看门狗（超时/窗口由 `cfg` 给出，[`WindowConfig::for_feed_interval`]
    /// 可按喂狗周期自动求解；**开启后不可关闭**，仅系统复位可停）
    ///
    /// 顺序：CR1(PRS/WINR) → CR0(WCNT 初值 + EN)。
    /// DeepSleep 期间停止计数，唤醒后恢复。
    pub fn start(&mut self, cfg: WindowConfig) {
        self.initial = cfg.wcnt;
        self.winr = cfg.winr;
        let cr1 =
            ((cfg.prs as u32 & 0x7) << ww::CR1_PRS_POS) | (cfg.winr as u32 & ww::CR1_WINR_MASK);
        self.regs.cr1.write(cr1);
        // 先设初值（EN=0 时安全），再置 EN（单次写同时带初值）
        self.regs.cr0.write(cfg.wcnt as u32 & ww::CR0_WCNT_MASK);
        self.regs
            .cr0
            .write(cfg.wcnt as u32 & ww::CR0_WCNT_MASK | ww::CR0_EN);
    }

    /// 喂狗（窗口安全版）
    ///
    /// 先读当前计数值：落在窗口内（≤ WINR）才刷新为初值；
    /// 过早喂狗返回 [`FeedError::TooEarly`]（真写会复位）。
    pub fn feed(&mut self) -> Result<(), FeedError> {
        let current = (self.regs.cr0.read() & ww::CR0_WCNT_MASK) as u8;
        if current > self.winr {
            return Err(FeedError::TooEarly);
        }
        self.regs.cr0.write(self.initial as u32);
        Ok(())
    }

    /// 直接刷新计数器（绕过窗口检查——仅在你明确知道时机合法时使用）
    pub fn feed_forced(&mut self) {
        self.regs.cr0.write(self.initial as u32);
    }

    /// 当前计数值（0x3F..=0x7F）
    pub fn count(&self) -> u8 {
        (self.regs.cr0.read() & ww::CR0_WCNT_MASK) as u8
    }

    /// 当前窗口值 WINR
    pub fn winr(&self) -> u8 {
        (self.regs.cr1.read() & ww::CR1_WINR_MASK) as u8
    }

    /// 喂狗时间窗 `(T_min_us, T_max_us)`（按给定 PCLK 计算）
    pub fn window_us(&self, pclk_hz: u32) -> (u64, u64) {
        let prs = ((self.regs.cr1.read() & ww::CR1_PRS_MASK) >> ww::CR1_PRS_POS) as u8;
        let tick = tick_ns(pclk_hz, prs);
        let wcnt = self.initial;
        // ns → µs（四舍五入）
        let to_us = |ns: u64| (ns + 500) / 1000;
        (
            to_us((wcnt - self.winr) as u64 * tick),
            to_us((wcnt - 0x3F) as u64 * tick),
        )
    }

    /// 预溢出标志（计到 0x40 时置位；写 0 清除）
    pub fn pre_overflow(&self) -> bool {
        self.regs.sr.read() & ww::SR_POV != 0
    }

    /// 清除预溢出标志
    pub fn clear_pre_overflow(&mut self) {
        self.regs.sr.write(0);
    }

    /// 寄存器块原始访问
    pub fn regs(&self) -> &'static pac::Wwdt {
        self.regs
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // 与阻塞版 cw32l012-hal/src/wdt.rs、src/wwdt.rs 的同名单元测试同源
    #[test]
    fn iwdt_timeout_short() {
        // 10ms：ticks = ceil(10×10/4) = 25 → PRS=0(÷4) ARR=24
        let (prs, arr) = calc_timeout(TimeoutMs(10)).unwrap();
        assert_eq!(prs, 0);
        assert_eq!(arr, 24);
    }

    #[test]
    fn iwdt_timeout_mid_uses_smallest_divisor() {
        // 500ms：÷4 → ticks=1250 ≤ 0xFFF ✓（无需升档）
        let (prs, arr) = calc_timeout(TimeoutMs(500)).unwrap();
        assert_eq!(prs, 0);
        assert_eq!(arr, 1249);
    }

    #[test]
    fn iwdt_timeout_large_needs_max_divisor() {
        // 200s：÷512 → ticks=ceil(200000×10/512)=3907 → ARR=3906
        let (prs, arr) = calc_timeout(TimeoutMs(200_000)).unwrap();
        assert_eq!(prs, 7);
        assert_eq!(arr, 3906);
    }

    #[test]
    fn iwdt_timeout_out_of_range() {
        // 上限：÷512、ARR=0xFFF → 4095×512/10 ≈ 209.6s
        assert!(calc_timeout(TimeoutMs(210_000)).is_err());
        assert!(calc_timeout(TimeoutMs(0)).is_err());
    }

    #[test]
    fn iwdt_timeout_rounding_never_shorter_than_requested() {
        // 计算出的实际超时必须 ≥ 请求值（向上取整保证）
        for ms in [1u32, 3, 7, 50, 123, 999, 5000, 60_000] {
            if let Ok((prs, arr)) = calc_timeout(TimeoutMs(ms)) {
                let div = 4u64 << prs;
                let actual_us = (arr as u64 + 1) * div * 100;
                assert!(
                    actual_us >= ms as u64 * 1000,
                    "{ms}ms: 实际 {actual_us}µs 偏短"
                );
            }
        }
    }

    #[test]
    fn wwdt_interval_50ms_at_96mhz() {
        // tick(prs)=4096×2^prs×1e6/96e6；prs=5 → 1365.3us；64×1365=87ms ≥ 50ms ✓
        let cfg = WindowConfig::for_feed_interval(96_000_000, 50).unwrap();
        assert_eq!(cfg.prs, 5);
        assert_eq!(cfg.wcnt, 0x64); // 0x3F + ceil(50000/1365)=37
        assert_eq!(cfg.winr, 0x52); // 0x64 - 18
    }

    #[test]
    fn wwdt_interval_1s_at_4mhz() {
        // 4MHz：tick(prs=4)=16384000ns；64×16.384ms=1.049s ≥ 1s ✓（prs=3 仅 0.52s）
        let cfg = WindowConfig::for_feed_interval(4_000_000, 1000).unwrap();
        assert_eq!(cfg.prs, 4);
        assert_eq!(cfg.wcnt, 0x7D); // 0x3F + ceil(1e9/16.384e6)=62
        assert_eq!(cfg.winr, 0x5E); // 0x7D - 31
    }

    #[test]
    fn wwdt_tmax_never_shorter_than_interval() {
        // ceil 保证：T_max = (wcnt-0x3F)×tick ≥ interval
        for &(pclk, ms) in &[
            (96_000_000u32, 1u32),
            (96_000_000, 30),
            (96_000_000, 300),
            (4_000_000, 100),
            (24_000_000, 200),
        ] {
            let cfg = WindowConfig::for_feed_interval(pclk, ms).unwrap();
            let tick = tick_ns(pclk, cfg.prs);
            let t_max = (cfg.wcnt - 0x3F) as u64 * tick;
            let t_min = (cfg.wcnt - cfg.winr) as u64 * tick;
            assert!(
                t_max >= ms as u64 * 1_000_000,
                "{ms}ms@{pclk}: T_max={t_max}ns 偏短"
            );
            assert!(
                t_min * 2 <= t_max,
                "{ms}ms@{pclk}: 窗口过窄 T_min={t_min}ns"
            );
            assert!(cfg.winr >= 0x40, "WINR 越界：0x{:02X}", cfg.winr);
            assert!(cfg.wcnt > cfg.winr, "WCNT 必须 > WINR");
        }
    }

    #[test]
    fn wwdt_out_of_range() {
        // 96MHz 最长 64×5461us ≈ 349ms
        assert!(WindowConfig::for_feed_interval(96_000_000, 400).is_none());
        assert!(WindowConfig::for_feed_interval(96_000_000, 0).is_none());
        assert!(WindowConfig::for_feed_interval(0, 100).is_none());
    }

    #[test]
    fn wwdt_window_matches_manual_example() {
        // 手册 §20.3.4 例：24MHz、PRS=1、WCNT=0x6F、WINR=0x4F →
        // T_min≈10.922ms、T_max=16.384ms
        let tick = tick_ns(24_000_000, 1); // 341333ns（精确值 341333.3）
        let t_min = (0x6F - 0x4F) as u64 * tick;
        let t_max = (0x6F - 0x3F) as u64 * tick;
        assert!(
            t_min >= 10_900_000 && t_min <= 10_940_000,
            "T_min={t_min}ns 应≈10922us"
        );
        assert!(
            t_max >= 16_380_000 && t_max <= 16_384_000,
            "T_max={t_max}ns 应≈16384us"
        );
    }

    #[test]
    fn wwdt_feed_semantics_unit_check() {
        // feed() 的判定逻辑（纯数值复现）：current > winr → TooEarly
        let winr = 0x52u8;
        assert!((0x60u8) > winr); // 过早
        assert!((0x52u8) <= winr); // 窗口开启
        assert!((0x40u8) <= winr); // 仍可喂（>0x3F）
        assert!((0x3Fu8) <= winr); // 但 0x3F 已触发复位
    }
}
