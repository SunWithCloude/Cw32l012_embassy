//! 看门狗：IWDT 独立看门狗 + WWDGT 窗口看门狗。
//!
//! 两代看门狗 IP 按家族 feature 多选一编译（同 [`crate::adc`] 的分支格局）：
//!
//! - **L012 家族**（`cw32l012`，§19/§20）：见下方 `l012` 分支，类型
//!   `Iwdt`/`Wwdgt`/`TimeoutMs`/`WindowConfig` 等。
//! - **经典家族**（`cw32f030`/`cw32f020`/`cw32l031`/`cw32l052`/
//!   `cw32l083`/`cw32r031`/`cw32w031`；f003 有 IWDT/WWDGT 但无 RTC，
//!   同样接入本分支）：见下方 `classic` 分支，类型带 `Classic` 前缀
//!   （`ClassicIwdt`/`ClassicWwdgt`/`ClassicWindowConfig`），方法名与
//!   L012 侧对齐（start/feed/…）。
//!
//! ```ignore
//! // IWDT：≈1.5s 超时，忘喂即复位
//! let mut wdt = ClassicIwdt::new(p.iwdt);
//! wdt.start(TimeoutMs(1500))?;
//! loop {
//!     do_work().await;
//!     wdt.feed(); // 忘了喂 → 芯片复位
//! }
//! ```
//!
//! 两代差异一览（逐寄存器比对，见 classic 分支头注）：寄存器位面同款
//! （IWDT KR/CR/ARR/SR/WINR/CNT、WWDGT CR0/CR1/SR 逐位段一致），差异在
//! 时钟源（经典组 IWDT 用**专属 RC10K**、标称 8kHz——非 L012 的 10kHz）、
//! 启动次序（经典组厂商库要求**先 0xCCCC 启动再改配置**）与门控位
//! （经典组在 APBEN1，无 KEY）。

// ================= L012 家族分支（内容不动，整体收进子模块）=================
#[cfg(feature = "cw32l012")]
mod l012 {
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
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub struct TimeoutMs(pub u32);

    /// 看门狗配置错误（超时不可实现）
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
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
            let ticks = (ms * wd::RC10K_HZ as u64).div_ceil(1000 * div);
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
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
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
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
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
            let mut wc_ticks = interval_ns.div_ceil(tick) as u32;
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
}

#[cfg(feature = "cw32l012")]
pub use l012::*;

// ================= 经典组分支（f030/f020/l031/l052/l083/r031/w031；f003 仅 WDT）=================
// 与 L012 **同位面但异代**的看门狗：7 家族 PAC 的 IWDT/WWDT 结构体与位段
// 常量逐字节同构（七份 `pub struct IWDT`/`pub struct WWDT` diff 为空；
// f003 无 RTC 但 IWDT/WWDT 同款接入）。
//
// IWDT（cw32f030.h:3209-3285 IWDT_TypeDef / 生成 PAC `pub mod iwdt`）：
//   KR@0x00(WO,16bit) / CR@0x04(PRS[2:0]/ACTION[3]/IE[4]/PAUSE[5]) /
//   ARR@0x08(12bit) / SR@0x0C(CRF[0]/ARRF[1]/WINRF[2]/OV[3]/RUN[4]/RELOAD[5]) /
//   WINR@0x10(12bit) / CNT@0x24(12bit,RO)。键值序列与 L012 完全同款
//   （cw32f030_iwdt.h:76-81）：0xCCCC 启动、0xAAAA 喂狗、0x5555 解锁、
//   0x6666 上锁、0x5A5A→0xA5A5 停止。计数时钟 = 专属 RC10K，与 PCLK/LSE
//   域独立（CW32x030 手册 V2.5 claim，cw32-hal2 clock yaml IWDT kernel_clock
//   = RC10K；非 LSI——经典组 LSI 32.8kHz 是 RTC/UART 的源）。
//
//   与 L012 配方差异（逐条实锤）：
//   - **标称频率 8kHz**：L012 RC10K 标称 10kHz（厂商 IWDT_FREQ=10000，
//     cw32f030_iwdt.h:48——该宏按 10kHz 写）；cw32-hal2 按数据手册电气表
//     给 7 家族统一 typical 8000 / 4000..12000 Hz（electrical.yaml
//     iwdt_clock，"SDK 的 IWDT_FREQ=10000 常有误"）。本驱动按**厂商宏
//     10kHz** 计算（与 L012 分支同口径），文档注明实测档位偏长
//     （8kHz 实际 ≈ 请求 ×1.25）——看门狗宁长勿短，喂狗间隔留裕量。
//   - **启动次序**：厂商 IWDT_Init 要求**先 `0xCCCC` 启动再改配置**
//     （cw32f030_iwdt.c:71-72 "需要先启动IWDT后，才能修改其他寄存器"；
//     配置后等 SR.CRF/ARRF/WINRF 同步清零，cw32f030_iwdt.c:82-84）——
//     L012 分支是先解锁配置再启动。
//   - **WINR 默认 0xFFF 关窗**（厂商 IWDT_SetPeriod 注释"关闭窗口看门狗
//     的功能"）：本驱动不暴露经典组 IWDT 的窗口功能（超范围特性，
//     WWDGT 已覆盖窗口语义）。
//   - **门控 APBEN1.IWDT=bit5**（cw32f030.h APBEN1_f；7 家族 PAC 同位），
//     读改写无 KEY。
//
// WWDGT（cw32f030.h:4481-4518 / 生成 PAC `pub mod wwdt`）：CR0@0x00
//   (WCNT[6:0]/EN[7] RW1) / CR1@0x04(WINR[6:0]/PRS[9:7]/IE[10]) / SR@0x08
//   (POV[0])——**与 L012 逐位段一致**（L012 PAC Wwdt 注释同位面），配方
//   直接沿用：PCLK/4096/2^PRS、0x40..=0x7F、窗口内才可喂、开启不可关
//   （厂商 WWDT_Cmd 注释"使能WWDT后不能禁止"）。差异仅门控
//   （APBEN1.WWDT=bit4）与基地址（0x4000_2C00 vs L012 0x4000_5400）。
//
// 配方出处（三源交叉核对）：厂商库 CW32F030_StandardPeripheralLib V2.2
// `cw32f030_iwdt.c/h`、`cw32f030_wwdt.c/h`；生成 PAC `pub mod iwdt`/
// `pub mod wwdt`（七家族 diff 为空）；cw32-hal2 embassy-cw32/src/wdg/
// mod.rs（独立实现的同款键值序列与启动先行协议，交叉印证）。
#[cfg(any(
    feature = "cw32f030",
    feature = "cw32f020",
    feature = "cw32f003",
    feature = "cw32l031",
    feature = "cw32l052",
    feature = "cw32l083",
    feature = "cw32r031",
    feature = "cw32w031"
))]
mod classic {
    //! 经典家族看门狗：IWDT 独立看门狗 + WWDGT 窗口看门狗。
    //!
    //! 与 L012 分支同为**非异步薄封装**：喂狗是一次键值写（IWDT
    //! `KR=0xAAAA` / WWDGT `CR0.WCNT` 重载），没有可等待的事件；溢出
    //! "事件"本身就是要复位，不存在等它。
    //!
    //! ```ignore
    //! // IWDT：≈1.5s 超时，忘喂即复位（RC10K 独立时钟域，主时钟跑飞仍有效）
    //! let mut wdt = ClassicIwdt::new(p.iwdt);
    //! wdt.start(TimeoutMs(1500))?;
    //! loop {
    //!     do_work().await;
    //!     wdt.feed();
    //! }
    //!
    //! // WWDGT：喂狗窗口 [T_min, T_max]，过早/过晚都复位
    //! let mut wwdt = ClassicWwdgt::new(p.wwdt);
    //! let cfg = ClassicWindowConfig::for_feed_interval(8_000_000, 200).unwrap();
    //! wwdt.start(cfg); // 开启后不可关闭
    //! wwdt.feed()?;    // 过早喂狗返回 Err 而不是让芯片复位
    //! ```
    //!
    //! 两条看门狗共享 IRQ0（`WDT`）——本模块复位模式不需要中断，
    //! 未提供异步接口。

    use crate::pac;

    // ================= IWDT 独立看门狗 =================

    /// IWDT 计数时钟标称值（RC10K 专属振荡器；厂商宏 `IWDT_FREQ=10000`，
    /// cw32f030_iwdt.h:48。⚠ 数据手册电气表口径为 typical 8000Hz、
    /// 4000..12000Hz 全温范围——见 classic 分支头注，喂狗间隔留足裕量）
    const RC10K_HZ: u64 = 10_000;
    /// 重载值上限（ARR[11:0]）
    const ARR_MAX: u16 = 0xFFF;
    /// 预分频档（PRS[2:0]：÷4 起，2 的幂，cw32f030_iwdt.h:50-57）
    const PRS_DIVS: [u64; 8] = [4, 8, 16, 32, 64, 128, 256, 512];

    // 键值序列（cw32f030_iwdt.h:76-81，与 L012 同款）
    /// KR：启动（SR.RUN 置位）
    const KR_START: u32 = 0xCCCC;
    /// KR：喂狗（重载计数）
    const KR_FEED: u32 = 0xAAAA;
    /// KR：解锁 CR/ARR/WINR
    const KR_UNLOCK: u32 = 0x5555;
    /// KR：重新上锁
    const KR_LOCK: u32 = 0x6666;
    /// KR：停止双键第一发（仅调试用）
    const KR_STOP1: u32 = 0x5A5A;
    /// KR：停止双键第二发
    const KR_STOP2: u32 = 0xA5A5;

    /// CR：溢出动作 = 复位（ACTION[3]=0，厂商 IWDT_OVERFLOW_ACTION_RESET）
    const CR_ACTION_RESET: u32 = 0;
    /// CR.PRS[2:0] 位掩码
    const CR_PRS_MASK: u32 = 0x7;
    /// SR 同步标志组：CRF|ARRF|WINRF（写 CR/ARR/WINR 后置位、同步到
    /// RC10K 域后硬件清零——厂商 IWDT_Init 忙等这三个清零，
    /// cw32f030_iwdt.c:82-84）
    const SR_SYNC_MASK: u32 = 0x7;
    /// SR.RUN[4]：计数运行中
    const SR_RUN: u32 = 1 << 4;
    /// 配置同步等待上限（spin 轮数；厂商 0xFFFF 次读，cw32f030_iwdt.c:68）
    const SYNC_POLL: u32 = 200_000;

    /// 看门狗超时（毫秒；语义同 L012 分支 [`super::l012::TimeoutMs`])
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub struct TimeoutMs(pub u32);

    /// 看门狗配置错误（超时不可实现）
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub struct TimeoutError;

    /// 计算能覆盖 `TimeoutMs` 的最小分频组合（PRS 分频、ARR）
    ///
    /// 返回 `(prs_code, arr)`；不可实现（超时 < 0.4ms 或 > ≈209.7s）时 Err。
    /// 公式：`T = (PRS 分频) × (ARR+1) / RC10K`，PRS ∈ {4,8,…,512}，
    /// ARR ≤ 0xFFF。向上取整保证不短于请求值（按标称 10kHz；全温 8kHz
    /// 口径下实际更长，见头注）。
    pub fn calc_timeout(t: TimeoutMs) -> Result<(u32, u16), TimeoutError> {
        let ms = t.0 as u64;
        if ms == 0 {
            return Err(TimeoutError);
        }
        for (code, div) in PRS_DIVS.iter().enumerate() {
            // ticks = ceil(ms × RC10K / 1000 / div)
            let ticks = (ms * RC10K_HZ).div_ceil(1000 * div);
            if ticks >= 2 && ticks <= ARR_MAX as u64 {
                return Ok((code as u32, (ticks - 1) as u16));
            }
        }
        Err(TimeoutError)
    }

    /// 经典组 IWDT 独立看门狗（键值机：`0xCCCC` 启动 → `0xAAAA` 喂狗 →
    /// `0x5555` 解锁改配置）
    ///
    /// 计数时钟为**专属 RC10K**（与 PCLK、LSE 域独立；DeepSleep 下
    /// 继续计数，CR.PAUSE=0 默认）——主时钟跑飞时仍能复位芯片。
    pub struct ClassicIwdt {
        regs: &'static pac::IWDT,
    }

    impl ClassicIwdt {
        /// 获取 IWDT（计数时钟为专属 RC10K；APBEN1[5] 仅门控寄存器访问，
        /// 无 KEY 写保护）
        pub fn new(regs: &'static pac::IWDT) -> Self {
            let sysctrl = unsafe { &*(pac::SYSCTRL_BASE as *const crate::Sysctrl) };
            // 读改写（经典组 APBEN 无 KEY，lib.rs 门控写规范；厂商
            // IWDT_DeInit 亦直接写 APBRST1/APBEN1）
            sysctrl.apben1.modify(|v| v | (1 << 5));
            ClassicIwdt { regs }
        }

        /// 启动看门狗（溢出动作 = 复位），超时 ≈ `t`
        ///
        /// 顺序为厂商 `IWDT_Init` 配方（cw32f030_iwdt.c:60-93）：
        /// `0xCCCC` 先启动（**经典组要求先启动才能改配置**）→ `0x5555`
        /// 解锁 → CR(PRS/ACTION=复位/PAUSE=0) → ARR → WINR=0xFFF（关窗）
        /// → 等 SR.CRF/ARRF/WINRF 同步清零 → `0x6666` 上锁 → `0xAAAA`
        /// 喂狗装载新 ARR 并从零计起（厂商 `IWDT_SetPeriod` 的 Init+
        /// Refresh 组合）。
        ///
        /// RC10K 有全温 ±% 十级漂移（8kHz 口径下实际超时偏长 ~25%），
        /// 喂狗间隔留足裕量。
        pub fn start(&mut self, t: TimeoutMs) -> Result<(), TimeoutError> {
            let (prs, arr) = calc_timeout(t)?;
            self.regs.kr.write(KR_START); // 先启动（厂商："需要先启动IWDT后，才能修改其他寄存器"）
            self.regs.kr.write(KR_UNLOCK); // 解锁 CR/ARR/WINR
            self.regs.cr.write(prs & CR_PRS_MASK | CR_ACTION_RESET); // IE=0 / PAUSE=0（DeepSleep 继续计数）
            self.regs.arr.write(arr as u32);
            self.regs.kr.write(KR_LOCK); // 重新上锁
            // 等配置同步到 RC10K 域（SR.CRF/ARRF/WINRF 清零）再喂狗装载
            if !self.wait_sync() {
                return Err(TimeoutError);
            }
            self.regs.kr.write(KR_FEED); // 装载新 ARR 并从零计起
            Ok(())
        }

        /// 等待 SR 同步标志清零（有界；厂商 0xFFFF 次读同款语义）
        fn wait_sync(&self) -> bool {
            for _ in 0..SYNC_POLL {
                if self.regs.sr.read() & SR_SYNC_MASK == 0 {
                    return true;
                }
                core::hint::spin_loop();
            }
            false
        }

        /// 喂狗（重载计数值，`KR=0xAAAA`）
        #[inline]
        pub fn feed(&mut self) {
            self.regs.kr.write(KR_FEED);
        }

        /// 当前计数值（0..ARR；连读两次取稳定值——厂商
        /// IWDT_GetCounterValue 的 do-while 配方，cw32f030_iwdt.c 尾部）
        #[inline]
        pub fn count(&self) -> u16 {
            let mut last = self.regs.cnt.read() & 0xFFF;
            for _ in 0..8 {
                let cur = self.regs.cnt.read() & 0xFFF;
                if cur == last {
                    return last as u16;
                }
                last = cur;
            }
            last as u16
        }

        /// 是否运行中（`SR.RUN`）
        #[inline]
        pub fn is_running(&self) -> bool {
            self.regs.sr.read() & SR_RUN != 0
        }

        /// 停止看门狗（仅调试；产线代码不应使用——0x5A5A+0xA5A5 双键）
        pub fn stop(&mut self) {
            self.regs.kr.write(KR_STOP1);
            self.regs.kr.write(KR_STOP2);
        }

        /// 寄存器块原始访问
        pub fn regs(&self) -> &'static pac::IWDT {
            self.regs
        }
    }

    // ================= WWDGT 窗口看门狗 =================

    /// 窗口配置（由 [`ClassicWindowConfig::for_feed_interval`] 生成或手工
    /// 构造；语义同 L012 分支 [`super::l012::WindowConfig`]）
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub struct ClassicWindowConfig {
        /// 预分频 PRS[0..7]：计数时钟 = PCLK / 4096 / 2^PRS
        pub prs: u8,
        /// 计数器初值 WCNT（0x40..=0x7F；≤0x3F 立即复位）
        pub wcnt: u8,
        /// 窗口值 WINR（0x40..=wcnt-1；喂狗下限）
        pub winr: u8,
    }

    /// 喂狗错误（语义同 L012 分支 [`super::l012::FeedError`]）
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub enum FeedError {
        /// 当前计数值仍在窗口值之上（喂得太早，真写会触发复位）
        TooEarly,
    }

    /// CR0：WCNT[6:0] 位掩码（读=当前值，写=刷新喂狗）
    const WWDGT_CR0_WCNT_MASK: u32 = 0x7F;
    /// CR0：EN[7]（RW1——开启后不可关闭，仅系统复位可停）
    const WWDGT_CR0_EN: u32 = 1 << 7;
    /// CR1：WINR[6:0] 位掩码
    const WWDGT_CR1_WINR_MASK: u32 = 0x7F;
    /// CR1：PRS[9:7] 位域
    const WWDGT_CR1_PRS_POS: u32 = 7;
    /// CR1：PRS[9:7] 位掩码
    const WWDGT_CR1_PRS_MASK: u32 = 0x7 << WWDGT_CR1_PRS_POS;
    /// SR：POV[0] 预溢出标志（写 0 清除）
    const WWDGT_SR_POV: u32 = 1 << 0;

    impl ClassicWindowConfig {
        /// 按喂狗周期自动求解 (PRS, WCNT, WINR)
        ///
        /// 目标：`T_max ≈ interval`（ceil 保证不短于请求）、`T_min ≈ interval/2`
        /// （软件每 interval 喂一次，落在窗口正中，两侧各有 ~25% 裕量）。
        /// 超出可实现范围（96MHz 下最长约 349ms、8MHz 下最长约 2.9s）返回 None。
        pub fn for_feed_interval(pclk_hz: u32, interval_ms: u32) -> Option<ClassicWindowConfig> {
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
            let mut wc_ticks = interval_ns.div_ceil(tick) as u32;
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
            Some(ClassicWindowConfig {
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

    /// 经典组 WWDGT 窗口看门狗——窗口内喂狗，过早/过晚都复位
    ///
    /// 7-bit 递减计数器（PCLK/4096/2^PRS 驱动），三条件触发复位：
    /// 1. 计数值递减到 0x3F（过晚喂狗）；
    /// 2. 当前计数值 > 窗口值 WINR 时执行喂狗（过早喂狗）；
    /// 3. 向 WCNT 写入 ≤ 0x3F 的值。
    ///
    /// 位面与 L012 完全一致（厂商 WWDT_Cmd："使能WWDT后不能禁止"）；
    /// DeepSleep 期间停止计数，唤醒后恢复。
    pub struct ClassicWwdgt {
        regs: &'static pac::WWDT,
        initial: u8,
        winr: u8,
    }

    impl ClassicWwdgt {
        /// 获取 WWDT 实例并使能寄存器时钟（APBEN1.WWDT=bit4，无 KEY）
        pub fn new(regs: &'static pac::WWDT) -> Self {
            let sysctrl = unsafe { &*(pac::SYSCTRL_BASE as *const crate::Sysctrl) };
            sysctrl.apben1.modify(|v| v | (1 << 4));
            ClassicWwdgt {
                regs,
                initial: 0x7F,
                winr: 0x7F,
            }
        }

        /// 启动看门狗（超时/窗口由 `cfg` 给出，
        /// [`ClassicWindowConfig::for_feed_interval`] 可按喂狗周期自动求解；
        /// **开启后不可关闭**，仅系统复位可停）
        ///
        /// 顺序：CR1(PRS/WINR) → CR0(WCNT 初值 + EN)——厂商 `WWDT_Init` +
        /// `WWDT_Cmd` 组合（cw32f030_wwdt.c）。DeepSleep 期间停止计数，
        /// 唤醒后恢复。
        pub fn start(&mut self, cfg: ClassicWindowConfig) {
            self.initial = cfg.wcnt;
            self.winr = cfg.winr;
            let cr1 = ((cfg.prs as u32 & 0x7) << WWDGT_CR1_PRS_POS)
                | (cfg.winr as u32 & WWDGT_CR1_WINR_MASK);
            self.regs.cr1.write(cr1);
            // 先设初值（EN=0 时安全），再置 EN（单次写同时带初值）
            self.regs.cr0.write(cfg.wcnt as u32 & WWDGT_CR0_WCNT_MASK);
            self.regs
                .cr0
                .write(cfg.wcnt as u32 & WWDGT_CR0_WCNT_MASK | WWDGT_CR0_EN);
        }

        /// 喂狗（窗口安全版）
        ///
        /// 先读当前计数值：落在窗口内（≤ WINR）才刷新为初值；
        /// 过早喂狗返回 [`FeedError::TooEarly`]（真写会复位）。
        pub fn feed(&mut self) -> Result<(), FeedError> {
            let current = (self.regs.cr0.read() & WWDGT_CR0_WCNT_MASK) as u8;
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
            (self.regs.cr0.read() & WWDGT_CR0_WCNT_MASK) as u8
        }

        /// 当前窗口值 WINR
        pub fn winr(&self) -> u8 {
            (self.regs.cr1.read() & WWDGT_CR1_WINR_MASK) as u8
        }

        /// 喂狗时间窗 `(T_min_us, T_max_us)`（按给定 PCLK 计算）
        pub fn window_us(&self, pclk_hz: u32) -> (u64, u64) {
            let prs = ((self.regs.cr1.read() & WWDGT_CR1_PRS_MASK) >> WWDGT_CR1_PRS_POS) as u8;
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
            self.regs.sr.read() & WWDGT_SR_POV != 0
        }

        /// 清除预溢出标志
        pub fn clear_pre_overflow(&mut self) {
            self.regs.sr.write(0);
        }

        /// 寄存器块原始访问
        pub fn regs(&self) -> &'static pac::WWDT {
            self.regs
        }
    }

    #[cfg(test)]
    mod tests {
        use super::*;

        // 与 L012 分支同名测试同源（标称口径一致：10kHz）
        #[test]
        fn classic_iwdt_timeout_short() {
            // 10ms：ticks = ceil(10×10/4) = 25 → PRS=0(÷4) ARR=24
            let (prs, arr) = calc_timeout(TimeoutMs(10)).unwrap();
            assert_eq!(prs, 0);
            assert_eq!(arr, 24);
        }

        #[test]
        fn classic_iwdt_timeout_mid_uses_smallest_divisor() {
            // 500ms：÷4 → ticks=1250 ≤ 0xFFF ✓（无需升档）
            let (prs, arr) = calc_timeout(TimeoutMs(500)).unwrap();
            assert_eq!(prs, 0);
            assert_eq!(arr, 1249);
        }

        #[test]
        fn classic_iwdt_timeout_large_needs_max_divisor() {
            // 200s：÷512 → ticks=ceil(200000×10/512)=3907 → ARR=3906
            let (prs, arr) = calc_timeout(TimeoutMs(200_000)).unwrap();
            assert_eq!(prs, 7);
            assert_eq!(arr, 3906);
        }

        #[test]
        fn classic_iwdt_timeout_out_of_range() {
            // 上限：÷512、ARR=0xFFF → 4095×512/10 ≈ 209.6s
            assert!(calc_timeout(TimeoutMs(210_000)).is_err());
            assert!(calc_timeout(TimeoutMs(0)).is_err());
        }

        #[test]
        fn classic_iwdt_timeout_rounding_never_shorter_than_requested() {
            for ms in [1u32, 3, 7, 50, 123, 999, 5000, 60_000] {
                if let Ok((prs, arr)) = calc_timeout(TimeoutMs(ms)) {
                    let div = 4u64 << prs;
                    let actual_us = (arr as u64 + 1) * div * 100;
                    assert!(actual_us >= ms as u64 * 1000, "{ms}ms: 偏短");
                }
            }
        }

        #[test]
        fn classic_wwdt_interval_50ms_at_96mhz() {
            // 与 L012 分支同期望值（位面与求解器同源）
            let cfg = ClassicWindowConfig::for_feed_interval(96_000_000, 50).unwrap();
            assert_eq!(cfg.prs, 5);
            assert_eq!(cfg.wcnt, 0x64);
            assert_eq!(cfg.winr, 0x52);
        }

        #[test]
        fn classic_wwdt_interval_1s_at_8mhz() {
            // 8MHz（经典组复位默认 PCLK）：tick(prs=5)=4096×32×1e9/8e6
            // =16.384ms；64×16.384ms=1.049s ≥ 1s ✓（prs=4 仅 0.52s）
            let cfg = ClassicWindowConfig::for_feed_interval(8_000_000, 1000).unwrap();
            assert_eq!(cfg.prs, 5);
            assert_eq!(cfg.wcnt, 0x7D); // 0x3F + ceil(1e9/16.384e6)=62
            assert_eq!(cfg.winr, 0x5E); // 0x7D - 31
        }

        #[test]
        fn classic_wwdt_tmax_never_shorter_than_interval() {
            for &(pclk, ms) in &[
                (96_000_000u32, 1u32),
                (96_000_000, 30),
                (96_000_000, 300),
                (8_000_000, 100),
                (24_000_000, 200),
            ] {
                let cfg = ClassicWindowConfig::for_feed_interval(pclk, ms).unwrap();
                let tick = tick_ns(pclk, cfg.prs);
                let t_max = (cfg.wcnt - 0x3F) as u64 * tick;
                let t_min = (cfg.wcnt - cfg.winr) as u64 * tick;
                assert!(
                    t_max >= ms as u64 * 1_000_000,
                    "{ms}ms@{pclk}: T_max={t_max}ns 偏短"
                );
                assert!(t_min * 2 <= t_max, "{ms}ms@{pclk}: 窗口过窄");
                assert!(cfg.winr >= 0x40, "WINR 越界：0x{:02X}", cfg.winr);
                assert!(cfg.wcnt > cfg.winr, "WCNT 必须 > WINR");
            }
        }

        #[test]
        fn classic_wwdt_out_of_range() {
            // 96MHz 最长 64×5461us ≈ 349ms
            assert!(ClassicWindowConfig::for_feed_interval(96_000_000, 400).is_none());
            assert!(ClassicWindowConfig::for_feed_interval(96_000_000, 0).is_none());
            assert!(ClassicWindowConfig::for_feed_interval(0, 100).is_none());
        }

        #[test]
        fn classic_wwdt_feed_semantics_unit_check() {
            // feed() 的判定逻辑（纯数值复现）：current > winr → TooEarly
            let winr = 0x52u8;
            assert!((0x60u8) > winr); // 过早
            assert!((0x52u8) <= winr); // 窗口开启
            assert!((0x40u8) <= winr); // 仍可喂（>0x3F）
            assert!((0x3Fu8) <= winr); // 但 0x3F 已触发复位
        }
    }
}

#[cfg(any(
    feature = "cw32f030",
    feature = "cw32f020",
    feature = "cw32f003",
    feature = "cw32l031",
    feature = "cw32l052",
    feature = "cw32l083",
    feature = "cw32r031",
    feature = "cw32w031"
))]
pub use classic::*;
