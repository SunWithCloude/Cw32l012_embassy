//! 异步实时时钟 RTC：BCD 日历读写 + 异步闹钟等待。
//!
//! 两套 IP 按家族 feature 多选一编译（同 [`crate::adc`] 的分支格局）：
//!
//! - **L012 家族**（`cw32l012`，§13）：见下方 `l012` 分支，类型 `Rtc`/
//!   `DateTime`/`Alarm`/`RtcClock` 等。
//! - **经典家族**（`cw32f030`/`cw32f020`/`cw32l031`/`cw32l052`/
//!   `cw32l083`/`cw32r031`/`cw32w031`，f030 手册 §12 同代位面）：见下方
//!   `classic` 分支，类型带 `Classic` 前缀（`ClassicRtc`/
//!   `ClassicDateTime`/…），**方法名与 L012 侧对齐**
//!   （start/stop/set_datetime/now/set_alarm_a/enable_alarm_a/
//!   wait_alarm…）。
//!
//! ```ignore
//! embassy_cw32::bind_interrupts!(struct Irqs {
//!     RTC => embassy_cw32::rtc::RtcIrq;          // L012
//!     RTC => embassy_cw32::rtc::ClassicRtcIrq;   // 经典组（同为 IRQ2）
//! });
//! ```
//!
//! 两代 RTC 的关键差异（逐寄存器比对，见 classic 分支头注）：
//!
//! | 项 | L012 | 经典组（7 家族同构） |
//! |---|---|---|
//! | 基地址 | 0x4000_4400 | 0x4000_2800 |
//! | 计时同步 | CR1.WAIT（RO） | CR1.ACCESS/WINDOW **运行中访问窗** |
//! | 预分频 | PSC1/PSC2 + SSCNT 亚秒 | **无**（内部固定 32768 分频） |
//! | 日历字段 | DAY[5:0]/MONTH[12:8] | DAY[7:0]/MONTH[15:8]（位段更宽） |
//! | 时钟门控 | APBEN2[1]（KEY） | APBEN1[3]（无 KEY） |
//! | LSI 标称 | 32768Hz（TRIM 9bit） | 32800Hz（TRIM 10bit） |

// ================= L012 家族分支（内容不动，整体收进子模块）=================
#[cfg(feature = "cw32l012")]
mod l012 {
    //! 异步实时时钟 RTC（§13）：BCD 日历读写 + 异步闹钟等待。
    //!
    //! **L012 家族先行**（`cw32l012` feature，同 [`crate::adc`]/[`crate::timer`]/
    //! [`crate::dma`] 的文档措辞）；寄存器配方与阻塞版 HAL **同源**
    //! （`cw32l012-hal/src/rtc.rs`，实机验证见其 `examples/rtc_demo.rs`——
    //! 跨日走时 + 闹钟 A 已上板通过）。
    //!
    //! # 时钟源（CR1.SOURCE）
    //!
    //! | 源 | 频率 | 说明 |
    //! |---|---|---|
    //! | LSE | 32768Hz | 外部晶体，最准；板上无晶振时 [`Rtc::new`] 返回
    //!   [`RtcError::ClockNotReady`]（起振等待约 2s，实例配方：先试 LSE 失败退 LSI） |
    //! | LSI | 32768Hz | 内部 RC，出厂已 TRIM（本板实测 32768Hz、误差 <1%——
    //!   阻塞版 rtc.rs 注释；PAC `LSI_HZ=32_800` 为标称值） |
    //!
    //! 默认预分频 `PSC1=0 / PSC2=0x3FFF` 把 32768Hz 直接分成 **2Hz TICKCLK**
    //! （日历每半秒走一步），[`Rtc::new`] 后无需再配。HSE/HSI 时基阻塞版虽
    //! 支持，但精度差、且预分频 32768↔2Hz 换算根本对不上，未移植。
    //!
    //! # 闹钟粒度（阻塞版注释的实测结论）
    //!
    //! 闹钟按 **秒字段 BCD 匹配**（时/分/秒任一字段可屏蔽，见 [`Alarm`]）：
    //! 日历每半秒走一步（TICKCLK 2Hz），匹配判定每 0.5s 一次——实际触发沿
    //! 最多滞后匹配时刻 ~0.5s；[`Rtc::subsecond`] 的 bit20 可读出当前半秒相位。
    //!
    //! # 写保护（KEY，手册表 13-6 注）
    //!
    //! 除 `KEY` / `ICR` 外全部锁定，**每次写都要先 `0xCA`→`0x53` 解锁**，
    //! 写完 `0xCA`→`0xFF` 重新上锁。本模块把解锁序列包进
    //! `critical_section::with`——IRQ 处理器（[`RtcIrq`]）也要动 IER，
    //! 两个上下文的 KEY 序列不会互相踩（中断插进解锁序列会静默丢写）。
    //!
    //! # 中断模型（异步闹钟）
    //!
    //! RTC 一条 NVIC 线 **IRQ2**（typelevel `interrupt::RTC`，全家族同号，
    //! PAC 中断表）。[`Rtc::wait_alarm`] 睡到闹钟 A 下一次匹配：等待期间才开
    //! `IER.ALARMA`（KEY 解锁写），IRQ 唤醒即单发压掉（防风暴，纪律同
    //! [`crate::uart`]）；ISR 标志由 poll 命中后经 `ICR`（R1W0）清除——
    //! **再次 `wait_alarm` 即等下一次匹配**，无需手动清标志。首次调用前若
    //! 担心热复位残留旧标志（IWDT 复位不清 RTC 域），先 [`Rtc::clear_flags`]。
    //!
    //! ```ignore
    //! embassy_cw32::bind_interrupts!(struct Irqs {
    //!     RTC => embassy_cw32::rtc::RtcIrq;
    //! });
    //! let mut rtc = Rtc::new(p.rtc, RtcClock::Lsi)?;
    //! rtc.set_datetime(&DateTime { year: 26, month: 10, day: 31, weekday: 6,
    //!                              hour: 23, minute: 59, second: 50 })?;
    //! rtc.start();
    //! rtc.set_alarm_a(&Alarm { weekmask: 0x7F, hour: None, minute: None,
    //!                          second: Some(0) });
    //! rtc.enable_alarm_a(true);
    //! rtc.wait_alarm().await; // 睡到 00:00:00，期间执行器照常调度其它任务
    //! ```
    //!
    //! ⚠ `wait_alarm` 依赖 RTC 中断已 `bind_interrupts!` 绑定到 [`RtcIrq`]；
    //! 未绑定则 NVIC 线落到弱默认向量，首个闹钟事件即挂死。IRQ 处理器会把
    //! **所有已挂起**的 RTC 中断源单发压掉——本版仅闹钟 A/B 提供异步等待，
    //! 其余源（周期中断/唤醒定时器/入侵检测）请经 [`Rtc::flags`] 轮询；
    //! 这四个特性阻塞版有，按需移植（本版未接入）。

    use core::future::poll_fn;
    use core::task::Poll;

    use embassy_sync::waitqueue::AtomicWaker;

    use crate::interrupt::{IrqHandler, interrupt};
    use crate::pac;
    use crate::pac::rtc as rc;

    pub use crate::pac::rtc::{
        IRQ_ALARMA, IRQ_ALARMB, IRQ_AWTIMER, IRQ_INTERVAL, IRQ_TAMP, IRQ_TAMPOV,
    };

    // ================= 时钟源 =================

    /// RTC 时钟源（CR1.SOURCE[10:8]，§13.5）
    #[allow(missing_docs)]
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub enum RtcClock {
        /// 外部 32.768kHz 晶体（最准；无晶振时起振等待超时）
        Lse,
        /// 内部 32.768kHz RC（出厂 TRIM，本板实测 32768Hz、误差 <1%）
        Lsi,
    }

    impl RtcClock {
        /// 源编码（`SRC_LSE=0 / SRC_LSI=2`，PAC §13.5）
        const fn code(self) -> u32 {
            match self {
                RtcClock::Lse => rc::SRC_LSE,
                RtcClock::Lsi => rc::SRC_LSI,
            }
        }
    }

    // ================= 日历数据 =================

    /// 日期时间（BCD 语义，字段是**十进制值**，模块内部转 BCD；同阻塞版结构体）
    #[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub struct DateTime {
        /// 年 0..=99（20xx）
        pub year: u8,
        /// 月 1..=12
        pub month: u8,
        /// 日 1..=31
        pub day: u8,
        /// 星期 0=周日、1..=6=周一到周六
        pub weekday: u8,
        /// 时 0..=23（24 小时制）
        pub hour: u8,
        /// 分 0..=59
        pub minute: u8,
        /// 秒 0..=59
        pub second: u8,
    }

    impl DateTime {
        /// 字段合法性（RTC 硬件也会拒写非法值，这里先挡一层）
        pub fn is_valid(&self) -> bool {
            self.year <= 99
                && (1..=12).contains(&self.month)
                && (1..=31).contains(&self.day)
                && self.weekday <= 6
                && self.hour <= 23
                && self.minute <= 59
                && self.second <= 59
        }
    }

    /// 闹钟配置（`None` = 该字段屏蔽，即"与它无关"；同阻塞版结构体）
    ///
    /// 例：`Alarm { hour: None, minute: None, second: Some(10), weekmask: 0x7F }`
    /// 表示"每分钟的第 10 秒"。
    #[derive(Debug, Clone, Copy)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub struct Alarm {
        /// 星期掩码：bit0=周日、bit1..=6=周一..周六；0x7F = 每天都响
        pub weekmask: u8,
        /// 时（None=屏蔽）
        pub hour: Option<u8>,
        /// 分（None=屏蔽）
        pub minute: Option<u8>,
        /// 秒（None=屏蔽）
        pub second: Option<u8>,
    }

    impl Alarm {
        /// 编码为 RTC_ALARMA/B 寄存器值（HOUREN 等三位：1 = 屏蔽）
        pub fn encode(&self) -> u32 {
            let mut v = ((self.weekmask as u32) & 0x7F) << rc::ALARM_WEEKMASK_POS;
            // 闹钟字段同样是 BCD（手册表 13-4 的 0x7F06 3000 → 06:30:00）
            match self.hour {
                Some(h) => v |= (to_bcd(h) & 0x3F) << rc::ALARM_HOUR_POS,
                None => v |= rc::ALARM_HOUREN,
            }
            match self.minute {
                Some(m) => v |= (to_bcd(m) & 0x7F) << rc::ALARM_MINUTE_POS,
                None => v |= rc::ALARM_MINUTEEN,
            }
            match self.second {
                Some(s) => v |= to_bcd(s) & rc::ALARM_SECOND_MASK,
                None => v |= rc::ALARM_SECONDEN,
            }
            v
        }
    }

    /// RTC 错误（语义同阻塞版 `rtc::RtcError`）
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub enum RtcError {
        /// 时钟源未就绪（LSE 未起振 / LSI 未稳定）
        ClockNotReady,
        /// 日期时间字段非法
        BadDateTime,
        /// 等待 `CR1.WAIT` 清零超时（RTC 同步逻辑异常）
        Timeout,
    }

    /// 十进制 → BCD
    #[inline]
    const fn to_bcd(v: u8) -> u32 {
        ((v / 10) as u32) << 4 | (v % 10) as u32
    }

    /// BCD → 十进制
    #[inline]
    const fn from_bcd(v: u32) -> u8 {
        (((v >> 4) & 0xF) * 10 + (v & 0xF)) as u8
    }

    // ================= waker =================

    /// 闹钟 A waker（IRQ2 派发用）
    static ALARMA_WAKER: AtomicWaker = AtomicWaker::new();
    /// 闹钟 B waker
    static ALARMB_WAKER: AtomicWaker = AtomicWaker::new();

    // ================= 驱动本体 =================

    /// 异步 RTC 驱动（单实例，RTC_BASE = 0x4000_4400，表 13-6）
    pub struct Rtc {
        regs: &'static pac::Rtc,
    }

    impl Rtc {
        /// 初始化：开 RTC 时钟门控、使能时钟源并等稳定、配 24 小时制与
        /// 2Hz TICKCLK
        ///
        /// `clk` 选 [`RtcClock::Lse`] / [`RtcClock::Lsi`] 时会自动使能对应
        /// 振荡器并等稳定，失败返回 [`RtcError::ClockNotReady`]（此时应换
        /// LSI 兜底——实例配方 `rtc_demo` 同款：先 LSE 失败退 LSI）。
        ///
        /// ⚠ 构造期为**短阻塞**（LSI 稳定等待忙等，STABLE 典型 <1ms、给足
        /// ~100ms；LSE 起振典型 200~500ms、给足 ~2s）——阻塞版
        /// `rcc::enable_lsi/enable_lse` 同款轮数，执行器尚未跑任务时调用无碍。
        pub fn new(regs: &'static pac::Rtc, clk: RtcClock) -> Result<Self, RtcError> {
            // APBEN2.RTC 时钟门控（bit1，KEY 保护；阻塞版 rcc.enable_rtc 配方：
            // 读改写 + KEY，保留其它外设使能位）
            let sysctrl = unsafe { &*(pac::SYSCTRL_BASE as *const crate::Sysctrl) };
            sysctrl
                .apben2
                .modify(|v| pac::sysctrl::KEY | (v & 0xFFFF) | pac::sysctrl::APBEN2_RTC);

            let me = Rtc { regs };
            let ok = match clk {
                RtcClock::Lse => me.enable_lse(),
                RtcClock::Lsi => me.enable_lsi(),
            };
            if !ok {
                return Err(RtcError::ClockNotReady);
            }
            // 24 小时制 + 1Hz 输出关 + 周期中断关（阻塞版配方）
            me.write_cr0(|v| (v & !(rc::CR0_INTERVAL_MASK | rc::CR0_RTC1HZ)) | rc::CR0_H24);
            me.write_cr1(|v| (v & !rc::CR1_SOURCE_MASK) | (clk.code() << rc::CR1_SOURCE_POS));
            // 32768Hz → TICKCLK 2Hz（复位默认值，显式写一遍更稳）
            me.with_unlocked(|| me.regs.psc.write(rc::PSC_32768_TO_2HZ));
            Ok(me)
        }

        /// 使能 LSI 并等稳定（阻塞版 `rcc::enable_lsi` 配方：出厂 TRIM 回填 +
        /// CR1[3] + LSI.STABLE 忙等 ~100ms）
        fn enable_lsi(&self) -> bool {
            let sysctrl = unsafe { &*(pac::SYSCTRL_BASE as *const crate::Sysctrl) };
            // 出厂 TRIM（9bit 有效范围 @0x0010_07C2）；未编程/异常值时保留
            // 硬件默认 TRIM
            let raw = unsafe { (pac::calib::LSI_TRIM_ADDR as *const u16).read_volatile() };
            if raw != 0 && raw != 0xFFFF && raw <= pac::sysctrl::LSI_TRIM_MASK as u16 {
                sysctrl
                    .lsi
                    .modify(|v| (v & !pac::sysctrl::LSI_TRIM_MASK) | raw as u32);
            }
            sysctrl
                .cr1
                .modify(|v| pac::sysctrl::KEY | (v & 0xFFFF) | pac::sysctrl::CR1_LSIEN);
            // STABLE 典型建立时间 <1ms；给足 ~100ms（复位默认 4MHz 下忙等）
            for _ in 0..200_000 {
                if sysctrl.lsi.read() & pac::sysctrl::LSI_STABLE != 0 {
                    return true;
                }
                core::hint::spin_loop();
            }
            false
        }

        /// 使能 LSE 并等稳定（阻塞版 `rcc::enable_lse` 配方：CR1[4] +
        /// LSE.STABLE 忙等 ~2s）
        ///
        /// 已稳定则直接返回 true（LSE 的 DRIVER/PINLOCK 只能被上电复位，
        /// 已在跑的晶体不重复动）。
        fn enable_lse(&self) -> bool {
            let sysctrl = unsafe { &*(pac::SYSCTRL_BASE as *const crate::Sysctrl) };
            if sysctrl.lse.read() & pac::sysctrl::LSE_STABLE != 0 {
                return true;
            }
            sysctrl
                .cr1
                .modify(|v| pac::sysctrl::KEY | (v & 0xFFFF) | pac::sysctrl::CR1_LSEEN);
            // 起振典型 200~500ms，给足 ~2s（板上无晶振时超时返回 false）
            for _ in 0..1_000_000 {
                if sysctrl.lse.read() & pac::sysctrl::LSE_STABLE != 0 {
                    return true;
                }
                core::hint::spin_loop();
            }
            false
        }

        /// 解锁 → 执行 → 上锁（每次写锁定寄存器都必须走这个）。
        ///
        /// 序列整体进 critical-section：IRQ 里的 IER 压位也是 KEY 序列，
        /// 中断插进解锁序列会把对方的写静默顶掉。
        fn with_unlocked<R>(&self, f: impl FnOnce() -> R) -> R {
            critical_section::with(|_| {
                self.regs.key.write(rc::KEY_UNLOCK1);
                self.regs.key.write(rc::KEY_UNLOCK2);
                let r = f();
                self.regs.key.write(rc::KEY_UNLOCK1);
                self.regs.key.write(0xFF); // 任意非 0x53 即重新上锁
                r
            })
        }

        fn write_cr0(&self, f: impl FnOnce(u32) -> u32) {
            let v = self.regs.cr0.read();
            let nv = f(v);
            self.with_unlocked(|| self.regs.cr0.write(nv));
        }

        fn write_cr1(&self, f: impl FnOnce(u32) -> u32) {
            let v = self.regs.cr1.read();
            let nv = f(v);
            self.with_unlocked(|| self.regs.cr1.write(nv));
        }

        fn write_cr2(&self, f: impl FnOnce(u32) -> u32) {
            let v = self.regs.cr2.read();
            let nv = f(v);
            self.with_unlocked(|| self.regs.cr2.write(nv));
        }

        /// 等待计时寄存器可访问（`CR1.WAIT==0`）
        ///
        /// ⚠ 仅**运行中**有意义：停表状态下 WAIT 不会清零，照手册等待会
        /// 误报超时（阻塞版实测结论，手册 §13.3.6 只写了运行中需等待）。
        pub fn wait_ready(&self) -> Result<(), RtcError> {
            for _ in 0..500_000 {
                if self.regs.cr1.read() & rc::CR1_WAIT == 0 {
                    return Ok(());
                }
                core::hint::spin_loop();
            }
            Err(RtcError::Timeout)
        }

        /// 启动计时
        pub fn start(&mut self) {
            self.write_cr0(|v| v | rc::CR0_START);
        }

        /// 停止计时（日期时间寄存器随即变为可写）
        pub fn stop(&mut self) {
            self.write_cr0(|v| v & !rc::CR0_START);
        }

        /// 是否正在计时
        pub fn is_running(&self) -> bool {
            self.regs.cr0.read() & rc::CR0_START != 0
        }

        /// 设置日期与时间（会自动先停表再启动，避免半写状态）
        pub fn set_datetime(&mut self, dt: &DateTime) -> Result<(), RtcError> {
            if !dt.is_valid() {
                return Err(RtcError::BadDateTime);
            }
            // 手册 §13.3.6：**运行中**写 DATE/TIME 才需要等 WAIT；停表后可直接写。
            // （实测：停表状态下 CR1.WAIT 不会清零，照手册等待会误报超时。）
            let was_running = self.is_running();
            if was_running {
                self.wait_ready()?;
                self.stop();
            }
            let date = ((dt.weekday as u32) << rc::DATE_WEEK_POS)
                | (to_bcd(dt.year) << rc::DATE_YEAR_POS)
                | (to_bcd(dt.month) << rc::DATE_MONTH_POS)
                | to_bcd(dt.day);
            let time = (to_bcd(dt.hour) << rc::TIME_HOUR_POS)
                | (to_bcd(dt.minute) << rc::TIME_MINUTE_POS)
                | to_bcd(dt.second);
            self.with_unlocked(|| {
                self.regs.date.write(date);
                self.regs.time.write(time);
            });
            if was_running {
                self.start();
            }
            Ok(())
        }

        /// 读当前日期时间（连读两次取稳定值，手册 §13.3.6 推荐做法）
        pub fn now(&self) -> DateTime {
            let mut last = self.read_once();
            for _ in 0..8 {
                let cur = self.read_once();
                if cur.0 == last.0 && cur.1 == last.1 {
                    return decode(cur.0, cur.1);
                }
                last = cur;
            }
            decode(last.0, last.1)
        }

        fn read_once(&self) -> (u32, u32) {
            // 先 date 后 time：time 的读取触发同步，顺序反了可能跨秒
            let d = self.regs.date.read();
            let t = self.regs.time.read();
            (d, t)
        }

        /// 亚秒计数（0..=2×(PSC2+1)-1，本配置下 0..=32767；0.5s 粒度由 bit20 给出）
        pub fn subsecond(&self) -> u32 {
            self.regs.sscnt.read() & (rc::SSCNT_HALF | rc::SSCNT_LOW_MASK)
        }

        // ---- 闹钟 ----

        /// 配置闹钟 A（不自动使能，需再调 [`Rtc::enable_alarm_a`]）
        pub fn set_alarm_a(&mut self, alarm: &Alarm) {
            let v = alarm.encode();
            self.with_unlocked(|| self.regs.alarma.write(v));
        }

        /// 配置闹钟 B
        pub fn set_alarm_b(&mut self, alarm: &Alarm) {
            let v = alarm.encode();
            self.with_unlocked(|| self.regs.alarmb.write(v));
        }

        /// 使能/关闭闹钟 A（`CR2.ALARMAEN`）
        pub fn enable_alarm_a(&mut self, on: bool) {
            self.write_cr2(|v| {
                if on {
                    v | rc::CR2_ALARMAEN
                } else {
                    v & !rc::CR2_ALARMAEN
                }
            });
        }

        /// 使能/关闭闹钟 B
        pub fn enable_alarm_b(&mut self, on: bool) {
            self.write_cr2(|v| {
                if on {
                    v | rc::CR2_ALARMBEN
                } else {
                    v & !rc::CR2_ALARMBEN
                }
            });
        }

        /// 异步睡到闹钟 A 下一次匹配（IER.ALARMA 中断 + waker）
        ///
        /// 语义：**睡到下一次匹配**——返回后 ISR.ALARMA 已清（ICR R1W0），
        /// 再次调用即等下一次匹配，无需手动清标志；若调用时匹配标志已在
        /// （如配置的目标秒刚过 / 热复位残留），本次**立即返回**，介意就先
        /// [`Rtc::clear_flags`]。等待期间才开 `IER.ALARMA`（KEY 解锁写），
        /// IRQ 唤醒即单发压掉（防风暴，纪律同 [`crate::uart`]）。
        ///
        /// 前置条件（模块文档详述）：RTC 中断已 `bind_interrupts!` 绑定到
        /// [`RtcIrq`]；NVIC 线由本方法幂等使能。闹钟本身须先
        /// [`Rtc::set_alarm_a`] + [`Rtc::enable_alarm_a`] 配好。
        pub async fn wait_alarm(&mut self) {
            self.wait_alarm_flag(IRQ_ALARMA, &ALARMA_WAKER).await;
        }

        /// 异步睡到闹钟 B 下一次匹配（同 [`Rtc::wait_alarm`]，配
        /// [`Rtc::set_alarm_b`] / [`Rtc::enable_alarm_b`] 使用）
        pub async fn wait_alarm_b(&mut self) {
            self.wait_alarm_flag(IRQ_ALARMB, &ALARMB_WAKER).await;
        }

        /// 闹钟等待公共体：NVIC 幂等使能 → poll 复查 ISR 标志（等待时才开
        /// 对应 IER 位，命中后压掉 IER 并清标志）
        async fn wait_alarm_flag(&mut self, flag: u32, waker: &'static AtomicWaker) {
            crate::interrupt::nvic_enable::<interrupt::RTC>();
            poll_fn(|cx| {
                waker.register(cx.waker());
                if self.regs.isr.read() & flag != 0 {
                    // 命中：压掉 IER 位（IRQ 侧已单发压过，这里幂等收尾）
                    // 并清标志（ICR R1W0），下次 wait 从干净状态开始
                    self.press_ier(flag);
                    self.clear_flags(flag);
                    Poll::Ready(())
                } else {
                    self.set_ier(flag); // 等待期间才开
                    Poll::Pending
                }
            })
            .await;
        }

        /// IER 置位（KEY 解锁写；仅等待期间打开）
        fn set_ier(&self, bit: u32) {
            self.with_unlocked(|| {
                let v = self.regs.ier.read();
                self.regs.ier.write(v | bit);
            });
        }

        /// IER 压位（KEY 解锁写；IRQ 唤醒后单发压掉，防中断风暴）
        fn press_ier(&self, bit: u32) {
            self.with_unlocked(|| {
                let v = self.regs.ier.read();
                self.regs.ier.write(v & !bit);
            });
        }

        // ---- 中断标志 ----

        /// 中断标志（`ISR`，RO）
        pub fn flags(&self) -> u32 {
            self.regs.isr.read()
        }

        /// 清中断标志（`ICR` 是 R1W0：写 0 清除；掩码用 [`IRQ_ALARMA`] 等）
        pub fn clear_flags(&self, mask: u32) {
            self.regs.icr.write(!mask);
        }

        /// 直接使能/关闭中断源（掩码用 [`IRQ_ALARMA`] 等；KEY 解锁写）
        ///
        /// 一般不用手工调——[`Rtc::wait_alarm`] 自带 IER 管理。⚠ 本版 IRQ
        /// 处理器会把所有已挂起的中断源单发压掉：为周期中断/唤醒定时器/
        /// 入侵检测开 IER 只会得到"IRQ 打进 [`RtcIrq`] 后被压掉"，请改用
        /// [`Rtc::flags`] 轮询。
        pub fn enable_interrupt(&mut self, mask: u32, on: bool) {
            self.with_unlocked(|| {
                let v = self.regs.ier.read();
                self.regs.ier.write(if on { v | mask } else { v & !mask });
            });
        }
    }

    fn decode(date: u32, time: u32) -> DateTime {
        DateTime {
            year: from_bcd((date & rc::DATE_YEAR_MASK) >> rc::DATE_YEAR_POS),
            month: from_bcd((date & rc::DATE_MONTH_MASK) >> rc::DATE_MONTH_POS),
            day: from_bcd(date & rc::DATE_DAY_MASK),
            weekday: ((date & rc::DATE_WEEK_MASK) >> rc::DATE_WEEK_POS) as u8,
            hour: from_bcd((time & rc::TIME_HOUR_MASK) >> rc::TIME_HOUR_POS),
            minute: from_bcd((time & rc::TIME_MINUTE_MASK) >> rc::TIME_MINUTE_POS),
            second: from_bcd(time & rc::TIME_SECOND_MASK),
        }
    }

    // ================= 中断派发 =================

    /// RTC 中断公共处理（IRQ2）：读 ISR，把所有已挂起源的 IER 位单发压掉
    /// （一次 KEY 序列完成，防任何源的风暴），闹钟 A/B 唤醒对应等待者。
    /// ISR 标志不动——poll 复查命中后经 ICR R1W0 清除（uart.rs 同款纪律）。
    fn on_irq(regs: &'static pac::Rtc) {
        let isr = regs.isr.read();
        if isr & rc::ICR_ALL != 0 {
            // 压掉全部已挂起源的 IER（下次 poll 会重新打开闹钟位；
            // 非闹钟源本版无等待者，压掉即止损）
            critical_section::with(|_| {
                let v = regs.ier.read();
                regs.ier.write(v & !rc::ICR_ALL);
            });
        }
        if isr & IRQ_ALARMA != 0 {
            ALARMA_WAKER.wake();
        }
        if isr & IRQ_ALARMB != 0 {
            ALARMB_WAKER.wake();
        }
    }

    /// RTC 中断标记（IRQ2，bind_interrupts! 绑定用）
    pub struct RtcIrq;
    impl IrqHandler<interrupt::RTC> for RtcIrq {
        fn call() {
            on_irq(unsafe { &*(pac::RTC_BASE as *const pac::Rtc) });
        }
    }

    #[cfg(test)]
    mod tests {
        use super::*;

        // 与阻塞版 cw32l012-hal/src/rtc.rs 的同名单元测试同源
        #[test]
        fn bcd_roundtrip() {
            for v in 0u8..=99 {
                assert_eq!(from_bcd(to_bcd(v)), v);
            }
            assert_eq!(to_bcd(59), 0x59);
            assert_eq!(to_bcd(23), 0x23);
            assert_eq!(to_bcd(9), 0x09);
        }

        #[test]
        fn datetime_encode_decode() {
            // 2026-10-31 周六 23:59:50
            let dt = DateTime {
                year: 26,
                month: 10,
                day: 31,
                weekday: 6,
                hour: 23,
                minute: 59,
                second: 50,
            };
            assert!(dt.is_valid());
            let date = (6 << rc::DATE_WEEK_POS)
                | (to_bcd(26) << rc::DATE_YEAR_POS)
                | (to_bcd(10) << rc::DATE_MONTH_POS)
                | to_bcd(31);
            let time = (to_bcd(23) << rc::TIME_HOUR_POS)
                | (to_bcd(59) << rc::TIME_MINUTE_POS)
                | to_bcd(50);
            assert_eq!(date, 0x0626_1031);
            assert_eq!(time, 0x0023_5950);
            assert_eq!(decode(date, time), dt);
        }

        #[test]
        fn invalid_datetime_rejected() {
            let base = DateTime {
                year: 26,
                month: 10,
                day: 31,
                weekday: 6,
                hour: 12,
                minute: 0,
                second: 0,
            };
            assert!(!DateTime { month: 13, ..base }.is_valid());
            assert!(!DateTime { day: 0, ..base }.is_valid());
            assert!(!DateTime { hour: 24, ..base }.is_valid());
            assert!(!DateTime { weekday: 7, ..base }.is_valid());
        }

        #[test]
        fn alarm_encoding_matches_manual_examples() {
            // 表 13-4 逐条：每天早上 6:30:00 → 0x7F06 3000
            let daily = Alarm {
                weekmask: 0x7F,
                hour: Some(6),
                minute: Some(30),
                second: Some(0),
            };
            assert_eq!(daily.encode(), 0x7F06_3000);
            // 每小时 50 分 → 0x7F80 5000
            let hourly = Alarm {
                weekmask: 0x7F,
                hour: None,
                minute: Some(50),
                second: Some(0),
            };
            assert_eq!(hourly.encode(), 0x7F80_5000);
            // 每分钟第 10 秒 → 0x7F80 8010（10 的 BCD 是 0x10，不是 0x0A）
            let each_min = Alarm {
                weekmask: 0x7F,
                hour: None,
                minute: None,
                second: Some(10),
            };
            assert_eq!(each_min.encode(), 0x7F80_8010);
        }

        #[test]
        fn alarm_all_none_only_matches_weekday() {
            let a = Alarm {
                weekmask: 0x01,
                hour: None,
                minute: None,
                second: None,
            };
            // weekmask[24]=0x01 + HOUREN[23] + MINUTEEN[15] + SECONDEN[7]
            assert_eq!(a.encode(), 0x0180_8080);
        }
    }
}

#[cfg(feature = "cw32l012")]
pub use l012::*;

// ================= 经典组分支（f030/f020/l031/l052/l083/r031/w031）=================
// 与 L012 同名不同构的另一套 RTC IP（任务面"RCMR 键值机"猜测不成立——
// 实锤为同款 0xCA→0x53 KEY + 另一套 ACCESS/WINDOW 运行中访问窗）。
// 7 家族 PAC 的 RTC 结构体与位段常量**逐字节同构**（cw32f030/f020/l031/
// l052/l083/r031/w031 七份 `pub struct RTC` diff 为空；f003 无 RTC 硬件）。
// 布局（cw32f030.h:3410-3626 / 生成 PAC `pub mod rtc`，其余 6 家族同）：
//
//   KEY@0x00(WO,8bit "Key = 0xCA - 0x53") / CR0@0x04(INTERVAL[2:0]/H24[3]/
//   RTC1HZ[6:5]/START[7]) / CR1@0x08(ACCESS[0] RW/WINDOW[1] RO/SOURCE[10:8]) /
//   CR2@0x0C(AWTSRC[2:0]/TAMPEDGE[3]/RTCOUT[5:4]/TAMPEN[6]/AWTEN[7]/
//   ALARMAEN[9]/ALARMBEN[10]) / COMPEN@0x10 / DATE@0x14(DAY[7:0]/MONTH[15:8]/
//   YEAR[23:16]/WEEK[26:24]) / TIME@0x18(SECOND[6:0]/MINUTE[14:8]/HOUR[21:16]) /
//   ALARMA@0x1C、ALARMB@0x20(SECOND[6:0]+SECONDEN[7]/MINUTE[14:8]+MINUTEEN[15]/
//   HOUR[21:16]+HOUREN[23]/WEEKMASK[30:24]——与 L012 闹钟位面同款) /
//   TAMPDATE/TAMPTIME@0x24/28 / AWTARR@0x2C / IER@0x30 / ISR@0x34(RO) /
//   ICR@0x38（R1W0 写 0 清，厂商 `CW_RTC->ICR = ~RTC_IT`）。
//
// 与 L012 的配方差异（逐条实锤）：
// - **无 PSC/SSCNT/AWTCNT**：日历由时钟源固定 32768 分频到 1Hz
//   （cw32-hal2 lse-qualified.yaml rtc_first_divisor=1/second=16384/
//   calendar=32768），无亚秒读数。LSI 标称 32800Hz（cw32f030_rcc.h:44
//   LSI_VALUE；cw32-hal2 electrical.yaml lsi_sysclk.nominal_hz=32800，
//   L031 组同值——rcc/l031_r031_w031/mod.rs:154"Factory-trimmed 32,800 Hz"），
//   走 LSI 时日历秒 ≈1.00098s（+85s/天量级）。
// - **运行中访问窗**（cw32f030_rtc.c:296-308 RTC_SetAlarm 配方）：RTC
//   运行中写 ALARMA/B、CR2、COMPEN、CR0.INTERVAL 须 `CR1.ACCESS=1` →
//   轮询 `CR1.WINDOW==1` → 写 → `ACCESS=0`；停表状态直接写即可（厂商
//   RTC_Cmd/RTC_SetTime 均不做窗）。IER/ICR **不**需要窗（RTC_ITConfig/
//   RTC_ClearITPendingBit 只走 KEY 解锁）。超时判定参照厂商
//   RTC_ACCESS_MAXTIMES=200（cw32f030_rtc.h:90）放宽为有界轮询。
// - **KEY 上锁字 0x55**：解锁 0xCA→0x53 同 L012，上锁 `0xCA→0x55`
//   （RTC_KEY_LOCKW，cw32f030_rtc.h:347；厂商 RTC_LOCK 宏）。
// - **时钟门控 APBEN1.RTC=bit3**（cw32f030.h:7528-7529；7 家族 PAC
//   sysctrl::apben1 同位），读改写**无 KEY**（lib.rs 门控写规范）。
// - **LSI/LSE 使能在 SYSCTRL.CR1[3]/[4]，CR1 带 0x5A5A KEY**
//   （厂商 RCC_LSI_Enable：`CR1 |= SYSCTRL_BYPASS | LSIEN`，rcc.c:305，
//   SYSCTRL_BYPASS=0x5A5A0000 rcc.h:47；生成 PAC 7 家族 sysctrl.cr1 均带
//   key[31:16] 位段）。LSE 起振另需 GPIOC14/15 模拟态（rcc.c:432-441），
//   本驱动不代管引脚——L012 分支同款取舍，文档注明。
// - 源编码同 L012：LSE=0、LSI=2（RTC_RTCCLK_FROM_LSI=2<<8，
//   cw32f030_rtc.h:137）；HSE 分频档（4..7<<8）不移植（精度/换算同 L012
//   分支的取舍）。
// - 闹钟匹配仍是秒字段 BCD + 星期掩码（位面与 L012 完全一致，
//   厂商 RTC_AlarmMask_All=0x7F808080 同款语义：1=屏蔽）。
//
// 中断模型与 L012 同款：IRQ2（7 家族 PAC interrupt::RTC=2）、IER 单发
// 纪律、ICR R1W0 清标志、异步 wait_alarm/wait_alarm_b。
//
// 配方出处（三源交叉核对）：厂商库 CW32F030_StandardPeripheralLib V2.2
// `cw32f030_rtc.c/h`；生成 PAC `pub mod rtc`（七家族位段 diff 为空）；
// cw32-hal2 embassy-cw32/src/rtc/mod.rs（独立实现的同款 WINDOW/ACCESS
// 协议与 0xCA/0x53/0x00 键序列，交叉印证）。
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
    //! 经典家族异步 RTC（f030/f020/l031/l052/l083/r031/w031）：
    //! BCD 日历读写 + 异步闹钟等待。
    //!
    //! ```ignore
    //! embassy_cw32::bind_interrupts!(struct Irqs {
    //!     RTC => embassy_cw32::rtc::ClassicRtcIrq;
    //! });
    //! let mut rtc = ClassicRtc::new(p.rtc, ClassicRtcClock::Lsi)?;
    //! rtc.set_datetime(&ClassicDateTime {
    //!     year: 26, month: 10, day: 31, weekday: 6,
    //!     hour: 23, minute: 59, second: 50,
    //! })?;
    //! rtc.start();
    //! rtc.set_alarm_a(&ClassicAlarm {
    //!     weekmask: 0x7F, hour: None, minute: None, second: Some(0),
    //! })?;
    //! rtc.enable_alarm_a(true)?;
    //! rtc.wait_alarm().await; // 睡到 00:00:00，期间执行器照常调度其它任务
    //! ```

    use core::future::poll_fn;
    use core::task::Poll;

    use embassy_sync::waitqueue::AtomicWaker;

    use crate::interrupt::{IrqHandler, interrupt};
    use crate::pac;

    // ---- 位常量面（cw32f030.h RTC_* 位段 + 生成 PAC `pub mod rtc`，
    //      七家族逐位段同款；与厂商宏值逐条核对）----
    /// CR0.INTERVAL[2:0]：周期中断间隔（0=关，厂商 RTC_INTERVAL_NONE）
    const CR0_INTERVAL_MASK: u32 = 0x7;
    /// CR0.H24[3]：1=24 小时制（RTC_HOUR24 = 1<<3，cw32f030_rtc.h:106）
    const CR0_H24: u32 = 1 << 3;
    /// CR0.RTC1HZ[6:5]：1Hz 输出源（本驱动关闭，0=NONE）
    const CR0_RTC1HZ_MASK: u32 = 0x3 << 5;
    /// CR0.START[7]：计时启动
    const CR0_START: u32 = 1 << 7;

    /// CR1.ACCESS[0]：运行中访问窗请求（RW）
    const CR1_ACCESS: u32 = 1 << 0;
    /// CR1.WINDOW[1]：访问窗指示（RO，1=可写）
    const CR1_WINDOW: u32 = 1 << 1;
    /// CR1.SOURCE[10:8]：时钟源
    const CR1_SOURCE_POS: u32 = 8;
    const CR1_SOURCE_MASK: u32 = 0x7 << CR1_SOURCE_POS;
    /// SOURCE 编码：外部 32.768kHz 晶体（RTC_RTCCLK_FROM_LSE=0）
    const SRC_LSE: u32 = 0;
    /// SOURCE 编码：内部低速 RC（RTC_RTCCLK_FROM_LSI=2<<8）
    const SRC_LSI: u32 = 2;

    /// CR2.ALARMAEN[9] / ALARMBEN[10]
    const CR2_ALARMAEN: u32 = 1 << 9;
    const CR2_ALARMBEN: u32 = 1 << 10;

    /// DATE：DAY[7:0] / MONTH[15:8] / YEAR[23:16] / WEEK[26:24]（BCD）
    const DATE_WEEK_POS: u32 = 24;
    const DATE_WEEK_MASK: u32 = 0x7 << DATE_WEEK_POS;
    const DATE_YEAR_POS: u32 = 16;
    const DATE_YEAR_MASK: u32 = 0xFF << DATE_YEAR_POS;
    const DATE_MONTH_POS: u32 = 8;
    const DATE_MONTH_MASK: u32 = 0xFF << DATE_MONTH_POS;
    const DATE_DAY_MASK: u32 = 0xFF;

    /// TIME：SECOND[6:0] / MINUTE[14:8] / HOUR[21:16]（BCD）
    const TIME_HOUR_POS: u32 = 16;
    const TIME_HOUR_MASK: u32 = 0x3F << TIME_HOUR_POS;
    const TIME_MINUTE_POS: u32 = 8;
    const TIME_MINUTE_MASK: u32 = 0x7F << TIME_MINUTE_POS;
    const TIME_SECOND_MASK: u32 = 0x7F;

    /// ALARMA/ALARMB 位面（与 L012 同款）：SECOND[6:0]+SECONDEN[7] /
    /// MINUTE[14:8]+MINUTEEN[15] / HOUR[21:16]+HOUREN[23] / WEEKMASK[30:24]
    const ALARM_WEEKMASK_POS: u32 = 24;
    const ALARM_HOUREN: u32 = 1 << 23;
    const ALARM_HOUR_POS: u32 = 16;
    const ALARM_MINUTEEN: u32 = 1 << 15;
    const ALARM_MINUTE_POS: u32 = 8;
    const ALARM_SECONDEN: u32 = 1 << 7;
    const ALARM_SECOND_MASK: u32 = 0x7F;

    /// IER/ISR/ICR：闹钟 A（bit0，厂商 RTC_IT_ALARMA）
    pub const IRQ_ALARMA: u32 = 1 << 0;
    /// 闹钟 B（bit1）
    pub const IRQ_ALARMB: u32 = 1 << 1;
    /// 唤醒定时器（bit2）
    pub const IRQ_AWTIMER: u32 = 1 << 2;
    /// 入侵检测（bit3）
    pub const IRQ_TAMP: u32 = 1 << 3;
    /// 入侵溢出（bit4）
    pub const IRQ_TAMPOV: u32 = 1 << 4;
    /// 周期中断（bit6）
    pub const IRQ_INTERVAL: u32 = 1 << 6;
    /// 全部已定义标志位（bit0..4 + bit6，bit5/7 保留）
    const ICR_ALL: u32 = 0x5F;

    /// RTC KEY 解锁第一字（RTC_KEY_WORD1）
    const KEY_UNLOCK1: u32 = 0xCA;
    /// RTC KEY 解锁第二字（RTC_KEY_WORD2）
    const KEY_UNLOCK2: u32 = 0x53;
    /// RTC KEY 重新上锁字（RTC_KEY_LOCKW，cw32f030_rtc.h:347）
    const KEY_LOCKW: u32 = 0x55;

    /// SYSCTRL CR1 写保护键（0x5A5A<<16；厂商 `SYSCTRL_BYPASS`，
    /// cw32f030_rcc.h:47；生成 PAC 7 家族 sysctrl.cr1 key[31:16] 位段同款）
    const SYSCTRL_CR1_KEY: u32 = 0x5A5A << 16;
    /// SYSCTRL.CR1[3]：LSI 振荡器使能
    const SYSCTRL_CR1_LSIEN: u32 = 1 << 3;
    /// SYSCTRL.CR1[4]：LSE 振荡器使能
    const SYSCTRL_CR1_LSEEN: u32 = 1 << 4;
    /// SYSCTRL.LSI/LSE.STABLE[15]：振荡器稳定标志（SYSCTRL_LSI_STABLE_Msk，
    /// cw32f030.h:7424/7431）
    const OSC_STABLE: u32 = 1 << 15;
    /// APBEN1 位：RTC=bit3 / WWDT=bit4 / IWDT=bit5（cw32f030.h APBEN1_f；
    /// 7 家族 PAC sysctrl::apben1 同位，无 KEY 保护）
    const APBEN1_RTC: u32 = 1 << 3;

    /// 运行中访问窗等待上限（spin 轮数）：厂商 RTC_ACCESS_MAXTIMES=200
    /// （cw32f030_rtc.h:90，单位未注明）放宽到远超 hal2 同协议实现
    /// （1000×10µs）的预算，超时按 Err 返回而非死等。
    const WINDOW_POLL: u32 = 200_000;
    /// LSI 稳定等待上限（~200ms @8MHz；STABLE 典型 <1ms）
    const LSI_POLL: u32 = 400_000;
    /// LSE 起振等待上限（~2-4s @8MHz；起振典型 200~500ms）
    const LSE_POLL: u32 = 2_000_000;
    /// ISR/DATE/TIME 连读稳定判定上限（厂商 do-while 连读，给上限防挂死）
    const READ_RETRIES: u32 = 8;

    // ================= 日历数据 =================

    /// 日期时间（BCD 语义，字段是**十进制值**，模块内部转 BCD；字段名与
    /// L012 分支 [`super::l012::DateTime`] 一致）
    #[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub struct ClassicDateTime {
        /// 年 0..=99（20xx）
        pub year: u8,
        /// 月 1..=12
        pub month: u8,
        /// 日 1..=31
        pub day: u8,
        /// 星期 0=周日、1..=6=周一到周六
        pub weekday: u8,
        /// 时 0..=23（24 小时制）
        pub hour: u8,
        /// 分 0..=59
        pub minute: u8,
        /// 秒 0..=59
        pub second: u8,
    }

    impl ClassicDateTime {
        /// 字段合法性（RTC 硬件也会拒写非法值，这里先挡一层）
        pub fn is_valid(&self) -> bool {
            self.year <= 99
                && (1..=12).contains(&self.month)
                && (1..=31).contains(&self.day)
                && self.weekday <= 6
                && self.hour <= 23
                && self.minute <= 59
                && self.second <= 59
        }
    }

    /// 闹钟配置（`None` = 该字段屏蔽；语义与 L012 分支 [`super::l012::Alarm`]
    /// 一致，位面同款：掩码位 1=与该字段无关）
    ///
    /// 例：`ClassicAlarm { hour: None, minute: None, second: Some(10),
    /// weekmask: 0x7F }` 表示"每分钟的第 10 秒"。
    #[derive(Debug, Clone, Copy)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub struct ClassicAlarm {
        /// 星期掩码：bit0=周日、bit1..=6=周一..周六；0x7F = 每天都响
        pub weekmask: u8,
        /// 时（None=屏蔽）
        pub hour: Option<u8>,
        /// 分（None=屏蔽）
        pub minute: Option<u8>,
        /// 秒（None=屏蔽）
        pub second: Option<u8>,
    }

    impl ClassicAlarm {
        /// 编码为 RTC_ALARMA/B 寄存器值（HOUREN/SECONDEN 等屏蔽位：1=屏蔽；
        /// 与厂商 `RTC_AlarmMask` 位面同款——RTC_AlarmMask_Hours=0x0080_0000、
        /// Minutes=0x0000_8000、Seconds=0x0000_0080、Week*=0x0100_0000..0x4000_0000，
        /// cw32f030_rtc.h:170-181）
        pub fn encode(&self) -> u32 {
            let mut v = ((self.weekmask as u32) & 0x7F) << ALARM_WEEKMASK_POS;
            // 闹钟字段同样是 BCD（经典组闹钟寄存器位段与 L012 同位面）
            match self.hour {
                Some(h) => v |= (to_bcd(h) & 0x3F) << ALARM_HOUR_POS,
                None => v |= ALARM_HOUREN,
            }
            match self.minute {
                Some(m) => v |= (to_bcd(m) & 0x7F) << ALARM_MINUTE_POS,
                None => v |= ALARM_MINUTEEN,
            }
            match self.second {
                Some(s) => v |= to_bcd(s) & ALARM_SECOND_MASK,
                None => v |= ALARM_SECONDEN,
            }
            v
        }
    }

    /// RTC 时钟源（CR1.SOURCE[10:8]；HSE 分频档不移植，取舍同 L012 分支）
    #[allow(missing_docs)]
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub enum ClassicRtcClock {
        /// 外部 32.768kHz 晶体（最准；无晶振/未配引脚时起振等待超时）
        Lse,
        /// 内部低速 RC（标称 32.8kHz，出厂 TRIM；走 LSI 时日历秒
        /// ≈1.00098s——32800/32768）
        Lsi,
    }

    impl ClassicRtcClock {
        /// 源编码（`RTC_RTCCLK_FROM_LSE=0 / FROM_LSI=2<<8`，cw32f030_rtc.h:136-137）
        const fn code(self) -> u32 {
            match self {
                ClassicRtcClock::Lse => SRC_LSE,
                ClassicRtcClock::Lsi => SRC_LSI,
            }
        }
    }

    /// RTC 错误（字段语义与 L012 分支 [`super::l012::RtcError`] 一致）
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub enum ClassicRtcError {
        /// 时钟源未就绪（LSE 未起振 / LSI 未稳定）
        ClockNotReady,
        /// 日期时间字段非法
        BadDateTime,
        /// 运行中访问窗（CR1.WINDOW）等待超时
        Timeout,
    }

    /// 十进制 → BCD
    #[inline]
    const fn to_bcd(v: u8) -> u32 {
        ((v / 10) as u32) << 4 | (v % 10) as u32
    }

    /// BCD → 十进制
    #[inline]
    const fn from_bcd(v: u32) -> u8 {
        (((v >> 4) & 0xF) * 10 + (v & 0xF)) as u8
    }

    // ================= waker =================

    /// 闹钟 A waker（IRQ2 派发用）
    static ALARMA_WAKER: AtomicWaker = AtomicWaker::new();
    /// 闹钟 B waker
    static ALARMB_WAKER: AtomicWaker = AtomicWaker::new();

    // ================= 驱动本体 =================

    /// 经典家族异步 RTC 驱动（单实例，RTC_BASE = 0x4000_2800）
    pub struct ClassicRtc {
        regs: &'static pac::RTC,
    }

    impl ClassicRtc {
        /// 初始化：开 APBEN1.RTC 门控（无 KEY 读改写）、使能时钟源并等稳定、
        /// 配 24 小时制；不启动计时（需再调 [`ClassicRtc::start`]）
        ///
        /// `clk` 选 [`ClassicRtcClock::Lse`] / [`ClassicRtcClock::Lsi`] 时会
        /// 自动使能对应振荡器并等稳定，失败返回
        /// [`ClassicRtcError::ClockNotReady`]（实例配方：先试 LSE 失败退 LSI，
        /// 同 L012 分支 `rtc_alarm` 示例的回退链）。
        ///
        /// ⚠ 构造期为**短阻塞**（LSI STABLE 典型 <1ms、给足 ~200ms；LSE
        /// 起振典型 200~500ms、给足 ~2-4s），执行器尚未跑任务时调用无碍。
        ///
        /// ⚠ LSE 依赖板上的 32.768kHz 晶体与引脚模拟态配置（F030 为
        /// PC14/PC15，厂商 `RCC_LSE_Enable` 会把两脚切 ANALOG——本驱动
        /// 不代管引脚，取舍同 L012 分支）；板上无晶振时请用 LSI。
        pub fn new(regs: &'static pac::RTC, clk: ClassicRtcClock) -> Result<Self, ClassicRtcError> {
            // APBEN1.RTC 门控（bit3；经典组 APBEN 无 KEY，读改写保留其它位
            // ——lib.rs 门控写规范，厂商 RTC_Init 亦为 `APBEN1_f.RTC = 1`，
            // cw32f030_rtc.c:82）
            let sysctrl = unsafe { &*(pac::SYSCTRL_BASE as *const crate::Sysctrl) };
            sysctrl.apben1.modify(|v| v | APBEN1_RTC);

            let me = ClassicRtc { regs };
            let ok = match clk {
                ClassicRtcClock::Lse => me.enable_lse(),
                ClassicRtcClock::Lsi => me.enable_lsi(),
            };
            if !ok {
                return Err(ClassicRtcError::ClockNotReady);
            }
            // 24 小时制 + 1Hz 输出关 + 周期中断关（厂商 RTC_Init 只配 H24，
            // INTERVAL/RTC1HZ 复位默认即关，显式清一遍更稳）
            me.write_cr0(|v| (v & !(CR0_INTERVAL_MASK | CR0_RTC1HZ_MASK)) | CR0_H24);
            // 时钟源：厂商 RTC_SetClockSource 为 CR1 整字写（同时清
            // ACCESS/WINDOW 位，cw32f030_rtc.c:515-522），本驱动读改写并
            // 显式清 ACCESS/WINDOW
            me.write_cr1(|v| {
                (v & !(CR1_SOURCE_MASK | CR1_ACCESS | CR1_WINDOW)) | (clk.code() << CR1_SOURCE_POS)
            });
            Ok(me)
        }

        /// 使能 LSI 并等 STABLE（厂商 `RCC_LSI_Enable` 配方：CR1 |= KEY |
        /// LSIEN 后忙等 LSI.STABLE，cw32f030_rcc.c:303-316）
        ///
        /// 与厂商差异：**不回填出厂 TRIM**（厂商写 TRIM@0x12602——该地址仅
        /// f030/f020 有厂商库实锤，七家族不通用；复位默认 TRIM 即出厂校准，
        /// cw32-hal2 经典组路径同样不动 TRIM）。
        fn enable_lsi(&self) -> bool {
            let sysctrl = unsafe { &*(pac::SYSCTRL_BASE as *const crate::Sysctrl) };
            // CR1 是 KEY 保护寄存器（0x5A5A<<16），读改写保留其它使能位
            sysctrl
                .cr1
                .modify(|v| SYSCTRL_CR1_KEY | ((v | SYSCTRL_CR1_LSIEN) & !(SYSCTRL_CR1_KEY)));
            for _ in 0..LSI_POLL {
                if sysctrl.lsi.read() & OSC_STABLE != 0 {
                    return true;
                }
                core::hint::spin_loop();
            }
            false
        }

        /// 使能 LSE 并等 STABLE（厂商 `RCC_LSE_Enable` 的 SYSCTRL 部分：
        /// CR1 |= KEY | LSEEN 后忙等 LSE.STABLE，cw32f030_rcc.c:424-458）
        ///
        /// 与厂商差异：不代管 GPIOC14/15 模拟态（板级职责）、不开时钟失效
        /// 切换（CLKCCS/LSECCS，厂商默认打开——失效自动切 LSI 属系统级
        /// 行为，本驱动不代做）。已稳定则直接返回 true。
        fn enable_lse(&self) -> bool {
            let sysctrl = unsafe { &*(pac::SYSCTRL_BASE as *const crate::Sysctrl) };
            if sysctrl.lse.read() & OSC_STABLE != 0 {
                return true;
            }
            sysctrl
                .cr1
                .modify(|v| SYSCTRL_CR1_KEY | ((v | SYSCTRL_CR1_LSEEN) & !(SYSCTRL_CR1_KEY)));
            for _ in 0..LSE_POLL {
                if sysctrl.lse.read() & OSC_STABLE != 0 {
                    return true;
                }
                core::hint::spin_loop();
            }
            false
        }

        /// 解锁 → 执行 → 上锁（每次写锁定寄存器都必须走这个）。
        ///
        /// 上锁字为 `0xCA→0x55`（RTC_KEY_LOCKW，厂商 RTC_LOCK 宏
        /// cw32f030_rtc.h:353-354）；解锁 `0xCA→0x53` 同 L012。
        /// 序列整体进 critical-section：IRQ 里的 IER/ICR 写也是 KEY 序列，
        /// 中断插进解锁序列会把对方的写静默顶掉。
        fn with_unlocked<R>(&self, f: impl FnOnce() -> R) -> R {
            critical_section::with(|_| {
                self.regs.key.write(KEY_UNLOCK1);
                self.regs.key.write(KEY_UNLOCK2);
                let r = f();
                self.regs.key.write(KEY_UNLOCK1);
                self.regs.key.write(KEY_LOCKW);
                r
            })
        }

        fn write_cr0(&self, f: impl FnOnce(u32) -> u32) {
            let v = self.regs.cr0.read();
            let nv = f(v);
            self.with_unlocked(|| self.regs.cr0.write(nv));
        }

        fn write_cr1(&self, f: impl FnOnce(u32) -> u32) {
            let v = self.regs.cr1.read();
            let nv = f(v);
            self.with_unlocked(|| self.regs.cr1.write(nv));
        }

        /// 是否正在计时
        pub fn is_running(&self) -> bool {
            self.regs.cr0.read() & CR0_START != 0
        }

        /// 启动计时
        pub fn start(&mut self) {
            self.write_cr0(|v| v | CR0_START);
        }

        /// 停止计时（日期时间寄存器随即变为可写）
        pub fn stop(&mut self) {
            self.write_cr0(|v| v & !CR0_START);
        }

        /// 运行中访问窗：`CR1.ACCESS=1` → 轮询 `WINDOW==1` → 执行 →
        /// `ACCESS=0`（厂商 RTC_SetAlarm/RTC_AlarmCmd 配方，
        /// cw32f030_rtc.c:296-308/349-369；**仅运行中需要**，停表直接写）
        ///
        /// 整体包进 KEY 解锁序列 + critical-section（与 IRQ 侧互斥）。
        fn with_running_access<R>(&self, f: impl FnOnce() -> R) -> Result<R, ClassicRtcError> {
            if !self.is_running() {
                // 停表状态无需访问窗（厂商同款：IS_RTC_START() 不成立时跳过）
                return Ok(self.with_unlocked(f));
            }
            self.with_unlocked(|| {
                self.regs.cr1.modify(|v| v | CR1_ACCESS);
                for _ in 0..WINDOW_POLL {
                    if self.regs.cr1.read() & CR1_WINDOW != 0 {
                        let r = f();
                        self.regs.cr1.modify(|v| v & !CR1_ACCESS);
                        return Some(r);
                    }
                    core::hint::spin_loop();
                }
                // 超时：撤回 ACCESS 请求，报错（厂商无界等，这里给上限）
                self.regs.cr1.modify(|v| v & !CR1_ACCESS);
                None
            })
            .ok_or(ClassicRtcError::Timeout)
        }

        /// 设置日期与时间（会自动先停表再启动，避免半写状态）
        ///
        /// 经典组 DATE/TIME 写没有 L012 的 WAIT 位同步语义——厂商流程
        /// （RTC_Init）也是停表后直写，运行中写需走 ACCESS/WINDOW 窗
        /// （cw32-hal2 的做法）；本驱动取厂商同款停表路径，简单且无窗超时。
        pub fn set_datetime(&mut self, dt: &ClassicDateTime) -> Result<(), ClassicRtcError> {
            if !dt.is_valid() {
                return Err(ClassicRtcError::BadDateTime);
            }
            let was_running = self.is_running();
            if was_running {
                self.stop();
            }
            let date = ((dt.weekday as u32) << DATE_WEEK_POS)
                | (to_bcd(dt.year) << DATE_YEAR_POS)
                | (to_bcd(dt.month) << DATE_MONTH_POS)
                | to_bcd(dt.day);
            let time = (to_bcd(dt.hour) << TIME_HOUR_POS)
                | (to_bcd(dt.minute) << TIME_MINUTE_POS)
                | to_bcd(dt.second);
            self.with_unlocked(|| {
                self.regs.date.write(date);
                self.regs.time.write(time);
            });
            if was_running {
                self.start();
            }
            Ok(())
        }

        /// 读当前日期时间（每寄存器连读两次取稳定值——厂商 RTC_GetTime/
        /// RTC_GetDate 的 do-while 配方，cw32f030_rtc.c:230-267）
        pub fn now(&self) -> ClassicDateTime {
            let (d, t) = self.read_stable();
            decode(d, t)
        }

        /// 连读直到稳定（DATE 先读，TIME 后读——顺序与 L012 分支一致）
        fn read_stable(&self) -> (u32, u32) {
            let mut last = self.read_once();
            for _ in 0..READ_RETRIES {
                let cur = self.read_once();
                if cur == last {
                    return cur;
                }
                last = cur;
            }
            last
        }

        fn read_once(&self) -> (u32, u32) {
            let d = self.regs.date.read();
            let t = self.regs.time.read();
            (d, t)
        }

        // ---- 闹钟 ----

        /// 配置闹钟 A（不自动使能，需再调 [`ClassicRtc::enable_alarm_a`]）
        ///
        /// 运行中写 ALARMA 走访问窗（厂商 RTC_SetAlarm 配方）。
        pub fn set_alarm_a(&mut self, alarm: &ClassicAlarm) -> Result<(), ClassicRtcError> {
            let v = alarm.encode();
            self.with_running_access(|| self.regs.alarma.write(v))
        }

        /// 配置闹钟 B（同 [`ClassicRtc::set_alarm_a`]）
        pub fn set_alarm_b(&mut self, alarm: &ClassicAlarm) -> Result<(), ClassicRtcError> {
            let v = alarm.encode();
            self.with_running_access(|| self.regs.alarmb.write(v))
        }

        /// 使能/关闭闹钟 A（`CR2.ALARMAEN`；运行中走访问窗——厂商
        /// RTC_AlarmCmd 配方）
        pub fn enable_alarm_a(&mut self, on: bool) -> Result<(), ClassicRtcError> {
            self.with_running_access(|| {
                self.write_cr2_raw(|v| {
                    if on {
                        v | CR2_ALARMAEN
                    } else {
                        v & !CR2_ALARMAEN
                    }
                })
            })
        }

        /// 使能/关闭闹钟 B（同 [`ClassicRtc::enable_alarm_a`]）
        pub fn enable_alarm_b(&mut self, on: bool) -> Result<(), ClassicRtcError> {
            self.with_running_access(|| {
                self.write_cr2_raw(|v| {
                    if on {
                        v | CR2_ALARMBEN
                    } else {
                        v & !CR2_ALARMBEN
                    }
                })
            })
        }

        /// CR2 写原语（调用方已持有 KEY 解锁/访问窗上下文时使用）
        fn write_cr2_raw(&self, f: impl FnOnce(u32) -> u32) {
            let v = self.regs.cr2.read();
            let nv = f(v);
            self.regs.cr2.write(nv);
        }

        /// 异步睡到闹钟 A 下一次匹配（IER.ALARMA 中断 + waker）
        ///
        /// 语义与 L012 分支 [`super::l012::Rtc::wait_alarm`] 一致：**睡到
        /// 下一次匹配**——返回后 ISR.ALARMA 已清（ICR R1W0），再次调用即等
        /// 下一次匹配；若调用时匹配标志已在，本次**立即返回**，介意就先
        /// [`ClassicRtc::clear_flags`]。等待期间才开 `IER.ALARMA`（KEY 解锁
        /// 写——IER 不需要访问窗，厂商 RTC_ITConfig 同款），IRQ 唤醒即单发
        /// 压掉（防风暴，纪律同 [`crate::uart`]）。
        ///
        /// 前置条件：RTC 中断已 `bind_interrupts!` 绑定到 [`ClassicRtcIrq`]；
        /// NVIC 线由本方法幂等使能。闹钟本身须先 [`ClassicRtc::set_alarm_a`]
        /// + [`ClassicRtc::enable_alarm_a`] 配好。
        pub async fn wait_alarm(&mut self) {
            self.wait_alarm_flag(IRQ_ALARMA, &ALARMA_WAKER).await;
        }

        /// 异步睡到闹钟 B 下一次匹配（同 [`ClassicRtc::wait_alarm`]，配
        /// [`ClassicRtc::set_alarm_b`] / [`ClassicRtc::enable_alarm_b`] 使用）
        pub async fn wait_alarm_b(&mut self) {
            self.wait_alarm_flag(IRQ_ALARMB, &ALARMB_WAKER).await;
        }

        /// 闹钟等待公共体：NVIC 幂等使能 → poll 复查 ISR 标志（等待时才开
        /// 对应 IER 位，命中后压掉 IER 并清标志）
        async fn wait_alarm_flag(&mut self, flag: u32, waker: &'static AtomicWaker) {
            crate::interrupt::nvic_enable::<interrupt::RTC>();
            poll_fn(|cx| {
                waker.register(cx.waker());
                if self.flags() & flag != 0 {
                    // 命中：压掉 IER 位（IRQ 侧已单发压过，这里幂等收尾）
                    // 并清标志（ICR R1W0），下次 wait 从干净状态开始
                    self.press_ier(flag);
                    self.clear_flags(flag);
                    Poll::Ready(())
                } else {
                    self.set_ier(flag); // 等待期间才开
                    Poll::Pending
                }
            })
            .await;
        }

        /// IER 置位（KEY 解锁写；仅等待期间打开）
        fn set_ier(&self, bit: u32) {
            self.with_unlocked(|| {
                let v = self.regs.ier.read();
                self.regs.ier.write(v | bit);
            });
        }

        /// IER 压位（KEY 解锁写；IRQ 唤醒后单发压掉，防中断风暴）
        fn press_ier(&self, bit: u32) {
            self.with_unlocked(|| {
                let v = self.regs.ier.read();
                self.regs.ier.write(v & !bit);
            });
        }

        // ---- 中断标志 ----

        /// 中断标志（`ISR`，RO；连读两次取稳定值——厂商 RTC_GetITState 的
        /// do-while 配方，cw32f030_rtc.c:117-127）
        pub fn flags(&self) -> u32 {
            let mut last = self.regs.isr.read();
            for _ in 0..READ_RETRIES {
                let cur = self.regs.isr.read();
                if cur == last {
                    return cur;
                }
                last = cur;
            }
            last
        }

        /// 清中断标志（`ICR` 是 R1W0：写 0 清除；掩码用 [`IRQ_ALARMA`] 等；
        /// 厂商 `CW_RTC->ICR = ~RTC_IT` 同款，KEY 解锁写）
        pub fn clear_flags(&self, mask: u32) {
            self.with_unlocked(|| self.regs.icr.write(!mask));
        }

        /// 直接使能/关闭中断源（掩码用 [`IRQ_ALARMA`] 等；KEY 解锁写）
        ///
        /// 一般不用手工调——[`ClassicRtc::wait_alarm`] 自带 IER 管理。⚠ 本版
        /// IRQ 处理器会把所有已挂起的中断源单发压掉：为周期中断/唤醒定时器/
        /// 入侵检测开 IER 只会得到"IRQ 打进 [`ClassicRtcIrq`] 后被压掉"，
        /// 请改用 [`ClassicRtc::flags`] 轮询。
        pub fn enable_interrupt(&mut self, mask: u32, on: bool) {
            self.with_unlocked(|| {
                let v = self.regs.ier.read();
                self.regs.ier.write(if on { v | mask } else { v & !mask });
            });
        }
    }

    fn decode(date: u32, time: u32) -> ClassicDateTime {
        ClassicDateTime {
            year: from_bcd((date & DATE_YEAR_MASK) >> DATE_YEAR_POS),
            month: from_bcd((date & DATE_MONTH_MASK) >> DATE_MONTH_POS),
            day: from_bcd(date & DATE_DAY_MASK),
            weekday: ((date & DATE_WEEK_MASK) >> DATE_WEEK_POS) as u8,
            hour: from_bcd((time & TIME_HOUR_MASK) >> TIME_HOUR_POS),
            minute: from_bcd((time & TIME_MINUTE_MASK) >> TIME_MINUTE_POS),
            second: from_bcd(time & TIME_SECOND_MASK),
        }
    }

    // ================= 中断派发 =================

    /// RTC 中断公共处理（IRQ2）：读 ISR，把所有已挂起源的 IER 位单发压掉
    /// （一次 KEY 序列完成，防任何源的风暴），闹钟 A/B 唤醒对应等待者。
    /// ISR 标志不动——poll 复查命中后经 ICR R1W0 清除（uart.rs 同款纪律）。
    fn on_irq(regs: &'static pac::RTC) {
        let isr = regs.isr.read();
        if isr & ICR_ALL != 0 {
            // 压掉全部已挂起源的 IER（下次 poll 会重新打开闹钟位；
            // 非闹钟源本版无等待者，压掉即止损）
            critical_section::with(|_| {
                let v = regs.ier.read();
                regs.ier.write(v & !ICR_ALL);
            });
        }
        if isr & IRQ_ALARMA != 0 {
            ALARMA_WAKER.wake();
        }
        if isr & IRQ_ALARMB != 0 {
            ALARMB_WAKER.wake();
        }
    }

    /// RTC 中断标记（IRQ2，bind_interrupts! 绑定用）
    pub struct ClassicRtcIrq;
    impl IrqHandler<interrupt::RTC> for ClassicRtcIrq {
        fn call() {
            on_irq(unsafe { &*(pac::RTC_BASE as *const pac::RTC) });
        }
    }

    #[cfg(test)]
    mod tests {
        use super::*;

        // 与 L012 分支同名测试同源（位面一致 → 期望值一致）
        #[test]
        fn classic_bcd_roundtrip() {
            for v in 0u8..=99 {
                assert_eq!(from_bcd(to_bcd(v)), v);
            }
            assert_eq!(to_bcd(59), 0x59);
            assert_eq!(to_bcd(23), 0x23);
        }

        #[test]
        fn classic_datetime_encode_decode() {
            // 2026-10-31 周六 23:59:50
            // DATE 位面 DAY[7:0]/MONTH[15:8]/YEAR[23:16]/WEEK[26:24]
            let dt = ClassicDateTime {
                year: 26,
                month: 10,
                day: 31,
                weekday: 6,
                hour: 23,
                minute: 59,
                second: 50,
            };
            assert!(dt.is_valid());
            let date = (6 << DATE_WEEK_POS)
                | (to_bcd(26) << DATE_YEAR_POS)
                | (to_bcd(10) << DATE_MONTH_POS)
                | to_bcd(31);
            let time = (to_bcd(23) << TIME_HOUR_POS) | (to_bcd(59) << TIME_MINUTE_POS) | to_bcd(50);
            assert_eq!(date, 0x0626_1031);
            assert_eq!(time, 0x0023_5950);
            assert_eq!(decode(date, time), dt);
        }

        #[test]
        fn classic_invalid_datetime_rejected() {
            let base = ClassicDateTime {
                year: 26,
                month: 10,
                day: 31,
                weekday: 6,
                hour: 12,
                minute: 0,
                second: 0,
            };
            assert!(!ClassicDateTime { month: 13, ..base }.is_valid());
            assert!(!ClassicDateTime { day: 0, ..base }.is_valid());
            assert!(!ClassicDateTime { hour: 24, ..base }.is_valid());
            assert!(!ClassicDateTime { weekday: 7, ..base }.is_valid());
        }

        #[test]
        fn classic_alarm_encoding_matches_vendor_mask_layout() {
            // 与 L012 分支同期望值（闹钟位面同款）+ 厂商掩码语义交叉核对：
            // RTC_AlarmMask_All=0x7F808080（全屏蔽）与"四字段全 None"等价
            let all_masked = ClassicAlarm {
                weekmask: 0x7F,
                hour: None,
                minute: None,
                second: None,
            };
            assert_eq!(all_masked.encode(), 0x7F80_8080);
            // 每天早上 6:30:00 → 0x7F06 3000（厂商 RTC_AlarmTypeDef 例同款）
            let daily = ClassicAlarm {
                weekmask: 0x7F,
                hour: Some(6),
                minute: Some(30),
                second: Some(0),
            };
            assert_eq!(daily.encode(), 0x7F06_3000);
            // 每分钟第 10 秒 → 0x7F80 8010（10 的 BCD 是 0x10，不是 0x0A）
            let each_min = ClassicAlarm {
                weekmask: 0x7F,
                hour: None,
                minute: None,
                second: Some(10),
            };
            assert_eq!(each_min.encode(), 0x7F80_8010);
            // 仅周日：WEEKMASK bit0=周日 → 0x0180 8080
            let sunday = ClassicAlarm {
                weekmask: 0x01,
                hour: None,
                minute: None,
                second: None,
            };
            assert_eq!(sunday.encode(), 0x0180_8080);
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
