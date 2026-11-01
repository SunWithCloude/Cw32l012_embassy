//! 异步定时器（§16）：GTIM1-4 单通道 PWM 输出 + 更新事件等待。
//!
//! **L012 家族先行**（`cw32l012` feature）；L010/L011/F030 的定时器
//! 寄存器面待实板回归后补齐（届时解除 lib.rs 的模块级 cfg）。
//!
//! 寄存器配方与阻塞版 HAL **同源**（`cw32l012-hal/src/gtim.rs` 的
//! `start_pwm`/`set_duty`，实机点灯验证见其 `examples/gtim_pwm.rs`）：
//! 边沿对齐 PWM 模式 1（CNT<CCR 输出有效电平，高有效）、PSC/ARR 十六位、
//! 影子寄存器预装载。第一版不做 ATIM 互补/死区。
//!
//! # 频率与占空比
//!
//! `f_pwm = PCLK / ((PSC+1) × (ARR+1))`，`start` 自动选 PSC 使 ARR 尽量大
//! （量化精度最高）。占空比以**万分比**（0..=10000）为满量程（同阻塞版
//! `PwmPin::MAX_DUTY`，喂给 `embedded_hal::pwm::SetDutyCycle` 时上层按
//! `max_duty_cycle()` 归一化）。
//!
//! # 中断模型
//!
//! PWM 本身是纯寄存器写（`set_duty_cycle` 无需异步等待），构造**不要求**
//! IRQ 绑定、不开 NVIC——与阻塞版一致。第二方法
//! [`GtimPwm::wait_update`] 用更新中断 UIF 做异步周期节拍
//! （等待时才开 DIER.UIE，IRQ 唤醒即压掉，防风暴）：
//!
//! ```ignore
//! embassy_cw32::bind_interrupts!(struct Irqs {
//!     GTIM1 => embassy_cw32::timer::Gtim1Irq;
//! });
//! ```
//!
//! ⚠ `wait_update` 依赖对应 GTIM 中断已绑定到本模块的
//! [`Gtim1Irq`]/[`Gtim2Irq`]/[`Gtim34Irq`]（GTIM3/4 共享 IRQ18 一条线）；
//! 未绑定则 NVIC 线落到弱默认向量，首个更新事件即挂死——用 PWM 不用
//! `wait_update` 则完全无需绑定。
//!
//! # 引脚（手册表 9-2，须配为对应 AF 复用输出）
//!
//! | 通道 | 常用引脚（阻塞版例程实机验证）      |
//! |------|-------------------------------------|
//! | GTIM1_CH1 | PB04（AF6）、PA00（AF10）等    |
//! | GTIM1_CH3 | PA03（AF6）等                  |
//!
//! 完整映射见手册表 9-2；引脚/AF 必须与所选通道对应。
//!
//! # 用法（GTIM1_CH1 @ PB04，1kHz、占空比 10%→90% 扫动）
//!
//! ```ignore
//! let pin = Pin::new(p.gpiob, 4);
//! let mut pwm = GtimPwm::new(p.gtim1, pin, 6, PwmConfig {
//!     channel: PwmChannel::Ch1,
//!     freq_hz: 1_000,
//!     duty_bp: 1_000,
//! })?;
//! loop {
//!     pwm.set_duty_bp(duty)?;
//!     pwm.wait_update().await; // 下一 PWM 周期节拍（需绑定 GTIM1 中断）
//! }
//! ```

use core::future::poll_fn;
use core::sync::atomic::Ordering;
use core::task::Poll;

use embassy_sync::waitqueue::AtomicWaker;

use crate::gpio::{GpioPort, Pin, Reg};
use crate::interrupt::{IrqHandler, interrupt};
use crate::pac;

// ================= 位常量面 =================
// 手写 PAC 的 gtim 常量与手册 §16.10 一致（STM32 风格）。GTIMx_ICR
// 已建模进 PAC（`Gtim::icr` @ 0x70，R1W0）；DIER.UIE 位常量 PAC 仍未
// 定义，此处按手册 §16.10.4 补。
use crate::pac::gtim as gt;

/// 更新中断使能（DIER bit0，§16.10.4——PAC 未定义故补）
const DIER_UIE: u32 = 1 << 0;

// ================= PWM 配置 =================

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
    /// 输出通道（须与引脚的 AF 复用通道一致，表 9-2）
    pub channel: PwmChannel,
    /// PWM 频率（Hz；PCLK/65536 ≤ freq ≤ PCLK/2 可实现）
    pub freq_hz: u32,
    /// 初始占空比（万分比 0..=10000）
    pub duty_bp: u16,
}

/// PWM 配置错误（语义同阻塞版 gtim::PwmError）
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum PwmError {
    /// 频率超出可实现范围（f > PCLK/2 或 f < PCLK/65536²）
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

// ================= 实例识别与 waker =================

/// 实例索引（0..3 = GTIM1..4，按基地址判定）
fn instance_of(regs: &pac::Gtim) -> usize {
    match regs as *const _ as usize {
        pac::GTIM1_BASE => 0,
        pac::GTIM2_BASE => 1,
        pac::GTIM3_BASE => 2,
        _ => 3, // GTIM4_BASE
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
    regs: &'static pac::Gtim,
    idx: usize,
    ch: PwmChannel,
}

impl GtimPwm {
    /// 初始化并启动一路 PWM（阻塞版 `Gtim::start_pwm` 配方）
    ///
    /// * `regs` — GTIM1..4 寄存器块（`p.gtim1` 等）
    /// * `pin` — PWM 输出引脚（内部配为 AF 复用输出：清模拟/开漏/上下拉，
    ///   DIR=0 输出，`set_af(af)`——同阻塞版 `into_af_output`）
    /// * `af` — 复用功能编号（表 9-2，如 GTIM1_CH1@PB04 = AF6）
    /// * `config` — 通道/频率/初始占空比
    pub fn new(
        regs: &'static pac::Gtim,
        pin: Pin<impl GpioPort>,
        af: u8,
        config: PwmConfig,
    ) -> Result<Self, PwmError> {
        if config.duty_bp > 10_000 {
            return Err(PwmError::BadDuty);
        }
        let pclk_hz = crate::PCLK_HZ.load(Ordering::Relaxed);
        let (psc, arr) = calc_pwm_psc_arr(pclk_hz, config.freq_hz).ok_or(PwmError::OutOfRange)?;
        // CCR = (ARR+1) × 万分比 / 10000（100% → CCR=ARR+1，CNT 永不大于 CCR → 恒有效）
        let ccr = ((arr as u32 + 1) * config.duty_bp as u32 / 10_000) as u16;

        // APBEN1.GTIMx 时钟（§4，bit6/7/11/12；KEY 写保护）。
        // 读改写 + KEY（阻塞版 rcc.rs set_bits 配方）：保留其它外设的
        // 使能位，不像整字写那样互相顶掉。
        let sysctrl = unsafe { &*(pac::SYSCTRL_BASE as *const crate::Sysctrl) };
        let idx = instance_of(regs);
        let gate = match idx {
            0 => pac::sysctrl::APBEN1_GTIM1,
            1 => pac::sysctrl::APBEN1_GTIM2,
            2 => pac::sysctrl::APBEN1_GTIM3,
            _ => pac::sysctrl::APBEN1_GTIM4,
        };
        sysctrl
            .apben1
            .modify(|v| pac::sysctrl::KEY | (v & 0xFFFF) | gate);

        // 引脚复用（阻塞版 into_af_output 配方：清附加态 → AF → 输出方向）
        let m = pin.mask();
        pin.port().mod_(Reg::OpEndrain, |v| v & !m);
        pin.port().mod_(Reg::Pur, |v| v & !m);
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
                regs.ccmr1.modify(|v| {
                    (v & !(gt::OC1M_MASK | gt::OC1PE | 0x3)) | gt::OC1M_PWM1 | gt::OC1PE
                });
                regs.ccer.modify(|v| (v & !gt::CC1P) | gt::CC1E);
            }
            PwmChannel::Ch2 => {
                regs.ccmr1.modify(|v| {
                    (v & !(gt::OC2M_MASK | gt::OC2PE | (0x3 << 8))) | gt::OC2M_PWM1 | gt::OC2PE
                });
                regs.ccer.modify(|v| (v & !gt::CC2P) | gt::CC2E);
            }
            PwmChannel::Ch3 => {
                regs.ccmr2.modify(|v| {
                    (v & !(gt::OC3M_MASK | gt::OC3PE | 0x3)) | gt::OC3M_PWM1 | gt::OC3PE
                });
                regs.ccer.modify(|v| (v & !gt::CC3P) | gt::CC3E);
            }
            PwmChannel::Ch4 => {
                regs.ccmr2.modify(|v| {
                    (v & !(gt::OC4M_MASK | gt::OC4PE | (0x3 << 8))) | gt::OC4M_PWM1 | gt::OC4PE
                });
                regs.ccer.modify(|v| (v & !gt::CC4P) | gt::CC4E);
            }
        }

        // UG 重装影子寄存器 → ARPE + EN 启动（§16.3.1.4）
        regs.egr.write(gt::EGR_UG);
        regs.cr1.write(gt::CR1_ARPE | gt::CR1_EN);
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
        let ccr = ((arr as u32 + 1) * duty_bp as u32 / 10_000) as u16;
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
        (((ccr as u32) * 10_000) / (arr as u32 + 1)) as u16
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
        self.regs.cr1.modify(|v| v & !gt::CR1_EN);
    }

    /// 恢复计数（保留已配置的 PSC/ARR/CCR）
    pub fn start(&mut self) {
        self.regs.cr1.modify(|v| v | gt::CR1_EN);
    }

    /// 异步等待一次更新事件（PWM 周期节拍；IER 使能 + waker 模式）
    ///
    /// 每次调用先清掉残留 UIF（`start` 配方的 EGR_UG 在 URS=0 时会置
    /// UIF，§16.10.5），再等**下一个**真实更新事件——返回时刻即一个
    /// PWM 周期边界。等待期间才开 DIER.UIE，IRQ 唤醒即压掉（防风暴）。
    ///
    /// 前置条件（模块文档详述）：对应 GTIM 中断已 `bind_interrupts!`
    /// 绑定到 [`Gtim1Irq`]/[`Gtim2Irq`]/[`Gtim34Irq`]；NVIC 线由本方法
    /// 幂等使能（GTIM3/4 共享 GTIM34 线）。
    pub async fn wait_update(&mut self) {
        let regs = self.regs;
        let idx = self.idx;
        // NVIC 线使能（幂等；GTIM3/4 共用 IRQ18）
        match idx {
            0 => crate::interrupt::nvic_enable::<interrupt::GTIM1>(),
            1 => crate::interrupt::nvic_enable::<interrupt::GTIM2>(),
            _ => crate::interrupt::nvic_enable::<interrupt::GTIM34>(),
        }
        // 前置清残留 UIF：PAC `Gtim::icr`（GTIMx_ICR @ 0x70）为 R1W0——
        // 写 0 清除对应标志、写 1 无功能，写 `!ICR_UIF` 即仅 bit0=0 清
        // 更新标志、其余位写 1 无副作用（与原裸指针写入 0xFFFF_FFFE 逐位一致）
        regs.icr.write(!gt::ICR_UIF);
        poll_fn(|cx| {
            GTIM_WAKER[idx].register(cx.waker());
            if regs.isr.read() & gt::ISR_UIF != 0 {
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

// ================= PSC/ARR 计算（照搬阻塞版） =================

/// 计算 PWM 的 (PSC, ARR)：PSC 取最小值使 ARR ≤ 0xFFFF。
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

// ================= 中断派发 =================

/// GTIM 更新中断公共处理：单发压掉 DIER.UIE 并唤醒等待者（ISR 的 UIF
/// 标志不动——poll 复查后由下次 wait_update 的前置 ICR 清除）。
fn on_irq(idx: usize, regs: &'static pac::Gtim) {
    if regs.isr.read() & gt::ISR_UIF != 0 {
        regs.dier.modify(|v| v & !DIER_UIE);
        GTIM_WAKER[idx].wake();
    }
}

/// GTIM1 中断标记（IRQ16，bind_interrupts! 绑定用）
pub struct Gtim1Irq;
impl IrqHandler<interrupt::GTIM1> for Gtim1Irq {
    fn call() {
        on_irq(0, unsafe { &*(pac::GTIM1_BASE as *const pac::Gtim) });
    }
}

/// GTIM2 中断标记（IRQ17）
pub struct Gtim2Irq;
impl IrqHandler<interrupt::GTIM2> for Gtim2Irq {
    fn call() {
        on_irq(1, unsafe { &*(pac::GTIM2_BASE as *const pac::Gtim) });
    }
}

/// GTIM3/4 共享中断标记（IRQ18 `GTIM34`——两个实例各自查各自 UIF，
/// 未挂起者空跑无害；时钟门控未开的实例 ISR 读回 0）
pub struct Gtim34Irq;
impl IrqHandler<interrupt::GTIM34> for Gtim34Irq {
    fn call() {
        on_irq(2, unsafe { &*(pac::GTIM3_BASE as *const pac::Gtim) });
        on_irq(3, unsafe { &*(pac::GTIM4_BASE as *const pac::Gtim) });
    }
}

#[cfg(test)]
mod tests {
    use super::*;

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
}
