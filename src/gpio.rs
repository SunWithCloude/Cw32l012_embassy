//! GPIO：异步 `Input`（等边沿 / 等电平）与 `Output`。
//!
//! 引脚类型 `Pin<P>` 泛型于端口（[`GpioPort`] 特征）：L012 单一 `Gpio`
//! 端口；F030 经典家族的 GPIOA/B、GPIOC、GPIOF 寄存器集各不相同
//! （GPIOC 无 AFRL、GPIOF 无 AFRH），特征按各自实际字段实现。
//!
//! CW32 每端口共享一个中断，等待期间注册 per-pin waker，端口中断里
//! 按位唤醒（软件触发位图防丢唤醒）。
//!
//! ```ignore
//! embassy_cw32::bind_interrupts!(struct Irqs { GPIOA => embassy_cw32::gpio::GpioAIrq; });
//! let btn = Input::new(Pin::new(p.gpioa, 0), Pull::Up);
//! btn.wait_for_falling_edge().await;
//! ```

use core::future::poll_fn;
use core::sync::atomic::Ordering;
use core::task::Poll;

use embassy_sync::waitqueue::AtomicWaker;

use crate::interrupt::{IrqHandler, interrupt};
use crate::pac;

// ================= 端口特征 =================

/// GPIO 端口寄存器访问面（驱动所需字段的最小集合 + 复用设置）。
///
/// 各家族端口结构体实现本特征；复用寄存器按端口实际具备的段实现
/// （如 F030 的 GPIOC 仅 AFRH——高脚复用，低脚无复用能力）。
pub trait GpioPort: 'static {
    /// 端口中断槽位（0..3 = A/B/C/F，按基地址判定——L012 四端口共用
    /// 一个结构体类型，无法用关联常量区分）
    fn bank(&self) -> usize;
    /// 读寄存器
    fn rd(&self, r: Reg) -> u32;
    /// 写寄存器
    fn wr(&self, r: Reg, v: u32);
    /// 读改写寄存器
    fn mod_(&self, r: Reg, f: impl FnOnce(u32) -> u32);
    /// 设置复用功能（AFRL pin0..7 / AFRH pin8..15，每脚 4bit；
    /// 端口不具备对应段时 panic）
    fn set_af(&self, pin: u8, af: u8);
}

/// 驱动用到的端口寄存器（功能式访问,隔离各 PAC 字段类型差异：
/// 如 F030 头文件把 ISR/ICR/IDR/BRR 都标成 RW,而 L012 是 Ro/Wo）
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Reg {
    /// 方向(1=输入)
    Dir,
    /// 模拟开关
    Analog,
    /// 开漏输出(F030 经典家族 I2C 必需)
    OpEndrain,
    /// 下拉
    Pdr,
    /// 上拉
    Pur,
    /// 上升沿使能
    Riseie,
    /// 下降沿使能
    Fallie,
    /// 中断标志
    Isr,
    /// 中断清零
    Icr,
    /// 输入数据
    Idr,
    /// 输出数据
    Odr,
    /// 位清零
    Brr,
    /// 位置位清零
    Bsrr,
    /// 位翻转
    Tog,
}

macro_rules! impl_port {
    ($typ:ty, $bank:expr, $isr:ident, $icr:ident, $idr:ident, $brr:ident) => {
        fn bank(&self) -> usize {
            $bank(self)
        }
        fn rd(&self, r: Reg) -> u32 {
            match r {
                Reg::Dir => self.dir.read(),
                Reg::Analog => self.analog.read(),
                Reg::OpEndrain => self.opendrain.read(),
                Reg::Pdr => self.pdr.read(),
                Reg::Pur => self.pur.read(),
                Reg::Riseie => self.riseie.read(),
                Reg::Fallie => self.fallie.read(),
                Reg::Isr => self.$isr.read(),
                Reg::Idr => self.$idr.read(),
                Reg::Odr => self.odr.read(),
                _ => unreachable!("只写寄存器不可读"),
            }
        }
        fn wr(&self, r: Reg, v: u32) {
            match r {
                Reg::Dir => self.dir.write(v),
                Reg::Analog => self.analog.write(v),
                Reg::OpEndrain => self.opendrain.write(v),
                Reg::Pdr => self.pdr.write(v),
                Reg::Pur => self.pur.write(v),
                Reg::Riseie => self.riseie.write(v),
                Reg::Fallie => self.fallie.write(v),
                Reg::Icr => self.$icr.write(v),
                Reg::Brr => self.$brr.write(v),
                Reg::Bsrr => self.bsrr.write(v),
                Reg::Tog => self.tog.write(v),
                _ => unreachable!("只读寄存器不可写"),
            }
        }
        fn mod_(&self, r: Reg, f: impl FnOnce(u32) -> u32) {
            match r {
                Reg::Dir => self.dir.modify(f),
                Reg::Analog => self.analog.modify(f),
                Reg::OpEndrain => self.opendrain.modify(f),
                Reg::Pdr => self.pdr.modify(f),
                Reg::Pur => self.pur.modify(f),
                Reg::Riseie => self.riseie.modify(f),
                Reg::Fallie => self.fallie.modify(f),
                Reg::Odr => self.odr.modify(f),
                _ => unreachable!("该寄存器只支持读/写"),
            }
        }
    };
}

#[cfg(feature = "cw32l012")]
impl GpioPort for pac::Gpio {
    impl_port!(
        pac::Gpio,
        |s: &Self| match s as *const _ as usize {
            pac::GPIOA_BASE => 0,
            pac::GPIOB_BASE => 1,
            pac::GPIOC_BASE => 2,
            _ => 3,
        },
        isr,
        icr,
        idr,
        brr
    );
    fn set_af(&self, pin: u8, af: u8) {
        set_af_impl(Some(&self.afrl), Some(&self.afrh), pin, af)
    }
}

#[cfg(feature = "cw32f030")]
impl GpioPort for pac::GPIO {
    impl_port!(
        pac::GPIO,
        |s: &Self| match s as *const _ as usize {
            pac::GPIOA_BASE => 0,
            pac::GPIOB_BASE => 1,
            _ => unreachable!("GPIO 基地址不在 A/B"),
        },
        isr,
        icr,
        idr,
        brr
    );
    fn set_af(&self, pin: u8, af: u8) {
        set_af_impl(Some(&self.afrl), Some(&self.afrh), pin, af)
    }
}

#[cfg(feature = "cw32f030")]
impl GpioPort for pac::GPIOC {
    impl_port!(pac::GPIOC, |_s: &Self| 2, isr, icr, idr, brr);
    fn set_af(&self, pin: u8, af: u8) {
        assert!(pin >= 8, "GPIOC 低脚无复用功能");
        set_af_impl(None, Some(&self.afrh), pin, af)
    }
}

#[cfg(feature = "cw32f030")]
impl GpioPort for pac::GPIOF {
    impl_port!(pac::GPIOF, |_s: &Self| 3, isr, icr, idr, brr);
    fn set_af(&self, pin: u8, af: u8) {
        assert!(pin < 8, "GPIOF 高脚无复用功能");
        set_af_impl(Some(&self.afrl), None, pin, af)
    }
}

/// AFRL/AFRH 公共写入（缺失的段传 `None`——对应脚位范围不存在，
/// 例如 F030 的 GPIOC 无低脚、GPIOF 无高脚）
fn set_af_impl(
    afrl: Option<&crate::pac::Rw<u32>>,
    afrh: Option<&crate::pac::Rw<u32>>,
    pin: u8,
    af: u8,
) {
    let m4 = 0xF << ((pin % 8) * 4);
    let v4 = ((af as u32) & 0xF) << ((pin % 8) * 4);
    if pin < 8 {
        afrl.expect("该端口无 AFRL(低脚无复用)")
            .modify(|v| (v & !m4) | v4);
    } else {
        afrh.expect("该端口无 AFRH(高脚无复用)")
            .modify(|v| (v & !m4) | v4);
    }
}

// ================= 引脚 =================

/// 引脚（端口 + 编号;端口泛型保证寄存器访问面）
pub struct Pin<P: GpioPort> {
    port: &'static P,
    pin: u8,
}

impl<P: GpioPort> Pin<P> {
    /// 绑定端口与引脚号（0..15，须与封装实际可用引脚一致）
    pub fn new(port: &'static P, pin: u8) -> Self {
        assert!(pin < 16);
        Self { port, pin }
    }

    /// 引脚位掩码
    pub fn mask(&self) -> u32 {
        1 << self.pin
    }

    /// 端口访问(供驱动做特征之外的专项配置,如 I2C 开漏)
    pub fn port(&self) -> &'static P {
        self.port
    }

    /// 设置复用功能（转发给端口实现）
    pub fn set_af(&self, af: u8) {
        self.port.set_af(self.pin, af);
    }

    /// 配置为**模拟引脚**（`GPIOx_ANALOG.PINy=1`），供 ADC/DAC/比较器
    /// 等模拟外设使用（手册 §25 转换通道要求）。[`Pin::new`] 及
    /// `Output`/`Input` 构造会清掉 ANALOG 位（数字模式），模拟用途
    /// 必须在把引脚交给外设前显式调用本方法。
    pub fn into_analog(self) -> Self {
        let m = self.mask();
        self.port.mod_(Reg::Analog, |v| v | m); // 1 = 模拟
        self
    }
}

/// 上/下拉配置。
///
/// ⚠ 下拉仅 PF3 有实际电阻（L012 手册 §9.6.3），其余引脚
/// [`Pull::Down`] 等同 [`Pull::None`]。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Pull {
    /// 浮空
    None,
    /// 上拉
    Up,
    /// 下拉（仅 L012 的 PF3 有效）
    Down,
}

/// 输出电平
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Level {
    /// 低电平
    Low,
    /// 高电平
    High,
}

impl Level {
    /// 翻转
    pub fn toggle(&mut self) {
        *self = match *self {
            Level::Low => Level::High,
            Level::High => Level::Low,
        };
    }
}

// ================= 输出 =================

/// 数字输出（推挽）。
pub struct Output<P: GpioPort> {
    pin: Pin<P>,
}

impl<P: GpioPort> Output<P> {
    /// 配置为推挽输出（方向 DIR=0、数字模式 ANALOG=0），并置初始电平。
    pub fn new(pin: Pin<P>, initial: Level) -> Self {
        let m = pin.mask();
        pin.port.mod_(Reg::Analog, |v| v & !m);
        pin.port.mod_(Reg::Dir, |v| v & !m);
        match initial {
            Level::Low => pin.port.wr(Reg::Brr, m),
            Level::High => pin.port.wr(Reg::Bsrr, m),
        }
        Self { pin }
    }

    /// 置高（BSRR 原子置位）
    pub fn set_high(&mut self) {
        self.pin.port.wr(Reg::Bsrr, self.pin.mask());
    }

    /// 置低（BRR 原子清零）
    pub fn set_low(&mut self) {
        self.pin.port.wr(Reg::Brr, self.pin.mask());
    }

    /// 翻转（TOG 原子翻转）
    pub fn toggle(&mut self) {
        self.pin.port.wr(Reg::Tog, self.pin.mask());
    }

    /// 设置电平
    pub fn set_level(&mut self, level: Level) {
        match level {
            Level::Low => self.set_low(),
            Level::High => self.set_high(),
        }
    }

    /// 读回输出锁存电平（ODR）
    pub fn get_output_level(&self) -> Level {
        if self.pin.port.rd(Reg::Odr) & self.pin.mask() != 0 {
            Level::High
        } else {
            Level::Low
        }
    }

    /// 释放引脚
    pub fn into_pin(self) -> Pin<P> {
        self.pin
    }
}

impl<P: GpioPort> embedded_hal::digital::ErrorType for Output<P> {
    type Error = core::convert::Infallible;
}

impl<P: GpioPort> embedded_hal::digital::OutputPin for Output<P> {
    fn set_low(&mut self) -> Result<(), Self::Error> {
        Output::set_low(self);
        Ok(())
    }
    fn set_high(&mut self) -> Result<(), Self::Error> {
        Output::set_high(self);
        Ok(())
    }
}

impl<P: GpioPort> embedded_hal::digital::StatefulOutputPin for Output<P> {
    fn is_set_high(&mut self) -> Result<bool, Self::Error> {
        Ok(self.get_output_level() == Level::High)
    }
    fn is_set_low(&mut self) -> Result<bool, Self::Error> {
        Ok(self.get_output_level() == Level::Low)
    }
}

// ================= 异步输入 =================

/// 数字输入（可上/下拉）。异步等待边沿与电平。
pub struct Input<P: GpioPort> {
    pin: Pin<P>,
}

/// 每端口 16 引脚的 waker 表
static WAKERS: [[AtomicWaker; 16]; 4] = [const { [const { AtomicWaker::new() }; 16] }; 4];

/// 每端口的软件触发位图（IRQ 清硬件标志后置位，任务侧消费；
/// 全程 critical section，避免丢唤醒）
static TRIGGERED: [core::sync::atomic::AtomicU32; 4] =
    [const { core::sync::atomic::AtomicU32::new(0) }; 4];

impl<P: GpioPort> Input<P> {
    /// 配置为数字输入（DIR=1、ANALOG=0）并设置上/下拉。
    pub fn new(pin: Pin<P>, pull: Pull) -> Self {
        let m = pin.mask();
        pin.port.mod_(Reg::Analog, |v| v & !m);
        pin.port.mod_(Reg::Dir, |v| v | m);
        match pull {
            Pull::None => {
                pin.port.mod_(Reg::Pur, |v| v & !m);
                pin.port.mod_(Reg::Pdr, |v| v & !m);
            }
            Pull::Up => {
                pin.port.mod_(Reg::Pur, |v| v | m);
                pin.port.mod_(Reg::Pdr, |v| v & !m);
            }
            Pull::Down => {
                pin.port.mod_(Reg::Pur, |v| v & !m);
                pin.port.mod_(Reg::Pdr, |v| v | m);
            }
        }
        Self { pin }
    }

    /// 当前电平（IDR）
    pub fn get_level(&self) -> Level {
        if self.pin.port.rd(Reg::Idr) & self.pin.mask() != 0 {
            Level::High
        } else {
            Level::Low
        }
    }

    /// 释放引脚
    pub fn into_pin(self) -> Pin<P> {
        self.pin
    }

    /// 等待任一边沿（上升或下降）。
    pub async fn wait_for_any_edge(&mut self) {
        self.wait_edge(true, true).await;
    }

    /// 等待上升沿。
    pub async fn wait_for_rising_edge(&mut self) {
        self.wait_edge(true, false).await;
    }

    /// 等待下降沿。
    pub async fn wait_for_falling_edge(&mut self) {
        self.wait_edge(false, true).await;
    }

    /// 等待高电平（边沿触发 + 电平复查）。
    pub async fn wait_for_high(&mut self) {
        loop {
            if self.get_level() == Level::High {
                return;
            }
            self.wait_for_any_edge().await;
        }
    }

    /// 等待低电平（边沿触发 + 电平复查）。
    pub async fn wait_for_low(&mut self) {
        loop {
            if self.get_level() == Level::Low {
                return;
            }
            self.wait_for_any_edge().await;
        }
    }

    /// 边沿等待公共实现。
    async fn wait_edge(&mut self, rise: bool, fall: bool) {
        let m = self.pin.mask();
        let port = self.pin.port;
        let bank = port.bank();
        // 使能 NVIC 中断线（按槽位;GPIOA..F 在两代家族中断号相同布局）
        match bank {
            0 => crate::interrupt::nvic_enable::<interrupt::GPIOA>(),
            1 => crate::interrupt::nvic_enable::<interrupt::GPIOB>(),
            2 => crate::interrupt::nvic_enable::<interrupt::GPIOC>(),
            _ => crate::interrupt::nvic_enable::<interrupt::GPIOF>(),
        }
        critical_section::with(|_| {
            port.wr(Reg::Icr, m);
            TRIGGERED[bank].store(
                TRIGGERED[bank].load(Ordering::Relaxed) & !m,
                Ordering::Relaxed,
            );
        });
        if rise {
            port.mod_(Reg::Riseie, |v| v | m);
        }
        if fall {
            port.mod_(Reg::Fallie, |v| v | m);
        }
        poll_fn(|cx| {
            WAKERS[bank][self.pin.pin as usize].register(cx.waker());
            let hit = critical_section::with(|_| {
                let t = TRIGGERED[bank].load(Ordering::Relaxed);
                if t & m != 0 {
                    TRIGGERED[bank].store(t & !m, Ordering::Relaxed);
                    true
                } else {
                    false
                }
            });
            if hit { Poll::Ready(()) } else { Poll::Pending }
        })
        .await;
        port.mod_(Reg::Riseie, |v| v & !m);
        port.mod_(Reg::Fallie, |v| v & !m);
        port.wr(Reg::Icr, m);
    }
}

impl<P: GpioPort> embedded_hal::digital::ErrorType for Input<P> {
    type Error = core::convert::Infallible;
}

impl<P: GpioPort> embedded_hal::digital::InputPin for Input<P> {
    fn is_high(&mut self) -> Result<bool, Self::Error> {
        Ok(self.get_level() == Level::High)
    }
    fn is_low(&mut self) -> Result<bool, Self::Error> {
        Ok(self.get_level() == Level::Low)
    }
}

// ================= 端口中断派发 =================

/// 端口中断公共处理：置软件触发位、清硬件标志、按位唤醒。
pub(crate) fn on_irq(bank: usize, port: &impl GpioPort) {
    let pending = port.rd(Reg::Isr);
    critical_section::with(|_| {
        let t = TRIGGERED[bank].load(Ordering::Relaxed);
        TRIGGERED[bank].store(t | pending, Ordering::Relaxed);
    });
    port.wr(Reg::Icr, pending);
    for (i, w) in WAKERS[bank].iter().enumerate() {
        if pending & (1 << i) != 0 {
            w.wake();
        }
    }
}

/// 生成端口中断标记类型与派发
macro_rules! port_irq {
    ($irq:ident, $handler:ident, $typ:ty, $base:expr, $bank:expr) => {
        /// 端口中断标记（bind_interrupts! 绑定用）
        pub struct $handler;
        impl IrqHandler<interrupt::$irq> for $handler {
            fn call() {
                let port: &$typ = unsafe { &*($base as *const $typ) };
                on_irq($bank, port);
            }
        }
    };
}

#[cfg(feature = "cw32l012")]
port_irq!(GPIOA, GpioAIrq, pac::Gpio, pac::GPIOA_BASE, 0);
#[cfg(feature = "cw32l012")]
port_irq!(GPIOB, GpioBIrq, pac::Gpio, pac::GPIOB_BASE, 1);
#[cfg(feature = "cw32l012")]
port_irq!(GPIOC, GpioCIrq, pac::Gpio, pac::GPIOC_BASE, 2);
#[cfg(feature = "cw32l012")]
port_irq!(GPIOF, GpioFIrq, pac::Gpio, pac::GPIOF_BASE, 3);

#[cfg(feature = "cw32f030")]
port_irq!(GPIOA, GpioAIrq, pac::GPIO, pac::GPIOA_BASE, 0);
#[cfg(feature = "cw32f030")]
port_irq!(GPIOB, GpioBIrq, pac::GPIO, pac::GPIOB_BASE, 1);
#[cfg(feature = "cw32f030")]
port_irq!(GPIOC, GpioCIrq, pac::GPIOC, pac::GPIOC_BASE, 2);
#[cfg(feature = "cw32f030")]
port_irq!(GPIOF, GpioFIrq, pac::GPIOF, pac::GPIOF_BASE, 3);
