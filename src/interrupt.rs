//! 中断绑定（embassy 风格编译期检查）。
//!
//! [`bind_interrupts!`](crate::bind_interrupts) 宏为每个中断生成强符号
//! 向量函数（覆盖 PAC `device.x` 里的 `PROVIDE(sym = DefaultHandler)` 弱
//! 默认），并把用户结构体标记为持有该中断的 [`Binding`]——异步外设构造
//! 时要求传入 `impl Binding<对应中断>`，绑错中断直接编译失败。
//!
//! ```ignore
//! embassy_cw32::bind_interrupts!(struct Irqs {
//!     UART1 => uart::Uart1Irq;
//!     GPIOA => gpio::GpioAIrq;
//! });
//!
//! // 之后：
//! let u = Uart::new(p.uart1, tx, rx, Irqs, Config::default());
//! ```

/// typelevel 中断类型（转发 PAC）
pub use cw32l012_pac::interrupt;

/// "某结构体持有中断 INT 的绑定" 标记。
///
/// 由 [`bind_interrupts!`](crate::bind_interrupts) 对用户结构体实现；
/// 驱动构造函数以 `impl Binding<pac::interrupt::UART1>` 参数做编译期校验。
/// # Safety
///
/// 实现者必须保证对应中断确实经 `bind_interrupts!` 绑定到了处理该
/// 外设的驱动（宏生成即满足）。
pub unsafe trait Binding<INT: interrupt::Interrupt> {}

/// 中断处理入口契约：驱动的 per-IRQ 标记类型实现它。
///
/// `call()` 在对应向量函数里被调用（中断上下文）。
pub trait IrqHandler<INT: interrupt::Interrupt> {
    /// 中断服务入口
    fn call();
}

/// 绑定中断到驱动处理类型。
///
/// 见模块文档。注意：一个中断在一套 [`Binding`] 里只能出现一次；
/// SPI2/3 共享 `SPI23` 中断（`Spi2Irq` 与 `Spi3Irq` 都实现
/// [`IrqHandler<SPI23>`](IrqHandler)，共享向量会依次派发）。
#[macro_export]
macro_rules! bind_interrupts {
    (struct $name:ident { $($irq:ident => $handler:ty;)* }) => {
        /// 中断绑定证明类型
        pub struct $name;

        $(
            unsafe impl $crate::interrupt::Binding<$crate::pac::interrupt::$irq> for $name {}

            #[unsafe(no_mangle)]
            #[allow(non_snake_case)]
            unsafe extern "C" fn $irq() {
                <$handler as $crate::interrupt::IrqHandler<$crate::pac::interrupt::$irq>>::call()
            }
        )*
    };
}
