//! CW32L083 异步 UART 回显示例（多实例 + 共享向量布线验证）。
//!
//! 用 **UART2**（PA06=TXD / PA07=RXD，AF2——厂商头 `PA06_AFx_UART2TXD()`/
//! `PA07_AFx_UART2RXD()`，数据手册表 5-3）验证：
//!
//! 1. 多实例构造：`new_uart2`（UART1..UART6 已全部布线，门控按实例查表
//!    UART2=APBEN1.7）；
//! 2. 共享向量：l083 的 UART2/UART5 共享 NVIC 线 IRQ28（向量符号
//!    `UART2_UART5`），`bind_interrupts!` 绑定 `UART2_UART5 =>
//!    uart::Uart2Uart5Irq;`，派发标记会依次检查组内两个实例。
//!
//! ⚠ 诚实标注：compiles-not-run——l083 实板未验证，本示例目前只证明
//! 布线在编译期成立（共享向量 IRQ 符号拼写错误会静默落到 DefaultHandler，
//! 编译期无法发现；运行验证需实板 + 串口适配器）。
//!
//! cargo build --target thumbv6m-none-eabi --release --no-default-features \
//!     --features "critical-section-impl,rt,cw32l083" --example uart_echo_l083

#![no_std]
#![no_main]

use embassy_executor::main;
use embassy_time::Timer;

use embassy_cw32::{
    Config, Peripherals,
    gpio::Pin,
    uart::{self, Uart},
};

embassy_cw32::bind_interrupts!(struct Irqs {
    UART2_UART5 => uart::Uart2Uart5Irq;
});

#[main]
async fn main(_spawner: embassy_executor::Spawner) {
    embassy_cw32::init(Config::default()); // l083 复位默认 8MHz

    let p = Peripherals::take().unwrap();
    embassy_cw32::sysctrl_enable_gpioa(p.sysctrl);

    let mut u = Uart::new_uart2(
        p.uart2,
        Pin::new(p.gpioa, 6), // PA06=TXD（AF2）
        Pin::new(p.gpioa, 7), // PA07=RXD（AF2）
        2,
        Irqs,
        Default::default(),
    );

    // 心跳兼发送指示：每秒发一帧，收到字节就回显
    loop {
        let n = u.read_byte().await;
        u.write_byte(n.unwrap_or(b'.')).await;
        Timer::after_millis(1000).await;
    }
}

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {
        core::hint::spin_loop();
    }
}
