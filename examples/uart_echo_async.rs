//! 异步 UART 回显示例（UART1：PA02=TXD / PA03=RXD，AF1，115200-8N1）。
//!
//! cargo build --target thumbv6m-none-eabi --release --example uart_echo_async

#![no_std]
#![no_main]

use embassy_executor::main;

use embassy_cw32::{
    Config, Peripherals,
    gpio::Pin,
    uart::{self, Uart},
};

embassy_cw32::bind_interrupts!(struct Irqs {
    UART1 => uart::Uart1Irq;
});

#[main]
async fn main(_spawner: embassy_executor::Spawner) {
    embassy_cw32::init(Config::default());

    let p = Peripherals::take().unwrap();

    let mut u = Uart::new_uart1(
        p.uart1,
        Pin::new(p.gpioa, 2),
        Pin::new(p.gpioa, 3),
        1, // AF1
        Irqs,
        Default::default(),
    );

    loop {
        match u.read_byte().await {
            Ok(b) => {
                u.write_byte(b).await;
            }
            Err(_e) => { /* 清错后继续 */ }
        }
    }
}

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {
        core::hint::spin_loop();
    }
}
