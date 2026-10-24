//! 缓冲式 UART 回显 + 心跳 LED 示例。
//!
//! UART1（PA02=TXD / PA03=RXD，AF1，115200-8N1）跑 `BufferedUart`：
//! 接收进 64 字节环形缓冲，两个任务并发——echo 任务回显，heartbeat
//! 任务翻转 PC13（1 秒周期，作为"系统活着"的可观测信号）。
//!
//! cargo build --target thumbv6m-none-eabi --release --features rt --example buffered_echo

#![no_std]
#![no_main]

use embassy_executor::{main, task};
use embassy_time::Timer;

use embassy_cw32::{
    gpio::{Level, Output, Pin},
    uart::{self, BufferedUart},
    Config, Peripherals,
};

embassy_cw32::bind_interrupts!(struct Irqs {
    UART1 => uart::Uart1Irq;
});

#[task]
async fn echo(mut rx: embassy_cw32::uart::BufferedUartRx, mut tx: embassy_cw32::uart::BufferedUartTx) {
    let mut buf = [0u8; 32];
    loop {
        let n = rx.read(&mut buf).await;
        tx.write(&buf[..n]).await;
        tx.flush().await;
    }
}

#[main]
async fn main(spawner: embassy_executor::Spawner) {
    embassy_cw32::init(Config::default());

    let p = Peripherals::take().unwrap();
    embassy_cw32::sysctrl_enable_gpioc(p.sysctrl);
    let mut led = Output::new(Pin::new(p.gpioc, 13), Level::High);

    let u = BufferedUart::new_uart1(
        p.uart1,
        Pin::new(p.gpioa, 2),
        Pin::new(p.gpioa, 3),
        1, // AF1
        Irqs,
        Default::default(),
    );
    let (tx, rx) = u.split();
    spawner.spawn(echo(rx, tx).unwrap());

    loop {
        Timer::after_millis(1000).await;
        led.toggle();
    }
}

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {
        core::hint::spin_loop();
    }
}
