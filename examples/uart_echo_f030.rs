//! CW32F030(立创天猛星)异步 UART 回显示例。
//!
//! UART1:PA08=TXD / PA09=RXD(板卡 BSP `PA08_AFx_UART1TXD()` 实锤)。
//! 立创天猛星未焊 USB 转串口桥,需 USB-TTL 适配器接 PA08/PA09 + GND
//! 才能实测回环;烧录后 CPU 侧可经寄存器读数验证初始化。
//!
//! cargo build --target thumbv6m-none-eabi --release --no-default-features \
//!     --features "rt,cw32f030,critical-section-impl" --example uart_echo_f030

#![no_std]
#![no_main]

use embassy_executor::main;
use embassy_time::Timer;

use embassy_cw32::{
    gpio::{Level, Output, Pin},
    uart::{self, Uart},
    Config, Peripherals,
};

embassy_cw32::bind_interrupts!(struct Irqs {
    UART1 => uart::Uart1Irq;
});

#[main]
async fn main(_spawner: embassy_executor::Spawner) {
    embassy_cw32::init(Config::default()); // F030 复位默认 8MHz

    let p = Peripherals::take().unwrap();
    embassy_cw32::sysctrl_enable_gpioa(p.sysctrl);
    embassy_cw32::sysctrl_enable_gpioc(p.sysctrl); // 心跳 LED 在 PC13

    let mut u = Uart::new_uart1(
        p.uart1,
        Pin::new(p.gpioa, 8),
        Pin::new(p.gpioa, 9),
        2, // AF2:PA08_AFx_UART1TXD()=AFRH_f.AFR8=2(厂商头 835 行)
        Irqs,
        Default::default(),
    );

    // 心跳兼发送指示:每秒发一帧,LED 翻转
    let mut led = Output::new(Pin::new(p.gpioc, 13), Level::High);
    let mut buf = [0u8; 32];
    loop {
        // 非阻塞读:可读才回显
        let n = u.read_byte().await;
        u.write_byte(n.unwrap_or(b'.')).await;
        Timer::after_millis(1000).await;
        led.toggle();
        let _ = &mut buf;
    }
}

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {
        core::hint::spin_loop();
    }
}
