//! CW32F030 异步 SPI 回环示例(需杜邦线短接 MOSI↔MISO)。
//!
//! SPI1 = PB12(SCK)/PB14(MISO)/PB15(MOSI)?不——以厂商宏为准:
//! SPI1 引脚经 GPIO AF 配置,示例用 PA05(SCK)/PA06(MISO)/PA07(MOSI)
//! (与 L012 相同排布的常见引脚;立创天猛星排针可接)。
//! 回环:发送字节序列,读回应与发送一致,LED 常亮=通过/闪烁=失败。
//!
//! cargo build --target thumbv6m-none-eabi --release --no-default-features \
//!     --features "rt,cw32f030,critical-section-impl" --example spi_loopback_f030

#![no_std]
#![no_main]

use embassy_executor::main;
use embassy_time::Timer;
use embedded_hal_async::spi::SpiBus as _;

use embassy_cw32::{
    Config, Peripherals,
    gpio::{Level, Output, Pin},
    spi::{self, Config as SpiConfig, Mode, Spi},
    sysctrl_enable_gpioa, sysctrl_enable_gpioc,
};

embassy_cw32::bind_interrupts!(struct Irqs {
    SPI1 => spi::Spi1Irq;
});

#[main]
async fn main(_spawner: embassy_executor::Spawner) {
    embassy_cw32::init(Config::default());

    let p = Peripherals::take().unwrap();
    sysctrl_enable_gpioa(p.sysctrl);
    sysctrl_enable_gpioc(p.sysctrl);
    let mut led = Output::new(Pin::new(p.gpioc, 13), Level::High);

    let mut spi = Spi::new_spi1(
        p.spi1,
        Pin::new(p.gpioa, 5),
        Pin::new(p.gpioa, 7),
        Pin::new(p.gpioa, 6),
        5, // AF5:PA05/06/07 的 SPI1 复用编号(厂商头 815/823/831 行)
        Irqs,
        SpiConfig {
            frequency: 500_000,
            mode: Mode::Mode0,
            ..Default::default()
        },
    );

    let tx = [0x0F, 0x55, 0xAA, 0xFF];
    let mut rx = [0u8; 4];
    loop {
        // 回环测试:MOSI 接 MISO 时 rx 应等于 tx
        spi.transfer(&mut rx, &tx).await;
        let ok = rx == tx;
        if ok {
            led.set_low(); // 立创板 LED 低电平亮?保守:双态可见
            Timer::after_millis(900).await;
            led.set_high();
            Timer::after_millis(100).await;
        } else {
            // 失败:快闪
            led.toggle();
            Timer::after_millis(100).await;
        }
    }
}

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {
        core::hint::spin_loop();
    }
}
