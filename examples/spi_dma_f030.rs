//! CW32F030 SPI1 DMA 传输示例（经典组，零接线可跑）：write_dma 固定图样
//! 只发一轮 + transfer_dma 双通道全双工一轮，PC13 心跳每轮翻转。
//!
//! SPI1 = PA05(SCK) / PA06(MISO) / PA07(MOSI)，AF5（spi_loopback_f030 同款
//! 引脚组合），Mode 0、500kHz。
//!
//! 判定：
//! - `ERR=0`（DMA 层无错）——零接线即可达成
//! - `MATCH=1` 需杜邦线短接 MOSI↔MISO（回环：transfer_dma 收到的 rx 与
//!   发出的 tx 图样一致）；未接线时 MISO 悬空，MATCH=0 属预期
//!
//! 位面（经典组）：SPI 的 DMA 请求使能位在 **CR1[17]=DMATX / CR1[16]=DMARX**
//! （厂商库 cw32f030_spi.h SPI_DMAReq_Tx/Rx，`SPI_DMACmd` 写 CR1——与 L012
//! 的 CR2 位面不同）；触发源 SPI1_RX=0x06 / SPI1_TX=0x07（§8.8.4 / 厂商库
//! cw32f030_dma.h DMA_HardTrig_SPI1_RXBufferNE/TXBufferE，即
//! `dma::hs::SPI1_RX/TX`）。DMA 占通道 1（RX）/通道 2（TX），两条 NVIC 线
//! DMACH1/DMACH23 都要绑定。
//!
//! ⚠ 语义级实现待实板验证（阻塞版 HAL 无 SPI DMA 配方）。
//!
//! 观察量（SRAM 静态量，`probe-rs read` 观测，地址以 `nm` 输出为准）：
//!
//! - `ROUNDS`: 完成的轮数（write_dma + transfer_dma 各一次为一轮）
//! - `ERR`: 0=无 1=BadLength 9=DMA 错误
//! - `MATCH`: 最近一轮 transfer_dma 回环比对（1=一致 0=不一致/未接线）
//! - `LAST_RX`: 最近一轮 transfer_dma 收到的数据
//!
//! cargo build --target thumbv6m-none-eabi --release --no-default-features \
//!     --features "rt,critical-section-impl,cw32f030" --example spi_dma_f030

#![no_std]
#![no_main]

use embassy_executor::main;
use embassy_time::Timer;

use embassy_cw32::{
    Config, Peripherals,
    gpio::{Level, Output, Pin},
    spi::{Config as SpiConfig, Spi},
    sysctrl_enable_gpioa, sysctrl_enable_gpioc,
};

// SPI1 中断（IRQ25）+ f030 三条 DMA NVIC 线（通道 1；2+3；4+5）
embassy_cw32::bind_interrupts!(struct Irqs {
    SPI1 => embassy_cw32::spi::Spi1Irq;
    DMACH1 => embassy_cw32::dma::Dmach1Irq;
    DMACH23 => embassy_cw32::dma::Dmach23Irq;
    DMACH45 => embassy_cw32::dma::Dmach45Irq;
});

/// 帧长（write_dma / transfer_dma 每次搬运的字节数）
const LEN: usize = 32;

/// 完成的轮数
#[used]
#[unsafe(no_mangle)]
pub static mut ROUNDS: u32 = 0;
/// 错误码：0=无 1=BadLength 9=DMA 错误
#[used]
#[unsafe(no_mangle)]
pub static mut ERR: u32 = 0;
/// 最近一轮 transfer_dma 回环比对：1=一致 0=不一致/未接线
#[used]
#[unsafe(no_mangle)]
pub static mut MATCH: u32 = 0;
/// 最近一轮 transfer_dma 收到的数据
#[used]
#[unsafe(no_mangle)]
pub static mut LAST_RX: [u8; LEN] = [0; LEN];

#[main]
async fn main(_spawner: embassy_executor::Spawner) {
    embassy_cw32::init(Config::default()); // 复位默认 HSI 48MHz/6 = 8MHz

    let p = Peripherals::take().unwrap();
    sysctrl_enable_gpioa(p.sysctrl);
    sysctrl_enable_gpioc(p.sysctrl);
    let mut led = Output::new(Pin::new(p.gpioc, 13), Level::High);

    let mut dma = embassy_cw32::dma::ClassicDma::new(p.dma, Irqs);
    // SPI1 引脚：PA05=SCK / PA07=MOSI / PA06=MISO，AF5（spi_loopback_f030
    // 同款组合）
    let mut spi = Spi::new_spi1(
        p.spi1,
        Pin::new(p.gpioa, 5),
        Pin::new(p.gpioa, 7),
        Pin::new(p.gpioa, 6),
        5,
        Irqs,
        SpiConfig::default(),
    );

    let mut tx = [0u8; LEN];
    let mut rx = [0u8; LEN];
    loop {
        // 只发一轮：固定图样走 write_dma（接收侧读走即弃）
        for (i, b) in tx.iter_mut().enumerate() {
            *b = (i as u8) * 7 + 3;
        }
        // 双通道全双工一轮：图样换序，若短接 MOSI↔MISO 则 rx 应与 tx 一致
        let mut pattern2 = tx;
        pattern2.reverse();
        let r = async {
            spi.write_dma(&mut dma, &tx).await?;
            spi.transfer_dma(&mut dma, &mut rx, &pattern2).await?;
            Ok(())
        };
        match r.await {
            Ok(()) => unsafe {
                ERR = 0;
                MATCH = (rx == pattern2) as u32;
                LAST_RX = rx;
                ROUNDS += 1;
            },
            Err(embassy_cw32::dma::DmaErrorClassic::BadLength) => unsafe { ERR = 1 },
            Err(_) => unsafe { ERR = 9 },
        }
        led.toggle();
        Timer::after_millis(500).await;
    }
}

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {
        core::hint::spin_loop();
    }
}
