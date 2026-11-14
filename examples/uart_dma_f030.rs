//! CW32F030 UART1 DMA 回显示例（经典组，embassy 异步）：read_dma 收一帧 →
//! write_dma 原样发回，PC13 心跳独立任务 500ms 翻转（回显挂起等待期间 LED
//! 不停闪，即"DMA 等待不阻塞执行器"的直观证据）。
//!
//! 接线：USB-TTL 交叉接 PA08=TXD / PA09=RXD（AF2，`PA08_AFx_UART1TXD()`
//! 实锤，115200-8N1），上位机一次发满 `FRAME` 字节即回显一帧（read_dma
//! 单次全量语义：收满才返回，无内置超时——需要限时用
//! `embassy_time::with_timeout` 包一层）。
//!
//! 位面（经典组）：UART 的 DMA 请求使能在 CR2[7]=DMATX / CR2[6]=DMARX
//! （厂商库 cw32f030_uart.h USART_DMAReq_Tx/Rx，`USART_DMACmd` 写 CR2）；
//! 触发源 UART1_RX=0x00 / UART1_TX=0x01（§8.8.4 / 厂商库 cw32f030_dma.h
//! DMA_HardTrig_UART1_RXBufferNE/TXBufferE，即 `dma::hs::UART1_RX/TX`）。
//! DMA 占通道 1（RX）/通道 2（TX），两条 NVIC 线 DMACH1/DMACH23 都要绑定。
//!
//! ⚠ 语义级实现待实板验证（阻塞版 HAL 无 UART DMA 配方）。
//!
//! 观察量（SRAM 静态量，`probe-rs read` 观测，地址以 `nm` 输出为准）：
//!
//! - `FRAMES`: 完成回显的帧数
//! - `ERR`: 0=无 1=BadLength 9=DMA 错误
//! - `LAST_FRAME`: 最近一帧内容（收到的原样字节）
//!
//! cargo build --target thumbv6m-none-eabi --release --no-default-features \
//!     --features "rt,cw32f030" --example uart_dma_f030

#![no_std]
#![no_main]

use embassy_executor::{Spawner, main, task};
use embassy_time::Timer;

use embassy_cw32::{
    Config, Peripherals,
    gpio::{Level, Output, Pin},
    uart::{self, Uart},
};

// UART1 收发中断（IRQ27）+ f030 三条 DMA NVIC 线（通道 1；2+3；4+5）
embassy_cw32::bind_interrupts!(struct Irqs {
    UART1 => uart::Uart1Irq;
    DMACH1 => embassy_cw32::dma::Dmach1Irq;
    DMACH23 => embassy_cw32::dma::Dmach23Irq;
    DMACH45 => embassy_cw32::dma::Dmach45Irq;
});

/// 帧长（read_dma 收满 / write_dma 发回的字节数）
const FRAME: usize = 32;

/// 完成回显的帧数
#[used]
#[unsafe(no_mangle)]
pub static mut FRAMES: u32 = 0;
/// 错误码：0=无 1=BadLength 9=DMA 错误
#[used]
#[unsafe(no_mangle)]
pub static mut ERR: u32 = 0;
/// 最近一帧内容
#[used]
#[unsafe(no_mangle)]
pub static mut LAST_FRAME: [u8; FRAME] = [0; FRAME];

/// PC13 心跳（独立任务，500ms 翻转；F030 的 GPIOC 是独立端口结构体）
#[task]
async fn heartbeat(mut led: Output<embassy_cw32::pac::GPIOC>) {
    loop {
        Timer::after_millis(500).await;
        led.toggle();
    }
}

#[main]
async fn main(spawner: Spawner) {
    embassy_cw32::init(Config::default()); // 复位默认 HSI 48MHz/6 = 8MHz

    let p = Peripherals::take().unwrap();
    embassy_cw32::sysctrl_enable_gpioa(p.sysctrl);
    embassy_cw32::sysctrl_enable_gpioc(p.sysctrl);
    let led = Output::new(Pin::new(p.gpioc, 13), Level::High);
    spawner.spawn(heartbeat(led).unwrap());

    let mut dma = embassy_cw32::dma::ClassicDma::new(p.dma, Irqs);
    // UART1 = PA08(TXD) / PA09(RXD)，AF2（uart_echo_f030 上板实锤组合）
    let mut u = Uart::new_uart1(
        p.uart1,
        Pin::new(p.gpioa, 8),
        Pin::new(p.gpioa, 9),
        2,
        Irqs,
        Default::default(),
    );

    let mut buf = [0u8; FRAME];
    loop {
        let r = async {
            u.read_dma(&mut dma, &mut buf).await?;
            u.write_dma(&mut dma, &buf).await?;
            Ok(())
        };
        match r.await {
            Ok(()) => unsafe {
                ERR = 0;
                LAST_FRAME = buf;
                FRAMES += 1;
            },
            Err(embassy_cw32::dma::DmaErrorClassic::BadLength) => unsafe { ERR = 1 },
            Err(_) => unsafe { ERR = 9 },
        }
    }
}

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {
        core::hint::spin_loop();
    }
}
