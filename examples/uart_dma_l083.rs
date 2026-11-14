//! CW32L083 UART2 DMA 回显示例（经典组六实例 DMA 接线验证）：read_dma 收
//! 一帧 → write_dma 原样发回。
//!
//! 用 **UART2**（PA06=TXD / PA07=RXD，AF2——厂商头 `PA06_AFx_UART2TXD()`/
//! `PA07_AFx_UART2RXD()`，数据手册表 5-3）验证 l083 多实例 DMA 触发源映射：
//! UART2_RX=0x02 / UART2_TX=0x03（§8.8.4 请求表全经典组同值，即
//! `dma::hs::UART2_RX/TX`）。UART4..6 的触发源（0x2B..0x30，仅 l083 手册表
//! 有）同表接线，把 `new_uart2` 换成 `new_uart4..6` 即可，映射自动跟上
//! （`cudma::tx_hs/rx_hs` 按实例 idx 查表）。
//!
//! 接线：USB-TTL 交叉接 PA06=TXD / PA07=RXD（115200-8N1），上位机一次发满
//! `FRAME` 字节即回显一帧（read_dma 单次全量语义：收满才返回，无内置超时
//! ——需要限时用 `embassy_time::with_timeout` 包一层）。
//!
//! 位面（经典组）：UART 的 DMA 请求使能在 CR2[7]=DMATX / CR2[6]=DMARX
//! （`uart::cr2::dmatx/dmarx_MASK`，厂商库 `USART_DMACmd` 同款）；DMA 占
//! 通道 1（RX）/通道 2（TX），三条 NVIC 线 DMACH1/DMACH23/DMACH45 都要
//! 绑定。UART2/UART5 共享中断线 IRQ28（`UART2_UART5 => uart::Uart2Uart5Irq;`）。
//!
//! ⚠ 诚实标注：compiles-not-run——l083 实板未验证，语义级实现待实板测试。
//!
//! 观察量（SRAM 静态量，`probe-rs read` 观测，地址以 `nm` 输出为准）：
//!
//! - `FRAMES`: 完成回显的帧数
//! - `ERR`: 0=无 1=BadLength 9=DMA 错误
//! - `LAST_FRAME`: 最近一帧内容（收到的原样字节）
//!
//! cargo build --target thumbv6m-none-eabi --release --no-default-features \
//!     --features "rt,cw32l083" --example uart_dma_l083

#![no_std]
#![no_main]

use embassy_executor::main;

use embassy_cw32::{
    Config, Peripherals,
    gpio::Pin,
    uart::{self, Uart},
};

// UART2/UART5 共享中断线（IRQ28）+ l083 三条 DMA NVIC 线（通道 1；2+3；4+5）
embassy_cw32::bind_interrupts!(struct Irqs {
    UART2_UART5 => uart::Uart2Uart5Irq;
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

#[main]
async fn main(_spawner: embassy_executor::Spawner) {
    embassy_cw32::init(Config::default()); // l083 复位默认 8MHz

    let p = Peripherals::take().unwrap();
    embassy_cw32::sysctrl_enable_gpioa(p.sysctrl);

    let mut dma = embassy_cw32::dma::ClassicDma::new(p.dma, Irqs);
    let mut u = Uart::new_uart2(
        p.uart2,
        Pin::new(p.gpioa, 6), // PA06=TXD（AF2）
        Pin::new(p.gpioa, 7), // PA07=RXD（AF2）
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
