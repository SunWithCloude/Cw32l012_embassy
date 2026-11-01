//! 异步 DMA 存储器→存储器验证示例（L012）：
//!
//! 64 个 u32 从 FLASH 常量表 `SRC` 搬到 SRAM `DST`（32 位宽、BULK、软件
//! 触发，`Dma::mem_to_mem` 占通道 3），逐字比对并记录首个失配下标；随后
//! 周期性重跑（每次都重新 configure——`CNT.REPEAT` 每轮传输后被硬件清零，
//! §8.5.2），ROUNDS 计数即异步 wait 路径的心跳。
//!
//! 结果落 SRAM 静态量，经 `probe-rs read` 观测（`#[used]#[no_mangle]`
//! 锁符号防优化；地址以 `nm` 输出为准）：
//!
//! ```text
//! probe-rs read b32 0x20000000 128     # DST[64]（应为 A5A5_0000 ^ i*0x0101_0101 图样）
//! probe-rs read b32 0x20000200 8       # 统计量（地址以 nm 为准）
//! ```
//!
//! 判定：`P1_OK=64`、`P1_BAD=0xFFFF_FFFF`、`P1_ERR=0`（阻塞版 dma_demo
//! P1 的同款判据，实机 2026-10-23 通过）。
//!
//! cargo build --target thumbv6m-none-eabi --release --features rt --example dma_mem2mem

#![no_std]
#![no_main]

use embassy_executor::main;
use embassy_time::Timer;

use embassy_cw32::{Config, Peripherals};

embassy_cw32::bind_interrupts!(struct Irqs {
    DMA12 => embassy_cw32::dma::Dma12Irq;
    DMA34 => embassy_cw32::dma::Dma34Irq;
});

/// P1 源数据（FLASH 常量表，编译期生成——阻塞版 dma_demo 同款图样）
static SRC: [u32; 64] = {
    let mut a = [0u32; 64];
    let mut i = 0;
    while i < 64 {
        a[i] = 0xA5A5_0000 ^ ((i as u32) * 0x0001_0101);
        i += 1;
    }
    a
};

/// P1 目的缓冲（DMA 写入）
#[used]
#[unsafe(no_mangle)]
pub static mut DST: [u32; 64] = [0; 64];
/// 比对通过的元素数（应为 64）
#[used]
#[unsafe(no_mangle)]
pub static mut P1_OK: u32 = 0;
/// 首个失配下标（0xFFFF_FFFF = 全部一致）
#[used]
#[unsafe(no_mangle)]
pub static mut P1_BAD: u32 = 0;
/// 最近一轮 DMA 错误码（0=无 1=BadChannel 2=UnsupportedWidth 3=BadLength
/// 4=AddressOutOfRange 5=Aborted 6=SourceError 7=DestError）
#[used]
#[unsafe(no_mangle)]
pub static mut P1_ERR: u32 = 0;
/// 完成的轮数（心跳：异步 wait 每轮唤醒一次 IRQ）
#[used]
#[unsafe(no_mangle)]
pub static mut ROUNDS: u32 = 0;

fn err_code(e: &embassy_cw32::dma::DmaError) -> u32 {
    use embassy_cw32::dma::DmaError;
    match e {
        DmaError::BadChannel => 1,
        DmaError::UnsupportedWidth => 2,
        DmaError::BadLength => 3,
        DmaError::AddressOutOfRange => 4,
        DmaError::Aborted => 5,
        DmaError::SourceError => 6,
        DmaError::DestError => 7,
    }
}

#[main]
async fn main(_spawner: embassy_executor::Spawner) {
    embassy_cw32::init(Config::default());
    let p = Peripherals::take().unwrap();
    let mut dma = embassy_cw32::dma::Dma::new(p.dma, Irqs);

    loop {
        // SRC 在 FLASH、DST 在 SRAM：阻塞版 dma_demo P1 的同款通路
        let dst = unsafe {
            core::slice::from_raw_parts_mut(
                core::ptr::addr_of_mut!(DST) as *mut u32,
                SRC.len(),
            )
        };
        match dma.mem_to_mem(&SRC, dst).await {
            Ok(()) => unsafe { P1_ERR = 0 },
            Err(e) => unsafe { P1_ERR = err_code(&e) },
        }

        // 逐字比对（DMA 已完成，TC 后读 SRAM；单核 M0+ 无缓存）
        unsafe {
            let mut ok = 0u32;
            let mut bad = 0xFFFF_FFFFu32;
            for i in 0..SRC.len() {
                if DST[i] == SRC[i] {
                    ok += 1;
                } else if bad == 0xFFFF_FFFF {
                    bad = i as u32;
                }
            }
            P1_OK = ok;
            P1_BAD = bad;
            ROUNDS += 1;
        }

        Timer::after_millis(500).await;
    }
}

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {
        core::hint::spin_loop();
    }
}
