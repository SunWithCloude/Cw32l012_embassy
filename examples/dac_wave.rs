//! 异步 DAC 波形输出验证示例（L012）：DMA 三角波 → DAC1（§26.7 DMA 模式）
//!
//! 触发链（手册 §26.6 表 26-1 + §26.7）：GTIM1 配 1kHz 更新 + TRGO
//! （CR2.MMS=update）→ DAC1 触发源 TSEL=0100（GTIM1_Trgo）→ 每次触发
//! 播放一个样点并请求 DMA 预载下一个（`hs::DAC1=0x10` 硬件触发，DMA
//! 通道 2 环形地址推进，[`write_dma`](embassy_cw32::dac::Dac::write_dma)）。
//! 32 点三角波 → 32ms 一个周期，每轮完成后 PC13 心跳翻转 + ROUNDS 计数。
//!
//! ⚠ **DAC DMA 模式为推断实现，待板验证**（阻塞版无 DAC DMA 实机配方）：
//! 寄存器语义逐条出自官方 UM V1.2 §26.7（DMAENx=1 + 硬件触发源=DAC 通道，
//! DAC 完成 DOR 加载后自动请求 DMA 搬下一个数据到 DHR；下溢标志
//! DORx.DMAUDR），但 hs::DAC1 请求线 × GTIM TRGO 供拍 × 末样点补播时序
//! 未上板核对。保守回退：把本例的 `write_dma` 换成周期调用即可逐轮播放。
//!
//! 输出观测（二选一）：
//! 1. **示波器**：默认不使能引脚输出（PA4=DAC_OUT1 与 SPI1_SCK 冲突，
//!    官方板屏幕占用）——需要引脚波形时解开 `enable_pin1()` 注释并把
//!    探头接 PA4；
//! 2. **probe-rs 静态量**：ERR=0 且 ROUNDS 递增 = 触发链/DMA 链全程活着；
//!    LAST_DOR 预期恒 0（三角波终点 = 末样点补播生效）。
//!
//! ```text
//! probe-rs read b32 0x20000000 8    # 静态量（地址以 nm 输出为准）
//! ```
//!
//! cargo build --target thumbv6m-none-eabi --release --features rt --example dac_wave

#![no_std]
#![no_main]

use embassy_executor::main;
use embassy_time::{Duration, Timer, with_timeout};

use embassy_cw32::{
    Config, Peripherals,
    dac::{Dac, DacChannel, DacError, DacTrig},
    gpio::{Level, Output, Pin},
};

embassy_cw32::bind_interrupts!(struct Irqs {
    DMA12 => embassy_cw32::dma::Dma12Irq;
    DMA34 => embassy_cw32::dma::Dma34Irq;
});

/// 32 点三角波（0→4095→0，步进 273；上坡 16 点、下坡镜像 16 点）。
/// 1kHz 供拍下一个周期 32ms。
static TRIANGLE: [u16; 32] = {
    let mut a = [0u16; 32];
    let mut i = 0;
    while i < 32 {
        // i<16 上坡 0..15，i>=16 下坡镜像（k = 31-i），峰值 15×273=4095
        let k = if i < 16 { i } else { 31 - i };
        a[i] = (k as u32 * 273) as u16;
        i += 1;
    }
    a
};

/// 完成的整段播放轮数（心跳：每轮一次 DMA TC 唤醒）
#[used]
#[unsafe(no_mangle)]
pub static mut ROUNDS: u32 = 0;
/// 错误码：0=无 1=超时（触发源停拍？） 2=DMA 下溢 3=DMA 通道错误
/// 4=值越界 5=缓冲长度非法
#[used]
#[unsafe(no_mangle)]
pub static mut ERR: u32 = 0;
/// 最后一轮播放完的 DOR1（预期恒 0 = 三角波终点，末样点补播生效）
#[used]
#[unsafe(no_mangle)]
pub static mut LAST_DOR: u32 = 0;

#[main]
async fn main(_spawner: embassy_executor::Spawner) {
    embassy_cw32::init(Config::default());
    let p = Peripherals::take().unwrap();

    // GPIOC 时钟 + PC13 LED 心跳
    embassy_cw32::sysctrl_enable_gpioc(p.sysctrl);
    let mut led = Output::new(Pin::new(p.gpioc, 13), Level::High);

    // ---- GTIM1 → TRGO 供拍（1kHz 更新；表 26-1 TSEL=0100 = GTIM1_Trgo）----
    // APBEN1.GTIM1（bit6，KEY 读改写——timer.rs 同款配方）
    let sc = p.sysctrl;
    sc.apben1
        .modify(|v| embassy_cw32::pac::sysctrl::KEY | (v & 0xFFFF) | embassy_cw32::pac::sysctrl::APBEN1_GTIM1);
    let gtim1 = p.gtim1;
    gtim1.cr1.write(0); // 停止计数以便配置
    // PSC=3、ARR=999：PCLK 4MHz /(3+1)/(999+1) = 1kHz 更新
    gtim1.psc.write(3);
    gtim1.arr.write(999);
    // CR2.MMS[6:4]=010（update → TRGO；厂商库 gtim.h GTIM_MASTER_TRGO_UPDATE，
    // embassy crate 的 PAC 未定义该位域故写裸值）
    gtim1.cr2.write(2 << 4);
    gtim1.egr.write(embassy_cw32::pac::gtim::EGR_UG); // 重装影子寄存器
    gtim1.cr1.write(embassy_cw32::pac::gtim::CR1_EN); // 跑起来，开始供拍

    // ---- DAC + DMA 波形链 ----
    let mut dac = Dac::new(p.dac);
    let mut dma = embassy_cw32::dma::Dma::new(p.dma, Irqs);
    dac.set_trigger(DacChannel::Ch1, DacTrig::Gtim1Trgo);
    // 需要引脚观测时解开（⚠ PA4 与 SPI1_SCK 冲突，官方板屏幕占用中）：
    // dac.enable_pin1();

    loop {
        // 32 样点 @1kHz = 32ms/轮；2s 超时兜底（触发源停拍时不挂死）
        let r = with_timeout(
            Duration::from_millis(2000),
            dac.write_dma(&mut dma, DacChannel::Ch1, &TRIANGLE),
        )
        .await;
        match r {
            Ok(Ok(())) => {
                unsafe {
                    ERR = 0;
                    LAST_DOR = dac.output1() as u32;
                }
            }
            Ok(Err(DacError::DmaUnderrun)) => unsafe { ERR = 2 },
            Ok(Err(DacError::Dma(_))) => unsafe { ERR = 3 },
            Ok(Err(DacError::ValueOutOfRange)) => unsafe { ERR = 4 },
            Ok(Err(DacError::BadLength)) => unsafe { ERR = 5 },
            Err(_) => {
                // 超时：触发链没跑起来/请求没到——停通道收尾再重试
                dac.stop_dma(&mut dma, DacChannel::Ch1);
                unsafe { ERR = 1 }
            }
        }
        unsafe { ROUNDS += 1 };
        led.toggle();
        Timer::after_millis(468).await;
    }
}

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {
        core::hint::spin_loop();
    }
}
