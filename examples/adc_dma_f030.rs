//! CW32F030 ADC + DMA 采样示例（经典组，embassy 异步）：
//!
//! PA0（AIN0）单通道 16 次转换（MODE=1，CR2.CNT=15），每次转换完成
//! （EOC）由 ADC 硬件发出 DMA 请求（CR1.DMAEN），DMA 通道 1 以硬件
//! 触发源 0x0A（ADC 转换完成，`dma::hs::ADC_COMPLETE`）把 RESULT0
//! （@+0x20，固定源）按 16 位宽搬入静态缓冲——CPU 不逐个取数
//! （[`read_dma`](embassy_cw32::adc::ClassicAdc::read_dma)）。
//! PC13 心跳灯 + ROUNDS 轮数计数。
//!
//! 结果落 SRAM 静态量，经 `probe-rs read` 观测（`#[used]#[no_mangle]`
//! 锁符号防优化；地址以 `nm` 输出为准）：
//!
//! ```text
//! probe-rs read b16 0x20000000 32      # BUF[16]（0..4095，悬空值不定）
//! probe-rs read b32 0x20000080 8       # 统计量（地址以 nm 为准）
//! ```
//!
//! ⚠ 经典组 ADC×DMA 路径为语义级实现——配方逐位段对照《CW32x030 用户
//! 手册 V2.5》§22.5.2/§22.13.2/§8.8.4 与厂商库 CW32F030_StandardPeripheral
//! Lib V2.2（cw32f030_adc.c ADC_SingleChMoreModeCfg/ADC_DMACmd、
//! cw32f030_dma.h DMA_HardTrig_ADC_TRANSCOMPLETE），尚未实板验证；首次
//! 上板请核对 BUF 数值合理性与 ROUNDS 心跳（无 DMA 请求则全部 Timeout）。
//!
//! cargo build --target thumbv6m-none-eabi --release --no-default-features \
//!     --features "rt,critical-section-impl,cw32f030" --example adc_dma_f030

#![no_std]
#![no_main]

use embassy_executor::main;
use embassy_time::Timer;

use embassy_cw32::{
    Config, Peripherals,
    adc::{ClassicAdc, ClassicAdcError, ClassicChannel, ClassicClk, ClassicSampleTime},
    gpio::{Level, Output, Pin},
};

// ADC 转换完成中断（IRQ12）+ f030 三条 DMA NVIC 线（通道 1；2+3；4+5）
embassy_cw32::bind_interrupts!(struct Irqs {
    ADC => embassy_cw32::adc::ClassicAdcIrq;
    DMACH1 => embassy_cw32::dma::Dmach1Irq;
    DMACH23 => embassy_cw32::dma::Dmach23Irq;
    DMACH45 => embassy_cw32::dma::Dmach45Irq;
});

/// 每轮采样的次数（≤256：CR2.CNT[7:0] 上限 = CNT+1 次转换）
const N: usize = 16;

/// 最近一轮 DMA 搬入的 PA0 原始采样（0..4095 右对齐，低 12 位有效）
#[used]
#[unsafe(no_mangle)]
pub static mut BUF: [u16; N] = [0; N];
/// 错误码：0=无 1=Timeout（含 DMA 侧错误）2=BadConfig
#[used]
#[unsafe(no_mangle)]
pub static mut ERR: u32 = 0;
/// 完成的轮数（心跳：每轮一次 DMA TC 中断唤醒）
#[used]
#[unsafe(no_mangle)]
pub static mut ROUNDS: u32 = 0;

fn err_code(e: ClassicAdcError) -> u32 {
    match e {
        ClassicAdcError::Timeout => 1,
        ClassicAdcError::BadConfig => 2,
    }
}

#[main]
async fn main(_spawner: embassy_executor::Spawner) {
    embassy_cw32::init(Config::default()); // 复位默认 HSI 48MHz/6 = 8MHz

    let p = Peripherals::take().unwrap();

    // GPIOA（ADC 输入脚）+ GPIOC（PC13 LED）时钟
    embassy_cw32::sysctrl_enable_gpioa(p.sysctrl);
    embassy_cw32::sysctrl_enable_gpioc(p.sysctrl);
    let mut led = Output::new(Pin::new(p.gpioc, 13), Level::High);

    let mut dma = embassy_cw32::dma::ClassicDma::new(p.dma, Irqs);
    // ADCCLK = PCLK/4 = 2MHz——§22.4.2 表 22-3 各电压域安全档（同
    // adc_read_f030）；DMA 请求随转换完成自动供拍，无逐次 CPU 介入
    let mut adc = ClassicAdc::new(p.adc, ClassicClk::Div4, Irqs);
    // PA00 → 模拟模式（AIN0，表 22-5 通道映射）
    let _pa0 = Pin::new(p.gpioa, 0).into_analog();

    loop {
        // 16 次转换整批经 DMA 搬入 buf（read_dma 内部占 DMA 通道 1）
        let buf =
            unsafe { core::slice::from_raw_parts_mut(core::ptr::addr_of_mut!(BUF) as *mut u16, N) };
        match adc
            .read_dma(&mut dma, ClassicChannel::In(0), ClassicSampleTime::S10, buf)
            .await
        {
            Ok(()) => unsafe { ERR = 0 },
            Err(e) => unsafe { ERR = err_code(e) },
        }
        unsafe {
            ROUNDS += 1;
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
