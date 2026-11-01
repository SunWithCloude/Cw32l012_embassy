//! 异步 ADC 序列 + DMA 验证示例（L012）：
//!
//! ADC1 序列扫描 3 通道（IN0=PA0 外部、IN15=1.2V 基准、IN14=温度传感器），
//! 序列完成（EOS）触发 DMA 请求，结果整块搬入缓冲（CPU 不逐通道取数，
//! [`read_sequence_dma`](embassy_cw32::adc::Adc::read_sequence_dma)）。
//! PC13 心跳灯 + ROUNDS 轮数计数。
//!
//! 结果落 SRAM 静态量，经 `probe-rs read` 观测（地址以 `nm` 输出为准）：
//!
//! ```text
//! probe-rs read b32 0x20000000 4    # RESULTS[0..3]：PA0 / Vref12 原始码
//! probe-rs read b32 0x20000200 8    # 统计量（地址以 nm 为准）
//! ```
//!
//! 判定：`ERR=0`；RESULTS[0] 随 PA0 电位变化、RESULTS[1] ≈ 4095×1.2/VDDA
//! （≈1520 @3.3V）、RESULTS[2] 本板 TS 无输出 ≈0（见 adc_async 说明）。
//!
//! cargo build --target thumbv6m-none-eabi --release --features rt --example adc_dma

#![no_std]
#![no_main]

use embassy_executor::main;
use embassy_time::Timer;

use embassy_cw32::{
    Config, Peripherals,
    adc::{Adc, AdcChannel, AdcClk, SampleCycles},
    gpio::{Level, Output, Pin},
};

embassy_cw32::bind_interrupts!(struct Irqs {
    ADC1 => embassy_cw32::adc::Adc1Irq;
    DMA12 => embassy_cw32::dma::Dma12Irq;
    DMA34 => embassy_cw32::dma::Dma34Irq;
});

/// 最近一轮序列结果（按通道序）：[0]=PA0（IN0）、[1]=1.2V 基准（IN15）、
/// [2]=温度传感器（IN14，本板 TS 无输出，预期 ≈0）
#[used]
#[unsafe(no_mangle)]
pub static mut RESULTS: [u32; 3] = [0; 3];
/// 最近一轮错误码（0=无 3=UnsupportedChannel 1=Timeout；映射同 adc_async）
#[used]
#[unsafe(no_mangle)]
pub static mut ERR: u32 = 0;
/// 完成的轮数（心跳：每轮一次 DMA TC 中断唤醒）
#[used]
#[unsafe(no_mangle)]
pub static mut ROUNDS: u32 = 0;

#[main]
async fn main(_spawner: embassy_executor::Spawner) {
    embassy_cw32::init(Config::default());
    let p = Peripherals::take().unwrap();

    // GPIOC 时钟 + PC13 LED 心跳
    embassy_cw32::sysctrl_enable_gpioc(p.sysctrl);
    let mut led = Output::new(Pin::new(p.gpioc, 13), Level::High);

    let mut dma = embassy_cw32::dma::Dma::new(p.dma, Irqs);
    let mut adc = Adc::new_adc1(p.adc1, AdcClk::Div2, Irqs);
    // PA0 → 模拟模式（IN0）
    let _pa0 = Pin::new(p.gpioa, 0).into_analog();

    // 序列：外部 + 两个内部通道（内部通道的 BGREN 稳定期由驱动自动处理）
    let chs = [AdcChannel::In(0), AdcChannel::Bgr1v2, AdcChannel::TempSensor];
    let samples = [SampleCycles::S24, SampleCycles::S166, SampleCycles::S262];
    let mut buf = [0u16; 3];

    loop {
        match adc.read_sequence_dma(&mut dma, &chs, &samples, &mut buf).await {
            Ok(()) => {
                unsafe {
                    ERR = 0;
                    for i in 0..3 {
                        RESULTS[i] = buf[i] as u32;
                    }
                }
            }
            Err(e) => unsafe {
                ERR = match e {
                    embassy_cw32::adc::AdcError::Timeout => 1,
                    embassy_cw32::adc::AdcError::UnsupportedChannel => 3,
                    _ => 9,
                };
            },
        }
        unsafe { ROUNDS += 1 };
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
