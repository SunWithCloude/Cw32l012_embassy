//! CW32L010 ADC 读取示例（低配代组 ADC，embassy 异步，软件触发）：
//!
//! 1. **PA0（IN0）** 单次读（`read`，`into_analog` 外部通道；悬空值不定）
//! 2. **PA1+PA2（IN1/IN2）** 序列扫描读（`read_sequence`，逐槽采样周期，
//!    一次 START 按 SQRCFR 槽序转换，结果落 RESULT0/RESULT1）
//! 3. PA7 心跳 LED（l010 的 PA7 不在 ADC 通道表内）
//!
//! L010/L011 的 ADC 是 L012 的"近亲但不同构" IP：SAMPLE/SQRCFR 逐槽
//! 4bit + ENS 序列 + 8 结果，但偏移/位序全不同，且 BGREN/TSEN 在 ADC_CR
//! 内；无 BGR 外设、无出厂校准字（内部通道 TS=IN14/BGR=IN15 读原始
//! counts）、无 DMA。l011 引脚映射不同（0..7=PA0..7、8=PB0、9=PB1、
//! 10..13=PA8..11），ADCCLK 下限 4MHz——本例按 l010 编写。
//!
//! ⚠ 低配组 ADC 驱动为语义级实现（配方对照 L010 手册 §20 经
//!   cw32-hal2 docs/adc-low-async.md 审计链）；尚未实板验证，首次上板
//!   请核对 RAW/SEQ 数值合理性（GND≈0、VDD≈4095）。
//!
//! cargo build --target thumbv6m-none-eabi --release --no-default-features \
//!     --features "rt,cw32l010" --example adc_read_l010

#![no_std]
#![no_main]

use embassy_executor::main;
use embassy_time::Timer;

use embassy_cw32::{
    Config, Peripherals,
    adc::{LowAdc, LowAdcError, LowChannel, LowClk, LowSampleCycles},
    gpio::{Level, Output, Pin},
};

embassy_cw32::bind_interrupts!(struct Irqs {
    ADC => embassy_cw32::adc::LowAdcIrq; // IRQ12
});

/// 最近一次 PA0（IN0）单次读原始值（0..4095 右对齐）
#[used]
#[unsafe(no_mangle)]
pub static mut RAW: u32 = 0;
/// 最近一轮序列结果：[0]=PA1（IN1）、[1]=PA2（IN2），低 12 位有效
#[used]
#[unsafe(no_mangle)]
pub static mut SEQ: [u16; 4] = [0; 4];
/// 错误码：0=无 1=Timeout 2=BadConfig
#[used]
#[unsafe(no_mangle)]
pub static mut ERR: u32 = 0;
/// 完成的轮数（心跳计数）
#[used]
#[unsafe(no_mangle)]
pub static mut SAMPLES: u32 = 0;

fn err_code(e: LowAdcError) -> u32 {
    match e {
        LowAdcError::Timeout => 1,
        LowAdcError::BadConfig => 2,
    }
}

#[main]
async fn main(_spawner: embassy_executor::Spawner) {
    embassy_cw32::init(Config::default()); // 复位默认 HSI 96MHz/24 = 4MHz

    let p = Peripherals::take().unwrap();

    // GPIOA（ADC 输入脚 + PA7 LED）时钟
    embassy_cw32::sysctrl_enable_gpioa(p.sysctrl);
    let mut led = Output::new(Pin::new(p.gpioa, 7), Level::High);

    // ADCCLK = PCLK/2 = 2MHz——l010 电压域 1.62V 档上限 4MHz，Div2 在
    // 各电压域都安全
    let mut adc = LowAdc::new(p.adc, LowClk::Div2, Irqs);
    // PA0/PA1/PA2 → 模拟模式（IN0/IN1/IN2，l010 通道映射）
    let _pa0 = Pin::new(p.gpioa, 0).into_analog();
    let _pa1 = Pin::new(p.gpioa, 1).into_analog();
    let _pa2 = Pin::new(p.gpioa, 2).into_analog();

    loop {
        // 单通道单次（悬空引脚读数不定，验证通路本身）
        match adc.read(LowChannel::In(0), LowSampleCycles::S262).await {
            Ok(raw) => unsafe { RAW = raw as u32 },
            Err(e) => unsafe { ERR = err_code(e) },
        }
        // 序列扫描：PA1、PA2 两通道，逐槽采样周期（本组特有）
        let chs = [LowChannel::In(1), LowChannel::In(2)];
        let samples = [LowSampleCycles::S102, LowSampleCycles::S262];
        let mut seq = [0u16; 4];
        match adc.read_sequence(&chs, &samples, &mut seq).await {
            Ok(()) => unsafe { SEQ = seq },
            Err(e) => unsafe { ERR = err_code(e) },
        }
        unsafe { SAMPLES += 1 };
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
