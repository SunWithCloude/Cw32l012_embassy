//! CW32L031 ADC 读取示例（8 槽扫描组 ADC，embassy 异步，软件触发）：
//!
//! 1. **PA0（AIN0）** 单次读（`read`，`into_analog` 外部通道；悬空值不定）
//! 2. **PA1+PA2（AIN1/AIN2）** 序列扫描读（`read_sequence`，MODE=4，
//!    一次 START 按 SQR0 槽序转换，结果落 RESULT0/RESULT1）
//! 3. PC13 心跳 LED（最小系统板 LED）
//!
//! 结果落 SRAM 静态量，经 `probe-rs read` 观测（debug 构建会把地址
//! 未逃逸的内部 static 优化掉，`#[used]#[unsafe(no_mangle)]` 锁符号）。
//!
//! ⚠ 扫描组 ADC 驱动为语义级实现——配方与经典组 f030 同源（cr0.MODE/
//!   REF/CLK/SAM + cr1.CHMUX + SQR0/SQR1 序列），寄存器面经
//!   cw32l031/cw32l052/cw32l083/cw32r031/cw32w031 五 PAC 逐寄存器比对
//!   同构；尚未实板验证，首次上板请核对 RAW/SEQ 数值合理性（GND≈0、
//!   VDDA≈4095）。
//!
//! cargo build --target thumbv6m-none-eabi --release --no-default-features \
//!     --features "rt,critical-section-impl,cw32l031" --example adc_read_l031

#![no_std]
#![no_main]

use embassy_executor::main;
use embassy_time::Timer;

use embassy_cw32::{
    Config, Peripherals,
    adc::{Scan8Adc, Scan8AdcError, Scan8Channel, Scan8Clk, Scan8SampleTime},
    gpio::{Level, Output, Pin},
};

embassy_cw32::bind_interrupts!(struct Irqs {
    ADC => embassy_cw32::adc::Scan8AdcIrq; // IRQ12
});

/// 最近一次 PA0（AIN0）单次读原始值（0..4095 右对齐）
#[used]
#[unsafe(no_mangle)]
pub static mut RAW: u32 = 0;
/// 最近一轮序列结果：[0]=PA1（AIN1）、[1]=PA2（AIN2），低 12 位有效
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

fn err_code(e: Scan8AdcError) -> u32 {
    match e {
        Scan8AdcError::Timeout => 1,
        Scan8AdcError::BadConfig => 2,
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

    // ADCCLK = PCLK/4 = 2MHz——扫描组电压域上限同经典组（VDDA 2~2.4V 档
    // 4MHz），Div4 在各电压域都安全（3.3V 供电下 Div1=8MHz 亦可）
    let mut adc = Scan8Adc::new(p.adc, Scan8Clk::Div4, Irqs);
    // PA00/PA01/PA02 → 模拟模式（AIN0/AIN1/AIN2，表 22-5 通道映射）
    let _pa0 = Pin::new(p.gpioa, 0).into_analog();
    let _pa1 = Pin::new(p.gpioa, 1).into_analog();
    let _pa2 = Pin::new(p.gpioa, 2).into_analog();

    loop {
        // 单通道单次（悬空引脚读数不定，验证通路本身）
        match adc.read(Scan8Channel::In(0), Scan8SampleTime::S10).await {
            Ok(raw) => unsafe { RAW = raw as u32 },
            Err(e) => unsafe { ERR = err_code(e) },
        }
        // 序列扫描：PA1、PA2 两通道（结果按序落 SEQ[0]、SEQ[1]）
        let chs = [Scan8Channel::In(1), Scan8Channel::In(2)];
        let mut seq = [0u16; 4];
        match adc
            .read_sequence(&chs, Scan8SampleTime::S10, &mut seq)
            .await
        {
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
