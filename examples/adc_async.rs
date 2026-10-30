//! 异步 ADC 验证示例（L012，ADC1）：
//!
//! 1. **VDDA**（IN15 内部基准 + 出厂校准，预期 ~3.2-3.3V）
//! 2. **PA0**（IN0，`into_analog` 外部通道演示；悬空时值不定）
//! 3. **温度**（IN14，本板 TS 硬件无输出，预期 `TEMP_ERR=4=SensorDead`
//!    ——正好验证失效保护路径不吐绝对零度假数据）
//!
//! 结果落 SRAM 静态量，经 `probe-rs read` 观测（debug 构建会把地址
//! 未逃逸的内部 static 优化掉，`#[used]#[no_mangle]` 锁符号）：
//!
//! ```text
//! probe-rs read b32 0x20000000 4        # 地址以 nm 输出为准
//! ```
//!
//! cargo build --target thumbv6m-none-eabi --release --features rt --example adc_async

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
});

/// 最近一轮结果：[0]=VDDA mV，[1]=PA0 原始值，[2]=温度 m℃（失败为 0）。
/// 单任务独写 + probe-rs 只读，static mut 可接受。
#[used]
#[unsafe(no_mangle)]
pub static mut RESULTS: [u32; 3] = [0; 3];
/// 错误码：0=无 1=Timeout 2=BadCalibration 3=UnsupportedChannel 4=SensorDead
#[used]
#[unsafe(no_mangle)]
pub static mut TEMP_ERR: u32 = 0;
/// 完成的轮数（心跳计数）
#[used]
#[unsafe(no_mangle)]
pub static mut SAMPLES: u32 = 0;

#[main]
async fn main(_spawner: embassy_executor::Spawner) {
    embassy_cw32::init(Config::default());

    let p = Peripherals::take().unwrap();

    // GPIOC 时钟 + PC13 LED 心跳
    embassy_cw32::sysctrl_enable_gpioc(p.sysctrl);
    let mut led = Output::new(Pin::new(p.gpioc, 13), Level::High);

    let mut adc = Adc::new_adc1(p.adc1, AdcClk::Div2, Irqs);
    // PA0 → 模拟模式（IN0）
    let _pa0 = Pin::new(p.gpioa, 0).into_analog();

    loop {
        // VDDA（内部基准通道，自动处理 BGREN 稳定期）
        if let Ok(mv) = adc.vdda_mv().await {
            unsafe { RESULTS[0] = mv as u32 };
        }
        // PA0 外部通道（悬空读数不定，验证通路本身）
        if let Ok(raw) = adc.read(AdcChannel::In(0), SampleCycles::S24).await {
            unsafe { RESULTS[1] = raw as u32 };
        }
        // 温度：本板 TS 无输出，应得 SensorDead（TEMP_ERR=4）
        match adc.temperature_mc().await {
            Ok(mc) => unsafe { RESULTS[2] = mc as u32 },
            Err(embassy_cw32::adc::AdcError::Timeout) => unsafe { TEMP_ERR = 1 },
            Err(embassy_cw32::adc::AdcError::BadCalibration) => unsafe { TEMP_ERR = 2 },
            Err(embassy_cw32::adc::AdcError::UnsupportedChannel) => unsafe { TEMP_ERR = 3 },
            Err(embassy_cw32::adc::AdcError::SensorDead) => unsafe { TEMP_ERR = 4 },
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
