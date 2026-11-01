//! 异步 DAC → ADC 片内回环自测（阻塞版 dac_adc_loop 例程的 embassy 版，
//! 零外部接线）
//!
//! 原理：DAC 通道 2 的输出在芯片内部直连 ADC1 的 IN12 通道（手册
//! 表 25-4；§26/表 9-2 的 CH2 内部互联）——写 DAC 一个值，立刻用 ADC
//! 读回来，比对误差。直写模式（TEN=0）为阻塞版实机配方：本例寄存器
//! 路径与其 `dac_adc_loop.rs` 完全同源（上板 15 轮扫描 MAX_ERR=3 LSB）。
//!
//! `MAX_ERR` 记录全程最大 |DAC 设定 − ADC 读回|（LSB）。
//! 预期：≤ 30 LSB（DAC/ADC 增益失调 + VDDA 噪声；典型 <15，本板 3）。
//! `RUNS` 记录完成的 ramp 轮数（持续递增 = 固件活着）。
//!
//! 结果落 SRAM 静态量，经 `probe-rs read` 观测（地址以 `nm` 输出为准）：
//!
//! ```text
//! probe-rs read b32 0x20000000 8
//! ```
//!
//! cargo build --target thumbv6m-none-eabi --release --features rt --example dac_adc_loop

#![no_std]
#![no_main]

use embassy_executor::main;
use embassy_time::Timer;

use embassy_cw32::{
    Config, Peripherals,
    adc::{Adc, AdcChannel, AdcClk, SampleCycles},
    dac::Dac,
    gpio::{Level, Output, Pin},
};

embassy_cw32::bind_interrupts!(struct Irqs {
    ADC1 => embassy_cw32::adc::Adc1Irq;
});

/// 全程最大回环误差（LSB）
#[used]
#[unsafe(no_mangle)]
pub static mut MAX_ERR: u32 = 0;
/// 完成 ramp 轮数（心跳计数）
#[used]
#[unsafe(no_mangle)]
pub static mut RUNS: u32 = 0;
/// 最近一次 ADC 读取错误码：0=无 1=Timeout 2=BadCalibration
/// 3=UnsupportedChannel 4=SensorDead 9=其它
#[used]
#[unsafe(no_mangle)]
pub static mut ADC_ERR: u32 = 0;
/// 最近一次设定的 DAC 码（观测用）
#[used]
#[unsafe(no_mangle)]
pub static mut LAST_SET: u32 = 0;

#[main]
async fn main(_spawner: embassy_executor::Spawner) {
    embassy_cw32::init(Config::default());
    let p = Peripherals::take().unwrap();

    // GPIOC 时钟 + PC13 LED 心跳
    embassy_cw32::sysctrl_enable_gpioc(p.sysctrl);
    let mut led = Output::new(Pin::new(p.gpioc, 13), Level::High);

    let mut dac = Dac::new(p.dac);
    let mut adc = Adc::new_adc1(p.adc1, AdcClk::Div2, Irqs);

    loop {
        // 0 → 4095 斜坡，每 256 LSB 一个测试点
        for step in 0..=16u16 {
            let set = (step * 256).min(4095);
            dac.set_ch2(set).ok(); // TEN=0 直写，1 个 dac_pclk 载入 DOR
            // 等待 DHR→DOR 传输 + 模拟建立（TEN=0 下 1 APCLK 转移，
            // 建立 RC 时间常数在 µs 级；给足裕量——阻塞版同款 50µs）
            Timer::after_micros(50).await;
            match adc.read(AdcChannel::In(12), SampleCycles::S24).await {
                Ok(raw) => {
                    let err = raw.abs_diff(set);
                    unsafe {
                        if err as u32 > MAX_ERR {
                            MAX_ERR = err as u32;
                        }
                        ADC_ERR = 0;
                        LAST_SET = set as u32;
                    }
                }
                Err(e) => unsafe {
                    ADC_ERR = match e {
                        embassy_cw32::adc::AdcError::Timeout => 1,
                        embassy_cw32::adc::AdcError::BadCalibration => 2,
                        embassy_cw32::adc::AdcError::UnsupportedChannel => 3,
                        embassy_cw32::adc::AdcError::SensorDead => 4,
                    };
                },
            }
        }
        unsafe { RUNS += 1 };
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
