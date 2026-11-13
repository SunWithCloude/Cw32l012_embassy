//! 异步 OPA 演示：跟随器 + PGA 增益，ADC 片内读回（零外部接线）。
//!
//! 链路（OPA 输出脚同时是 ADC 通道，手册表 29-1 + 表 25-4）：
//!
//! 1. **跟随器**（板验路径）：`DAC_OUT1` 内部 → OPA1 正端 INP4 → 增益 1 →
//!    `OPA1_OUT = PB00` → `ADC1_IN8` 读回。寄存器路径与阻塞版
//!    `cw32l012-hal/examples/analog_loop.rs` 同源（其 DAC→OPA1 跟随器
//!    →PB00→VC1 链路 2026-10-23 上板通过），本例把观测端换成 ADC。
//!    预期 `MAX_ERR` ≤ 30 LSB（运放失调 + 增益误差 + ADC 抖动）。
//! 2. **PGA 增益 ×2**（API 演示，待板验证）：`DAC_OUT2` 内部 → OPA2 正端
//!    INP4 → PGA ×2 → `OPA2_OUT = PB01` → `ADC1_IN9` 读回，期望
//!    ≈ min(2×设定, 4095)。⚠ 内部 PGA 反馈网络是否需要负端外接未上板
//!    核对（阻塞版无 PGA 实测配方），`PGA_ERR` 只记录实测偏差供核对。
//!
//! 上电后先对 OPA1 做一次**触发校准**（自动结束，`is_calibrating` 轮询）。
//!
//! 观测静态量（`probe-rs read b32 0x20000000 16`，地址以 `nm` 为准）：
//! `RUNS`（心跳）、`MAX_ERR`（跟随器最大误差 LSB）、`PGA_ERR`（PGA×2
//! 最大误差 LSB）、`OPA_ERR`（ADC 错误码，编码同 dac_adc_loop）。
//!
//! cargo build --target thumbv6m-none-eabi --release --features rt --example opa

#![no_std]
#![no_main]

use embassy_executor::main;
use embassy_time::Timer;

use embassy_cw32::{
    Config, Peripherals,
    adc::{Adc, AdcChannel, AdcClk, SampleCycles},
    dac::Dac,
    gpio::{Level, Output, Pin},
    opa::{CalPeriod, Opa, OpaMode, PgaGain},
};

embassy_cw32::bind_interrupts!(struct Irqs {
    ADC1 => embassy_cw32::adc::Adc1Irq;
});

/// 完成 ramp 轮数（心跳）
#[used]
#[unsafe(no_mangle)]
pub static mut RUNS: u32 = 0;
/// 跟随器最大回环误差（LSB）
#[used]
#[unsafe(no_mangle)]
pub static mut MAX_ERR: u32 = 0;
/// PGA ×2 最大回环偏差（LSB；待板验证，见文件头注）
#[used]
#[unsafe(no_mangle)]
pub static mut PGA_ERR: u32 = 0;
/// 最近一次 ADC 读取错误码：0=无 1=Timeout 2=BadCalibration
/// 3=UnsupportedChannel 4=SensorDead 9=其它
#[used]
#[unsafe(no_mangle)]
pub static mut OPA_ERR: u32 = 0;

#[main]
async fn main(_spawner: embassy_executor::Spawner) {
    embassy_cw32::init(Config::default());
    let p = Peripherals::take().unwrap();

    // GPIOC 时钟 + PC13 LED 心跳
    embassy_cw32::sysctrl_enable_gpioc(p.sysctrl);
    let mut led = Output::new(Pin::new(p.gpioc, 13), Level::High);

    let mut dac = Dac::new(p.dac);
    let mut adc = Adc::new_adc1(p.adc1, AdcClk::Div2, Irqs);

    // OPA1 跟随器：INP4 = DAC_OUT1 内部，输出 PB00（OPA1_OUT）
    let mut opa1 = Opa::new_opa1(p.opa1); // APBEN2.OPA 时钟
    opa1.enable_output_pin(); // PB00 → 模拟（TESTING.md 实测坑：不配模拟输出被短路）
    opa1.enable_inputs(embassy_cw32::pac::opa::CR_INP4EN, 0); // INP4 = DAC 内部
    opa1.set_mode(OpaMode::Follower);
    opa1.enable(); // 自动启动 BGR（§25.12.19）

    // 上电校准（消除失调）：软件触发一次，自动结束
    opa1.trigger_calibration(CalPeriod::C1024); // 1024 OPACLK ≈ 256µs @PCLK/1
    while opa1.is_calibrating() {}

    // OPA2 PGA ×2：INP4 = DAC_OUT2 内部，输出 PB01（OPA2_OUT）
    let mut opa2 = Opa::new_opa2(p.opa2);
    opa2.enable_output_pin();
    opa2.enable_inputs(embassy_cw32::pac::opa::CR_INP4EN, 0);
    opa2.configure(OpaMode::Pga(PgaGain::X2), embassy_cw32::opa::Bias::Ua8);
    opa2.enable();

    loop {
        // 跟随器回环：0 → 4095 斜坡，ADC IN8（=PB00）读回
        for step in 0..=16u16 {
            let set = (step * 256).min(4095);
            dac.set_ch1(set).ok(); // INP4 直连 DAC_OUT1，增益 1
            Timer::after_micros(50).await; // OPA 建立（偏置 8µA 约 4µs）+ ADC 采样前稳定
            match adc.read(AdcChannel::In(8), SampleCycles::S24).await {
                Ok(raw) => unsafe {
                    let err = raw.abs_diff(set) as u32;
                    if err > MAX_ERR {
                        MAX_ERR = err;
                    }
                    OPA_ERR = 0;
                },
                Err(e) => unsafe {
                    OPA_ERR = adc_err_code(&e);
                },
            }
        }

        // PGA ×2 回环：DAC2 → OPA2 → ADC IN9（=PB01），期望 min(2×设定, 4095)
        for step in 0..=16u16 {
            let set = (step * 128).min(2047); // ×2 后仍 <4095，避免饱和段
            dac.set_ch2(set).ok();
            Timer::after_micros(50).await;
            match adc.read(AdcChannel::In(9), SampleCycles::S24).await {
                Ok(raw) => unsafe {
                    let want = (set as u32 * 2).min(4095);
                    let err = (raw as u32).abs_diff(want);
                    if err > PGA_ERR {
                        PGA_ERR = err;
                    }
                    OPA_ERR = 0;
                },
                Err(e) => unsafe {
                    OPA_ERR = adc_err_code(&e);
                },
            }
        }

        unsafe { RUNS += 1 };
        led.toggle();
        Timer::after_millis(500).await;
    }
}

/// AdcError → 观测码（与 dac_adc_loop 例程同编码）
fn adc_err_code(e: &embassy_cw32::adc::AdcError) -> u32 {
    match e {
        embassy_cw32::adc::AdcError::Timeout => 1,
        embassy_cw32::adc::AdcError::BadCalibration => 2,
        embassy_cw32::adc::AdcError::UnsupportedChannel => 3,
        embassy_cw32::adc::AdcError::SensorDead => 4,
    }
}

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {
        core::hint::spin_loop();
    }
}
