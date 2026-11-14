//! CW32F003 ADC 读取示例（经典组 ADC 的 f003 变体，embassy 异步，软件触发）：
//!
//! 1. **PA1（IN1）** 单次读（`read`，`into_analog` 外部通道；悬空值不定）
//! 2. **PA4+PA6（IN2/IN3）** 序列扫描读（`read_sequence`，MODE=4，
//!    一次 START 按 SQR0 槽序转换，结果落 RESULT0/RESULT1）
//! 3. PA0 心跳 LED（f003 引脚少，PA0 非 ADC 通道）
//!
//! f003 的通道-引脚映射与 f030 不同（IN0=PB2、IN1=PA1、IN2=PA4、
//! IN3=PA6、IN4=PA7、IN5..7=PC0..2、IN8=PB0、IN9=PB1、IN10=PB6、
//! IN11=PB5、IN12=PB3），且无 DMA（CR1 无 DMAEN）——本例只用软件触发。
//!
//! ⚠ 经典组 ADC 驱动为语义级实现（f003 寄存器面与 f030 逐位段同款，
//!   尾部 14 位扩展 RESULT140..143/ENABLE14 不触碰）；尚未实板验证，
//!   首次上板请核对 RAW/SEQ 数值合理性（GND≈0、VDDA≈4095）。
//!
//! cargo build --target thumbv6m-none-eabi --release --no-default-features \
//!     --features "rt,cw32f003" --example adc_read_f003

#![no_std]
#![no_main]

use embassy_executor::main;
use embassy_time::Timer;

use embassy_cw32::{
    Config, Peripherals,
    adc::{ClassicAdc, ClassicAdcError, ClassicChannel, ClassicClk, ClassicSampleTime},
    gpio::{Level, Output, Pin},
};

embassy_cw32::bind_interrupts!(struct Irqs {
    ADC => embassy_cw32::adc::ClassicAdcIrq; // IRQ12
});

/// 最近一次 PA1（IN1）单次读原始值（0..4095 右对齐）
#[used]
#[unsafe(no_mangle)]
pub static mut RAW: u32 = 0;
/// 最近一轮序列结果：[0]=PA4（IN2）、[1]=PA6（IN3），低 12 位有效
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

    // GPIOA（ADC 输入脚 + PA0 LED）时钟
    embassy_cw32::sysctrl_enable_gpioa(p.sysctrl);
    let mut led = Output::new(Pin::new(p.gpioa, 0), Level::High);

    // ADCCLK = PCLK/4 = 2MHz——电压域上限同经典组（VDDA 2~2.4V 档 4MHz），
    // Div4 在各电压域都安全（3.3V 供电下 Div1=8MHz 亦可）
    let mut adc = ClassicAdc::new(p.adc, ClassicClk::Div4, Irqs);
    // PA1/PA4/PA6 → 模拟模式（IN1/IN2/IN3，f003 通道映射）
    let _pa1 = Pin::new(p.gpioa, 1).into_analog();
    let _pa4 = Pin::new(p.gpioa, 4).into_analog();
    let _pa6 = Pin::new(p.gpioa, 6).into_analog();

    loop {
        // 单通道单次（悬空引脚读数不定，验证通路本身）
        match adc
            .read(ClassicChannel::In(1), ClassicSampleTime::S10)
            .await
        {
            Ok(raw) => unsafe { RAW = raw as u32 },
            Err(e) => unsafe { ERR = err_code(e) },
        }
        // 序列扫描：PA4、PA6 两通道（结果按序落 SEQ[0]、SEQ[1]）
        let chs = [ClassicChannel::In(2), ClassicChannel::In(3)];
        let mut seq = [0u16; 4];
        match adc
            .read_sequence(&chs, ClassicSampleTime::S10, &mut seq)
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
