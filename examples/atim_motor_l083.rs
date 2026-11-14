//! 经典 ATIM 三相互补 PWM 验证示例（l083，LQFP64 引脚面）：
//! CH1A@PA08 + CH1B@PB13、CH2A@PA09 + CH2B@PB14、CH3A@PA10 + CH3B@PB15，
//! 全部 AF7。20kHz 载波、500ns 对称死区，三路同斜坡 10%→40%→10% 循环，
//! 每步 50 个 PWM 周期（wait_update 节拍，验证 UIF 更新中断 + waker 异步
//! 路径）；PC13 LED 心跳。
//!
//! 引脚/AF 出处（双源一致，完整引脚表见 `src/atim_classic.rs` 模块文档）：
//! - 数据手册《CW32L083 DataSheet V1.9》表 5-3（印刷页 33：PA08=ATIM_CH1A、
//!   PA09=ATIM_CH2A、PA10=ATIM_CH3A）、表 5-4（印刷页 34：PB13=ATIM_CH1B、
//!   PB14=ATIM_CH2B、PB15=ATIM_CH3B）
//! - SDK 宏 `PA08_AFx_ATIMCH1A`/`PA09_AFx_ATIMCH2A`/`PA10_AFx_ATIMCH3A`
//!   （`cw32l083_gpio.h:1530/1538/1546`）与 `PB13_AFx_ATIMCH1B`/
//!   `PB14_AFx_ATIMCH2B`/`PB15_AFx_ATIMCH3B`（`:1695/1703/1711`），
//!   `CW32L083_StandardPeripheralLib_V2.2.zip`（sha256 2d587655…）
//! - 语义证据：用户手册《CW32L083 UM V2.0》§16.3.3.5（互补+死区配方，
//!   印刷页 317-318）、§16.7.1/§16.7.5/§16.7.9（CR/ICR/DTR，页 335-336/
//!   340/345）——死区四段/ICR 写 0 清/MODE=10 与 f030 逐字一致
//!
//! 验证方式（无示波器也可）：
//! - `PERIOD` 静态量记录 (ARR<<16 | PRS 档位原始值)——20kHz@8MHz 复位默认
//!   预期 ARR=399、档位=0（分频 1）
//! - `DT_TICKS`/`DT_CODE`——500ns@8MHz TCLK = 4 tick → 编码 2（预期 4/2）
//! - `DUTY_A/B/C` 三路读回应同步（同斜坡）
//! - `UPDATES` 计数随时间线性增长（≈20000/s，即 wait_update 真在等
//!   UIF 更新中断而非空转）
//! - 接示波器/逻辑分析仪：PA8-10/PB13-15 三对互补方波，对边沿死区约 500ns
//!   （4 tick @ 125ns），占空比呈同步斜坡
//! - ⚠ **compiles-not-run**：本示例未上板跑过；驱动安全边界（MOE 默认
//!   关、端点占空比拒绝、无刹车源）见 `atim_classic` 模块文档
//!
//! cargo build --target thumbv6m-none-eabi --release --no-default-features --features "rt,critical-section-impl,cw32l083" --example atim_motor_l083

#![no_std]
#![no_main]

use embassy_executor::main;

use embassy_cw32::{
    Config, Peripherals,
    atim_classic::{AtimChannel, AtimError, AtimPair, AtimPwm, AtimPwmConfig},
    gpio::{Level, Output, Pin},
};

// wait_update 需要 ATIM 更新中断（IRQ13）
embassy_cw32::bind_interrupts!(struct Irqs {
    ATIM => embassy_cw32::atim_classic::AtimIrq;
});

/// (ARR<<16 | PRS 档位原始值) 配置回读
#[used]
#[unsafe(no_mangle)]
pub static mut PERIOD: u32 = 0;
/// 死区 TCLK tick 数（0 = DTEN 关）
#[used]
#[unsafe(no_mangle)]
pub static mut DT_TICKS: u32 = 0;
/// 死区编码（0xFF = DTEN 关）
#[used]
#[unsafe(no_mangle)]
pub static mut DT_CODE: u8 = 0xFF;
/// 三路占空比读回（万分比）
#[used]
#[unsafe(no_mangle)]
pub static mut DUTY_A: u16 = 0;
#[used]
#[unsafe(no_mangle)]
pub static mut DUTY_B: u16 = 0;
#[used]
#[unsafe(no_mangle)]
pub static mut DUTY_C: u16 = 0;
/// wait_update 完成次数（预期 ≈20000/s）
#[used]
#[unsafe(no_mangle)]
pub static mut UPDATES: u32 = 0;
/// 构造错误码：0=无 1=NoPairs 2=OutOfRange 3=BadDuty 4=DeadTimeTooLong
#[used]
#[unsafe(no_mangle)]
pub static mut ERR: u32 = 0;

#[main]
async fn main(_spawner: embassy_executor::Spawner) {
    embassy_cw32::init(Config::default()); // 经典组复位默认 SysClk/PCLK = 8MHz

    let p = Peripherals::take().unwrap();

    // GPIOA/B/C 端口时钟（AHBEN bit4/5/6；经典组无 KEY，库内读改写）
    embassy_cw32::sysctrl_enable_gpioa(p.sysctrl);
    embassy_cw32::sysctrl_enable_gpiob(p.sysctrl);
    embassy_cw32::sysctrl_enable_gpioc(p.sysctrl);
    let mut led = Output::new(Pin::new(p.gpioc, 13), Level::High);

    // 三相互补：门控 APBEN2 bit7 由 AtimPwm::new 内部读改写打开；
    // 构造后 MOE=0（无输出），须显式 enable()
    let pwm = AtimPwm::new(
        p.atim,
        Some(AtimPair::new(
            Pin::new(p.gpioa, 8), // CH1A AF7（表 5-3 印刷页 33 + gpio.h:1530）
            7,
            Pin::new(p.gpiob, 13), // CH1B AF7（表 5-4 印刷页 34 + gpio.h:1695）
            7,
        )),
        Some(AtimPair::new(
            Pin::new(p.gpioa, 9), // CH2A AF7（表 5-3 + gpio.h:1538）
            7,
            Pin::new(p.gpiob, 14), // CH2B AF7（表 5-4 + gpio.h:1703）
            7,
        )),
        Some(AtimPair::new(
            Pin::new(p.gpioa, 10), // CH3A AF7（表 5-3 + gpio.h:1546）
            7,
            Pin::new(p.gpiob, 15), // CH3B AF7（表 5-4 + gpio.h:1711）
            7,
        )),
        AtimPwmConfig {
            freq_hz: 20_000,
            dead_time_ns: 500,
            duty_bp: 1_000, // 初始 10%（内部值）
        },
    );
    let mut pwm = match pwm {
        Ok(t) => t,
        Err(e) => {
            unsafe {
                ERR = match e {
                    AtimError::NoPairs => 1,
                    AtimError::OutOfRange => 2,
                    AtimError::BadDuty => 3,
                    AtimError::DeadTimeTooLong => 4,
                    AtimError::ChannelUnavailable => 5,
                };
            }
            loop {
                core::hint::spin_loop();
            }
        }
    };

    let (arr, prs_raw) = pwm.period_config();
    unsafe {
        PERIOD = (arr as u32) << 16 | prs_raw as u32;
        DT_TICKS = pwm.dead_time_ticks();
        DT_CODE = pwm.dead_time_code().unwrap_or(0xFF);
    }

    pwm.enable(); // DTR.MOE=1：三对输出一起开（构造默认关）

    // 占空比 10%→40%→10% 循环斜坡（每步 0.1%，每 50 个 PWM 周期一步：
    // 完整三角 ≈ 1.5s；占空比写 CCRA 影子，下一 UEV 生效）
    let mut duty: u16 = 1_000;
    let mut dir_up = true;
    loop {
        for ch in [AtimChannel::Ch1, AtimChannel::Ch2, AtimChannel::Ch3] {
            pwm.set_duty_bp(ch, duty).ok();
        }
        for _ in 0..50 {
            pwm.wait_update().await; // ATIM UIF 更新中断 + waker 异步路径
            unsafe { UPDATES += 1 };
        }
        unsafe {
            DUTY_A = pwm.duty_bp(AtimChannel::Ch1).unwrap_or(0xFFFF);
            DUTY_B = pwm.duty_bp(AtimChannel::Ch2).unwrap_or(0xFFFF);
            DUTY_C = pwm.duty_bp(AtimChannel::Ch3).unwrap_or(0xFFFF);
            if UPDATES % 20_000 == 0 {
                led.toggle(); // 20kHz 节拍下 1Hz 心跳
            }
        }
        if dir_up {
            duty += 10;
            if duty >= 4_000 {
                dir_up = false;
            }
        } else {
            duty -= 10;
            if duty <= 1_000 {
                dir_up = true;
            }
        }
    }
}

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {
        core::hint::spin_loop();
    }
}
