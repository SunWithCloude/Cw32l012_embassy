//! 异步 GTIM PWM 验证示例（L012，GTIM1_CH1 @ PB04，AF6）：
//!
//! 1kHz、占空比 10%→90%→10% 万分比扫动；每个 PWM 周期用
//! `wait_update().await` 节拍（验证 UIF 更新中断 + waker 异步路径）。
//! 验证方式（无示波器也可）：
//! - `PERIOD` 静态量记录 (ARR<<16 | PSC) 回读——预期 ARR=3999、PSC=0
//!   （1kHz @ PCLK 4MHz）
//! - `UPDATES` 计数随时间线性增长（≈1000/s，即 wait_update 真在等
//!   更新中断而非空转）
//! - `CNT_MIN/MAX` 采样计数器滚动范围——预期铺满 0..ARR
//! - PC13 LED 每 500 个更新事件翻转（1Hz 心跳）
//! - 接示波器/逻辑分析仪到 PB04 可看 1kHz 方波，占空比呈呼吸扫动
//!
//! 结果落 SRAM 静态量，经 `probe-rs read` 观测（debug 构建会把地址
//! 未逃逸的内部 static 优化掉，`#[used]#[no_mangle]` 锁符号）。
//!
//! cargo build --target thumbv6m-none-eabi --release --no-default-features --features "rt,cw32l012" --example timer_pwm

#![no_std]
#![no_main]

use embassy_executor::main;

use embassy_cw32::{
    Config, Peripherals,
    gpio::{Level, Output, Pin},
    timer::{GtimPwm, PwmChannel, PwmConfig},
};

// wait_update 需要 GTIM1 更新中断（纯 PWM 用法则无需绑定任何中断）
embassy_cw32::bind_interrupts!(struct Irqs {
    GTIM1 => embassy_cw32::timer::Gtim1Irq;
});

/// (ARR<<16 | PSC) 配置回读
#[used]
#[unsafe(no_mangle)]
pub static mut PERIOD: u32 = 0;
/// 占空比读回（万分比，由 CCR/ARR 反算）
#[used]
#[unsafe(no_mangle)]
pub static mut DUTY_RB: u16 = 0;
/// 当前设定的占空比（万分比）
#[used]
#[unsafe(no_mangle)]
pub static mut DUTY: u16 = 0;
/// wait_update 完成次数（预期 ≈1000/s）
#[used]
#[unsafe(no_mangle)]
pub static mut UPDATES: u32 = 0;
/// CNT 采样最小值
#[used]
#[unsafe(no_mangle)]
pub static mut CNT_MIN: u16 = 0xFFFF;
/// CNT 采样最大值
#[used]
#[unsafe(no_mangle)]
pub static mut CNT_MAX: u16 = 0;
/// 构造错误码：0=无 1=BadDuty 2=OutOfRange
#[used]
#[unsafe(no_mangle)]
pub static mut ERR: u32 = 0;

#[main]
async fn main(_spawner: embassy_executor::Spawner) {
    embassy_cw32::init(Config::default()); // 复位默认 SysClk/PCLK = 4MHz

    let p = Peripherals::take().unwrap();

    // GPIOB(PB04 PWM 脚) + GPIOC(PC13 LED) 同写一个 AHBEN——KEY 保护
    // 寄存器整字写，分两次写会互相顶掉使能位（lib.rs 位定义：B=5/C=6）
    p.sysctrl
        .ahben
        .write(embassy_cw32::SYSCTRL_KEY | (1 << 5) | (1 << 6));
    let mut led = Output::new(Pin::new(p.gpioc, 13), Level::High);

    // PWM：PB04 = GTIM1_CH1（AF6，手册表 9-2；阻塞版 gtim_pwm.rs 同脚）
    let pwm = GtimPwm::new(
        p.gtim1,
        Pin::new(p.gpiob, 4),
        6,
        PwmConfig {
            channel: PwmChannel::Ch1,
            freq_hz: 1_000,
            duty_bp: 1_000, // 初始 10%
        },
    );
    let mut pwm = match pwm {
        Ok(t) => t,
        Err(embassy_cw32::timer::PwmError::BadDuty) => {
            unsafe { ERR = 1 };
            loop {
                core::hint::spin_loop();
            }
        }
        Err(embassy_cw32::timer::PwmError::OutOfRange) => {
            unsafe { ERR = 2 };
            loop {
                core::hint::spin_loop();
            }
        }
    };

    let (arr, psc) = pwm.period_config();
    unsafe { PERIOD = (arr as u32) << 16 | psc as u32 };

    // 占空比 10%→90%→10% 循环扫动（每步 5%，每 PWM 周期一步）
    let mut duty: u16 = 1_000;
    let mut dir_up = true;
    loop {
        // 万分比直接控（等价于 SetDutyCycle：duty = max_duty_cycle() * x / 100）
        pwm.set_duty_bp(duty).ok();
        // 等一个真实 PWM 周期（GTIM1 更新中断 + waker 异步路径）
        pwm.wait_update().await;
        unsafe {
            DUTY = duty;
            UPDATES += 1;
            DUTY_RB = pwm.duty_bp();
            let c = pwm.count();
            if c < CNT_MIN {
                CNT_MIN = c;
            }
            if c > CNT_MAX {
                CNT_MAX = c;
            }
            if UPDATES % 500 == 0 {
                led.toggle(); // 1kHz 更新节拍下 1Hz 心跳
            }
        }
        if dir_up {
            duty += 500;
            if duty >= 9_000 {
                dir_up = false;
            }
        } else {
            duty = duty.saturating_sub(500);
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
