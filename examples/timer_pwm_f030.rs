//! 经典组异步 GTIM PWM 验证示例（f030，GTIM1_CH1 @ PB04，AF6）：
//!
//! 1kHz、占空比 10%→90%→10% 万分比扫动；每个 PWM 周期用
//! `wait_update().await` 节拍（验证 OV 溢出中断 + waker 异步路径）。
//!
//! 引脚/AF 出处：PB04 = GTIM1_CH1 = AF6——厂商库宏
//! `PB04_AFx_GTIM1CH1()`（CW32F030_StandardPeripheralLib V2.2
//! `cw32f030_gpio.h:931`，`CW_GPIOB->AFRL_f.AFR4 = 6`），与数据手册
//! 《CW32F030》V1.9 表 5-4（PB4 行）一致；PC13 是最小系统板 LED，
//! 不占作 PWM 脚，另作心跳指示。
//!
//! 验证方式（无示波器也可）：
//! - `PERIOD` 静态量记录 (ARR<<16 | 分频档原始值) 回读——预期 ARR=7999、
//!   分频档=0（f030 无独立 PSC 寄存器，CR0.PRS=0 即 2^0=1 分频；
//!   1kHz @ PCLK 8MHz 复位默认）
//! - `UPDATES` 计数随时间线性增长（≈1000/s，即 wait_update 真在等
//!   溢出中断而非空转）
//! - `CNT_MIN/MAX` 采样计数器滚动范围——预期铺满 0..ARR
//! - PC13 LED 每 500 个更新事件翻转（1Hz 心跳）
//! - 接示波器/逻辑分析仪到 PB04 可看 1kHz 方波，占空比呈呼吸扫动
//!
//! 结果落 SRAM 静态量，经 `probe-rs read` 观测（debug 构建会把地址
//! 未逃逸的内部 static 优化掉，`#[used]#[no_mangle]` 锁符号）。
//!
//! cargo build --target thumbv6m-none-eabi --release --no-default-features --features "rt,cw32f030" --example timer_pwm_f030

#![no_std]
#![no_main]

use embassy_executor::main;

use embassy_cw32::{
    Config, Peripherals,
    gpio::{Level, Output, Pin},
    timer::{ClassicPwm, PwmChannel, PwmConfig},
};

// wait_update 需要 GTIM1 溢出中断（IRQ16；纯 PWM 用法则无需绑定任何中断）
embassy_cw32::bind_interrupts!(struct Irqs {
    GTIM1 => embassy_cw32::timer::Gtim1Irq;
});

/// (ARR<<16 | 分频档原始值) 配置回读（f030 分频档 = CR0.PRS 场值，2^PRS 倍分频）
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
    embassy_cw32::init(Config::default()); // 经典组复位默认 SysClk/PCLK = 8MHz

    let p = Peripherals::take().unwrap();

    // GPIOB(PB04 PWM 脚) + GPIOC(PC13 LED)：经典组 AHBEN 无 KEY，读改写
    // 互不顶位（AHBEN bit5=B / bit6=C，同 lib.rs 位定义）
    p.sysctrl.ahben.modify(|v| v | (1 << 5) | (1 << 6));
    let mut led = Output::new(Pin::new(p.gpioc, 13), Level::High);

    // PWM：PB04 = GTIM1_CH1（AF6，厂商库宏 PB04_AFx_GTIM1CH1，见文件头出处）
    // 门控 APBEN1 bit1 由 ClassicPwm::new 内部读改写打开
    let pwm = ClassicPwm::new(
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

    let (arr, div_raw) = pwm.period_config();
    unsafe { PERIOD = (arr as u32) << 16 | div_raw as u32 };

    // 占空比 10%→90%→10% 循环扫动（每步 5%，每 PWM 周期一步）
    // ⚠ 经典 GTIM 无预装载：CCR 立即生效，可能截断当前脉冲（与 L012 不同）
    let mut duty: u16 = 1_000;
    let mut dir_up = true;
    loop {
        // 万分比直接控（等价于 SetDutyCycle：duty = max_duty_cycle() * x / 100）
        pwm.set_duty_bp(duty).ok();
        // 等一个真实 PWM 周期（GTIM1 溢出中断 + waker 异步路径）
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
