//! 异步 GTIM PWM 验证示例（CW32L010，GTIM1 单实例，GTIM1_CH1 @ PB6，AF6）：
//!
//! 1kHz、占空比 10%→90%→10% 万分比扫动；每个 PWM 周期用
//! `wait_update().await` 节拍（验证 IRQ16 更新中断 + waker 异步路径）。
//!
//! # 引脚核对（双源：数据手册表 5-4 + SDK 宏 PB06_AFx_GTIM1CH1）
//!
//! l010 的 GTIM1 路线（AF6）：CH1 = PA6/PB6、CH2 = PA5/PB5、
//! CH3 = PA4/PB4、CH4 = PA3/PB3（hal2 证据链
//! `buffered-pwm-route-evidence.json` CW32L010 retained=8，PA1 的 CH3
//! 路线被 OSC_OUT 占用排除）。⚠ 手册里 PB4 的 AF7 是 ATIM_CH1、AF6 才是
//! GTIM1_CH3——接 ATIM 互补 PWM 用 PB4@AF7，本例走 GTIM 用 PB6@AF6。
//!
//! # 验证方式（无示波器也可）
//!
//! - `PERIOD` 静态量记录 (ARR<<16 | PSC) 回读——预期 ARR=3999、PSC=0
//!   （1kHz @ PCLK 4MHz，复位默认时钟）
//! - `UPDATES` 计数随时间线性增长（≈1000/s，即 wait_update 真在等
//!   更新中断而非空转）
//! - `CNT_MIN/MAX` 采样计数器滚动范围——预期铺满 0..ARR
//! - l010 为 20 脚封装、无标准 LED 脚（PA7=SWDIO/PA8=SWCLK 不可动），
//!   心跳用 `UPDATES % 1000 == 0` 的 HEARTBEAT 计数代替
//! - 接示波器/逻辑分析仪到 PB6 可看 1kHz 方波，占空比呈呼吸扫动
//!
//! 结果落 SRAM 静态量，经 `probe-rs read` 观测（debug 构建会把地址
//! 未逃逸的内部 static 优化掉，`#[used]#[no_mangle]` 锁符号）。
//!
//! ⚠ compiles-not-run：l010 分支无阻塞版同源背书，寄存器位面按三家族
//! PAC 逐位比对 + hal2 证据链核对，首次上板先看 PERIOD/UPDATES 合理性。
//!
//! cargo build --target thumbv6m-none-eabi --release --no-default-features --features "rt,cw32l010" --example timer_pwm_l010

#![no_std]
#![no_main]

use embassy_executor::main;

use embassy_cw32::{
    Config, Peripherals,
    timer::{GtimPwm, PwmChannel, PwmConfig},
};

// wait_update 需要 GTIM1 更新中断（IRQ16；纯 PWM 用法则无需绑定任何中断）
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
/// 心跳计数（每 1000 次更新 +1 = 每秒 +1）
#[used]
#[unsafe(no_mangle)]
pub static mut HEARTBEAT: u32 = 0;
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

    // GPIOB 端口时钟（AHBEN bit5，KEY 内嵌 [31:16]——lib.rs SYSCTRL_KEY）
    p.sysctrl.ahben.write(embassy_cw32::SYSCTRL_KEY | (1 << 5));

    // PWM：PB6 = GTIM1_CH1（AF6，数据手册表 5-4 + SDK PB06_AFx_GTIM1CH1）
    let pwm = GtimPwm::new(
        p.gtim1,
        Pin::new(p.gpiob, 6),
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
            if UPDATES % 1000 == 0 {
                HEARTBEAT += 1; // 1kHz 更新节拍下每秒 +1
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
