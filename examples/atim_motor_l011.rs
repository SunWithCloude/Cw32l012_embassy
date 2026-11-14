//! 三相互补 PWM + 死区 + 刹车演示（CW32L011 ATIM，L012 同构寄存器面）：
//!
//! 20kHz 三相互补对（六路，1µs 死区），占空比 0→100%→0 三角波斜坡、
//! 三路固定错 120°（粗略旋转矢量观感，**非正弦 SVPWM**——示例只演示
//! 互补输出与周期节拍，不做电机控制）。每个 PWM 周期
//! `wait_update().await` 供拍（验证 IRQ13 + waker 异步路径）。
//!
//! # 接线（数据手册表 5-3/5-4，全部 AF7；hal2 证据链
//! atim-pwm-route/atim-complementary-route 双源核对）
//!
//! | 信号 | 引脚 | 信号 | 引脚 |
//! |------|------|------|------|
//! | CH1  | PA8  | CH1N | PA7  |
//! | CH2  | PA9  | CH2N | PB0  |
//! | CH3  | PA10 | CH3N | PB1  |
//! | BK   | PA0  | （刹车输入，低电平有效；不接则传 None） |
//!
//! # BK 刹车接线示例（低电平有效；**可选接法**——`bk=None` 六路波形
//! 照常输出，只是没有硬件刹车保护，BIF 永不置位）
//!
//! ```text
//!  过流比较器 / 保护 IC（开漏输出）──┬── PA0（ATIM_BK，AF7）
//!                                    │
//!                                  10kΩ
//!                                    │
//!                                   VDD
//! ```
//!
//! 比较器拉低 PA0 → 硬件**异步**清 MOE，六路输出立即进入空闲安全态
//! （OISx=0 → 上/下管全关，不经软件、不经中断、不经死区逻辑）。软件侧
//! ISR.BIF 置位（`is_braked()`）；**刹车源活跃期间 BIF 清不掉**
//! （`clear_break()` 返回 false）。排除故障 → `clear_break()` →
//! `enable()` 手动恢复（本例 `aoe=false`：不自动恢复，恢复决策权留给
//! 上层）。BKF=0 无数字滤波；毛刺敏感场景给 0..=15 的滤波档。
//!
//! ⚠ **l010 家族没有本示例的刹车接线可用**：l010 无任何合格外部 BK
//! 引脚路线（BK 候选 PA1=OSC_OUT/PA7=SWDIO 均被排除），`AtimPwm::new`
//! 的 bk 参数必须传 None、驱动不接外部刹车——同一 API，引脚表见
//! src/atim.rs 模块文档。
//!
//! # 验证方式（无示波器也可）
//!
//! - `PERIOD` 静态量记录 (PSC<<16 | ARR) 回读——预期 PSC=0、ARR=199
//!   （20kHz @ PCLK 4MHz，复位默认时钟）
//! - `DTG` 记录 BDTR.DTG 编码回读——1µs @ 4MHz = 4 tick，预期 0x04
//! - `UPDATES` 计数随时间线性增长（≈20000/s，wait_update 真在等更新
//!   中断而非空转）
//! - `BRAKES` / `ENABLE_FAILS`：PA0 拉低再释放——BRAKES +1（刹车进入）、
//!   ENABLE_FAILS +1（刹车压着时 enable() 置不上 MOE），释放并
//!   clear_break + enable 后恢复斜坡
//! - 接示波器/逻辑分析仪到 PA8/PA7 可看互补方波 + 1µs 死区（高/低电平
//!   交叠处两路同低），PA9/PB0、PA10/PB1 同理且占空比错 120°
//!
//! 结果落 SRAM 静态量，经 `probe-rs read` 观测（debug 构建会把地址
//! 未逃逸的内部 static 优化掉，`#[used]#[no_mangle]` 锁符号）。
//!
//! ⚠ compiles-not-run：l010/l011 分支无阻塞版同源背书（L012 有），驱动
//! 语义见 src/atim.rs 模块文档（实机验证待板，上板前勿接真电机——功率级
//! 先用逻辑分析仪 + 小电压验证）。
//!
//! cargo build --target thumbv6m-none-eabi --release --no-default-features --features "rt,cw32l011,critical-section-impl" --example atim_motor_l011

#![no_std]
#![no_main]

use embassy_executor::main;

use embassy_cw32::{
    Config, Peripherals,
    atim::{AtimError, AtimPwm, AtimPwmConfig},
    gpio::Pin,
};

// wait_update 需要 ATIM 更新中断（IRQ13，三家族同号线）
embassy_cw32::bind_interrupts!(struct Irqs {
    ATIM => embassy_cw32::atim::AtimIrq;
});

/// (PSC<<16 | ARR) 配置回读（预期 PSC=0、ARR=199）
#[used]
#[unsafe(no_mangle)]
pub static mut PERIOD: u32 = 0;
/// BDTR.DTG 死区编码回读（预期 0x04 = 4 tick @ 4MHz 的 1µs）
#[used]
#[unsafe(no_mangle)]
pub static mut DTG: u32 = 0;
/// wait_update 完成次数（预期 ≈20000/s）
#[used]
#[unsafe(no_mangle)]
pub static mut UPDATES: u32 = 0;
/// 斜坡步进序号
#[used]
#[unsafe(no_mangle)]
pub static mut STEP: u32 = 0;
/// 刹车进入次数（PA0 拉低事件）
#[used]
#[unsafe(no_mangle)]
pub static mut BRAKES: u32 = 0;
/// 刹车压着时 enable() 失败次数（MOE 置不上）
#[used]
#[unsafe(no_mangle)]
pub static mut ENABLE_FAILS: u32 = 0;
/// 构造错误码：0=无 1=FreqOutOfRange 2=DeadTimeTooLong 3=BadBreakConfig
/// 4=BadDuty
#[used]
#[unsafe(no_mangle)]
pub static mut ERR: u32 = 0;

/// 三角波占空比（万分比）：0 → 10000 → 0，200 步一循环
fn ramp(step: u32) -> u16 {
    let s = step % 200;
    let v = if s < 100 { s * 100 } else { (200 - s) * 100 };
    (v.min(10_000)) as u16
}

#[main]
async fn main(_spawner: embassy_executor::Spawner) {
    embassy_cw32::init(Config::default()); // 复位默认 SysClk/PCLK = 4MHz

    let p = Peripherals::take().unwrap();

    // GPIOA(bit4: PA0/PA7-PA10) + GPIOB(bit5: PB0/PB1) 端口时钟——KEY
    // 内嵌 [31:16] 的保护寄存器整字写一次到位，分两次写会互相顶掉使能位
    // （lib.rs SYSCTRL_KEY 说明）
    p.sysctrl
        .ahben
        .write(embassy_cw32::SYSCTRL_KEY | (1 << 4) | (1 << 5));

    // 三相互补 + 死区 + 刹车（引脚/AF 见模块头注接线表；BK 不接传 None）
    // 不用的通道传 None：APIT 端口类型无法从 None 推断，需显式标注
    let no_pin: Option<(Pin<embassy_cw32::Gpio>, u8)> = None;
    let pwm = AtimPwm::new(
        p.atim,
        AtimPwmConfig {
            freq_hz: 20_000,     // 20kHz 载波（4MHz PCLK 下 ARR=199）
            dead_time_ns: 1_000, // 1µs 死区（4 tick，向上取整）
            aoe: false,          // 刹车不自动恢复——clear_break + enable 显式来
        },
        Some((Pin::new(p.gpioa, 8), 7)),  // CH1  = PA8  AF7
        Some((Pin::new(p.gpioa, 7), 7)),  // CH1N = PA7  AF7
        Some((Pin::new(p.gpioa, 9), 7)),  // CH2  = PA9  AF7
        Some((Pin::new(p.gpiob, 0), 7)),  // CH2N = PB0  AF7
        Some((Pin::new(p.gpioa, 10), 7)), // CH3  = PA10 AF7
        Some((Pin::new(p.gpiob, 1), 7)),  // CH3N = PB1  AF7
        no_pin,                           // CH4 单端（本例不用）
        Some((Pin::new(p.gpioa, 0), 7)),  // BK   = PA0  AF7（低有效；不接传 None）
        false,                            // BKP=0：低电平有效
        0,                                // BKF=0：无数字滤波
        Irqs,
    );
    let mut pwm = match pwm {
        Ok(t) => t,
        Err(e) => {
            unsafe {
                ERR = match e {
                    AtimError::FreqOutOfRange => 1,
                    AtimError::DeadTimeTooLong => 2,
                    AtimError::BadBreakConfig => 3,
                    AtimError::BadDuty => 4,
                };
            }
            loop {
                core::hint::spin_loop();
            }
        }
    };

    let (psc, arr) = pwm.period_config();
    unsafe {
        PERIOD = (psc as u32) << 16 | arr as u32;
        DTG = pwm.dtg() as u32;
    }

    // 置 MOE：构造时 MOE=0（输出空闲全低）。BK 已活跃则置不上 → 返回 false
    if !pwm.enable() {
        unsafe { ENABLE_FAILS += 1 };
    }

    // 占空比 0→100%→0 三角斜坡，三路错 120°（万分比 3334/6667）：
    // 非正弦，仅演示三相互补输出与 120° 相位观感
    let mut step: u32 = 0;
    loop {
        let d = ramp(step);
        pwm.set_duty_bp(1, d).ok();
        pwm.set_duty_bp(2, (d + 3_334) % 10_000).ok();
        pwm.set_duty_bp(3, (d + 6_667) % 10_000).ok();

        // 等一个真实 PWM 周期（ATIM 更新中断 + waker 异步路径）
        pwm.wait_update().await;

        // 刹车监控：BIF 置位即六路输出已被硬件异步压到安全态（MOE=0）。
        // clear_break 在刹车源仍活跃时返回 false；源释放后重新 enable。
        if pwm.is_braked() {
            unsafe { BRAKES += 1 };
            if pwm.clear_break() && !pwm.enable() {
                unsafe { ENABLE_FAILS += 1 };
            }
        }

        unsafe {
            STEP = step;
            UPDATES += 1;
        }
        step += 1;
    }
}

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {
        core::hint::spin_loop();
    }
}
