//! 异步 RTC 闹钟示例（L012，零接线）：
//!
//! 1. **时基**：优先 LSE（外部 32.768kHz 晶体，起振等待 ~2s），失败退
//!    LSI（内部 RC，出厂 TRIM，本板实测 32768Hz、误差 <1%——阻塞版
//!    rtc_demo 实机配方同款回退链）
//! 2. **设日期**：2026-10-31（周六）23:59:50，启动计时（后续闹钟若跨
//!    零点可顺带观测跨日进位）
//! 3. **异步闹钟**：闹钟 A = 当前秒 +5（分/时屏蔽 → 每分钟该秒触发），
//!    `wait_alarm().await` 睡到触发后重设下一次——唤醒计数落静态量
//! 4. **心跳**：独立任务 500ms 翻转 PC13——闹钟等待期间 LED 不停闪，
//!    即"等闹钟不阻塞执行器"的直观证据
//!
//! 观察量（SRAM 静态量，`probe-rs read` 观测，地址以 nm 输出为准；
//! debug 构建会把未逃逸的内部 static 优化掉，`#[used]#[no_mangle]`
//! 锁符号）：
//!
//! - `CLK_SRC`: 1=LSE、2=LSI、0=双失败（挂死待查）
//! - `SET_AT`: 设定时间（hour<<16 | minute<<8 | second）
//! - `WAKE_COUNT`: 闹钟唤醒计数（每 ~5s +1）
//! - `WAKE_AT`: 最近一次唤醒时刻（同上打包）
//! - `ERR`: 0=无、1=BadDateTime、2=ClockNotReady
//!
//! cargo build --target thumbv6m-none-eabi --release --features rt --example rtc_alarm

#![no_std]
#![no_main]

use embassy_executor::{Spawner, main};
use embassy_time::Timer;

use embassy_cw32::{
    Config, Peripherals,
    gpio::{Level, Output, Pin},
    rtc::{Alarm, DateTime, Rtc, RtcClock},
};

embassy_cw32::bind_interrupts!(struct Irqs {
    RTC => embassy_cw32::rtc::RtcIrq;
});

/// 实际使用的时钟源：1=LSE、2=LSI、0=双失败
#[used]
#[unsafe(no_mangle)]
pub static mut CLK_SRC: u32 = 0;
/// 设定时间（hour<<16 | minute<<8 | second）
#[used]
#[unsafe(no_mangle)]
pub static mut SET_AT: u32 = 0;
/// 闹钟唤醒计数（静态量，每 ~5s +1）
#[used]
#[unsafe(no_mangle)]
pub static mut WAKE_COUNT: u32 = 0;
/// 最近一次唤醒时刻（hour<<16 | minute<<8 | second）
#[used]
#[unsafe(no_mangle)]
pub static mut WAKE_AT: u32 = 0;
/// 错误码：0=无 1=BadDateTime 2=ClockNotReady
#[used]
#[unsafe(no_mangle)]
pub static mut ERR: u32 = 0;

/// PC13 心跳：独立任务 500ms 翻转，闹钟等待期间持续闪烁
#[embassy_executor::task]
async fn heartbeat(mut led: Output<embassy_cw32::Gpio>) {
    loop {
        led.toggle();
        Timer::after_millis(500).await;
    }
}

fn pack_time(d: &DateTime) -> u32 {
    ((d.hour as u32) << 16) | ((d.minute as u32) << 8) | d.second as u32
}

#[main]
async fn main(spawner: Spawner) {
    embassy_cw32::init(Config::default()); // 复位默认 SysClk = 4MHz

    let p = Peripherals::take().unwrap();
    embassy_cw32::sysctrl_enable_gpioc(p.sysctrl);
    let led = Output::new(Pin::new(p.gpioc, 13), Level::High);
    // embassy-executor 0.10：task 宏生成 Result<SpawnToken, SpawnError>，
    // spawn() 返回 ()（失败时内部 panic）
    spawner.spawn(heartbeat(led).unwrap());

    // ---- 时基：LSE 优先，失败退 LSI（阻塞版 rtc_demo 实机配方）----
    // 注：LSE 失败后 CR1.LSEEN 仍置位（只能 POR 清），无晶振板上无害
    let mut rtc = match Rtc::new(p.rtc, RtcClock::Lse) {
        Ok(r) => {
            unsafe { CLK_SRC = 1 };
            r
        }
        Err(_) => match Rtc::new(p.rtc, RtcClock::Lsi) {
            Ok(r) => {
                unsafe { CLK_SRC = 2 };
                r
            }
            Err(_) => {
                unsafe { ERR = 2 }; // ClockNotReady：双时钟源都失败
                loop {
                    Timer::after_secs(1).await; // 心跳仍在跑，便于观测挂死态
                }
            }
        },
    };

    // ---- P1：设 2026-10-31（周六）23:59:50，启动计时 ----
    let t0 = DateTime {
        year: 26,
        month: 10,
        day: 31,
        weekday: 6,
        hour: 23,
        minute: 59,
        second: 50,
    };
    if rtc.set_datetime(&t0).is_err() {
        unsafe { ERR = 1 }; // BadDateTime
    }
    unsafe { SET_AT = pack_time(&t0) };
    rtc.start();

    // ---- P2：闹钟 A = 当前秒 +5（分/时屏蔽 → 每分钟该秒触发）----
    let now = rtc.now();
    let target = (now.second + 5) % 60;
    rtc.set_alarm_a(&Alarm {
        weekmask: 0x7F,
        hour: None,
        minute: None,
        second: Some(target),
    });
    rtc.enable_alarm_a(true);
    // 清热复位残留标志（IWDT 复位不清 RTC 域）：保证首次 wait 睡到
    // 下一次真实匹配而不是立即返回
    rtc.clear_flags(embassy_cw32::rtc::IRQ_ALARMA);

    loop {
        rtc.wait_alarm().await; // ← 异步睡到闹钟（IRQ2 唤醒，心跳不受影响）
        unsafe {
            WAKE_COUNT += 1;
            WAKE_AT = pack_time(&rtc.now());
        }
        // 下一个闹钟：当前秒 +5（闹钟粒度秒级匹配，见 rtc 模块文档）
        let now = rtc.now();
        rtc.set_alarm_a(&Alarm {
            weekmask: 0x7F,
            hour: None,
            minute: None,
            second: Some((now.second + 5) % 60),
        });
    }
}

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {
        core::hint::spin_loop();
    }
}
