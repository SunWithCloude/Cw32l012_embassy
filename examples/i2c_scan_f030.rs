//! CW32F030(立创天猛星)异步 I2C 总线扫描示例。
//!
//! I2C1 主机:PA09=SCL / PA10=SDA(AF3,厂商宏 `PA09_AFx_I2C1SCL()`
//! / `PA10_AFx_I2C1SDA()`),标准模式 100kHz。总线上需外接 4.7kΩ
//! 上拉到 VDD,并与板卡共地。
//!
//! 周期性扫描地址 0x08..0x78:对每个地址发 START+ADDR+W(空数据,
//! i2cdetect 式探测),经 `embedded-hal-async` 的 `write()`——ACK 即
//! 命中、NACK(经典家族 I2C 状态机 STAT=0x20)即跳过;ACK 命中时
//! 状态码为 0x18(SLA+W 已 ACK)/0x28(数据字节已 ACK)。命中地址
//! 记入静态数组(宿主可经 probe-rs 读内存核对,无需串口)。
//!
//! LED(PC13)目视判定:
//! - 有命中:900ms 亮 / 100ms 灭(常亮间隔闪)
//! - 无命中:500ms 翻转(慢闪,1s 周期)
//!
//! 板上未接任何从器件时全地址 NACK,LED 慢闪即为正确行为。
//!
//! cargo build --target thumbv6m-none-eabi --release --no-default-features \
//!     --features "rt,cw32f030,critical-section-impl" --example i2c_scan_f030

#![no_std]
#![no_main]

use core::cell::RefCell;

use critical_section::Mutex;
use embassy_executor::main;
use embassy_time::Timer;
use embedded_hal_async::i2c::I2c as _;

use embassy_cw32::{
    Config, Peripherals,
    gpio::{Level, Output, Pin},
    i2c::{self, I2c},
    sysctrl_enable_gpioa, sysctrl_enable_gpioc,
};

embassy_cw32::bind_interrupts!(struct Irqs {
    I2C1 => i2c::I2c1Irq;
});

/// 扫描地址范围(i2cdetect 惯例:避开 0x00-0x07 保留段与 0x78 起的
/// 10 位地址前缀)
const SCAN_RANGE: core::ops::Range<u8> = 0x08..0x78;
/// 命中地址最多记录条数(超出只计总数)
const MAX_HITS: usize = 16;

/// 扫描命中记录(静态存放,便于宿主经 probe-rs 读内存核对):
/// `[0]` = 命中个数,其后依次为地址,空位保持 0。
static HITS: Mutex<RefCell<[u8; 1 + MAX_HITS]>> = Mutex::new(RefCell::new([0; 1 + MAX_HITS]));

/// 未命中原因计数(上板验证用):`[0]`=NACK(正常无器件)、
/// `[1]`=总线错误、`[2]`=其它错误。空总线上应全为 NACK。
/// `#[used]`:固件只写不读,防止链接器把符号裁掉(宿主要读内存)。
#[used]
static MISS: Mutex<RefCell<[u8; 3]>> = Mutex::new(RefCell::new([0; 3]));

/// 把本轮命中与未命中原因写入静态记录。
fn record_round(hits: &[u8], miss: [u8; 3]) {
    critical_section::with(|cs| {
        let mut slot = HITS.borrow_ref_mut(cs);
        slot[0] = hits.len() as u8;
        for (dst, src) in slot[1..].iter_mut().zip(hits) {
            *dst = *src;
        }
        *MISS.borrow_ref_mut(cs) = miss;
    });
}

#[main]
async fn main(_spawner: embassy_executor::Spawner) {
    embassy_cw32::init(Config::default()); // F030 复位默认 8MHz(HSI 48M/6)

    let p = Peripherals::take().unwrap();
    sysctrl_enable_gpioa(p.sysctrl);
    sysctrl_enable_gpioc(p.sysctrl); // 心跳 LED 在 PC13

    let mut led = Output::new(Pin::new(p.gpioc, 13), Level::High);

    // I2C 引脚电气(数字模式/输出方向/开漏/上拉)由 `I2c::new_i2c1`
    // 驱动内部处理——2026-10-24 上板实锤:缺 ANALOG 清零或 DIR=输出,
    // 状态机挂死(CR.STA 置位但 START 永不发出,STAT 停 0xF8)。

    let mut i2c = I2c::new_i2c1(
        p.i2_c1,
        Pin::new(p.gpioa, 9),  // SCL,AF3 = PA09_AFx_I2C1SCL()
        Pin::new(p.gpioa, 10), // SDA,AF3 = PA10_AFx_I2C1SDA()
        3,
        Irqs,
        i2c::Config::default(), // 默认 = 标准模式 100kHz
    );

    loop {
        // 一轮扫描:对每个地址发 START+ADDR+W,ACK=命中 / NACK=跳过
        // (经典家族状态码:0x18/0x28=ACK 命中;0x20/0x30/0x48=NACK)
        let mut hits = [0u8; MAX_HITS];
        let mut n = 0usize;
        let mut miss = [0u8; 3];
        for addr in SCAN_RANGE {
            match i2c.write(addr, &[]).await {
                Ok(()) => {
                    if n < MAX_HITS {
                        hits[n] = addr;
                    }
                    n += 1;
                }
                Err(i2c::I2cError::Nack) => miss[0] = miss[0].saturating_add(1),
                Err(i2c::I2cError::Arbitration) => miss[2] = miss[2].saturating_add(1),
                Err(_) => miss[1] = miss[1].saturating_add(1),
            }
        }
        record_round(&hits[..n.min(MAX_HITS)], miss);

        if n > 0 {
            // 总线上有器件:常亮间隔闪(极性未实测,亮灭占比 9:1 双态可辨)
            led.set_low();
            Timer::after_millis(900).await;
            led.set_high();
            Timer::after_millis(100).await;
        } else {
            // 总线空:慢闪(1s 周期翻转)
            led.toggle();
            Timer::after_millis(500).await;
        }
    }
}

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {
        core::hint::spin_loop();
    }
}
