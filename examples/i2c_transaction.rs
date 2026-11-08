//! CW32L012 异步 I2C 单事务示例：写寄存器指针 + 重复起始读。
//!
//! I2C1 主机:PA04=SCL / PA05=SDA(厂商宏 `PA04_AFx_I2C1SCL()`
//! / `PA05_AFx_I2C1SDA()`,均 AF2),标准模式 100kHz。总线上需外接
//! 4.7kΩ 上拉到 VDD,并与板卡共地。
//!
//! 演示 `embedded-hal-async::i2c::I2c::transaction` 的单事务语义:
//! 整个 operations 序列一次 START,`Write`→`Read` 方向切换处硬件自动
//! 发重复 START(repeated START),末操作后 STOP——即"写寄存器指针 +
//! 读数据"类器件(LM75/TMP102、EEPROM 随机读、绝大多数数字传感器)
//! 的通用随机读模式,总线上不出现中间 STOP:
//!
//! ```text
//! Master: ST SAD+W  0x00  SR SAD+R      MAK  NMAK SP
//! Slave:         SAK      SAK  T1 MSB    T1 LSB
//! ```
//!
//! 目标器件:LM75 系列温度传感器(地址 0x48,指针寄存器 0x00=温度,
//! 2 字节大端)。无板上验证,仅编译通过——任何"写指针+重复起始读"
//! 器件同款适配(改地址/指针/长度即可)。
//!
//! 结果记录:16 位温度原始值写入静态数组(宿主可经 probe-rs 读内存
//! 核对,无需串口);温度换算 LM75 口径 0.125°C/LSB(9 位有效)。
//!
//! LED(PC13)目视判定:
//! - 事务成功:100ms 亮 / 900ms 灭(短促闪)
//! - NACK(无器件):500ms 翻转(慢闪,1s 周期)
//! - 其它错误:常亮
//!
//! cargo build --target thumbv6m-none-eabi --release --no-default-features \
//!     --features "rt,cw32l012,critical-section-impl" --example i2c_transaction

#![no_std]
#![no_main]

use core::cell::RefCell;

use critical_section::Mutex;
use embassy_executor::main;
use embassy_time::Timer;
use embedded_hal_async::i2c::{I2c as _, Operation};

use embassy_cw32::{
    Config, Peripherals,
    gpio::{Level, Output, Pin},
    i2c::{self, I2c},
    sysctrl_enable_gpioa, sysctrl_enable_gpioc,
};

embassy_cw32::bind_interrupts!(struct Irqs {
    I2C1 => i2c::I2c1Irq;
});

/// LM75 系列温度传感器 7 位地址(A0=A1=A2=接地)
const LM75_ADDR: u8 = 0x48;
/// 指针寄存器:0x00 = 温度(16 位大端,0.125°C/LSB)
const LM75_TEMP_PTR: u8 = 0x00;

/// 最近一次事务读到的温度原始值(`[0]`=状态,0=成功;`[1..3]`=大端
/// 原始值。`#[used]`:固件只写不读,防止链接器把符号裁掉——宿主经
/// probe-rs 读内存用)。
#[used]
static RESULT: Mutex<RefCell<[u8; 3]>> = Mutex::new(RefCell::new([0xFF; 3]));

#[main]
async fn main(_spawner: embassy_executor::Spawner) {
    embassy_cw32::init(Config::default()); // 复位默认 HCLK=PCLK=4MHz

    let p = Peripherals::take().unwrap();
    sysctrl_enable_gpioa(p.sysctrl);
    sysctrl_enable_gpioc(p.sysctrl); // 心跳 LED 在 PC13

    let mut led = Output::new(Pin::new(p.gpioc, 13), Level::High);

    // 开漏复用电气由驱动内部 MCR2.PINCFG=OD_IO 处理
    let mut i2c = I2c::new_i2c1(
        p.i2c1,
        Pin::new(p.gpioa, 4), // SCL,AF2 = PA04_AFx_I2C1SCL()
        Pin::new(p.gpioa, 5), // SDA,AF2 = PA05_AFx_I2C1SDA()
        2,
        Irqs,
        i2c::Config::default(), // 默认 = 标准模式 100kHz
    );

    // 写寄存器指针 + 重复起始读:单事务,中间无 STOP
    let mut buf = [0u8; 2];
    let res = i2c
        .transaction(
            LM75_ADDR,
            &mut [
                Operation::Write(&[LM75_TEMP_PTR]),
                Operation::Read(&mut buf),
            ],
        )
        .await;

    critical_section::with(|cs| {
        let mut slot = RESULT.borrow_ref_mut(cs);
        match res {
            Ok(()) => {
                slot[0] = 0;
                slot[1] = buf[0];
                slot[2] = buf[1];
            }
            Err(_e) => {
                slot[0] = 1; // 事务失败(空总线 = NACK)
            }
        }
    });

    loop {
        match res {
            Ok(()) => {
                // 成功:100ms 亮 / 900ms 灭
                led.set_low();
                Timer::after_millis(100).await;
                led.set_high();
                Timer::after_millis(900).await;
            }
            Err(i2c::I2cError::Nack) => {
                // 无器件:慢闪
                led.toggle();
                Timer::after_millis(500).await;
            }
            Err(_) => {
                // 其它错误:常亮
                led.set_low();
                Timer::after_millis(1000).await;
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
