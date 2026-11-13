//! FLASH 自编程示例（L012，零接线）：掉电保持的 u64 启动计数器。
//!
//! 在第 127 页（`0x0000_FE00`，页大小 512B，FLASH 末页）存一条 20 字节
//! 记录（5 个字，4 字节对齐 → 纯字编程路径）：
//!
//! ```text
//! [0..4)   magic  u32  "FLS1"（0x3153_4C46，LE）
//! [4..12)  count  u64  LE  ← 掉电累加的计数器本体
//! [12..20) check  u64  = !count（反码校验）
//! ```
//!
//! 每次复位：读旧记录 → 校验 magic + 反码 → 有效则 count+1，否则从 1
//! 重计 → 擦页 → 编程 → 回读验证。**复位/掉电不清零**——最朴素的
//! 参数存储 / NVM 用法（阻塞版 cw32l012-hal 同名例程的结构移植）。
//!
//! 掉电安全语义（如实声明）：
//! - **编程中途掉电**：反码校验失败 → 记录判无效，计数回到 1（检测得到，
//!   不回写半截数据）；
//! - **擦除后、编程前掉电**：整页已 0xFF，旧记录丢失（单页单记录方案的
//!   固有窗口；要消除须 A/B 双页+序号提交，本示例不展开）。
//!
//! 观察量（SRAM 静态量，`probe-rs read` 观测，地址以 nm 输出为准）：
//!
//! - `MAGIC`：读出的旧 magic（首次上电应为 0xFFFF_FFFF）
//! - `OLD_COUNT_LO/HI`：旧计数（u64 拆两个 u32 读——u64 须 8 字节对齐，
//!   记录里它落在偏移 4，只能拆读）
//! - `NEW_COUNT_LO/HI`：写回的新计数
//! - `ERASED`：擦除后回读页首（应为 0xFFFF_FFFF，验证"确实变 0xFF"）
//! - `RB_OK`：回读校验（magic/count/反码全对 = 1）
//! - `SECURITY`：读保护等级（0=Level0）
//! - `ETEXT`：代码段末（cortex-m-rt `__etext`，页 127 远在其后——擦除
//!   不会碰代码区的直观证据）
//! - `ERR`：0=无、1=erase 失败、2=program 失败、3=回读校验失败
//!
//! cargo build --target thumbv6m-none-eabi --release --features rt --example flash_store

#![no_std]
#![no_main]

use embassy_executor::{Spawner, main};
use embassy_time::Timer;

use embassy_cw32::{
    Config, Peripherals,
    flash::{Flash, PAGE_SIZE},
    gpio::{Level, Output, Pin},
};

/// 记录所在页（FLASH 末页，0x0000_FE00..0x0001_0000）
const PAGE: u32 = 127;
/// 记录地址
const ADDR: u32 = PAGE * PAGE_SIZE;
/// 魔数 "FLS1"（embassy 版 flash_store；阻塞版用 "FLHC"，换魔数避免两版
/// 固件互相把对方的记录当有效）
const MAGIC: u32 = 0x3153_4C46;

// cortex-m-rt link.x 的代码段末符号（观测用；库驱动不引用链接脚本符号）
unsafe extern "C" {
    static __etext: u8;
}

/// 读出的旧 magic
#[used]
#[unsafe(no_mangle)]
pub static mut OLD_MAGIC: u32 = 0;
/// 旧计数低 32 位
#[used]
#[unsafe(no_mangle)]
pub static mut OLD_COUNT_LO: u32 = 0;
/// 旧计数高 32 位
#[used]
#[unsafe(no_mangle)]
pub static mut OLD_COUNT_HI: u32 = 0;
/// 新计数低 32 位
#[used]
#[unsafe(no_mangle)]
pub static mut NEW_COUNT_LO: u32 = 0;
/// 新计数高 32 位
#[used]
#[unsafe(no_mangle)]
pub static mut NEW_COUNT_HI: u32 = 0;
/// 擦除后回读页首（应为 0xFFFF_FFFF）
#[used]
#[unsafe(no_mangle)]
pub static mut ERASED: u32 = 0;
/// 回读校验结果：1=通过
#[used]
#[unsafe(no_mangle)]
pub static mut RB_OK: u32 = 0;
/// 读保护等级
#[used]
#[unsafe(no_mangle)]
pub static mut SECURITY: u32 = 0;
/// 代码段末（`__etext`）
#[used]
#[unsafe(no_mangle)]
pub static mut ETEXT: u32 = 0;
/// 错误码：0=无 1=erase 2=program 3=回读校验
#[used]
#[unsafe(no_mangle)]
pub static mut ERR: u32 = 0;

/// 读记录里的 u64 字段（偏移 4 起非 8 字节对齐，M0+ 拆两个 u32 读）
fn read_u64_split(addr: u32) -> Result<u64, embassy_cw32::flash::FlashError> {
    let lo = Flash::read_u32(addr)? as u64;
    let hi = Flash::read_u32(addr + 4)? as u64;
    Ok(lo | (hi << 32))
}

#[main]
async fn main(_spawner: Spawner) {
    embassy_cw32::init(Config::default()); // 复位默认 SysClk = 4MHz

    let p = Peripherals::take().unwrap();
    embassy_cw32::sysctrl_enable_gpioc(p.sysctrl);
    let mut led = Output::new(Pin::new(p.gpioc, 13), Level::High);

    // ---- 0. 控制器 + 观测面 ----
    let mut nvm = Flash::new(p.flashctrl); // 自动开 AHBEN.FLASH 配置时钟
    // SAFETY: 裸机单例，仅此一处引用
    unsafe {
        ETEXT = core::ptr::addr_of!(__etext) as u32;
        SECURITY = nvm.security_level() as u32;
    }

    // ---- 1. 读旧记录（首次上电全 0xFF → 判无效 → 从 1 重计）----
    let old_magic = Flash::read_u32(ADDR).unwrap_or(0);
    let old_count = read_u64_split(ADDR + 4).unwrap_or(u64::MAX);
    let old_check = read_u64_split(ADDR + 12).unwrap_or(0);
    unsafe {
        OLD_MAGIC = old_magic;
        OLD_COUNT_LO = old_count as u32;
        OLD_COUNT_HI = (old_count >> 32) as u32;
    }
    let valid = old_magic == MAGIC && old_check == !old_count;
    let count = if valid && old_count < 0xFFFF_0000_0000_0000 {
        old_count + 1
    } else {
        1
    };
    unsafe {
        NEW_COUNT_LO = count as u32;
        NEW_COUNT_HI = (count >> 32) as u32;
    }

    // ---- 2. 组记录：magic + count + !count（20 字节 = 5 字，纯字路径）----
    let mut buf = [0u8; 20];
    buf[0..4].copy_from_slice(&MAGIC.to_le_bytes());
    buf[4..12].copy_from_slice(&count.to_le_bytes());
    buf[12..20].copy_from_slice(&(!count).to_le_bytes());

    // ---- 3~4. 擦除 → 编程（页擦典型 3~5ms 忙等，当前任务阻塞）----
    // SAFETY: 第 127 页（0xFE00）在代码段之外（ETEXT 观测量可查），且
    // 本固件独占该页——不存常量、无他任务读它
    let mut err = 0u32;
    let erased = unsafe {
        match nvm.erase_page(PAGE) {
            Ok(()) => Flash::read_u32(ADDR).unwrap_or(0),
            Err(_) => {
                err = 1;
                0
            }
        }
    };
    unsafe { ERASED = erased };
    if err == 0 && unsafe { nvm.program_slice(ADDR, &buf) }.is_err() {
        err = 2;
    }

    // ---- 5. 回读校验 ----
    let rb_magic = Flash::read_u32(ADDR).unwrap_or(0);
    let rb_count = read_u64_split(ADDR + 4).unwrap_or(0);
    let rb_check = read_u64_split(ADDR + 12).unwrap_or(0);
    let ok = err == 0 && rb_magic == MAGIC && rb_count == count && rb_check == !count;
    unsafe { RB_OK = ok as u32 };
    if !ok && err == 0 {
        err = 3;
    }
    unsafe { ERR = err };

    // ---- 6. 反馈：成功闪 3 下（0xA0），失败常亮；之后慢闪待命 ----
    if ok {
        for _ in 0..3 {
            led.set_low();
            Timer::after_millis(120).await;
            led.set_high();
            Timer::after_millis(120).await;
        }
    } else {
        led.set_low(); // 失败常亮
    }
    loop {
        Timer::after_secs(1).await;
        led.toggle();
    }
}

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {
        core::hint::spin_loop();
    }
}
