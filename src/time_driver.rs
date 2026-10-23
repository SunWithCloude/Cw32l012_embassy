//! SysTick 时间驱动（embassy-time-driver 实现）。
//!
//! Cortex-M0+ 的 SysTick 是 24 位倒计数器，没有比较寄存器，本驱动用
//! "动态单次闹钟"方式实现精确调度：
//!
//! - `now()`：64 位软件扩展。COUNTFLAG 读清语义的折算统一在本模块
//!   `now_systicks` / SysTick IRQ 之间互斥完成（都在临界区内），双读
//!   校验消除"wrap 落在标志读与 CVR 读之间"的窗口。
//! - `schedule_wake()`：队列头变化时把 SysTick 重编为一次性闹钟
//!   （`RVR = 剩余 systicks`，写 `CVR = 0` 重启计数；写 CVR 只清
//!   COUNTFLAG，不触发中断）。长限期退化为最大周期，wrap 中断里
//!   重新检查队列。
//!
//! tick 频率固定 1MHz（embassy-time 默认）。时间基准是 HCLK
//! （见 [`crate::Config`]）。空闲时无任何周期性唤醒，对低功耗友好。

use core::cell::{Cell, RefCell};
use core::sync::atomic::{AtomicU32, Ordering};
use core::task::Waker;

use critical_section::Mutex;
use embassy_time_driver::Driver;
use embassy_time_queue_utils::Queue;

const SYST_CSR: *mut u32 = 0xE000_E010 as *mut u32;
const SYST_RVR: *mut u32 = 0xE000_E014 as *mut u32;
const SYST_CVR: *mut u32 = 0xE000_E018 as *mut u32;

const CSR_ENABLE: u32 = 1 << 0;
const CSR_TICKINT: u32 = 1 << 1;
const CSR_CLKSOURCE: u32 = 1 << 2; // 1 = HCLK
const CSR_COUNTFLAG: u32 = 1 << 16;

/// 24 位计数上限（4MHz 下约 4.19s，48MHz 下约 0.35s）
const MAX_PERIOD: u64 = 1 << 24;
/// 最小编程周期（systicks）：必须显著长于中断处理耗时，防止处理期间
/// 再次 wrap 丢计数。到期时间更近的闹钟最多晚这么多个 systick 触发。
const MIN_PERIOD: u64 = 100;

/// 时间基数（systicks，wrapping 语义；重编程时会回退接近一个周期，
/// 起步阶段可能"负"，wrapping 运算保证一致性）。
/// thumbv6m 无 64 位原子，全部访问都在临界区内，用 Cell 即可。
static ABS: Mutex<Cell<u64>> = Mutex::new(Cell::new(0));
/// 当前编程的周期（systicks）
static PERIOD: AtomicU32 = AtomicU32::new(MAX_PERIOD as u32);

/// 单调钳制用的"最近一次读数"（systicks；仅临界区内访问）
static LAST_NOW: Mutex<Cell<u64>> = Mutex::new(Cell::new(0));

struct TimeDriver {
    queue: Mutex<RefCell<Queue>>,
}

embassy_time_driver::time_driver_impl!(static DRIVER: TimeDriver = TimeDriver {
    queue: Mutex::new(RefCell::new(Queue::new())),
});

impl Driver for TimeDriver {
    fn now(&self) -> u64 {
        now_us()
    }

    fn schedule_wake(&self, at: u64, waker: &Waker) {
        critical_section::with(|cs| {
            let mut queue = DRIVER.queue.borrow(cs).borrow_mut();
            if queue.schedule_wake(at, waker) {
                let mut next = queue.next_expiration(now_us());
                while !set_alarm(next) {
                    next = queue.next_expiration(now_us());
                }
            }
        })
    }
}

/// SysTick 初始化：最大周期自由运行 + 开中断。
pub(crate) fn init() {
    critical_section::with(|_| {
        // 先关再配（EN=0 时 RVR/CVR 可安全写入）
        unsafe { SYST_CSR.write_volatile(0) };
        critical_section::with(|cs| {
            ABS.borrow(cs).set(0);
            LAST_NOW.borrow(cs).set(0);
        });
        unsafe { SYST_RVR.write_volatile((MAX_PERIOD - 1) as u32) };
        unsafe { SYST_CVR.write_volatile(0) };
        unsafe { SYST_CSR.write_volatile(CSR_ENABLE | CSR_TICKINT | CSR_CLKSOURCE) };
    });
}

/// 当前时刻（embassy-time ticks，1MHz）。
fn now_us() -> u64 {
    let hz = crate::HCLK_HZ.load(Ordering::Relaxed) as u64;
    now_systicks().wrapping_mul(1_000_000) / hz.max(1)
}

/// 读清 COUNTFLAG；返回置位前的值。
#[inline]
fn flag_read_clear() -> bool {
    unsafe { SYST_CSR.read_volatile() & CSR_COUNTFLAG != 0 }
}

/// 折算一个已完成的周期到基数（仅在临界区内调用）。
#[inline]
fn fold_period() {
    let p = PERIOD.load(Ordering::Relaxed) as u64;
    critical_section::with(|cs| {
        let cell = ABS.borrow(cs);
        cell.set(cell.get().wrapping_add(p));
    });
}

/// 读时间基数（仅在临界区内调用）。
#[inline]
fn abs_get() -> u64 {
    critical_section::with(|cs| ABS.borrow(cs).get())
}

/// 写时间基数（仅在临界区内调用）。
#[inline]
fn abs_set(v: u64) {
    critical_section::with(|cs| ABS.borrow(cs).set(v));
}

/// 当前时刻（systicks，64 位扩展）。仅在临界区内调用。
///
/// 双读 COUNTFLAG 消除折算窗口：若第二次读仍见到 wrap（说明发生在
/// 标志读与 CVR 读之间），折算后重读——wrap 间隔 ≥ MIN_PERIOD，
/// 循环至多两轮。
///
/// 单调钳制：重编程写 CVR=0 后的第一拍（计数器尚未重载到 RVR）读出
/// 会超前约一个周期，用 LAST_NOW 压回，保证 now() 永不回退。
fn now_systicks() -> u64 {
    critical_section::with(|cs| {
        loop {
            if flag_read_clear() {
                fold_period();
            }
            let cvr = (unsafe { SYST_CVR.read_volatile() } & 0x00FF_FFFF) as u64;
            if flag_read_clear() {
                fold_period();
                continue;
            }
            let period = PERIOD.load(Ordering::Relaxed) as u64;
            let v = abs_get().wrapping_add(period - 1).wrapping_sub(cvr);
            let last = LAST_NOW.borrow(cs);
            return if v > last.get() {
                last.set(v);
                v
            } else {
                last.get()
            };
        }
    })
}

/// 把 SysTick 编程为一次性闹钟（`at` 为 embassy-time ticks）。
///
/// 返回 `false` 表示 `at` 已过期（调用方重取队列下一个到期项，过期项
/// 会被 `next_expiration` 直接唤醒）。超过 24 位的长限期编程为最大
/// 周期，到期中断里重新检查队列，不丢失精度。
fn set_alarm(at: u64) -> bool {
    // 已在临界区内（schedule_wake / SysTick IRQ 持有；临界区可重入）
    let hz = crate::HCLK_HZ.load(Ordering::Relaxed) as u64;
    let now = now_systicks();
    let at_ticks = at.wrapping_mul(hz) / 1_000_000;
    let diff = at_ticks.wrapping_sub(now);
    if diff == 0 || diff >= 1 << 63 {
        return false; // 已过期（回绕差值表现为极大数）
    }
    let period = diff.clamp(MIN_PERIOD, MAX_PERIOD);

    // 重编程：RVR=period-1、写 CVR=0 重启计数（写 CVR 只清 COUNTFLAG，
    // 不触发中断；计数器下一拍重载到 RVR=period-1 开始倒数）。
    // 基数必须存"现在"本身：此后 now = ABS + (period-1) - cvr 随 cvr
    // 从 period-1 递减到 0 线性推进，到点时 now ≈ ABS + period-1。
    // （写 CVR 后那一拍 cvr 仍为 0、读数会短暂超前 period-1，由
    // now_systicks 的单调钳制压回。）
    abs_set(now);
    PERIOD.store(period as u32, Ordering::Relaxed);
    unsafe { SYST_RVR.write_volatile((period - 1) as u32) };
    unsafe { SYST_CVR.write_volatile(0) };
    unsafe { SYST_CSR.write_volatile(CSR_ENABLE | CSR_TICKINT | CSR_CLKSOURCE) };
    true
}

/// SysTick 中断：折算完成的周期（经 `now_systicks` 的读清折算），
/// 派发到期的定时器并续排下一个闹钟。
#[cfg(all(target_arch = "arm", target_os = "none"))]
#[unsafe(no_mangle)]
unsafe extern "C" fn SysTick() {
    critical_section::with(|cs| {
        let _ = now_systicks(); // 读清 COUNTFLAG 并折算本次 wrap
        let mut queue = DRIVER.queue.borrow(cs).borrow_mut();
        let mut next = queue.next_expiration(now_us());
        while !set_alarm(next) {
            next = queue.next_expiration(now_us());
        }
    });
}
