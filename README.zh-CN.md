# embassy-cw32

[![license](https://img.shields.io/badge/license-MIT%20OR%20Apache--2.0-blue.svg)](#许可)

[English](README.md) | **简体中文**

武汉芯源半导体（CMSemicon）**CW32L012**（Cortex-M0+）的 embassy 异步 HAL。

基于 [cw32l012-pac](https://github.com/SunWithCloude/Cw32l012_pac)（寄存器数据转录自
《CW32L012 用户手册》V1.4，与 cw32l012-hal 同源）。

## 提供内容（v0.1）

| 模块 | 内容 |
|------|------|
| `time_driver` | SysTick 时间驱动（embassy-time-driver 0.2 接口）。1MHz tick、动态单次闹钟（`RVR` 重编程），**空闲时无周期性唤醒**，低功耗友好 |
| `bind_interrupts!` | embassy 风格编译期中断绑定（绑错中断直接编译失败） |
| `gpio` | 异步 `Input`（`wait_for_rising_edge` / `wait_for_falling_edge` / `wait_for_any_edge` / `wait_for_high` / `wait_for_low`）+ `Output`（含 `embedded-hal` 1.0 trait） |
| `uart` | 异步 UART1/2/3（中断驱动逐字节，`embedded-io-async` 的 `Read`/`Write`，可 `split()` 收发半部） |
| `i2c` | 异步 I2C1/2 主机（`embedded-hal-async::i2c::I2c`，命令 FIFO 式外设） |
| `spi` | 异步 SPI1/2/3 主机（`SpiBus`，软件 CS） |
| `critical-section-impl` | PRIMASK 实现（默认开启；全固件唯一） |

## 快速开始

本 crate 依赖同级的 `cw32l012-pac` 目录，**两个仓库要放在一起**：

```
Desktop/
├── cw32l012-pac/
└── embassy-cw32/      ← 本仓库
```

```toml
[dependencies]
embassy-cw32 = { path = "../embassy-cw32", features = ["rt"] }
embassy-executor = { version = "0.10", features = ["platform-cortex-m", "executor-thread"] }
embassy-time = "0.5"
```

```rust,no_run
#![no_std]
#![no_main]

use embassy_executor::main;
use embassy_time::Timer;
use embassy_cw32::{gpio::{Level, Output, Pin}, sysctrl_enable_gpioc, Config, Peripherals};

#[main]
async fn main(_spawner: embassy_executor::Spawner) {
    embassy_cw32::init(Config::default()); // 复位默认 SysClk = 4MHz
    let p = Peripherals::take().unwrap();
    sysctrl_enable_gpioc(p.sysctrl);
    let mut led = Output::new(Pin::new(p.gpioc, 13), Level::High);
    loop {
        Timer::after_millis(500).await;
        led.toggle();
    }
}

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! { loop {} }
```

构建：

```bash
cargo build --target thumbv6m-none-eabi --release --features rt --example blink_async
```

`.cargo/config.toml` 已配置 `-Tlink.x`（自动 INCLUDE 本 crate 的 `memory.x`
与 PAC 生成的 `device.x`）。

## 时钟约定

v0.1 不做时钟树配置。`Config { hclk_hz, pclk_hz }` 只是**声明**当前频率
（复位默认均 4MHz），用于 SysTick 计时与外设波特率换算。需要变频的固件
请先用寄存器层配好时钟再 `init`，并传入对应频率。

## 中断绑定

每个异步外设要求传入 `impl Binding<对应中断>` 证明（编译期检查）：

```rust,ignore
embassy_cw32::bind_interrupts!(struct Irqs {
    UART1 => uart::Uart1Irq;
    GPIOA => gpio::GpioAIrq;
});

let mut u = Uart::new_uart1(p.uart1, tx, rx, 1 /*AF*/, Irqs, Default::default());
```

未绑定的中断经 `device.x` 的 `PROVIDE(sym = DefaultHandler)` 弱别名落到
死循环默认处理。

## 时间驱动实现说明

M0+ 的 SysTick 没有 compare 寄存器。本驱动把 SysTick 编程为**一次性
闹钟**（`RVR = 距下次到期的 systicks`，写 `CVR=0` 重启计数），到点中断
里派发 embassy-time 定时器队列并续排下一个闹钟；超过 24 位的长限期自动
退化为最大周期并在 wrap 中断里复查。64 位扩展的 wrap 折算在临界区内经
COUNTFLAG 双读校验完成，无竞态。

到期时间距现在不足 ~100 systicks 的闹钟最多晚这么久触发（只晚不早，
embassy-time 语义安全）。`now()` 带单调钳制（`LAST_NOW`），消除重编程
写 `CVR=0` 后第一拍的超前读数。

> 实板调试记录（2026-10-24）：初版把重编程基数写成了
> `now - (period-1)`（误以为写 CVR=0 后计数器从 0 起数；实际下一拍
> 重载到 `RVR=period-1` 再倒数），导致 `now()` 在一个周期内徘徊、
> 永远到不了 deadline——LED 常亮不闪。经 probe-rs 读 ODR/SysTick/
> RAM 静态定位后修复为基数存 `now` 本身。

## 尚未支持（路线图）

- DMA（UART/SPI 缓冲收发、ADC、memory-to-memory）
- 异步 ADC / DAC / 定时器 PWM / RTC 闹钟唤醒
- `Output` 的 `wait_for_settle` 类高级 API、GPIO 低功耗唤醒配置
- 时钟树运行时配置（RCC async API）
- I2C repeated-start（v0.1 每个操作独立 START…STOP）
- defmt 支持（feature 门控）

## 与 cw32l012-hal 的关系

两套并行方案，按需选用：

| | cw32l012-hal | embassy-cw32 |
|---|---|---|
| 编程模型 | 阻塞式 + 自包含运行时 | async / embassy |
| 依赖 | 零依赖 | embassy 生态 |
| 向量表 | 自带（符号 `UART1_irq` 等） | cortex-m-rt + PAC `rt`（符号 `UART1` 等） |

⚠ **不要同时启用**两家的 critical-section 实现
（`cw32l012-hal/critical-section-impl` 与本 crate 默认实现），链接会冲突。

## 验证状态

- `thumbv6m-none-eabi`：lib + 两个示例，debug/release/clippy 零警告；
  链接产物向量表经 `nm`/`objdump` 核验（`__INTERRUPTS@0x40`、SysTick →
  时间驱动、UART1 → 绑定处理器、未绑定项 → DefaultHandler）；
- `cargo test`（宿主机）通过（布局/文档测试）；
- **blink_async 已于 2026-10-24 在 CW32L012 实板验证**（probe-rs +
  ST-Link V2）：PC13 经 `Timer::after_millis(500)` 精确 1 秒周期翻转
  （ODR 寄存器轮询确认），SysTick 闹钟周期 1,997,499 ≈ 2,000,000
  systicks（500ms @ 4MHz HCLK）——异步执行器、时间驱动、任务唤醒
  全链路工作正常。期间发现并修复了 set_alarm 基数回退错误（见上）；
- uart_echo_async 已烧录验证运行（无硬件故障），但异步串口数据回环
  待接 USB 串口适配器后实测。

## 许可

MIT OR Apache-2.0（与 cw32l012-pac / cw32l012-hal 相同）。
