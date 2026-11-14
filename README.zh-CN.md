# embassy-cw32

[![license](https://img.shields.io/badge/license-MIT%20OR%20Apache--2.0-blue.svg)](#许可)

[English](README.md) | **简体中文**

武汉芯源半导体（CMSemicon）**CW32 系列**（Cortex-M0+）的 embassy 异步 HAL——**11 个家族**，
由互斥的 Cargo feature 选择：`cw32l012`（默认）、`cw32l010`、`cw32l011`、`cw32f030`
（CW32F030/A030）、`cw32f020`、`cw32f003`、`cw32l031`、`cw32l052`、`cw32l083`、
`cw32r031`、`cw32w031`。

两代硬件：**L012 代组**（`cw32l010`/`cw32l011`/`cw32l012`）的 SYSCTRL 门控写入要带
0x5A5A 键；**经典代组**（其余 8 家族）无键（直接读改写）。外设寄存器面也随代不同
（UART 错误位、门控布局，以及 I2C/ADC 这类整套不同的 IP）。

## 生态系统

- [cw32l012-pac](https://github.com/SunWithCloude/Cw32l012_pac)——默认家族的 PAC，
  寄存器数据转录自《CW32L012 用户手册》V1.4，逐项核对。
- [cw32-pacs](https://github.com/SunWithCloude/cw32-pacs)——其余 10 家族的 PAC，由厂商
  SDK 头文件生成（同一份生成器、同一种产物风格）。
- [cw32-bsp](https://github.com/SunWithCloude/cw32-bsp)——全部 11 家族的通用 BSP：
  引脚映射表 + `blink` / `uart_echo` 工程模板。

## 提供内容

| 模块 | 内容 |
|------|------|
| `time_driver` | SysTick 时间驱动（embassy-time-driver 0.2 接口）。1MHz tick、动态单次闹钟（`RVR` 重编程），**空闲时无周期性唤醒**，低功耗友好 |
| `bind_interrupts!` | embassy 风格编译期中断绑定（绑错中断直接编译失败）；驱动构造时自动使能对应 NVIC 线 |
| `gpio` | 异步 `Input`（`wait_for_rising_edge` / `wait_for_falling_edge` / `wait_for_any_edge` / `wait_for_high` / `wait_for_low`）+ `Output`（含 `embedded-hal` 1.0 trait），全家族；每端口一条共享中断按位唤醒 per-pin waker（l052/l083 的共享向量 `GPIOC_GPIOD` / `GPIOE_GPIOF` 成对派发端口对） |
| `uart` | 异步 UART（`embedded-io-async`），全家族：逐字节 `Uart` + **`BufferedUart`**（后台环形缓冲收发，embassy-sync `Pipe`，收/发各 64 字节，多任务并发读写，`flush` 等 TC）。L012/F030 有 UART1/2/3，**l083 有 UART1..6**（共享向量 UART1/4、UART2/5、UART3/6 成对派发），其余家族单 UART1。L012 另有 `write_dma`/`read_dma`（语义级实现，待实板验证）。设计吸收自 cw32-hal2，见 `THIRD_PARTY.md` |
| `spi` | 异步 SPI 主机（`SpiBus`，软件 CS），全家族：L012 SPI1/2/3、F030 SPI1/2、**l083 SPI1/2**，其余单实例；两代同守 ICR R1W0 纪律。L012 另有 `transfer_dma`/`write_dma`（语义级实现，待实板验证） |
| `i2c` | 异步 I2C 主机（`embedded-hal-async::i2c::I2c`），11 家族中覆盖 10 个（L011 无 I2C 硬件）：L012 保留命令 FIFO 式控制器；其余 9 家族为经典 8051 风格状态机控制器，事务单次 START、操作间发重复起始（F030 上板验证，其余经典家族为逐 PAC 位面比对的编译级接入） |
| `timer` | （L012 代组：`cw32l012`/`cw32l010`/`cw32l011`）GTIM 单通道边沿对齐 PWM（`SetDutyCycle`，万分比 0..=10000 满量程），`wait_update()` 异步等更新事件；纯 PWM 无需绑中断。实例数：L012 = GTIM1-4、l011 = GTIM1/2、l010 = GTIM1 |
| `atim` | （L012 代组：`cw32l012`/`cw32l010`/`cw32l011`）ATIM 高级定时器互补 PWM（手册 §16，三相电机 PWM 路线）：CH1-3 主/互补输出对（死区插入）+ CH4 单端 + BK 刹车输入。死区时钟 = PCLK（CKD 固定 /1），DTG 四段编码**向上取整**（绝不取不足值，上限 1008 tick），刹车事件硬件异步清 MOE、刹车源活跃期间 BIF 清不掉。l010 **无外部刹车引脚路线**（`bk` 必须传 `None`；只能内部 VC 源）。`AtimPwm::new` / `set_duty_bp` / `enable`-`disable` / `is_braked`-`clear_break` / `wait_update`。⚠ compiles-not-run：实机验证待板（l010/l011 另无阻塞版同源背书），上板前勿驱动真电机 |
| `atim_classic` | （经典代组：`cw32f030`/`cw32l052`/`cw32l083`）另一套 ATIM IP——三相互补 PWM + 硬件死区：CH1-3 A/B 对（整字寄存器配方 `COMP=1` + `PWM2S=1` + `MODE=0b10` 边沿对齐，`BUFPEN`/`BUFEA` 影子双缓冲），死区走 `DTR` 四段编码（2..1010 TCLK tick，请求**向上取整**，只收内部占空比——端点拒绝），MOE 在 `DTR`（默认关输出，须显式 `enable()`），`ICR` 写 0 清除。三家族 PAC 寄存器面逐位核对一致；死区公式/ICR 语义/MODE 与 PRS 编码逐家族对照各家族手册核验（页码见模块文档）。引脚路线 SDK 宏 + 数据手册 AF 表双源核对（l083 的 PE9 单源孤证拒收）。`AtimPwm::new` / `set_duty_bp` / `enable`-`disable` / `is_braked`-`clear_break` / `wait_update`。⚠ compiles-not-run：实机验证待板，上板前勿驱动真电机 |
| `adc` | 一个模块、两套 IP：L012 ADC1/2 异步单次/序列转换，含内部通道（经 1.2V 基准的 `vdda_mv()`、`temperature_mc()`）与 `read_sequence_dma()`；经典组（F030/F020，完全不同的 IP）独立实现——异步单次/4 结果序列，内部通道 VDDA/3 + 温度 + 1.2V 基准，软件触发路径 |
| `rtc` | （L012）BCD 日历读写（`now`/`set_datetime`）+ 异步闹钟（`wait_alarm`，KEY 解锁序列包进临界区），LSE/LSI 时基 |
| `wdt` | （L012）IWDT 独立看门狗 + WWDGT 窗口看门狗，非异步薄封装，带喂狗窗口求解器（`WindowConfig::for_feed_interval`） |
| `dac` | （L012）双通道 12 位输出（`set_ch1`/`set_ch2`/`set_both` + 回读），DMA 波形输出——单发 `write_dma` 与 `start_dma_loop`/`stop_dma` 环形循环（GTIM TRGO 供拍）；含 DAC→ADC 片内回环示例 |
| `dma` | 两代 IP：L012 `Dma`——4 通道（IRQ9 `DMA12` = 通道 1/2、IRQ10 `DMA34` = 通道 3/4），`mem_to_mem`/`periph_to_mem`/`mem_to_periph`；经典组 `ClassicDma`（F020/F030/L031/L052/L083/R031/W031）——F030/F020/L083 5 通道、L031 组 4 通道，**F003/L010/L011 无 DMA 硬件**（F020 通道 4/5 无 NVIC 线，仅轮询）。两代各配硬件触发源表（`pac::dma::hs`） |
| `critical-section-impl` | PRIMASK 实现（默认开启；全固件唯一——家族 PAC 自身不带实现） |
| `defmt` | 可选 feature：驱动错误/配置类型（30+）derive `defmt::Format`；`rtt_hello` 示例打通 defmt-rtt 端到端日志 |

`examples/` 下有 39 个可运行示例（blink、UART echo/缓冲/DMA、SPI 回环/DMA、I2C 扫描、
ADC 异步/DMA/F030、DAC 波形与 DAC→ADC 回环、RTC 闹钟、看门狗、两代 DMA mem2mem、
定时器 PWM、L012 代与经典代 ATIM 三相互补电机 PWM、RTT hello），各自按家族 feature
门控（见 `Cargo.toml`）。

## 快速开始

全部家族 PAC 来自 crates.io——**不需要克隆任何 sibling 仓库**：

```toml
[dependencies]
embassy-cw32 = { version = "0.4", features = ["rt"] }
embassy-executor = { version = "0.10", features = ["platform-cortex-m", "executor-thread"] }
embassy-time = "0.5"
```

三个家族 crate 带 `-um` 后缀（用户手册转录版）——因为原名在 crates.io 被
第三方占用：`cw32f030-pac-um`、`cw32l010-pac-um`、`cw32l012-pac-um`。其余
八个用原名（`cw32f003-pac`、`cw32f020-pac`、`cw32l011-pac`、`cw32l031-pac`、
`cw32l052-pac`、`cw32l083-pac`、`cw32r031-pac`、`cw32w031-pac`）。

开发本仓时用 `[patch.crates-io]` 把 PAC 依赖指回本地检出即可，CHANGELOG
0.4.0 条目有说明。

```rust,no_run
#![no_std]
#![no_main]

use embassy_executor::main;
use embassy_time::Timer;
use embassy_cw32::{gpio::{Level, Output, Pin}, sysctrl_enable_gpioc, Config, Peripherals};

#[main]
async fn main(_spawner: embassy_executor::Spawner) {
    embassy_cw32::init(Config::default()); // 复位默认：L012 代组 4MHz / 经典代组 8MHz
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

本 crate 不做时钟树配置。`Config { hclk_hz, pclk_hz }` 只是**声明**当前频率，
用于 SysTick 计时与外设波特率换算；`Config::default()` 按复位默认取值——
**L012 代组 4MHz**（HSI 96M/24），**经典代组 8MHz**（HSI 48M/6）。需要变频的
固件请先用寄存器层配好时钟再 `init`，并传入对应频率。

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

- 非 L012 家族的 `timer` / `rtc` / `wdt` / `dac`（逐家族寄存器面；经典组
  ADC 已先行落地 F030/F020）
- L031 组 / F003 / L010 / L011 的 ADC（另一种 IP，尚未接入）
- L012 命令 FIFO 式 I2C 的 repeated-start（经典控制器已支持单 START 事务）
- 经典代的 UART/SPI DMA（L012 已有，语义级实现）
- F030 `rcc` 之外的时钟树运行时配置（async RCC API）
- `Output` 的 `wait_for_settle` 类高级 API、GPIO 低功耗唤醒配置

## 与 cw32l012-hal 的关系

两套并行方案，按需选用：

| | cw32l012-hal | embassy-cw32 |
|---|---|---|
| 编程模型 | 阻塞式 + 自包含运行时 | async / embassy |
| 依赖 | 零依赖 | embassy 生态 |
| 向量表 | 自带（符号 `UART1_irq` 等） | cortex-m-rt + PAC `rt`（符号 `UART1` 等） |

⚠ **不要在一个固件里启用两个 critical-section 实现**——例如
`cw32l012-hal/critical-section-impl` 与本 crate 默认实现，链接会冲突。

## 验证状态

- `thumbv6m-none-eabi`：lib 与示例在 CI 中零警告构建（11 家族互斥矩阵 +
  defmt-on 两类 job）；链接产物向量表经 `nm`/`objdump` 核验
  （`__INTERRUPTS@0x40`、SysTick → 时间驱动、UART1 → 绑定处理器、
  未绑定项 → DefaultHandler）；
- `cargo test`（宿主机）通过（布局/文档测试）；
- **blink_async 已于 2026-10-24 在 CW32L012 实板验证**（probe-rs +
  ST-Link V2）：PC13 经 `Timer::after_millis(500)` 精确 1 秒周期翻转
  （ODR 寄存器轮询确认），SysTick 闹钟周期 1,997,499 ≈ 2,000,000
  systicks（500ms @ 4MHz HCLK）——异步执行器、时间驱动、任务唤醒
  全链路工作正常。期间发现并修复了 set_alarm 基数回退错误（见上）；
- uart_echo_async 已烧录验证运行（无硬件故障），但异步串口数据回环
  待接 USB 串口适配器后实测；
- **buffered_echo 已于 2026-10-24 上板验证**：心跳 LED 证明执行器 +
  双任务并发（echo 任务停驻管道读）；`ISER bit27=1` 证明 v0.2 的
  NVIC 使能修复生效；经调试口向 `ISPR` 注入 UART1 挂起后 `ICPR`
  自动清零、心跳不停——向量 → `bind_interrupts!` → `buffered_irq` →
  临界区 的完整中断管线实机跑通。缓冲数据的实际收发仍需串口适配器。

## F030 家族备注

- `--no-default-features --features "rt,cw32f030"` 构建
  CW32F030/A030（生成版 `cw32f030-pac`，cw32-pac-gen 产物）。
- 复位默认时钟 **8MHz**（HSI 48M/6）——`Config::default()` 已处理。
- 本代 SYSCTRL 的 AHBEN/APBEN **无 KEY 保护**（不同于 L012 的 0x5A5A
  键控写），门控走读改写；CR0/CR1/IER 则带 KEY。`rcc` 模块（PLL 配方
  转录自厂商库）处理键控写，切频后用 `set_frequencies` 重新声明时钟。
- 经典 I2C 是另一套外设（8051 风格状态机），已有独立驱动；
  `i2c_scan_f030` 上板验证（空总线 112/112 地址 NACK，寄存器链逐项核对）。
- LCKFB CW32F030C8T7 实板验证：blink_async_f030（500ms 心跳）、UART1
  初始化链（PA08/PA09 AF2，BRRI=4 @115200）、SPI1（CR1=0x1E7C，
  AFRL=0x55500000）。

## 相关工作

[fan2nd/cw32-hal2](https://github.com/fan2nd/cw32-hal2)（2026-10，早于本
crate 出现）走全家桶路线：chiptool/metapac 架构覆盖 CW32F/L/A 多家族，
驱动面广（UART DMA/LIN、SPI PIO、深度定时器），时间驱动用 GTIM 硬件
比较。本 crate 定位不同：逐家族 PAC（L012 手写核对版对 UM V1.4 逐项
核对，其余家族由厂商 SDK 头文件生成），驱动按各家手册核对，有板子的
地方上板验证。吸收了哪些内容、依据什么许可，见 `THIRD_PARTY.md`。

## 许可

MIT OR Apache-2.0（与 cw32l012-pac / cw32l012-hal 相同）。
