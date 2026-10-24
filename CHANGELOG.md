# Changelog

遵循 SemVer；格式参考 Keep a Changelog。

## [0.3.0] - 2026-10-24

### 新增

- **多家族支持(feature 互斥)**:`cw32l012`(默认)与 `cw32f030`(CW32F030/
  CW32A030 寄存器兼容)。`rt` 经弱依赖 feature 透传给所选家族。
- `blink_async_f030` 示例;F030 实板验证(执行器 + 时间驱动 + 异步任务 +
  GPIO,PC13 精确 500ms 翻转)。
- `GpioPort` 功能式端口特征(`rd`/`wr`/`mod_` + `Reg` 枚举):隔离两套
  PAC 的字段访问类型差异;`Pin<P>` 泛型化。F030 三种端口结构体分别建模
  (GPIOA/B 全量、GPIOC 仅 AFRH、GPIOF 仅 AFRL)。
- UART 家族位常量面:TXBE/TC/RC 两代共用;错误位/时钟门控/ICR 清除
  按家族(F030 经典:FE3/PE4、UART1=APBEN2.9 等;ICR 同为 R1W0,依据
  厂商库 `REGBITS_CLR(ICR, flag)`)。
- `nvic_enable::<typelevel>()`:统一走 typelevel 中断号(两套 PAC 的
  枚举变体命名不同)。

### 修复

- F030 复位默认时钟为 **8MHz**(HSI 48M/6),非 48MHz——48M 假设下
  SysTick 慢 6 倍、blink 周期 6 秒;上板排障发现,DFP `Dclock=8000000`
  佐证。
- 生成器(cw32-pac-gen):IRQ 名全族退化 RESERVED 的 dict 键序 bug;
  头文件宏补全 UART2/UART3/GPIOB 实例;位段常量输出。

### 变更

- **I2C 多家族化**（收尾补记）:F030 经典 I2C 是 8051 风格状态机外设
  （9 寄存器、STAT 状态码），与 L012 命令 FIFO 式完全不同——按公共 API
  重写驱动而非位面适配。单 START + 重复起始读（强于 L012 v0.1 的逐操作
  START…STOP）;BRR = fPCLK/(8×fSCL) − 1（厂商库注释唯一出处）;两处上板
  实锤的引脚坑收编进驱动:F030 复位默认 ANALOG=0x9fff 仅 SWD 数字使能、
  DIR 保持输入态时 START 永不发出（现由 `Reg::OpEndrain` 新增访问面 +
  `I2c::new_inner` 统一处理）。`i2c_scan_f030` 扫描示例上板:112/112
  地址 NACK（空总线预期）,寄存器链逐项验证。
- "i2c/spi 暂为 L012 专属"状态就此终结:四大件（GPIO/UART/SPI/I2C）
  双家族全覆盖。

## [0.2.0] - 2026-10-24

### 新增

- `uart::BufferedUart`（+ `BufferedUartTx` / `BufferedUartRx`）：后台
  环形缓冲收发（embassy-sync `Pipe`，收/发各 64 字节），多任务并发
  读写，`flush` 等 TC。**设计吸收自 fan2nd/cw32-hal2**（MIT OR
  Apache-2.0，见 `THIRD_PARTY.md`）：IER 单发屏蔽、R1W0 清标志纪律、
  写 TDR 前清 TC。
- `THIRD_PARTY.md` 上游归属声明。
- 示例 `buffered_echo`（echo 任务 + 心跳 LED 任务并发）。

### 修复

- **v0.1 全部异步驱动未使能 NVIC 中断线**（向量函数有了但 ISER 没
  开，中断根本进不来）。现在 `Uart` / `BufferedUart` 构造、GPIO
  `wait_edge`、`Spi` / `I2c` 构造都会使能对应 NVIC 线。此前 blink
  上板验证走的是 SysTick（内核异常，不经 NVIC 外设线），所以漏洞
  一直没暴露。

## [0.1.0] - 2026-10-24

- 首版：SysTick 单次闹钟时间驱动、`bind_interrupts!`、异步
  GPIO/UART/I2C/SPI、PRIMASK critical-section、blink_async 实板验证
  （含时间驱动基数 bug 的上板修复记录，见 README）。
