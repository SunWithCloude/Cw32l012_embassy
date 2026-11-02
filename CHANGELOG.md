# Changelog

遵循 SemVer；格式参考 Keep a Changelog。

## [未发布]（0.4.0 增量，随 DMA/RTC 收尾定档）

### 新增

- **家族矩阵扩至 11 个**：`cw32l010`/`cw32l011`（L012 代组，KEY 写保护 +
  UART 错误位 8..11 + APBEN1 布局）与 `cw32l031`/`cw32l052`/`cw32l083`/
  `cw32r031`/`cw32w031`/`cw32f020`/`cw32f003`（经典代，无 KEY + 错误位
  3/4 + APBEN2 布局）。家族差异逐 PAC 核对，复位默认时钟以各家族数据
  手册原文为准（L010/L011/L012=4MHz，其余=8MHz——L031 Rev1.8 p14 等原文
  纠正了此前按 4MHz 的归类）。CI 增设 11 家族互斥矩阵 job。
- `timer` 模块（L012 先行）：GTIM1-4 单通道边沿对齐 PWM（`SetDutyCycle`，
  万分比满量程）+ `wait_update()` 异步等更新事件；寄存器配方与阻塞版
  `cw32l012-hal` 的 `gtim.rs` 逐行同源（实机验证过）。发现手写 PAC 的
  GTIM 块漏建模 ICR（§16.10.6，BASE+0x70，R1W0）——以基地址偏移访问
  绕过，PAC 侧待补。
- `adc` 模块（L012 先行）：异步单次/序列转换（EOC/EOS 中断驱动，10ms
  软超时）、内部通道（1.2V BGR → `vdda_mv()`、温度传感器 →
  `temperature_mc()` 含 SensorDead 失效保护——该板 TS 硬件无输出的
  实测结论收敛进错误路径）。`Pin::into_analog()` 新增。
- `rtt_hello` 示例：defmt-rtt + panic-probe，`_defmt_timestamp` 以
  embassy-time uptime 毫秒手写（不引额外 crate）。
- F003 的 I2C（经典状态机同构，中断名 `I2C` 无数字）、l083 的共享向量
  命名（GPIOC_GPIOD / GPIOE_GPIOF / UART1_UART4 等）接入。

- **GPIO 全端口接线（l052/l083）**：PAC 实例回填带来的 GPIOD/GPIOE 已接入
  embassy——共享向量 handler 成对派发（`GPIOC_GPIOD` 服务 C+D、
  `GPIOE_GPIOF` 服务 E+F），waker/触发位图 bank 扩为 6 槽避免同 pin 位
  互相污染；`wait_edge` 的 NVIC 表补 bank 4/5。
- `bsp_pin!`/BSP 引脚表随 PAC 回填解锁 gpiob+（cw32-bsp 6a6b773/4c4218b）。

- `rtc` 模块（L012 先行）：异步闹钟（`wait_alarm`，KEY 解锁序列包进
  critical-section——IRQ 与任务共享 KEY 状态机的必要加固）、日历读写
  （`now`/`set_datetime`）、LSE/LSI 时基（LSI 精度实测结论入文档）。
  `wdt` 模块：IWDT/WWDGT 薄封装（窗口配置求解器 `for_feed_interval`），
  选择建议表入文档。16 项宿主测试。
- `dma` 模块（L012 先行）：4 通道异步 DMA（IRQ9 DMA12/IRQ10 DMA34 共享
  向量，单发 IER 纪律），`mem_to_mem`/`periph_to_mem`/`mem_to_periph`
  便捷入口 + `adc.read_sequence_dma()`（IER.DMAEOS + ADC1_SEQ 触发——
  ⚠ 阻塞版实机配方为单次路径的推广，首次上板核对缓冲内容）。经典组
  DMA 为不同 IP（6 通道、4bit 打包 ISR、无 remaining 回读），保持
  L012 独占。
- `defmt` feature（库级日志支持）：开启后各驱动公共错误/配置类型 derive
  `defmt::Format`（uart/spi/i2c/adc/dma/timer/rtc/wdt/gpio；dac 待合并后
  补一行），默认关闭、关闭时零开销；`rtt_hello` 示例（defmt-rtt +
  panic-probe）形成端到端日志链路。

- `dac` 模块（L012 先行）：双通道 12 位（`set_ch1/ch2/both` + 回读）、
  DMA 波形输出（`write_dma`/`start_dma_loop` 环形循环——GTIM TRGO 供拍 +
  `hs::DACx` 请求线，§26.6-26.8 语义逐条标注实机/推断状态）、DAC→ADC
  内部通道闭环示例（阻塞版实机例程的 embassy 版）。门控在 APBEN2[10]
  （纠正了任务面的 APBEN1 笔误）。
- `defmt` feature：全驱动错误/配置类型（30+）`Format` derive，CI 增设
  defmt-on job。

- **经典组 DMA**（`ClassicDma`，f020/f030/l031/l052/l083/r031/w031）：与
  L012 不同的 IP——每通道独立寄存器块（@0x20 步进）、ISR/ICR 每通道
  4bit 打包、无 remaining 回读。通道数 f030/f020/l083=5、l031 组=4、
  f003/l010/l011 无 DMA。中断分组 DMACH1(9)/DMACH23(10)/DMACH45(11)
  （l031 组 DMACH4、f020 的 IRQ11 保留→通道 4/5 仅轮询）。触发源表
  全家族就绪（各家手册 §8.8.4，cw32-hal2 cw32-data/dma 数据）。门控
  **AHBEN bit0**（纠正了任务面的 APBEN1 误判）。
- **经典组 ADC**（f030/f020 归组）：与 L012 完全不同的 IP——cr0/cr1/
  sqr 4 结果架构。EOC/EOS 异步、软件触发先行、**内部通道实锤存在**
  （VDDA/3、温度、Vref1.2 + 出厂校准字——纠正了"经典组无内部通道"的
  先前结论，按 sha 锁定的厂商库 V2.2 配方实现）。l031 组/f003/l010/
  l011 的 ADC 为另外的 IP，排除待后续。
- **经典组 ADC×DMA 联动**（`ClassicAdc::read_dma`，f030/f020）：硬件
  DMA 请求线实锤——CR1.DMAEN=转换完成发 DMA 请求（手册 V2.5 §22.13.2
  bit7；厂商库 cw32f030_adc.c:697-727 `ADC_DMACmd`），请求源 HARDSRC
  =0x0A「ADC 转换完成」（§8.4.4 表 8-2/§8.8.4 p130/p140；厂商库
  cw32f030_dma.h:117 `DMA_HardTrig_ADC_TRANSCOMPLETE=0x0AUL<<2`，即
  dma.rs 既有 `hs::ADC_COMPLETE`）。路径取 MODE=1 单通道 CR2.CNT+1 次
  有界转换（§22.5.2：每次转换完成 EOC+RESULT0、完成后 EOA+START 自清；
  厂商库 `ADC_SingleChMoreModeCfg` cw32f030_adc.c:405-412 含强制 ACCEN
  配方），DMA 侧 RESULT0 固定源 B16 搬入缓冲、占通道 1。序列模式不走
  DMA：结果按序列位轮转 RESULT0..3、手册无按请求选源机制，不做推断。

### 变更

- 门控写规范：经典组 AH BEN/APBEN 无 KEY（modify 读改写）；L012 代组
  KEY 写。以 cw32-hal2 各家族 clock sidecar 的 `enable_write_key` 元数据
  与 F030 上板实测交叉验证。

- timer：GTIM 清更新标志改走 PAC 的 `Gtim::icr` 字段（cw32l012-pac 补录
  0x70 处 ICR，§16.10.6 R1W0，厂商库 `GTIM_TypeDef` Size=116 交叉核对），
  撤掉驱动里的裸指针偏移 hack，行为逐位一致。
- rtt_hello 示例的时间戳提供者改走 defmt 1.x 的 `timestamp!` 宏（旧的手写
  `_defmt_timestamp` 签名在 defmt 1.x 下不会被采纳——defmt 0.3→1.0 签名
  变更，旧写法静默失效；nm 实证符号与 `{=u64}` 元数据已配对）。


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
