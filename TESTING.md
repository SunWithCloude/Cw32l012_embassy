# TESTING.md —— embassy-cw32 实机验证台账

阻塞版 `cw32l012-hal` 的实测记录见其仓内 TESTING.md(23 项);本文件只记
**异步侧**(embassy-cw32)。状态口径:

- ✅ **已实测**——板上跑过,带观察数据与日期;
- ⏳ **仅编译**——CI 全绿、语义有同源旁证(阻塞版配方/寄存器读数),板上未跑。

## 通用操作

烧录/观测配方与 hal 仓一致(chip 名走 cw32-probe-rs-compat 的 target YAML):

```text
# L012:probe-rs 0.32 + ST-Link 0483:3748,chip CW32L012x8(48 脚,LQFP)
probe-rs download --chip CW32L012x8 --protocol swd <elf>
probe-rs reset   --chip CW32L012x8 --protocol swd
probe-rs read    b32 <addr> 1          # 读 SRAM 静态量
# F030(立创天猛星):chip CW32F030C8(F030 GPIOC@0x48000800,与 L012 不同)
```

各例程的预期读数/示波器观察点写在示例文件头注(`//! # 验证方式`),
静态量统一 `#[used]#[unsafe(no_mangle)]` 锁符号(debug 构建不会剔除)。

## 已实测

| 例程 | 日期 | 实测内容 |
|------|------|----------|
| blink_async | 2026-10-24 | PC13 心跳(执行器 + 双任务);embassy 首次上板 |
| buffered_echo | 2026-10-24 | ISER bit27=1(NVIC 使能修复生效);经 probe-rs write 往 ISPR 注入 UART1 挂起 → ICPR 自动清零且心跳不停(向量 → buffered_irq → 临界区全链路) |
| blink_async_f030 | 2026-10-25 | 500ms 精确翻转——F030 上首个异步 Rust 程序 |
| i2c_scan_f030 | 2026-10-25 | 112/112 NACK(空总线预期);两大引脚坑实锤收编驱动:F030 复位默认 GPIOA_ANALOG=0x9fff(仅 SWD 数字使能)、DIR=输入态时 START 永不发不出(经 OpEndrain 访问面统一处理) |

旁证(非异步侧):阻塞版 23 项全过(hal 仓 TESTING.md),含 GTIM PWM、
ATIM 互补+死区+刹车(BDTR=0x94FF)、ADC 序列、RTC、IWDT/WWDT、FLASH 自
编程、LPTIM 深度休眠、HALLTIM、OPA 环路——embassy 驱动的寄存器配方与
阻塞版同源,其正确性由这批实测背书。

## 待验证(⏳ 仅编译,CI 全绿)

### L012(feature = "cw32l012")

| 例程 | 首查点 |
|------|--------|
| uart_echo_async | PA02/PA03 接 USB 串口适配器(COM5);回显字节一致 |
| adc_async | IN12 精确跟随 DAC 设定(±1 LSB,BGR=1488 理论值旁证阻塞版) |
| timer_pwm | GTIM1_CH1@PB04 1kHz 方波;`PERIOD`=0x0F9F_0000 |
| **atim_motor** | 三相互补六路(PA8/PA7、PA9/PB0、PA10/PB1,全 AF7)+1µs 死区;`PERIOD` PSC=0/ARR=199、`DTG`=0x04、`UPDATES`≈20000/s;PA6 拉低 → 六路全灭 + `BRAKES`+1,释放 → `clear_break`+`enable` 恢复斜坡。**上板前勿接真电机** |
| rtt_hello | `probe-rs logs` 或 VS Code 插件面板见 defmt 输出 |
| dma_mem2mem / adc_dma / uart_dma / spi_dma | 静态量校验字;心跳独立翻转(挂起等待期间不停) |
| dac_wave / dac_adc_loop | DAC1 三角波;片内回环 IN12 跟随(阻塞版同源已实测) |
| rtc_alarm | 闹钟唤醒(阻塞版 rtc_demo 跨日走时已实测) |
| wdt_demo | 复位循环:PC13 快闪 5 下 + 灭 1.5s(阻塞版同源已实测) |
| i2c_transaction | 写指针 + 重复起始读;上拉 4.7kΩ |
| flash_store | `BOOT_ID` 掉电保持递增(阻塞版 flash_store 已实测) |
| opa | 跟随器 + PGA 增益,ADC 片内读回(阻塞版 analog_loop 已实测) |

### 经典家族(feature = "cw32f030")

| 例程 | 首查点 |
|------|--------|
| uart_echo_f030 / uart_dma_f030 | 8MHz 复位默认时钟;115200-8N1 回显 |
| spi_loopback_f030 | MOSI↔MISO 杜邦短接;图样一致 |
| adc_read_f030 / adc_dma_f030 | 厂商库 V2.2 配方;基准电压读数 |
| dma_mem2mem_f030 / spi_dma_f030 | 静态量校验字 |
| timer_pwm_f030 | GTIM1_CH1@PB04(PRS 分频路径) |
| rtc_alarm_f030 / wdt_demo_f030 | ACCESS/WINDOW 访问窗版 RTC;IWDT 复位循环 |
| **atim_motor_f030** | PWM2S 三相:CH1A@PA08+CH1B@PB13、CH2A@PA09+CH2B@PB14、CH3A@PA10+CH3B@PB15(LQFP48/C8T7);死区 500ns;DTR 编码回读。**经典 ATIM 是独立 IP(MSCR/DTR/CHxCCRA+B),驱动 compiles-not-run,先逻辑分析仪后电机** |

### 其它家族

- l083(`cw32l083\):UART×6/SPI×2/I2C×2 多实例布线已落地(`uart_echo_l083`
  示例,共享向量 UART2_UART5 等 ELF 向量槽实测通过);DMA 触发源映射
  UART×6/SPI×2 全接线(`uart_dma_l083` = UART2 DMA 回显,触发源
  UART2_RX/TX=0x02/03,UART4..6=0x2B..0x30 仅 l083 手册表有;I2C 不在
  手册 §8.8.4 DMA 请求表——CN_V2.0 印刷页 147-149 无 I2C 行,不接 DMA)。
  ⏳ 仅编译。
- **l083 ATIM 互补 PWM 已接**(`atim_classic`,与 f030 一套驱动):死区四段/
  ICR 写 0 清/MODE=10 逐条对照《CW32L083 UM V2.0》§16.3.3.5(印刷页
  317-318)/§16.7.5(p340)/§16.7.9(p345)核实,引脚 SDK 宏 + DS V1.9
  表 5-3..5-7 双源(PE9 单源孤证拒收)。首板核对项
  `atim_motor_l083`(PA8-10/PB13-15 三相全 AF7 + PC13 心跳):
  `PERIOD`=ARR=399/档位 0(20kHz@8MHz)、`DT_TICKS`=4/`DT_CODE`=2
  (500ns 死区)、`UPDATES`≈20000/s、三对互补方波对边沿死区 ≈500ns。
  **上板前勿接真电机**。⏳ 仅编译。
- **l010/l011 PWM 已接**(`cw32l010`/`cw32l011`):GTIM 简单 PWM
  (`timer_pwm_l010`)与 ATIM 互补 PWM(`atim_motor_l011`)落地,与 L012
  一套驱动(位面经三家族 PAC 逐位比对 + hal2 证据链核对)。⏳ **仅编译,
  compiles-not-run**——两家族无阻塞版同源背书,首板核对项:
  - timer_pwm_l010:GTIM1_CH1@PB6(AF6)1kHz 方波;`PERIOD`=0x0F9F_0000、
    `UPDATES`≈1000/s、`CNT_MIN/MAX` 铺满 0..3999
  - atim_motor_l011:三相互补六路(PA8/PA7、PA9/PB0、PA10/PB1,全 AF7)
    +1µs 死区;`PERIOD` PSC=0/ARR=199、`DTG`=0x04、`UPDATES`≈20000/s;
    PA0(BK,AF7)拉低 → 六路全灭 + `BRAKES`+1,释放 → `clear_break`+
    `enable` 恢复。**上板前勿接真电机**
  - l010 差异:无外部刹车脚(`bk` 必须 None——BK 候选 PA1/PA7 被振荡器/
    SWDIO 占用排除)、无 CH4/CH4N 路线;GPIO 无 PDR 下拉寄存器
- l031/l010/l011/f003/l052/r031/w031:`cargo check` 矩阵全绿。

## 已知问题(复测注意)

- **F030 调试口楔死史**(2026-10-25):git 版 probe-rs 无 `--protocol`
  默认走 JTAG,ST-Link 共享引脚的 JTAG 扫描把 SWD TAP 打死(粘滞错误
  只能物理断电重插)。复测一律带 `--protocol swd`;对照脚本
  `Desktop/probe-rs-4551-test.bat`。
- **异步 UART 回环**尚未闭环:串口适配器接线后先跑 uart_echo_async。
- **TS 内置温度传感器**:本板个体异常(hal 仓 TESTING.md 有排查记录,
  代码无错)。
- 驱动时间基数:复位默认 SysClk/PCLK——L012 = 4MHz,F030 = 8MHz
  (HSI48M/6);假设错会让波特率/节拍整体偏 6 倍。
