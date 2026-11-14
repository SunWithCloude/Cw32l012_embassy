# Changelog

遵循 SemVer；格式参考 Keep a Changelog。

## [未发布]

- **经典 ATIM 互补 PWM 接入 l052/l083（f030 同款驱动一套三家族）**：
  `atim_classic` 从 cfg(cw32f030) 扩为 `any(cw32f030, cw32l052,
  cw32l083)`，API 与寄存器配方不变。依据（每家族手册级证据，逐条
  对照后才接入；页码为手册 PDF 0 基页索引，三本手册页脚印刷页与之一致）：
  - **寄存器面**：三家族 PAC `pac::ATIM` 结构体 20 字段逐字节一致
    （`dtr@0x30`、`icr@0x14`）、CR/ISR/ICR/FLTR/CHxCR/DTR 位域常量逐位
    一致、IRQ `ATIM` 同为 13；唯一差异 l052 PAC 的 MSCR `msm@bit4`
    （l052 手册标 bit4 保留）——驱动 MSCR 恒写 0，两解释行为相同。
    hal2 证据链指纹：f030/l083 同 `atim_v1`（c68086ae…），l052 单列
    `atim_cw32l052_v1`（1932ea2a…，即 msm 一位之差）。
  - **l052**（《CW32L052 UM V1.5》）：互补配方 §16.3.3.5 p305-306；
    DTR 四段死区 §16.7.9 p333（tick 范围锚点 2~129/130~256/258~506/
    514~1010 与 f030 p305 逐字一致）；ICR 写 0 清/复位 0x7_FFFF §16.7.5
    p328；MODE 0b10=边沿对齐/PRS 111=256/PWM2S=1 单点 §16.7.1 p323-324。
    门控 APBEN2.ATIM=bit7（PAC 位域 + SDK `cw32l052.h:7851` 双源）。
  - **l083**（《CW32L083 UM V2.0》）：互补配方 §16.3.3.5 p317-318；
    DTR 四段死区 §16.7.9 p345；ICR §16.7.5 p340；CR §16.7.1 p335-336。
    门控 APBEN2.ATIM=bit7（PAC 位域 + SDK `cw32l083.h:8425` 双源）。
  - **引脚（SDK 宏 + 数据手册 AF 表双源一致才收）**：l052 29 条
    CHxA/B 路线全收（DS V1.3 表 5-3/5-4/5-5，印刷页 30/31/32）；l083
    45 条 SDK 宏收 44 条——**PE9 拒收**（SDK `PE09_AFx_ATIMCH2A`
    `cw32l083_gpio.h:2006` 有而 DS V1.9 表 5-7 印刷页 37 的 PE09 行无
    ATIM_CH2A，单源孤证宁缺毋滥）。全表进 `src/atim_classic.rs` 模块
    文档；AF 编号按家族/引脚而异（A/B 口主 AF7、C/D/E 口主 AF6），
    `AtimPair` 的 af 参数按表填。
  - 示例 `atim_motor_l083`（l083 三相互补 20kHz/500ns 死区，
    CH1A@PA8+CH1B@PB13、CH2A@PA9+CH2B@PB14、CH3A@PA10+CH3B@PB15 全
    AF7，PC13 心跳）。
  - ⚠ **compiles-not-run**（三家族皆然）：thumbv6m 矩阵全绿；波形/
    死区/刹车路径未上板。上板前勿驱动真电机。

## [0.5.0] - 2026-11-14

### 新增

- **PWM 接入 l010/l011（L012 代组低配家族）——GTIM 简单 PWM + ATIM 互补
  PWM，三家族一套驱动**：`timer` 的 `GtimPwm` 与 `atim` 的 `AtimPwm`
  从 cfg(cw32l012) 扩为 `any(cw32l012, cw32l010, cw32l011)`，API 同名同
  参。依据：hal2 证据链 `docs/atim-complementary-evidence.json` 的
  `register_variant` 三家族同为 `cw32l010_v1`（同一套 STM32 风格 IP，
  基址同为 0x4000_1400），`qualified_subset` 的死区时钟/DTG 编码/刹车
  语义 l010/l011 与 L012 逐条相同。落地方式：
  - **位常量本地化**：l010/l011 PAC 是小写字段模块风格
    （`pac::atim::cr1::cen_POS/cen_MASK`），驱动按 L012 同款命名在
    embassy 侧定义整字常量（注明对应 PAC 字段模块），三家族共用；新增
    锚点测试逐位比对三家族 PAC（`gtim_bitplane_matches_family_pacs`/
    `bitplane_matches_l012_pac_whole_word_consts`）。
  - **字段名薄适配**：`dier`→`ier`（0x0C）、`ccmr1/ccmr2`→
    `ccmr1cap/ccmr2cap`（0x18/0x1C，比较位面在 PAC `ccmr1cmp/ccmr2cmp`）
    经内联访问函数收敛；`isr` l010 PAC 标 RW/l011 标 RO（驱动只读，语义
    一致）；l010/l011 的 GPIO 无 PDR 下拉寄存器，清下拉那步仅 L012。
  - **GTIM 实例**：l010 = GTIM1 单实例（IRQ16）、l011 = GTIM1/GTIM2
    （IRQ16/17 独立）、L012 = GTIM1..4 不变；有独立 PSC 寄存器，与 L012
    同一线性分频配方（`calc_pwm_psc_arr` 复用）。
  - **l010 无外部刹车**：hal2 证据链
    `atim-complementary-route-evidence.json` external_bk_routes=0（手册
    BK 唯二候选 PA1=OSC_OUT/PA7=SWDIO 被排除）——`AtimPwm::new` 的 bk
    参数 l010 下必须 None（传 Some 运行时 panic），刹车只能内部 VC 源
    （AF1.BKVC1E/BKVC2E，本驱动不建模）；l011 有 4 条 BK 路线（PA0/
    PA6 AF7、PA12/PB5 AF5）。引脚表（SDK 宏 + 数据手册表 5-3/5-4 双源，
    振荡器/调试口占用路线已排除）进 src/atim.rs 模块文档。
  - 示例 `timer_pwm_l010`（GTIM1_CH1@PB6 AF6——任务简报的"PB4=CH1 AF7"
    经证据链复核系 ATIM_CH1@AF7 与 GTIM1_CH3@AF6 的混排，GTIM1_CH1 实为
    PB6/PA6@AF6）与 `atim_motor_l011`（三相互补 20kHz/1µs 死区，BK 可选
    接法）；宿主测试 68（L012）/33（l010）/33（l011）全绿。
  - ⚠ **compiles-not-run**：l010/l011 无阻塞版同源背书，实机验证待板；
    上板前勿驱动真电机。

- **l083 DMA 触发源映射全接线（UART×6/SPI×2）**：UART4/5/6 触发源
  0x2B..0x30、SPI2 0x08/09（均 l083 手册 §8.8.4 请求表，印刷页 147-149）。
  I2C1/2 经原手册核实**不在 DMA 请求表**（无硬件触发源），维持中断驱动。
  示例 `uart_dma_l083`。⚠ compiles-not-run。

### 修复

- **全家族 KEY 门控读改写收敛**：lib.rs 的 sysctrl_enable_gpioa/b/c/d、
  uart L012、adc L012、i2c L012/l010 的整字 `write(KEY|gate)` 全部改为
  `modify(|v| KEY | v | gate)`——同寄存器其它外设使能位不再被顶掉
  （l010/l011 侧同款修复的推广；F030 上板实测过整字写会清位）。

### 修复

- **uart（l010/l011）门控整字写顶掉其它外设使能位**：原
  `apben1.write(SYSCTRL_KEY | (1<<3))` 会清掉同寄存器的 ADC/VC/SPI1/
  UART2/ATIM/GTIM1/GTIM2 使能位（与 PWM 门控共存时后初始化者顶掉先
  初始化者的时钟）。改为读改写 `modify(|v| KEY | (v & 0xFFFF) | uart1en)`
  ——与 timer/atim 驱动同配方。顺带完成任务项的 KEY 协议查证：l010/l011
  的 SYSCTRL **没有独立 KEY 寄存器**，KEY 内嵌于各受保护寄存器 [31:16]
  （cw32l011.h:6018-6019 `SYSCTRL_APBEN1_KEY_Pos=16`、cw32l010.h:5662
  同；PAC SYSCTRL 结构体首字段是 CR0 而非 KEY，独立 KEY@0x00 的是
  RTC）——原"高位内嵌 KEY"协议本身正确，仅写法从整字写收敛为读改写。

## [0.4.0] - 2026-11-14

### 破坏性变更（0.4.0）

- **依赖全部走 crates.io**（为发布本 crate 铺路）：10 个生成家族 PAC 改为
  版本依赖并精确钉版（`=0.1.2`），cw32l012-pac 改名 **cw32l012-pac-um**
  （`=0.3.0`）——crates.io 上 `cw32l012_pac` 已被第三方缺陷版占用。f030 与
  l010 同因改名 `cw32f030-pac-um`（占用者 cw32-rust）与 `cw32l010-pac-um`
  （占用者 Lioyae）。`cw32-pacs` 仓 0.1.2 起以新名发布；sibling path 依赖
  不再需要，本地开发用 `[patch.crates-io]` 指回检出目录。
- **critical-section：实现保留本 crate 的 PRIMASK 版**（feature
  `critical-section-impl`，默认开启，restore-state=u8）。家族 PAC 0.1.2
  的 `Peripherals::take` 是静态 bool 标记、不带任何 critical-section
  实现，与本实现无冲突；"一个固件只能有一个实现、与 cw32l012-hal 的
  实现互斥"的警告不变。（评审中曾试过切到 cortex-m 的
  `critical-section-single-core`，因 restore-state 统一与生态一致性
  问题未采纳，留档在 wip 分支。）

### 新增

- **ATIM 高级定时器互补 PWM**（`atim` 模块，L012 家族独占，手册 §16；
  三相电机 PWM 路线）：CH1-3 主/互补输出对（死区插入）+ CH4 单端 +
  BK 刹车输入。API：`AtimPwm::new`（9 路 `Option<(Pin, AF)>` + 刹车
  极性/滤波 + `bind_interrupts!` 绑定证明；不用的通道传 `None` 需显式
  标注端口类型——嵌套 APIT 推断限制）、`set_duty_bp(ch 1..=4, 万分比)`、
  `enable()`（置 MOE，返回写入后实态——刹车活跃时硬件异步清 MOE、置
  不上）/`disable()`（封波）、`is_braked()`/`clear_break()`（刹车源
  活跃时 BIF 清不掉、返回 false）、`stop()`/`start()`、`dtg()`/
  `period_config()` 回读、`wait_update()`（UIF 周期节拍，DIER.UIE 单发
  纪律同 GTIM；`AtimIrq` 防御性压 COMIF/BIE）；错误 `AtimError`
  （FreqOutOfRange/BadDuty/DeadTimeTooLong/BadBreakConfig，实现
  `embedded_hal::pwm::Error`）。关键语义三源核对（本仓 PAC `pac::atim`
  位面 + 手册 §16 + fan2nd/cw32-hal2 `docs/atim-complementary-evidence.json`
  证据链，MIT）：死区时钟 = f_DTS = PCLK（CKD 固定 /1，死区不走 PSC）、
  DTG 四段编码最大 1008 tick（`dead_time_ns` **向上取整**、绝不饱和到
  不足值，`encode_dtg`/`decode_dtg` 助手）、推荐保守面（OSSI/OSSR=1 +
  OIS\*=0 + 高有效 + LOCK=0 + BK2/DTR2 不碰）、ICR 为 R1W0（清标志写
  "全 1 除该位"，不能整字写 0）。PAC 缺失的位常量（CR2 的 CCPC/CCUS/
  CCDS/MMS/OIS\*、CCMR2 的 OC3/OC4 面、DIER/ISR 全套、BDTR.BKF/BK2F）
  本地补齐并注明 §16。引脚表（数据手册 V1.0 表 5-3，SDK 双源验证）进
  模块文档；刹车输入脚 GPIO 配方与输出脚分离（DIR=1 输入、保留调用方
  上下拉、不清开漏）。示例 `atim_motor`（三相互补 20kHz/1µs 死区 +
  120° 占空比斜坡 + 刹车监控/显式恢复 + BK 开漏比较器接线图）。
  8 项新宿主测试（DTG 四段边界/穷举最小向上取整/256 编码回环/上限/
  ns 换算/ICR 清位字），45 项不减、clippy -D warnings 干净。
  ⚠ **compiles-not-run**：实机验证待板，上板前勿驱动真电机（模块文档
  含诚实的安全边界声明：不做电机安全、不做故障闩锁承诺、死区不构成
  防直通保证）。
- **l083 串行多实例布线补齐（UART×6 / SPI×2 / I2C×2）**：此前 l083 只接了
  UART1/SPI1/I2C1（旧注释称"PAC 只收编 UART1 寄存器块"已过时——cw32l083-pac
  0.1.2 的 `UART1..UART6`/`SPI1..SPI2`/`I2C1..I2C2` 基址与实例**全都有**，
  基址 UART1@0x40013800/UART2@0x40004400/UART3@0x40004800/UART4@0x40004c00/
  UART5@0x40005000/UART6@0x40011400、SPI1@0x40013000/SPI2@0x40003800、
  I2C1@0x40005400/I2C2@0x40005800）。本轮只做 IRQ+构造布线：
  - **uart.rs**：`instance_of` 按基址 match 扩为 UART1..6→0..5；per-实例
    waker/管道/标记静态量从 3 槽扩为按家族实例数（l083=6，其余 3 不变）；
    新增 `new_uart2..new_uart6`（同步与 BufferedUart 两套，门控按实例查表：
    UART1=APBEN2.9、UART2=APBEN1.7、UART3=APBEN1.8、UART4=APBEN1.9、
    UART5=APBEN1.10、UART6=APBEN2.1——PAC apben1/apben2 位域，无 KEY 读改写）。
    **共享向量纪律**：l083 的 UART 两两共享 NVIC 线（UART1/4=IRQ27
    `UART1_UART4`、UART2/5=IRQ28 `UART2_UART5`、UART3/6=IRQ29
    `UART3_UART6`，typelevel 拼写以 PAC interrupt 模块标记类型/device.x
    符号为准），参照 SPI2/3 共享 `SPI23` 的做法新增 `Uart1Uart4Irq`/
    `Uart2Uart5Irq`/`Uart3Uart6Irq` 派发标记（依次检查组内两个实例），
    l083 原 `Uart1Irq` 移除（只派发半个共享向量会留下另一半 IER 常开，
    有中断风暴风险）。示例 `uart_echo_l083`（UART2 PA06/PA07 AF2 +
    `UART2_UART5 => Uart2Uart5Irq` 绑定）；链接产物 ELF 向量槽实测：
    IRQ28 → 0xFD（强符号 `UART2_UART5` 处理函数），未绑定向量恒为
    DefaultHandler 0xF5——证明共享向量符号拼写与 device.x `PROVIDE`
    弱符号正确对接（拼写错会静默落到 DefaultHandler，编译期不可见）。
  - **spi.rs**：l083 双实例独立向量（SPI1=IRQ25/SPI2=IRQ26）——`new`
    （l083 要求同时绑定 `SPI1`+`SPI2`）、`new_spi2`、`Spi2Irq` 标记；
    门控 SPI2=APBEN1.6（与 f030 同位，PAC 位域核对）。
  - **i2c.rs**：l083 双实例独立向量（I2C1=IRQ23/I2C2=IRQ24）——`new`
    （同时绑定）、`new_i2c1`/`new_i2c2`、`I2c2Irq` 标记；门控
    I2C1=APBEN1.11、I2C2=APBEN1.12；风暴防护的 NVIC 压线/重开按实例
    （经典分支无 IER，只能动 NVIC 线）。
  - 引脚路线写进各模块文档表（数据手册表 5-3..5-8 摘要；全表见
    cw32-hal2 数据仓库 `af/cw32l083-serial.yaml`）。
  - **DMA 触发源映射**：0.4.0 发布时为防呆 panic，0.5.0 起全接线——
    `cudma::tx_hs/rx_hs` 扩到 idx 0..5：UART1..3=0x00..0x05（全经典组
    请求表同值）、UART4..6=UARTx_TX/RX=0x2B..0x30（**仅 l083 手册表有**，
    l083=CN_V2.0 p147-149），UART2/3 臂从 cfg(f030) 放宽为
    cfg(any(f030,l083))（instance_of 同款 cfg）；spi.rs 的 SPI2 同步接线
    （SPI2_RX/TX=0x08/0x09，f030 与 l083 请求表同值）。A23 的"未接实例
    显式 panic"防呆换成按实例查表（兜底 unreachable——单实例家族
    `instance_of` 只产 0，不会越界）。**I2C 查证结论**：l083 手册 §8.8.4
    `DMA_TRIGy.HARDSRC[7:2]` 请求表（CN_V2.0 印刷页 147-149/PDF 页
    148-150）0x00..0x32 全部 51 行**无任何 I2C 行**——I2C1/I2C2 无硬件
    DMA 触发源，无从接线，i2c.rs 保持中断驱动并文档注明出处。示例
    `uart_dma_l083`（UART2 DMA 回显，证明多实例触发映射可用）。
  - 顺带修复：①f030 的 `new_uart2`/`new_uart3` 门控此前 assert
    `idx==0`，构造即 panic——改为查表 UART2=APBEN1.7/UART3=APBEN1.8
    （F030 PAC 位域）；②lib.rs 家族互斥宏名单补上漏掉的 `cw32l083`
    （此前 l083 与其它家族 feature 同开不会触发互斥 compile_error，
    会以更隐晦的 `pac` 重导出冲突报错）；③timer.rs 经典分支 `arr.read()`
    返回类型两族不同（l012=u16/l083=u32），l083 feature 下 clippy 报
    unnecessary_cast——按家族中立写法 + 局部 allow 收敛（l083 示例
    clippy -D warnings 随之全绿）。
  - ⚠ **compiles-not-run**：l083 实板未验证——本轮改动经 11 家族矩阵
    check、clippy -D warnings、宿主测试与 l083 示例全链接（含向量槽
    ELF 核验）保证编译期正确；运行语义（共享向量派发、门控查表）待实板。
- **FLASH 自编程**（`flash` 模块，L012 先行——经典家族 FLASH 是另一套
  IP，本轮未做，模块头注已注明）：片上 FLASH 读 / 页擦除 / 编程，
  参数存储与校准数据落盘的底层。API：`Flash::new(p.flashctrl)`（自动开
  `AHBEN.FLASH` 配置时钟——hal 上板实锤：缺时钟时 CR1/CR2/PAGELOCK 写
  全被静默丢弃）、关联函数 `read_u8`/`read_u32`/`read_slice`（范围校验 +
  `Result`，不 panic）、`unsafe erase_page(page)` / `program_slice(addr,
  &[u8])`（半字对齐 + 奇数尾 0xFF 填充凑半字 + 强制同页，4 对齐且 4 倍数
  长度走纯字路径）/ `write_page(page, &[u8])`（擦+写组合）；错误类型
  `FlashError`（BadPage/AddrRange/CrossPage/BadAlign/Timeout + ISR 错误位
  翻译 NotErased/PageLocked/PcPage/CacheOn/SdkRegion）。页解锁/回锁为每
  操作自动（阻塞版同款选择）。寄存器配方三源交叉核对：阻塞版
  `cw32l012-hal/src/flash.rs`（**实机验证过**，TESTING.md flash_store
  条目：第 127 页掉电计数器跨复位累加 OK，两条静默失败坑——AHBEN.FLASH
  时钟、MODE 须"或"进现有 CR1——都收进本驱动）；厂商库
  `cw32l012_flash.c`（ErasePage 字节写触发 / WriteWords 字对齐 /
  WriteHalfWords 半字对齐 / PAGELOCK 每 8 页一位 / 0x5A5A KEY）；手写 PAC
  `flash::` 位定义（§7.10）。页参数 512B×128 页（厂商
  FLASH_PAGES_SIZE/FLASH_BYTES_SIZE）。BUSY 无中断可用（IER 仅
  PROG/PAGELOCK/PC 三个错误位）→ 擦写为**有界忙等**（页擦典型 3~5ms，
  hal 同款 200 万轮上限），模块文档如实披露阻塞面；安全注释逐条给出处：
  擦写期间取指由硬件停顿（厂商库 flash 内运行配方 + hal 上板），唯一硬件
  禁区是 PC 所在页（ISR.PC），代码区其余页的误擦防护交 unsafe 契约（不引
  链接脚本符号，保持库对自定义链接脚本中立）。读入口与阻塞版差异：hal
  是"安全函数内裸解引用任意地址"，本版改为范围校验后解引用（0..0x10000
  恒映射）+ Err，声音性更严。示例 `flash_store`（u64 掉电计数器：magic +
  count + 反码 20 字节记录，擦→写→回读验证，掉电语义如实声明——编程中
  途掉电靠反码检出、擦除窗口丢记录为单页方案固有；PC13 反馈 + `__etext`
  观测量）。4 项新单测（页参数 / 拆分路径 / 覆盖不变量），38 项不减、
  clippy -D warnings 干净。⚠ 语义级实现待实板验证（阻塞版同配方，字
  路径已由 hal 上板背书，半字引子/收尾路径未单独上板）。
- **`opa` 模块（L012 独占）**：OPA1/OPA2 片上运放（手册 §29）。家族覆盖
  结论（11 个家族 PAC 逐一 grep `Opa` 块，2026-11-14）：只有
  cw32l012-pac 有运放（OPA1/OPA2 @0x4000_00B0/B8）；经典组
  f003/f020/f030/l031/l052/l083/r031/w031 与 l010/l011 均无 OPA 硬件
  （有 COMP 比较器、无运放），故非 `cw32l012` feature 不编译、无经典
  分支。API 与阻塞版 `cw32l012-hal/src/opa.rs` 对齐、embassy 所有权风格
  （`Opa::new_opa1/new_opa2` 自行使能 **APBEN2[9]** 时钟——TESTING.md
  analog_loop 实测坑：门控在 APBEN2 不在 APBEN1，写错 CR 读回恒为复位
  值 0xE000、配置静默失效）：`configure`/`set_mode`（External/
  `Pga(×2..×32)`/Follower）、`enable_inputs`（CR 导通位掩码，hal 同
  签名）+ 按实例引脚映射的 `enable_input_pin`/`enable_output_pin`
  （表 29-1：OPA1 INP1=PA03/INP3=PB01/OUT=PB00，OPA2 INP1=PA04/
  INP3=PB00/OUT=PB01，厂商库 cw32l012_opa.h `AFx_*` 宏同款只动
  GPIOx_ANALOG；端口时钟读改写保留其它外设使能位）、电平/触发双校准
  （`start_level_calibration`/`stop_calibration`/`trigger_calibration`/
  `is_calibrating`）、`set_cal_clock_div`（>7 返回
  `OpaError::BadCalClockDiv`）。校准时长按本仓 PAC 口径
  `8 << CALPERIOD` 个 OPACLK（§29.6；厂商库宏名 AZCLK16.. 暗示
  `16 << code`，差一倍不影响自动结束，文档注明待板核对）。输出脚即
  ADC 通道（表 25-4：IN8=PB00/IN9=PB01），示例 `opa`：跟随器片内回环
  （DAC_OUT1→OPA1→PB00→ADC IN8 读回——阻塞版 analog_loop 2026-10-23
  板验路径的 embassy 版）+ PGA ×2 演示（DAC_OUT2→OPA2→PB01→ADC IN9，
  ⚠ 内部 PGA 反馈是否需负端外接未上板核对，PGA_ERR 记录实测偏差）。
  7 项宿主测试（PGA/偏置/校准编码 + 引脚映射表逐项断言）。
- **经典组 RTC**（`ClassicRtc`，f030/f020/l031/l052/l083/r031/w031；f003
  无 RTC 硬件）：与 L012 不同 IP 的另一套日历——同款 `0xCA`→`0x53` KEY
  解锁（**上锁字 0x55**，RTC_KEY_LOCKW，厂商 cw32f030_rtc.h:347；L012
  侧为任意非 0x53），但运行中访问走 **ACCESS/WINDOW 窗**而非 L012 的
  WAIT 位（厂商 RTC_SetAlarm/RTC_AlarmCmd 配方，cw32f030_rtc.c:296-308/
  349-369：CR1.ACCESS=1 → 轮询 WINDOW==1 → 写 → ACCESS=0；停表直写、
  IER/ICR 不需要窗），无 PSC/SSCNT 预分频（日历=时钟源固定 32768 分频，
  cw32-hal2 lse-qualified.yaml rtc_calendar_divisor），DATE 日/月字段更宽
  （DAY[7:0]/MONTH[15:8] vs L012 DAY[5:0]/MONTH[12:8]），闹钟位面与
  L012 完全一致（厂商 RTC_AlarmMask_All=0x7F808080 同语义）。API 与
  L012 侧对齐：`start`/`stop`/`set_datetime`/`now`/`set_alarm_a/b`/
  `enable_alarm_a/b`/`wait_alarm`/`wait_alarm_b`/`flags`/`clear_flags`/
  `enable_interrupt`（返回 `Result`——运行中访问窗有界轮询可超时，厂商
  RTC_ACCESS_MAXTIMES=200 语义）。门控 APBEN1.RTC=bit3 读改写**无 KEY**
  （纠正任务面"KEY 读改写"假设）；LSI/LSE 使能在 SYSCTRL.CR1[3]/[4]，
  **CR1 带 0x5A5A KEY**（厂商 RCC_LSI_Enable `CR1 |= SYSCTRL_BYPASS |
  LSIEN`，cw32f030_rcc.c:305，SYSCTRL_BYPASS=0x5A5A0000 rcc.h:47；生成
  PAC 7 家族 sysctrl.cr1 key[31:16] 位段同款）。LSI 标称 32800Hz
  （cw32f030_rcc.h:44；L031 组同值，cw32-hal2 rcc l031 mod.rs:154），
  走 LSI 时日历秒 ≈1.00098s；不回填出厂 TRIM（回填地址仅 f030/f020 有
  厂商库实锤，复位默认即出厂校准，cw32-hal2 经典组同样不动）。异步闹钟
  语义与 L012 同款（经典组有闹钟中断 → `wait_alarm` 同款异步实现：等待
  期间才开 IER、IRQ 单发压掉、ICR R1W0 清标志；IRQ2，7 家族 PAC
  interrupt::RTC=2 同号）。七家族 PAC 的 RTC 结构体与位段常量**逐字节
  同构**（diff 为空）；cw32-hal2 rtc/mod.rs 的独立实现交叉印证同款键
  序列与窗协议。示例 `rtc_alarm_f030`（LSE 失败退 LSI 回退链 + PC13
  心跳观测面）。⚠ 语义级实现待实板验证。
- **经典组 IWDT/WWDGT**（`ClassicIwdt`/`ClassicWwdgt`，f030/f020/f003/
  l031/l052/l083/r031/w031——f003 无 RTC 但两条看门狗都在）：键值序列与
  L012 **完全同款**（0xCCCC 启动/0xAAAA 喂狗/0x5555 解锁/0x6666 上锁/
  0x5A5A→0xA5A5 停止，cw32f030_iwdt.h:76-81；cw32-hal2 wdg/mod.rs 交叉
  印证），配方差异实锤三条：①厂商库要求**先 0xCCCC 启动再改配置**
  （cw32f030_iwdt.c:71-72 注释原文），配置后等 SR.CRF/ARRF/WINRF 同步
  清零再喂狗装载（IWDT_Init+IWDT_SetPeriod 组合）——L012 分支是先配置
  再启动；②计数时钟为**专属 RC10K**（CW32x030 手册 V2.5 claim，非
  LSI），标称按厂商宏 IWDT_FREQ=10000（cw32f030_iwdt.h:48）——数据手册
  电气表口径 typical 8000Hz/全温 4-12kHz（cw32-hal2 electrical.yaml
  iwdt_clock），实际超时偏长、喂狗间隔留裕量，文档双口径注明；③门控
  APBEN1：IWDT=bit5/WWDGT=bit4，读改写无 KEY。WWDGT 位面与 L012 **逐
  位段一致**（CR0.WCNT/EN RW1、CR1.WINR/PRS[9:7]/IE[10]、SR.POV——
  L012 PAC 注释同位面），配方沿用：窗口求解器
  `ClassicWindowConfig::for_feed_interval`、feed 窗口检查、开启不可关。
  示例 `wdt_demo_f030`（喂 5 次停喂的复位循环观测面，L012 版同款）。⚠
  语义级实现待实板验证。

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
- **经典组 UART×DMA**（`Uart::write_dma`/`Uart::read_dma`，f030/f020/
  l031/l052/l083/r031/w031）：ClassicDma 通道抽象单次全量搬运——发送
  mem→TDR（占通道 2、`hs::UARTx_TX` 触发）、接收 RDR→mem（占通道 1、
  `hs::UARTx_RX` 触发），次序与厂商库 `USART_DMACmd` 一致。位面出处：
  请求使能位与 L012 **同位布局**——f030 厂商库 cw32f030_uart.h:188-189
  `USART_DMAReq_Tx=0x0080`(CR2[7])/`USART_DMAReq_Rx=0x0040`(CR2[6])、
  cw32f030_uart.c:462-467 `USART_DMACmd` 写 UARTx->CR2；其余家族厂商头
  `UARTx_CR2_DMATX_Msk=0x80`/`DMARX_Msk=0x40` 逐家抽验同值（cw32f020.h
  :7032-7035、cw32l031.h:6001-6004、cw32l052.h:8047-8050、cw32l083.h
  :8663-8666、cw32r031.h:6001-6004、cw32w031.h:6353-6356）。触发源全
  经典组 §8.8.4 请求表同段同值（f030 厂商库 cw32f030_dma.h:105-110
  `DMA_HardTrig_UARTx_RXBufferNE=0x00<<2`…；cw32-hal2 cw32-data/dma
  各家族 yaml UART1..3 request 0..5 交叉核对），复用 dma.rs 既有
  `hs::UART1..3_RX/TX` 常量，无新增。f020 的通道 1/2 各有 NVIC 线
  （DMACH1/DMACH2），`wait` 可用（避开 IRQ11 保留的通道 4/5）。
  示例 `uart_dma_f030`（PA08/PA09 AF2 回显 + PC13 心跳）。⚠ 语义级实现
  待实板验证。
- **经典组 SPI×DMA**（`Spi::transfer_dma`/`Spi::write_dma`，f030/f020/
  l031/l052/l083/r031/w031）：transfer_dma 双通道全双工（RX=通道 1、
  TX=通道 2，配置即弃句柄、等待按通道号重取）+ write_dma 只发。**关键
  家族差异：DMA 请求使能位在 CR1 而非 CR2**——CR1[17]=DMATX/
  CR1[16]=DMARX（f030 厂商库 cw32f030_spi.h:195-196
  `SPI_DMAReq_Tx=0x00020000`/`Rx=0x00010000`、cw32f030_spi.c:326-337
  `SPI_DMACmd` 写 SPIx->CR1；其余家族厂商头 `SPIx_CR1_DMATX_Pos=17`/
  `DMARX_Pos=16` 逐家抽验同值：cw32f020.h:6535-6538、cw32l031.h
  :5556-5559、cw32l052.h:7556-7559、cw32l083.h:8103-8106、cw32r031.h
  :5556-5559、cw32w031.h:5910-5913）。触发源 `hs::SPI1_RX/TX`=0x06/0x07
  全经典组、`hs::SPI2_RX/TX`=0x08/0x09 有 SPI2 的家族（f030 厂商库
  cw32f030_dma.h:112-115），同样复用既有常量。ICR 清标志保位：经典组
  ICR bit0=FLUSH **写 0 会清空发送缓冲/移位寄存器**（cw32f030_spi.c
  `SPI_FlushSendBuff`），清 OV/RXNE 的掩码除目标位外全 1。f003 无 DMA
  硬件不接入。示例 `spi_dma_f030`（PA05/06/07 AF5，write_dma +
  transfer_dma 回环判定）。⚠ 语义级实现待实板验证。
- **ADC 补齐剩余家族（三组新 IP，分组经各 PAC 逐寄存器比对锁定）**：
  - **8 槽扫描组**（l031/l052/l083/r031/w031 → `Scan8Adc` 分支）：
    经典代配方的 8 槽扩展——五家族 PAC 的 ADC 结构体逐寄存器同构
    （cw32l031/cw32r031/cw32w031 三份 adc 位段常量 diff 为空；l052/
    l083 仅 TRIGGER 多外设触发源位），vs f030 classic 差异：无 SQR@0x0C、
    序列拆 SQR0（ENS[18:16] 3bit + 槽 0..3）+SQR1（槽 4..7）、结果 8 个、
    CR1 无 DMAEN、CR0 无 BGREN。API：`new`/`read`/`read_sequence`
    （1..=8 通道）/`set_vref`/`enable_trigger`（五家族公共触发位子集）/
    `disable_trigger`；ADC-DMA 联动本轮不做。示例 `adc_read_l031`。
  - **f003 并入经典组分支**（`ClassicAdc` cfg 扩家族）：寄存器面与
    f030 逐位段同款（cw32f003-pac：CR0 含 BGREN bit4/BUF bit13、
    SQR.ENS[17:16]、READY bit7），差异=CR1 无 DMAEN（无 DMA 硬件，
    `read_dma`/`ClassicTrig` 仅 f030/f020 可用）+ 尾部 14 位扩展
    RESULT140..143/ENABLE14（不触碰，仍 12 位）+ 通道-引脚映射不同
    （IN0=PB2、IN1=PA1…）。`disable_trigger` 三家族通用。示例
    `adc_read_f003`。
  - **L010/L011 低配代组**（`LowAdc` 分支）：**与 L012 不同构的近亲
    IP**（纠正任务面「门控同 L012 → 疑同构」假设）——厂商头文件比对
    （cw32l010.h:164-379 Size=128 / cw32l011.h:167-388 vs cw32l012.h
    Size=124）：START@0x08/SAMPLE@0x28/SQRCFR@0x2C/RESULT0@0x40/
    IER@0x74，CR 位序 EN/BGREN/TSEN/CONT/CLK[5:4]/ENS[8:6]（L012 为
    EN/CONT/CLK[3:2]/ENS[6:4] 且 BGREN/TSEN 在独立 BGR 控制器）。无
    BGR 外设/无出厂校准字/无 READY 位/无 DMA；内部通道 TS(IN14)/
    BGR1.2(IN15) 实锤存在（40µs 采集下限，原始 counts）。采样档同
    L012 表但末档 **390**（非 518）+ 15 比较周期。API：`new`（APBEN1
    bit0 KEY 读改写）/`read`/`read_sequence`（1..=8 槽，per-槽采样
    周期）。示例 `adc_read_l010`。
- **经典组 GTIM PWM**（`ClassicPwm`，f030/f020/l031/l052/l083/r031/w031
  全 7 家族）：经典代 GTIM 的边沿对齐向上计数单通道 PWM + `wait_update()`
  溢出节拍（OV 标志 + IER 单发纪律，ICR R1W0 写 `!OV` 清除、保全复位值
  1 的 bit7/8）。**选 GTIM 不选 ATIM**：f020 无 ATIM（CW32F020 SDK 无
  atim 文件、手册外设表亦无），GTIM 才是 7/7 全覆盖；厂商库简单 PWM
  配方即 `GTIM_OCInit`+`GTIM_SetCompareN`+`GTIM_Cmd`。寄存器面与 L012
  无一处同构（CR0 EN/MODE + CMMR CCxM 4bit + IER/ISR/ICR；分频两子面：
  f030/f020 走 CR0.PRS 2^n（厂商库 `GTIM_PRESCALER_DIV1..DIV32768`，
  cw32f030.h `GTIMx_CR0_PRS_Msk=0x780`），l031 组/l052/l083 有 PSC
  寄存器线性 `div-1`（cw32l031.h `GTIMx_PSC_PSC_Msk=0xFFFF`；l031 组
  PAC 结构体以块首址 0x400_0700 起排、绝对地址与 f030 的 +0x300 布局
  相同））；无 EGR/无影子寄存器——ARR/CCR 立即生效、0%/100% 端点走
  强制电平模式（CCxM=8/9，CCR 16bit 装不下 65536 周期）、PWM 模式
  CCxM=15（CNT<CCR 输出高，各家手册 CMMR 表：14 为 CNT≥CCR 输出高，
  与厂商库宏名方向相反，逐家手册核对过）。门控 GTIM1/2=APBEN1 bit1/2、
  GTIM3/4=APBEN2 bit10/11（按家族存在性），NVIC 线独立 16/17/18/19
  （l031 组仅前两档）。`timer` 模块重构为 l012/classic 两子模块
  （L012 行为不变），`calc_pwm_psc_arr` 提为两代共用并新增
  `calc_pwm_pow2_div`（PRS 变体量化，宿主测试 5 项）。示例
  `timer_pwm_f030`（GTIM1_CH1@PB04 AF6，厂商库宏
  `PB04_AFx_GTIM1CH1` cw32f030_gpio.h:931 + 数据手册 V1.9 表 5-4）。
- **经典 ATIM 三相互补 PWM**（`atim_classic` 模块，`AtimPwm`，cfg
  `cw32f030` 一个家族先行——l031/l052/l083/r031/w031 的 ATIM 指令语义
  另有出入、f003 是另一变体、f020 无 ATIM，本轮一律不接）：经典代 ATIM
  是与 L012 ATIM **完全不同的 IP**（寄存器面无一处同构：单 CR 含
  COMP/PWM2S/PRS[6:4]/BUFPEN/URS/UG 命令位 + MSCR/FLTR/TRIG/CHxCR/
  DTR/RCR + CHxCCRA/CCRB 双缓冲，f030 PAC `pac::ATIM` 基址 0x4001_2C00
  整字 + 小写字段常量模块）。API：`AtimPwm::new`（三对互补引脚
  `Option<AtimPair>` 任选、至少一对，构造即门控 APBEN2 bit7 + 配置 +
  启动计数但 **DTR.MOE=0 无输出**）/`enable`/`disable`（MOE 主输出）/
  `stop`/`start`（计数 EN）/`set_duty_bp(ch, bp)`（CCRA 影子预装载，
  下一 UEV 生效，仅内部比较值 0<compare<period）/`duty_bp`/`is_braked`
  （ISR.BIF 观测）/`clear_break`（ICR 清标志，**不承诺**刹车恢复序列——
  BKE/AOE 保持 0，无刹车源）/`period_config`/`dead_time_code`/
  `dead_time_ticks`/`wait_update`（UIF 更新节拍，IRQ13，UIE 单发纪律）。
  硬件语义（CR 配方 COMP+PWM2S+MODE=10 边沿对齐+BUFPEN+URS、FLTR 三路
  OCM=110 PWM1 非反相、CHxCR 仅 BUFEA、UG 命令字装载影子后 ICR 写 0
  清全标志、DTR.MOE@12/DTEN@9 在 DTR 不在 MSCR）对照《CW32x030 UM
  V2.5》p277/278/296/300/305 核对；DTR 死区四段编码（2~129/130~256/
  258~506/514~1010 tick，上限 1010）收 ns 向上取整、四段边界+单调性
  + 编码下限搜索宿主测试全覆盖（p278 第四例 63.25us 系排版误按公式
  64.25us）；**PRS 档位无 128**（000..110=1..64、111=256，手册锚点
  "111：256"，任务草稿"0..7→1..128"有误已修正），分频/ARR 计算
  `calc_prs_arr` 宿主可测；ICR 为 RW 写 0 清（写 1 无效、复位 0x7FFFF、
  保留 bit1 保全），与 GTIM 同纪律。引脚/AF 双源（数据手册 V1.9 表
  5-3/5-4 + SDK 宏）：CH1A=PA5/PA8/PB2/PB5、CH1B=PA7/PA15/PB13、
  CH2A=PA4/PA9/PB6/PB10、CH2B=PB0/PB3/PB14、CH3A=PA3/PA10/PB7/PB11、
  CH3B=PB1/PB4/PB15（全 AF7）；CH4 内部通道无引出无互补不暴露。
  诚实标注 compiles-not-run：未上板验证波形/死区/刹车路径；死区按标称
  PCLK 向上取整（RCC 无 ClockBounds 包络，不做 ±容差保守包络）。
  示例 `atim_motor_f030`（三相互补+500ns 死区+斜坡，LQFP48/C8T7）。

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
- **L012 I2C 升级为单事务重复起始**：`embedded-hal-async::i2c::I2c::
  transaction` 从"逐 Operation 独立 START…STOP"改为整个事务单次
  START、方向切换处重复 START、末操作后 STOP（与 F030 经典分支及
  eha 契约对齐；`read`/`write`/`write_read` 走 trait 默认实现自动
  受益）。命令序列出处：MTDR 寄存器表对 CMD=100/101 的原文即
  "(Repeated)START"（UM V1.2 §23.8.11，页 554；V1.4 同款）；主机发/
  收流程以 CMD=100 发 SLA+W/SLA+R、收尾"发 STOP 或 Repeated START"
  （§23.4.4.1/§23.4.4.2，页 538-539）；厂商库
  `I2C_MasterSTART_Addr7b`/`I2C_MasterReSTART_Addr7b` 写入的 MTDR 字
  逐字相同、仅 BUSY 预检不同（cw32l012_lpi2c.c:476-493/504-517，
  h:186 `I2C_Cmd_Tx_RepSTART_Data_Rx_ACK`），写指针+重复起始读参考
  `I2C_MasterReceiveDataFromSlave`（c:1105-1200）。**顺带修正**：读
  方向地址字节由 `CMD_START_NACK`（CMD=101 期待 NACK）改为
  `CMD_START_ACK`（CMD=100）——v0.1 编码与上述手册/厂商库/源
  cw32l012-hal（`start_addr` 的 `expect_nack` 恒传 false）三处证据
  冲突；读长 >256 字节按 RXCOUNT 上限分块排队（DATA 字段 8 位，
  块间不重发地址，修复 v0.1 的 `(n-1) as u8` 静默截断）；主机侧
  ACK/NACK（STAR.TXNACK，§23.8.22）保持复位默认不动，与厂商库主机
  接收流程一致。新增 `i2c_transaction` 示例（LM75 写指针+重复起始
  读，⚠ 仅编译未上板）。29 项测试不减、`cargo fmt -p` 干净。

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
