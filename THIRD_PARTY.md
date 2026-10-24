# 第三方来源声明

## fan2nd/cw32-hal2

- 来源：<https://github.com/fan2nd/cw32-hal2>
- 许可：MIT OR Apache-2.0（原仓库 `LICENSE-MIT` / `LICENSE-APACHE`，
  其 MIT 版权行为 "Embassy project contributors"——该项目的中断绑定与
  时间驱动设计本身改编自 Embassy 上游）
- 吸收范围（v0.2.0，2026-10-24）：**设计层面**——缓冲式 USART 的
  IER 单发屏蔽策略、R1W0 清标志纪律、写 TDR 前清 TC 防陈旧 flush。
  未逐行复制其代码：本 crate 的实现基于零依赖 PAC 寄存器原语与
  embassy-sync `Pipe` 重写。
- 上游溯源：该项目的 provenance 记录见其仓库 `NOTICE.md` 与
  `docs/upstream-file-provenance.json`。

## Embassy

- 来源：<https://github.com/embassy-rs/embassy>
- 许可：MIT OR Apache-2.0
- `bind_interrupts!` 宏与 `Binding` 模式为 Embassy 公开架构模式的
  独立实现（非复制）。
