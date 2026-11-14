//! 异步 ADC：四套完全不同的 IP，按家族 feature 多选一编译。
//!
//! - **L012 家族**（`cw32l012`，§25，ADC1/ADC2）：见 `l012` 分支，
//!   含内部通道（温度/1.2V 基准）+ 出厂校准 + DMA 序列。
//! - **经典家族**（`cw32f030`/`cw32f020`/`cw32f003`，§22，单 ADC）：见
//!   [`classic`] 分支。寄存器面与 L012 无一处同构（cr0/cr1/start/sqr/
//!   cr2/vth/vtl/trigger/result0..3/resultacc/ier/icr/isr vs L012 的
//!   cr/start/sqrcfr/sample/result0..7），实现独立。f003 与 f030/f020
//!   同款但无 DMA 硬件（CR1 无 DMAEN 位，尾部多 14 位扩展
//!   RESULT140..143 + ENABLE14，本驱动不触碰）。DMA 路径为单通道多次转换
//!   （[`ClassicAdc::read_dma`]，仅 f030/f020——f003 无 DMA，每次转换
//!   完成一个 DMA 请求），与 L012 的整序列 DMA（`read_sequence_dma`）
//!   形状不同。
//! - **8 槽扫描组**（`cw32l031`/`cw32l052`/`cw32l083`/`cw32r031`/
//!   `cw32w031`）：见 [`scan8`] 分支。与 f030 同为经典代配方（CR0.MODE/
//!   REF/CLK/SAM/BUF、CR1.CHMUX、EOC/EOS、READY），但序列面扩为
//!   SQR0（ENS[18:16] 3bit + 槽 0..3）+ SQR1（槽 4..7）、结果 8 个、
//!   0x0C 处无 SQR（空洞）——五家族 PAC 的 ADC 结构体逐寄存器同构
//!   （TRIGGER 位面 l052/l083 多几个源，与本驱动的软件触发路径无关）。
//!   有 ClassicDma 硬件但 ADC-DMA 联动本轮不做。
//! - **L010/L011 低配代组**（`cw32l010`/`cw32l011`）：见 [`low`] 分支。
//!   单实例 @0x4000_0000，寄存器面是 L012 的"近亲但不同构"：同为
//!   SAMPLE/SQRCFR 逐槽 4bit + ENS 序列 + 8 结果 + EOC/EOS + START 写 0
//!   复位游标，但偏移/位序全不同（START@0x08、SAMPLE@0x28、SQRCFR@0x2C、
//!   RESULT0@0x40、IER@0x74；CR 的 CLK[5:4]/ENS[8:6] 且 **BGREN/TSEN 在
//!   ADC_CR 内**——L012 在独立 BGR 控制器）。无 BGR 外设、无出厂校准字、
//!   无 READY 位、无 DMA；内部通道 TS(IN14)/BGR1.2(IN15) 存在（原始
//!   counts，40µs 级采集/稳定期）。采样档与 L012 同表但末档 390（非
//!   518）+ 15 比较周期。
//!
//! ```ignore
//! embassy_cw32::bind_interrupts!(struct Irqs {
//!     ADC1 => adc::Adc1Irq;          // L012
//!     ADC  => adc::ClassicAdcIrq;    // 经典组/扫描组/低配组（IRQ12）
//! });
//! ```
//!
//! 各分支的公共类型不混用：L012 用 `Adc`/`AdcChannel`/`AdcError` 等，
//! 经典组用 `ClassicAdc`/`ClassicChannel`/`ClassicAdcError` 等，扫描组
//! 用 `Scan8Adc`/`Scan8Channel`/…，低配组用 `LowAdc`/`LowChannel`/…
//! （均经 `pub use` 平铺到本模块）。
//!
//! 外部引脚均须先 [`crate::gpio::Pin::into_analog`] 转模拟。

// ================= L012 家族分支（内容不动，整体收进子模块）=================
#[cfg(feature = "cw32l012")]
mod l012 {
    //! L012 家族异步 ADC（§25）：单次与序列转换，EOC/EOS 中断驱动，
    //! ADC1/ADC2 双实例（`cw32l012` feature）。
    //!
    //! 通道映射（表 25-4，ADC1 视角）：IN0..7=PA0..PA7、IN8=PB0、IN9=PB1、
    //! IN10=PB10、IN11=PB2、IN12/13=DAC 输出（内部）、IN14=温度传感器
    //! （内部，仅 ADC1）、IN15=1.2V 基准（内部，仅 ADC1）。ADC2 的外部
    //! 通道映射不同（IN0=PA05 …，表 25-4）且无内部通道。
    //!
    //! 等待转换用 EOC/EOS 中断（等待时才开 IER 位，完毕/超时即压掉），
    //! 10ms 软超时（最长转换 ≈550 ADCCLK，即使 Div8@1MHz 也 <1ms）。
    use core::future::poll_fn;
    use core::sync::atomic::Ordering;
    use core::task::Poll;

    use embassy_sync::waitqueue::AtomicWaker;
    use embassy_time::{Duration, Timer, with_timeout};

    use crate::interrupt::{Binding, IrqHandler, interrupt};
    use crate::pac;

    // ================= 家族位常量面 =================
    // L012 手写 PAC 的 adc 常量与手册 §25 一致。IER 位面与 ISR 同布局
    // （§25.7.19 IER：bit0 EOCIE / bit1 EOSIE，PAC 未定义故此处补）。
    #[cfg(feature = "cw32l012")]
    pub(crate) mod ab {
        pub use crate::pac::adc as raw;
        /// EOC 中断使能（IER bit0）
        pub const IER_EOC: u32 = 1 << 0;
        /// EOS 中断使能（IER bit1）
        pub const IER_EOS: u32 = 1 << 1;
        /// EOS 触发 DMA 请求（IER bit6，PAC 有定义；序列 DMA 配套——
        /// 阻塞版 TESTING.md 坑 5：序列配 DMAEOS+ADC1_SEQ、单次配 DMAEOC+ADC1_SINGLE）
        pub const IER_DMAEOS: u32 = crate::pac::adc::IER_DMAEOS;
        /// BGR 控制器 BGREN（复位默认已开，BGR_CR=0x105）
        pub const BGREN: u32 = crate::pac::bgr::CR_BGREN;
        /// BGR 控制器 TSEN
        pub const TSEN: u32 = crate::pac::bgr::CR_TSEN;
    }

    use ab::raw;

    /// ADC 时钟分频（ADCCLK = PCLK / N）
    #[allow(missing_docs)]
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    #[repr(u32)]
    pub enum AdcClk {
        Div1 = 0,
        Div2 = 1,
        Div4 = 2,
        Div8 = 3,
    }

    /// 采样周期档位（表 25-2，单位 ADCCLK）
    #[allow(missing_docs)]
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    #[repr(u8)]
    pub enum SampleCycles {
        S6 = 0,
        S7 = 1,
        S9 = 2,
        S12 = 3,
        S18 = 4,
        S24 = 5,
        S30 = 6,
        S42 = 7,
        S54 = 8,
        S70 = 9,
        S102 = 10,
        S134 = 11,
        S166 = 12,
        S198 = 13,
        S262 = 14,
        S518 = 15,
    }

    /// 转换通道（表 25-4）
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub enum AdcChannel {
        /// 外部引脚 / 内部信号（0..=13，映射见模块文档）
        In(u8),
        /// 内置温度传感器（IN14，仅 ADC1）
        TempSensor,
        /// 1.2V 内核基准（IN15，仅 ADC1）
        Bgr1v2,
    }

    impl AdcChannel {
        fn code(self) -> u32 {
            match self {
                AdcChannel::In(n) => n as u32 & 0xF,
                AdcChannel::TempSensor => ab::raw::CH_TS,
                AdcChannel::Bgr1v2 => ab::raw::CH_BGR,
            }
        }
    }

    /// ADC 错误
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub enum AdcError {
        /// 转换未在 10ms 内完成（时钟门控没开 / 序列没启动？）
        Timeout,
        /// 校准值异常（0/0xFFFF，出厂未编程）
        BadCalibration,
        /// 内部通道在 ADC2 上不可用
        UnsupportedChannel,
        /// 温度传感器输出异常（原始值 <100 counts ≈ <0.08V，正常 ~0.78V）。
        ///
        /// 某板实测（2026-10-21 终审，`ts_probe` 判定矩阵 + 受控 POR）：
        /// 该芯片个体 TS 模拟块输出级无输出（硬件状态），非驱动问题。
        SensorDead,
    }

    /// per-实例 waker（0=ADC1，1=ADC2）
    static ADC_WAKER: [AtomicWaker; 2] = [const { AtomicWaker::new() }; 2];

    /// 异步 ADC
    pub struct Adc {
        regs: &'static crate::Adc,
        idx: usize,
    }

    impl Adc {
        /// 初始化 ADC1（表 25-4 全通道 + 内部通道可用）。
        pub fn new_adc1(
            regs: &'static crate::Adc,
            clk: AdcClk,
            _irqs: impl Binding<interrupt::ADC1>,
        ) -> Self {
            Self::new_inner(regs, 0, clk)
        }

        /// 初始化 ADC2（外部通道映射不同，无内部通道）。
        pub fn new_adc2(
            regs: &'static crate::Adc,
            clk: AdcClk,
            _irqs: impl Binding<interrupt::ADC2_DAC>,
        ) -> Self {
            Self::new_inner(regs, 1, clk)
        }

        fn new_inner(regs: &'static crate::Adc, idx: usize, clk: AdcClk) -> Self {
            // APBEN1.ADC 时钟（L012 带写保护键）
            let sysctrl = unsafe { &*(pac::SYSCTRL_BASE as *const crate::Sysctrl) };
            // 读改写:GTIM/ATIM/UART 等的门控同住 APBEN1,整字写会顶掉
            // 先初始化外设的使能位(rcc.rs set_bits 实机配方)
            sysctrl
                .apben1
                .modify(|v| pac::sysctrl::KEY | (v & 0xFFFF) | pac::sysctrl::APBEN1_ADC);

            // 模拟块复位 + 分频 + 上电（CR bit8 复位为 1 须保留，经 CR_RESET 置回）
            regs.cr.modify(|v| {
                (v & !raw::CR_CLK_MASK) | raw::CR_RESET | ((clk as u32) << raw::CR_CLK_POS)
            });
            regs.cr.modify(|v| v | raw::CR_EN);
            // 模拟电路初始化约 1µs（手册流程步骤 3）：构造函数里短阻塞，
            // 圈数按每圈 ~4 HCLK 周期估，各频率下约 4µs
            let n = crate::HCLK_HZ.load(Ordering::Relaxed) / 1_000_000;
            for _ in 0..n {
                core::hint::spin_loop();
            }

            match idx {
                0 => crate::interrupt::nvic_enable::<interrupt::ADC1>(),
                _ => crate::interrupt::nvic_enable::<interrupt::ADC2_DAC>(),
            }

            Adc { regs, idx }
        }

        /// 内部通道前置条件：确保 BGREN（及需要时的 TSEN）已使能并度过
        /// 稳定期。BGREN 复位默认已开（BGR_CR=0x105），常见路径零等待；
        /// 稳定期按保守 40µs（手册 §25.9 30µs + 裕量）异步等待。
        async fn ensure_internal_ready(&self, want_ts: bool) {
            let bgr = unsafe { &*(pac::BGR_BASE as *const pac::Bgr) };
            let cr = bgr.cr.read();
            let missing_bgr = cr & ab::BGREN == 0;
            let missing_ts = want_ts && cr & ab::TSEN == 0;
            if missing_bgr || missing_ts {
                let mask = ab::BGREN | if want_ts { ab::TSEN } else { 0 };
                bgr.cr.modify(|v| v | mask);
                Timer::after_micros(40).await;
            }
        }

        /// 单次转换一个通道（等 EOC，返回 0..4095）
        pub async fn read(
            &mut self,
            ch: AdcChannel,
            sample: SampleCycles,
        ) -> Result<u16, AdcError> {
            if self.idx == 1 && matches!(ch, AdcChannel::TempSensor | AdcChannel::Bgr1v2) {
                return Err(AdcError::UnsupportedChannel);
            }
            self.ensure_internal_ready(matches!(ch, AdcChannel::TempSensor))
                .await;
            let regs = self.regs;
            regs.start.write(0); // 停止并复位序列
            regs.sqrcfr.write(ch.code() << raw::SQRCH0_POS);
            regs.sample.write((sample as u32) << raw::SQRCH0_POS);
            regs.icr.write(raw::ICR_CLEAR_EOC_EOS);
            regs.cr.modify(|v| v & !raw::CR_ENS_MASK); // 单通道模式（ENS=0）
            regs.start.write(raw::START);
            match with_timeout(
                Duration::from_millis(10),
                wait_flag(regs, raw::ISR_EOC, ab::IER_EOC),
            )
            .await
            {
                Ok(()) => {}
                Err(_) => {
                    regs.start.write(0);
                    return Err(AdcError::Timeout);
                }
            }
            let r = (regs.result0.read() & 0xFFF) as u16;
            regs.icr.write(raw::ICR_CLEAR_EOC_EOS);
            Ok(r)
        }

        /// 序列扫描转换（ENS 模式，2..=8 通道，一次启动按序全部转换）
        ///
        /// * `chs` / `samples` — 通道与采样周期（等长）；结果按通道顺序写入
        ///   `out`（长度 ≥ chs.len()）。等待 EOS（整序列完成）。
        /// * 内部通道前置条件自动处理；ADC2 上不可含内部通道。
        pub async fn read_sequence(
            &mut self,
            chs: &[AdcChannel],
            samples: &[SampleCycles],
            out: &mut [u16],
        ) -> Result<(), AdcError> {
            let n = chs.len();
            if n == 0 || n > 8 || samples.len() != n || out.len() < n {
                return Err(AdcError::UnsupportedChannel);
            }
            let any_ts = chs.iter().any(|c| matches!(c, AdcChannel::TempSensor));
            if self.idx == 1 && any_ts {
                return Err(AdcError::UnsupportedChannel);
            }
            if self.idx == 1 && chs.iter().any(|c| matches!(c, AdcChannel::Bgr1v2)) {
                return Err(AdcError::UnsupportedChannel);
            }
            self.ensure_internal_ready(any_ts).await;
            let regs = self.regs;
            let mut sqrcfr = 0u32;
            let mut sample = 0u32;
            for (i, ch) in chs.iter().enumerate() {
                sqrcfr |= ch.code() << (4 * i);
                sample |= (samples[i] as u32) << (4 * i);
            }
            regs.start.write(0); // 复位序列
            regs.sqrcfr.write(sqrcfr);
            regs.sample.write(sample);
            regs.icr.write(raw::ICR_CLEAR_EOC_EOS);
            regs.cr
                .modify(|v| (v & !raw::CR_ENS_MASK) | ((n as u32 - 1) << raw::CR_ENS_POS));
            regs.start.write(raw::START);
            match with_timeout(
                Duration::from_millis(10),
                wait_flag(regs, raw::ISR_EOS, ab::IER_EOS),
            )
            .await
            {
                Ok(()) => {}
                Err(_) => {
                    regs.start.write(0);
                    regs.cr.modify(|v| v & !raw::CR_ENS_MASK);
                    return Err(AdcError::Timeout);
                }
            }
            let results = [
                regs.result0.read(),
                regs.result1.read(),
                regs.result2.read(),
                regs.result3.read(),
                regs.result4.read(),
                regs.result5.read(),
                regs.result6.read(),
                regs.result7.read(),
            ];
            for (slot, r) in out.iter_mut().zip(results.iter()) {
                *slot = (r & 0xFFF) as u16;
            }
            regs.icr.write(raw::ICR_CLEAR_EOC_EOS);
            regs.cr.modify(|v| v & !raw::CR_ENS_MASK); // 恢复单通道模式
            Ok(())
        }

        /// 序列扫描转换（DMA 版）：整段序列的结果由 DMA 硬件触发自动搬入缓冲，
        /// CPU 不逐通道取数（[`read_sequence`](Self::read_sequence) 的 DMA 版）
        ///
        /// * `dma` — DMA 控制器（内部占用**通道 1**，与阻塞版 dma_demo 给
        ///   ADC 的选路一致；HARDSRC 与通道号无关，任一通道皆可接 ADC 触发）
        /// * `chs` / `samples` — 通道与采样周期（等长，2..=8 通道）
        /// * `out` — 结果缓冲（长度 ≥ chs.len()，按通道顺序 0..4095）
        ///
        /// 配方（阻塞版 dma_demo P2 的序列推广 + 其 TESTING.md 坑 4/5）：
        ///
        /// - 触发链：序列完成（EOS）→ ADC 发 DMA 请求（`IER.DMAEOS`，bit6）→
        ///   HARDSRC = `ADC1_SEQ`/`ADC2_SEQ`（表 8-8：序列配 SEQ、单次配
        ///   SINGLE，两者别配错——阻塞版 TESTING.md 坑 5）。
        /// - 一次请求搬一整块：`CNT=1`、`REPEAT=n`（§8 数据块模型：一次硬件
        ///   请求搬一个数据块 = REPEAT 个元素；n ≤ 8 ≤ REPEAT 上限 15）。
        /// - 32 位宽 + 源自增：结果寄存器 RESULT0..7 @ +0x30..+0x4C 步进 4
        ///   字节（表 25-7），16 位宽的自增量 2 字节对不上——阻塞版 TESTING.md
        ///   坑 4「要么配 32bit，要么用 REPEAT 整块搬」，此处两者同用。
        ///
        /// 每个结果落在暂存 u32 的低 12 位（RESULT[11:0]），紧凑成 u16 写入
        /// `out`。⚠ 序列路径（DMAEOS + REPEAT 整块）在阻塞版只做过单次/连续
        /// 路径（DMAEOC + ADC1_SINGLE）的实机验证，首次上板请核对缓冲内容。
        pub async fn read_sequence_dma(
            &mut self,
            dma: &mut crate::dma::Dma,
            chs: &[AdcChannel],
            samples: &[SampleCycles],
            out: &mut [u16],
        ) -> Result<(), AdcError> {
            let n = chs.len();
            if n == 0 || n > 8 || samples.len() != n || out.len() < n {
                return Err(AdcError::UnsupportedChannel);
            }
            let any_ts = chs.iter().any(|c| matches!(c, AdcChannel::TempSensor));
            if self.idx == 1 && (any_ts || chs.iter().any(|c| matches!(c, AdcChannel::Bgr1v2))) {
                return Err(AdcError::UnsupportedChannel);
            }
            self.ensure_internal_ready(any_ts).await;
            let regs = self.regs;
            let mut sqrcfr = 0u32;
            let mut sample = 0u32;
            for (i, ch) in chs.iter().enumerate() {
                sqrcfr |= ch.code() << (4 * i);
                sample |= (samples[i] as u32) << (4 * i);
            }

            // 暂存缓冲：每个结果寄存器 32 位读出（低 12 位有效），DMA 整块
            // 写入后紧凑成 u16；栈上 [u32; 8] 天然 4 字节对齐
            let mut stage = [0u32; 8];

            // 通道配置：源 = RESULT0（随 SRCINC 逐结果寄存器推进）、CNT=1 +
            // REPEAT=n（一次 EOS 请求整块搬）、B32、BULK、硬件触发 ADCx_SEQ
            let (base, hs) = match self.idx {
                0 => (pac::ADC1_BASE, crate::pac::dma::hs::ADC1_SEQ),
                _ => (pac::ADC2_BASE, crate::pac::dma::hs::ADC2_SEQ),
            };
            let cfg = crate::dma::Config {
                src: base as u32 + 0x30, // RESULT0（表 25-7）
                dst: stage.as_mut_ptr() as u32,
                len: 1,
                repeat: n as u8,
                width: crate::dma::Width::B32,
                mode: crate::dma::Mode::Bulk,
                trigger: crate::dma::Trigger::Hardware(hs),
                src_inc: true,
                dst_inc: true,
                src_reload: false,
                dst_reload: false,
                restart: false,
            };
            let mut dch = dma.channel(1);
            dch.configure(&cfg); // EN=1，等 EOS 请求

            // 配序列（同 read_sequence 步骤）→ 开序列 DMA 请求 → 启动
            regs.start.write(0); // 复位序列
            regs.sqrcfr.write(sqrcfr);
            regs.sample.write(sample);
            regs.icr.write(raw::ICR_CLEAR_EOC_EOS);
            regs.cr
                .modify(|v| (v & !raw::CR_ENS_MASK) | ((n as u32 - 1) << raw::CR_ENS_POS));
            regs.ier.modify(|v| v | ab::IER_DMAEOS);
            regs.start.write(raw::START);

            // 序列时长上限：8 通道 × 最长采样 518+12 ADCCLK ≈ 4.3k ADCCLK，
            // 最低速 ADCCLK（PCLK/8@4MHz = 500kHz）下 ≈ 8.6ms，取 20ms 裕量
            //（read_sequence 的单通道 10ms 档对 8 通道贴边，这里放宽）。
            // DMA 侧错误（TE/中止）与超时一并归入 Timeout：序列结果没到位
            let r = match with_timeout(Duration::from_millis(20), dch.wait()).await {
                Ok(Ok(())) => Ok(()),
                _ => Err(AdcError::Timeout),
            };

            // 收尾：关序列 DMA 请求、停序列、恢复单通道模式、停 DMA 通道
            regs.ier.modify(|v| v & !ab::IER_DMAEOS);
            regs.start.write(0);
            regs.cr.modify(|v| v & !raw::CR_ENS_MASK);
            regs.icr.write(raw::ICR_CLEAR_EOC_EOS);
            dch.stop();

            r?;
            // DMA 已完成（TC 后读 SRAM；单核 M0+ 无缓存，阻塞版 dma_demo 同款直读）
            for (slot, v) in out.iter_mut().zip(stage.iter()) {
                *slot = (v & 0xFFF) as u16;
            }
            Ok(())
        }

        /// 测量 VDDA（经 1.2V 基准通道 + 出厂校准，返回 mV）
        ///
        /// 公式（手册 §25.8 步骤 13）：`VDD = cal × 4095 / RESULT0`，
        /// cal 为 0x0010_07D2 处的 16-bit 工厂校准值（单位 mV）。
        pub async fn vdda_mv(&mut self) -> Result<u32, AdcError> {
            let cal = unsafe { (pac::calib::VDDA_ADDR as *const u16).read_volatile() };
            if cal == 0 || cal == 0xFFFF {
                return Err(AdcError::BadCalibration);
            }
            let raw = self.read(AdcChannel::Bgr1v2, SampleCycles::S166).await?;
            if raw == 0 {
                return Err(AdcError::BadCalibration);
            }
            Ok(cal as u32 * 4095 / raw as u32)
        }

        /// 测量环境温度（返回毫摄氏度 m℃，经 VDDA 自校准）
        ///
        /// 公式（手册 §25.9）：
        /// `T = T0×0.5 + 0.095765×(Vref×AdcValue − 4.095×Trim)`
        /// 其中 T0@0x0010_07CD（0.5℃ 单位）、Trim@0x0010_07CE（16bit）。
        pub async fn temperature_mc(&mut self) -> Result<i32, AdcError> {
            let vdda = self.vdda_mv().await?;
            self.temperature_mc_with_vref(vdda).await
        }

        /// 指定 Vref（mV）测量温度（避免重复测 VDDA）
        pub async fn temperature_mc_with_vref(&mut self, vref_mv: u32) -> Result<i32, AdcError> {
            let t0 = unsafe { (pac::calib::TS_T0_ADDR as *const u8).read_volatile() } as i64;
            let trim = unsafe { (pac::calib::TS_TRIM_ADDR as *const u16).read_volatile() } as i64;
            if trim == 0 || trim == 0xFFFF {
                return Err(AdcError::BadCalibration);
            }
            let adc = self
                .read(AdcChannel::TempSensor, SampleCycles::S262)
                .await? as i64;
            // 传感器失效保护：正常室温下 TS 输出 ≈0.6~0.9V（raw 750~1100），
            // 读到 <100 counts 说明传感器无输出——拒绝计算，避免输出
            // "-271℃（绝对零度）"式假数据（见 [`AdcError::SensorDead`]）
            if adc < 100 {
                return Err(AdcError::SensorDead);
            }
            let diff = vref_mv as i64 * adc - 4095 * trim;
            let t_mc = t0 * 500 + 95_765 * diff / 1_000_000;
            Ok(t_mc as i32)
        }
    }

    /// 轮询等待 ISR 标志置位：等待时才开对应 IER 位，IRQ 唤醒后由本
    /// poll 复查（标志清除由调用方负责——读结果后经 ICR R1W0 清）。
    async fn wait_flag(regs: &'static crate::Adc, flag: u32, ier_bit: u32) {
        poll_fn(|cx| {
            ADC_WAKER[instance_idx(regs)].register(cx.waker());
            if regs.isr.read() & flag != 0 {
                Poll::Ready(())
            } else {
                regs.ier.modify(|v| v | ier_bit);
                Poll::Pending
            }
        })
        .await;
    }

    fn instance_idx(regs: &'static crate::Adc) -> usize {
        if regs as *const _ as usize == pac::ADC1_BASE {
            0
        } else {
            1
        }
    }

    // ================= 中断派发 =================

    fn on_irq(idx: usize, regs: &'static crate::Adc) {
        let isr = regs.isr.read();
        // 唤醒等待者并单发压掉 IER 位（下次 poll 会重新打开）；ISR 标志
        // 由驱动读结果后经 ICR 清除，此处不动
        if isr & raw::ISR_EOC != 0 {
            regs.ier.modify(|v| v & !ab::IER_EOC);
            ADC_WAKER[idx].wake();
        }
        if isr & raw::ISR_EOS != 0 {
            regs.ier.modify(|v| v & !ab::IER_EOS);
            ADC_WAKER[idx].wake();
        }
    }

    /// ADC1 中断标记（IRQ12）
    pub struct Adc1Irq;
    impl IrqHandler<interrupt::ADC1> for Adc1Irq {
        fn call() {
            on_irq(0, unsafe { &*(pac::ADC1_BASE as *const crate::Adc) });
        }
    }

    /// ADC2/DAC 中断标记（IRQ30）
    pub struct Adc2Irq;
    impl IrqHandler<interrupt::ADC2_DAC> for Adc2Irq {
        fn call() {
            on_irq(1, unsafe { &*(pac::ADC2_BASE as *const crate::Adc) });
        }
    }
}

#[cfg(feature = "cw32l012")]
pub use l012::*;

// ================= 经典组分支（f030 / f020 / f003，CW32x030 手册 §22）=================
// 12bit SAR ADC，1MSPS 顶速，单实例 @0x4001_2400（IRQ12）。寄存器面：
// cr0/cr1/start/sqr/cr2/vth/vtl/trigger/result0..3/resultacc/ier/icr/isr
// （f030/f020/f003 三 PAC 的 ADC 结构体与位段逐项比对同款；f020 的
// TRIGGER 无 ATIM 位——无高级定时器；f003 的 CR1 无 DMAEN 位、TRIGGER
// 位名为引脚组（PA10/PB32 等）且尾部多 14 位扩展 RESULT140..143 +
// ENABLE14（KEY 保护 WO）——14 位模式本驱动不触碰，仍为 12 位右对齐）。
//
// f003 的外部通道-引脚映射与 f030 不同（IN0=PB2、IN1=PA1、IN2=PA4、
// IN3=PA6、IN4=PA7、IN5..7=PC0..2、IN8=PB0、IN9=PB1、IN10=PB6、
// IN11=PB5、IN12=PB3，cw32-data af/cw32f003-analog.yaml），内部通道
// 同为 13=VDDA/3、14=温度、15=Vref1.2（classic-adc-scans.yaml
// CW32F003 facts）。f003 无 DMA 硬件（dma 模块未编译），read_dma 与
// TRIGGER 位面相关的 ClassicTrig 仅 f030/f020 可用。
//
// DMA 路径（[`ClassicAdc::read_dma`]）：CR1.DMAEN=转换完成发 DMA 请求
// （§22.13.2；厂商库 ADC_DMACmd），请求源 HARDSRC=0x0A（§8.8.4，厂商库
// DMA_HardTrig_ADC_TRANSCOMPLETE）——单通道 MODE=1 有界多次转换 +
// RESULT0 固定源搬运；序列模式的结果按位轮转 RESULT0..3、无按请求
// 选源机制，不做 DMA 推断。
//
// 配方出处（三源交叉核对，逐位段一致）：
// - 《CW32x030 用户手册》V2.5（whxy.com，sha256 与 cw32-hal2 sidecar 锁定
//   值一致）§22：22.4.1 时序（EN→READY ≤40µs、转换 = 采样 5..10 + 19
//   ADCCLK）、22.4.2 时钟/速度限制、22.5 各 MODE 流程、22.13 寄存器。
// - 厂商库 CW32F030_StandardPeripheralLib V2.2（whxy.com 直连下载，zip
//   内 cw32f030.h sha256 与 sidecar 锁定值一致）`cw32f030_adc.c/h`：
//   ADC_Init/ADC_Enable/ADC_SingleChCommon/ADC_SerialChCommon/
//   ADC_BufEnSingleCh/ADC_SoftwareStartConvCmd/ADC_ExtTrigCfg。
// - 生成 PAC cw32f030-pac / cw32f020-pac 的 `pub mod adc` 位段常量。
//
// ⚠ 全部路径为语义级实现（配方逐条对照上述来源），尚未实板验证；
//   首次上板请按 examples/adc_read_f030.rs 回归。
#[cfg(any(feature = "cw32f030", feature = "cw32f020", feature = "cw32f003"))]
mod classic {
    use core::future::poll_fn;
    use core::sync::atomic::Ordering;
    use core::task::Poll;

    use embassy_sync::waitqueue::AtomicWaker;
    use embassy_time::{Duration, with_timeout};

    use crate::interrupt::{Binding, IrqHandler, interrupt};
    use crate::pac;

    // ---- 位常量面（手册 §22.13 寄存器位段；与厂商头 cw32f030.h
    //      ADC_CR0_*/ADC_CR1_* 等 _Pos/_Msk 及生成 PAC 位段常量一致）----
    /// CR0.EN：模块使能（EN 0→1 后 ≤40µs ISR.READY 置位，§22.4.1）
    const CR0_EN: u32 = 1 << 0;
    /// CR0.MODE[3:1]：工作模式
    const CR0_MODE_MASK: u32 = 0xE;
    /// MODE=0：单通道单次转换（ADC_SingleChOneMode，§22.5.1）
    const MODE_SINGLE_ONE: u32 = 0 << 1;
    /// MODE=1：单通道多次转换——CR2.CNT+1 次，每次转换完成置 EOC、结果
    /// 落 RESULT0，全部完成置 EOA 且 START 自动清零（厂商库
    /// ADC_SingleChMoreMode，§22.5.2；DMA 路径 [`ClassicAdc::read_dma`]
    /// 用此模式：每次转换完成发一个 DMA 请求，源固定读 RESULT0）
    #[cfg(any(feature = "cw32f030", feature = "cw32f020"))]
    const MODE_SINGLE_MORE: u32 = 1 << 1;
    /// MODE=4：序列扫描——一次 START 按序转换整段序列，逐通道置 EOC、
    /// 序列完置 EOS 且 START 自动清零（ADC_SerialChScanMode，§22.5.5）
    const MODE_SERIAL_SCAN: u32 = 4 << 1;
    /// CR0.BGREN：BGR 1.2V 基准使能（读 Vref1.2 通道必需，厂商库
    /// ADC_BufEnSingleCh）
    const CR0_BGREN: u32 = 1 << 4;
    /// CR0.TSEN：内置温度传感器使能（读 TS 通道必需）
    const CR0_TSEN: u32 = 1 << 5;
    /// CR0.REF[7:6]：参考电压选择
    const CR0_REF_MASK: u32 = 0xC0;
    /// CR0.CLK[10:8]：ADCCLK = PCLK / N（N=1..128，表 22-1）
    const CR0_CLK_MASK: u32 = 0x700;
    /// CR0.SAM[12:11]：采样周期 5/6/8/10 ADCCLK（表 22-2）
    const CR0_SAM_MASK: u32 = 0x1800;
    /// CR0.BUF：输入跟随器（内部通道 VDDA/3、TS、Vref1.2 必需，厂商库
    /// 头文件「必须使用输入增益」注释 + ADC_BufEnSingleCh）
    const CR0_BUF: u32 = 1 << 13;
    /// CR1.CHMUX[3:0]：单通道模式待转换通道（表 22-5）
    const CR1_CHMUX_MASK: u32 = 0xF;
    /// CR1.ALIGN：对齐（0=右对齐，本驱动固定右对齐）
    const CR1_ALIGN: u32 = 1 << 6;
    /// CR1.DMAEN：转换完成发出 DMA 请求（§22.13.2 bit7；厂商库
    /// ADC_DMACmd/ADC_DMAEnable = REGBITS_SET/CLR(CR1, DMAEN_Msk)，
    /// cw32f030_adc.c:697-727。构造时恒关，[`ClassicAdc::read_dma`]
    /// 传输期间临时打开）。f003 无此位段（无 DMA 硬件）
    #[cfg(any(feature = "cw32f030", feature = "cw32f020"))]
    const CR1_DMAEN: u32 = 1 << 7;
    /// f003 无 DMAEN 位段（bit7 保留），按 0 处理保持调用面一致
    #[cfg(not(any(feature = "cw32f030", feature = "cw32f020")))]
    const fn dmaen_mask() -> u32 {
        0
    }
    #[cfg(any(feature = "cw32f030", feature = "cw32f020"))]
    const fn dmaen_mask() -> u32 {
        CR1_DMAEN
    }
    /// CR2.CNT[7:0]：MODE=1/MODE=5 的转换次数 −1（共 CNT+1 次，§22.13.3）
    #[cfg(any(feature = "cw32f030", feature = "cw32f020"))]
    const CR2_CNT_MASK: u32 = 0xFF;
    /// CR2.ACCEN：转换结果累加进 RESULTACC（bit8；厂商库 MODE=1 配置
    /// 强制置位——cw32f030_adc.c:408「Must be added」，照抄）
    #[cfg(any(feature = "cw32f030", feature = "cw32f020"))]
    const CR2_ACCEN: u32 = 1 << 8;
    /// CR2.ACCRST：写 1 清零 RESULTACC（bit9，R0W1 自清脉冲，§22.13.3）
    #[cfg(any(feature = "cw32f030", feature = "cw32f020"))]
    const CR2_ACCRST: u32 = 1 << 9;
    /// START.START：软件启动。转换完成后硬件自动清零（§22.5.1 时序图）
    const START_START: u32 = 1 << 0;
    /// SQR.ENS[17:16]：序列长度 1..=4（00=仅 SQR0 … 11=SQR0..SQR3，§22.5.4 表 22-6）
    const SQR_ENS_POS: u32 = 16;
    /// ISR/IER/ICR 公共位面：EOC=bit0（单次/每通道转换完）、EOS=bit1
    /// （整段序列完）。ICR 为 R1W0——写 0 清对应 ISR 标志、写 1 无效
    /// （§22.13.11，复位值 0x7F；厂商库 ADC_ClearITPendingBit 用
    /// REGBITS_CLR、中断例程直接 `CW_ADC->ICR = 0x00`）
    const ISR_EOC: u32 = 1 << 0;
    const ISR_EOS: u32 = 1 << 1;
    /// ISR.READY=bit7：模块启动完成电平标志（无 ICR 位，不可清）
    const ISR_READY: u32 = 1 << 7;
    const IER_EOC: u32 = 1 << 0;
    const IER_EOS: u32 = 1 << 1;
    /// TRIGGER[15:0]：外部触发源使能位面（§22.13.7；厂商库 ADC_ExtTrigCfg）
    const TRIGGER_ALL: u32 = 0xFFFF;

    /// ADC 时钟分频（ADCCLK = PCLK / N，§22.4.1 表 22-1）。
    ///
    /// 上限约束（§22.4.2 表 22-3）：VDDA/ExRef 2.7~5.5V 档 24MHz、
    /// 2.4~2.7V 12MHz、2~2.4V 4MHz、1.65~1.8V 500kHz；内部 2.5V 基准档
    /// 4MHz。复位默认 PCLK=8MHz 时 Div1=8MHz 已覆盖多数低压档，建议
    /// 按供电电压取档。
    #[allow(missing_docs)]
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    #[repr(u32)]
    pub enum ClassicClk {
        Div1 = 0,
        Div2 = 1,
        Div4 = 2,
        Div8 = 3,
        Div16 = 4,
        Div32 = 5,
        Div64 = 6,
        Div128 = 7,
    }

    impl ClassicClk {
        fn code(self) -> u32 {
            (self as u32) << 8
        }
    }

    /// 采样周期档位（§22.4.2 表 22-2，单位 ADCCLK；总转换时间 = 采样 + 19）
    #[allow(missing_docs)]
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    #[repr(u32)]
    pub enum ClassicSampleTime {
        S5 = 0,
        S6 = 1,
        S8 = 2,
        S10 = 3,
    }

    impl ClassicSampleTime {
        fn code(self) -> u32 {
            (self as u32) << 11
        }
    }

    /// 转换通道（§22.5.1 表 22-5，CR1.CHMUX / SQR.SQRy 4bit 编码）
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub enum ClassicChannel {
        /// 外部引脚通道 AIN0..=AIN12：
        /// 0..7=PA00..PA07、8..10=PB00..PB02、11=PB10、12=PB11
        In(u8),
        /// VDDA/3（AIN13；需输入跟随器，驱动自动开 CR0.BUF）
        VddaDiv3,
        /// 内置温度传感器（AIN14；需跟随器 + TSEN，自动开）
        TempSensor,
        /// BGR 1.2V 基准（AIN15；需跟随器 + BGREN，自动开）
        Bgr1v2,
    }

    impl ClassicChannel {
        fn code(self) -> Option<u32> {
            match self {
                ClassicChannel::In(n) if n <= 12 => Some(n as u32),
                ClassicChannel::In(_) => None,
                ClassicChannel::VddaDiv3 => Some(13),
                ClassicChannel::TempSensor => Some(14),
                ClassicChannel::Bgr1v2 => Some(15),
            }
        }
    }

    /// 参考电压（CR0.REF[7:6]，§22.4.2 + 厂商库 ADC_VrefSel）
    #[allow(missing_docs)]
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    #[repr(u32)]
    pub enum ClassicVref {
        /// BGR 1.5V（VDDA 1.8~2V 档）
        Bgr1p5 = 0,
        /// BGR 2.5V（厂商库 ADC_StructInit 默认）
        Bgr2p5 = 1,
        /// 外部基准 ExRef（PB00 引脚；须先把 PB00 `into_analog`，
        /// 厂商库 ADC_Init 对此档的处理）
        Extern1 = 2,
        /// VDDA（0..VDDA 满量程，最常用）
        Vdda = 3,
    }

    /// ADC 错误（经典组无 L012 那套内部通道出厂校准概念，故无
    /// SensorDead/BadCalibration）
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub enum ClassicAdcError {
        /// 转换未在 10ms 内完成（时钟门控没开 / ADCCLK 超频？）
        Timeout,
        /// 配置非法（外部通道号 >12 / 序列长度 0 或 >4 / out 缓冲不足）
        BadConfig,
    }

    /// 硬件触发源（TRIGGER[15:0] 位面，§22.13.7；厂商库 ADC_ExtTrigCfg）。
    /// 置位后对应外设的事件等效一次 START。本版驱动的转换等待流程只在
    /// 软件触发路径验证过语义，硬件触发为寄存器面直配——待板验证。
    ///
    /// 仅 f030/f020（位面同源）。f003 的 TRIGGER 位为引脚组命名
    /// （GTIM/PA10/PA32/…/PB74，cw32f003-pac adc::trigger），语义不同
    /// 不在此暴露。
    #[allow(missing_docs)]
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    #[cfg(any(feature = "cw32f030", feature = "cw32f020"))]
    pub enum ClassicTrig {
        /// ATIM（仅 `cw32f030`——f020 无高级定时器，TRIGGER 无 bit0）
        #[cfg(feature = "cw32f030")]
        Atim,
        Gtim1,
        Gtim2,
        Gtim3,
        Gtim4,
        Btim1,
        Btim2,
        Btim3,
        Uart1,
        Uart2,
        Uart3,
        Spi1,
        Spi2,
        I2c1,
        I2c2,
    }

    #[cfg(any(feature = "cw32f030", feature = "cw32f020"))]
    impl ClassicTrig {
        fn code(self) -> u32 {
            match self {
                #[cfg(feature = "cw32f030")]
                ClassicTrig::Atim => 1 << 0,
                ClassicTrig::Gtim1 => 1 << 1,
                ClassicTrig::Gtim2 => 1 << 2,
                ClassicTrig::Gtim3 => 1 << 3,
                ClassicTrig::Gtim4 => 1 << 4,
                ClassicTrig::Btim1 => 1 << 5,
                ClassicTrig::Btim2 => 1 << 6,
                ClassicTrig::Btim3 => 1 << 7,
                ClassicTrig::Uart1 => 1 << 8,
                ClassicTrig::Uart2 => 1 << 9,
                ClassicTrig::Uart3 => 1 << 10,
                ClassicTrig::Spi1 => 1 << 11,
                ClassicTrig::Spi2 => 1 << 12,
                ClassicTrig::I2c1 => 1 << 13,
                ClassicTrig::I2c2 => 1 << 14,
            }
        }
    }

    /// per-实例 waker（经典组单 ADC 实例）
    static ADC_WAKER: AtomicWaker = AtomicWaker::new();

    /// 异步 ADC（经典组）
    pub struct ClassicAdc {
        regs: &'static pac::ADC,
    }

    impl ClassicAdc {
        /// 初始化：开时钟 → 基础配置 → 使能等 READY → 挂 NVIC。
        ///
        /// * `clk` — ADCCLK 分频（ADCCLK=PCLK/N，注意 §22.4.2 电压域上限）
        /// * 默认参考电压 VDDA（可用 [`Self::set_vref`] 改）
        pub fn new(
            regs: &'static pac::ADC,
            clk: ClassicClk,
            _irqs: impl Binding<interrupt::ADC>,
        ) -> Self {
            // SYSCTRL.APBEN2.ADC（bit2）。经典家族 APBEN 无 KEY 写保护，
            // 读改写即可（厂商库 ADC_PortInit 直接 REGBITS_SET；lib.rs
            // 「便捷时钟门控」注释：F030 上板实测 KEY 写会顶掉使能位）
            let sysctrl = unsafe { &*(pac::SYSCTRL_BASE as *const crate::Sysctrl) };
            sysctrl.apben2.modify(|v| v | (1 << 2));

            // 基础配置（厂商库 ADC_Init + ADC_StructInit 配方：CR0 的
            // BUF/SAM/CLK/REF/TSEN/MODE 一把写，CR1 的 DMAEN/ALIGN）：
            // 分频按参数，参考 VDDA，采样 5CLK，单通道单次模式（MODE=0），
            // 跟随器/温度传感器/BGR 关（内部通道在 read 时按需开），
            // 右对齐、DMA 关
            regs.cr0.modify(|v| {
                (v & !(CR0_CLK_MASK | CR0_REF_MASK | CR0_SAM_MASK | CR0_MODE_MASK))
                    | clk.code()
                    | (ClassicVref::Vdda as u32) << 6
                    | ClassicSampleTime::S5.code()
            });
            // f003 的 CR1 无 DMAEN 位段（bit7 保留），清 0 无副作用
            regs.cr1
                .modify(|v| v & !(CR1_ALIGN | CR1_CHMUX_MASK | dmaen_mask()));

            // 使能 + 等 READY（§22.4.1 步骤 4/5；厂商库 ADC_Enable 同样
            // 忙等 ISR.READY）。EN 0→1 到 READY ≤40µs，这里给 ~1ms 上限
            //（构造函数是同步上下文，不能 await；超限不报错，首次 read
            // 会以 Timeout 暴露）
            regs.cr0.modify(|v| v | CR0_EN);
            let n = (crate::HCLK_HZ.load(Ordering::Relaxed) / 4_000).max(1);
            for _ in 0..n {
                if regs.isr.read() & ISR_READY != 0 {
                    break;
                }
                core::hint::spin_loop();
            }

            crate::interrupt::nvic_enable::<interrupt::ADC>();
            ClassicAdc { regs }
        }

        /// 切换参考电压（CR0.REF[7:6]；厂商库 ADC_SetVref）
        pub fn set_vref(&mut self, vref: ClassicVref) {
            self.regs
                .cr0
                .modify(|v| (v & !CR0_REF_MASK) | (vref as u32) << 6);
        }

        /// 使能外部硬件触发源（TRIGGER |= bit；厂商库 ADC_ExtTrigCfg）。
        /// 触发事件出现时等效一次 START。见 [`ClassicTrig`] 的待板验证注。
        #[cfg(any(feature = "cw32f030", feature = "cw32f020"))]
        pub fn enable_trigger(&mut self, src: ClassicTrig) {
            self.regs.trigger.modify(|v| v | src.code());
        }

        /// 关闭全部外部触发源（TRIGGER 清零，仅软件 START）
        pub fn disable_trigger(&mut self) {
            self.regs.trigger.modify(|v| v & !TRIGGER_ALL);
        }

        /// 内部通道前置条件：按厂商库 ADC_BufEnSingleCh（cw32f030_adc.c）
        /// 开跟随器；Vref1.2 另开 BGREN，TS 另开 TSEN。置位即粘滞（厂商
        /// 库从不回清，后续普通通道读取不受影响）
        fn apply_channel_analog(&self, ch: ClassicChannel) {
            let regs = self.regs;
            match ch {
                ClassicChannel::In(_) => {}
                ClassicChannel::VddaDiv3 => regs.cr0.modify(|v| v | CR0_BUF),
                ClassicChannel::TempSensor => regs.cr0.modify(|v| v | CR0_BUF | CR0_TSEN),
                ClassicChannel::Bgr1v2 => regs.cr0.modify(|v| v | CR0_BUF | CR0_BGREN),
            }
        }

        /// 单次转换一个通道（§22.5.1 MODE=0 单通道单次 + 软件触发），
        /// 等 EOC 异步唤醒，返回 0..4095（右对齐）。
        ///
        /// 配方（厂商库 ADC_SingleChOneModeCfg + adc_sgl_sw 例程）：
        /// 配 CHMUX/SAM → 清标志 → START=1 → 等 EOC → 读 RESULT0。
        pub async fn read(
            &mut self,
            ch: ClassicChannel,
            sample: ClassicSampleTime,
        ) -> Result<u16, ClassicAdcError> {
            let code = ch.code().ok_or(ClassicAdcError::BadConfig)?;
            let regs = self.regs;
            regs.start.write(0); // MODE=0 下 START 转换完自动清零，此处清残留保险
            self.apply_channel_analog(ch);
            regs.cr1.modify(|v| (v & !CR1_CHMUX_MASK) | code);
            regs.cr0.modify(|v| {
                (v & !(CR0_SAM_MASK | CR0_MODE_MASK)) | sample.code() | MODE_SINGLE_ONE
            });
            regs.icr.write(0); // R1W0：写 0 清全部标志
            regs.start.write(START_START);
            // 时长上限：采样 10 + 19 = 29 ADCCLK；最低速档 PCLK/128@8MHz
            // = 62.5kHz 时 ≈0.47ms，10ms 软超时裕量充足
            match with_timeout(Duration::from_millis(10), wait_flag(regs, ISR_EOC, IER_EOC)).await {
                Ok(()) => {}
                Err(_) => {
                    regs.start.write(0);
                    regs.ier.modify(|v| v & !IER_EOC);
                    regs.icr.write(0);
                    return Err(ClassicAdcError::Timeout);
                }
            }
            let r = (regs.result0.read() & 0xFFF) as u16;
            regs.icr.write(0);
            Ok(r)
        }

        /// 序列扫描转换（§22.5.5 MODE=4：一次 START 按序转换 SQR0..SQRn-1
        /// 各通道一次，逐通道置 EOC、整段完置 EOS 且 START 自动清零；
        /// 结果按传入顺序落 RESULT0..RESULT3），1..=4 通道。
        ///
        /// * `chs` — 通道序列（写入 SQR0..SQR3 位段）
        /// * `sample` — 采样周期（CR0.SAM 全局一档，无 per-通道位面）
        /// * `out` — 结果缓冲（长度 ≥ chs.len()，0..4095 右对齐）
        ///
        /// 配方（厂商库 ADC_SerialChScanModeCfg + adc_sqr_irq_sw 例程）：
        /// 配 SQR.ENS+SQRy → 清标志 → START=1 → 等 EOS → 读 RESULT0..3。
        pub async fn read_sequence(
            &mut self,
            chs: &[ClassicChannel],
            sample: ClassicSampleTime,
            out: &mut [u16],
        ) -> Result<(), ClassicAdcError> {
            let n = chs.len();
            if n == 0 || n > 4 || out.len() < n {
                return Err(ClassicAdcError::BadConfig);
            }
            // ENS = n-1；SQRy 位段各 4bit，从低位起按序排
            let mut sqr = (n as u32 - 1) << SQR_ENS_POS;
            for (i, ch) in chs.iter().enumerate() {
                let code = ch.code().ok_or(ClassicAdcError::BadConfig)?;
                sqr |= code << (4 * i);
                self.apply_channel_analog(*ch);
            }
            let regs = self.regs;
            regs.start.write(0);
            regs.cr0.modify(|v| {
                (v & !(CR0_SAM_MASK | CR0_MODE_MASK)) | sample.code() | MODE_SERIAL_SCAN
            });
            regs.sqr.write(sqr);
            regs.icr.write(0);
            regs.start.write(START_START);
            // 时长上限：4 通道 × 29 ADCCLK ≈ 116 ADCCLK，最低速档 ≈1.9ms
            match with_timeout(Duration::from_millis(10), wait_flag(regs, ISR_EOS, IER_EOS)).await {
                Ok(()) => {}
                Err(_) => {
                    // 序列中途超时不再硬停（EOS 到来后 START 自清，ADC 回
                    // 空闲），只压掉中断与标志并恢复单通道模式
                    regs.ier.modify(|v| v & !IER_EOS);
                    regs.icr.write(0);
                    regs.cr0.modify(|v| v & !CR0_MODE_MASK);
                    regs.sqr.write(0);
                    return Err(ClassicAdcError::Timeout);
                }
            }
            let results = [
                regs.result0.read(),
                regs.result1.read(),
                regs.result2.read(),
                regs.result3.read(),
            ];
            for (slot, r) in out.iter_mut().zip(results.iter()) {
                *slot = (r & 0xFFF) as u16;
            }
            regs.icr.write(0);
            // 恢复单通道单次模式 + ENS=0，避免影响下次 read()
            regs.cr0.modify(|v| v & !CR0_MODE_MASK);
            regs.sqr.write(0);
            Ok(())
        }

        /// 单通道多次采样 + DMA 搬运（MODE=1 + CR1.DMAEN）：对 `ch` 连续
        /// 转换 `buf.len()` 次，每次转换完成（EOC）由 ADC 硬件发出一个
        /// DMA 请求，DMA 通道把 RESULT0 搬进 `buf`，CPU 不逐个取数。
        ///
        /// * `dma` — 经典组 DMA 控制器（内部占用**通道 1**，同
        ///   [`ClassicDma::periph_to_mem`] 的选路）
        /// * `buf` — 结果缓冲（1..=256 个；12bit 右对齐 0..4095，
        ///   CR1.DISCARD=0 时 RESULT 高 4bit 恒 0）
        ///
        /// 配方出处（逐位段核对）：
        ///
        /// - **每次转换完成发一个 DMA 请求**：CR1.DMAEN（§22.13.2 bit7
        ///   「转换完成→DMA 请求」；厂商库 cw32f030_adc.c:697-727
        ///   `ADC_DMACmd`/`ADC_DMAEnable`）。请求源编码 0x0A=ADC 转换完成
        ///   （手册 §8.4.4 表 8-2/§8.8.4 p130/p140，二进制 001010 行；
        ///   厂商库 cw32f030_dma.h:117 `DMA_HardTrig_ADC_TRANSCOMPLETE
        ///   = 0x0AUL<<2`，即 [`crate::dma::hs::ADC_COMPLETE`]）。
        /// - **MODE=1 有界转换**：CR2.CNT+1 次（§22.5.2——每次转换完成
        ///   置 EOC、结果落 RESULT0；全部完成后置 EOA 且 START 自动清零，
        ///   恰好发出 buf.len() 个请求后 ADC 自停，DMA 侧 CNT 同值归零
        ///   出 TC，无溢出窗口）。厂商库 `ADC_SingleChMoreModeCfg`
        ///   （cw32f030_adc.c:405-412）：MODE=001 + `CR2_f.CNT` + 强制
        ///   ACCEN（「Must be added」）——此处照抄，另加 ACCRST 清历史
        ///   累加值（累加只影响 RESULTACC，不影响逐次 RESULT0）。
        /// - **源固定 RESULT0**（@+0x20，PAC 寄存器块；单通道模式每次
        ///   结果都落 RESULT0，B16 读低半字即 [11:0]）+ 目的自增，B16、
        ///   REPEAT=1（厂商 DMA_Init 恒写 `CNT=0x1_0000|CNT` 同款）。
        ///
        /// 序列模式（MODE=3/4）不走 DMA：结果按序列位置轮转落
        /// RESULT0..3，手册没有"按请求选结果寄存器"的机制，不做推断。
        // f003 无 DMA 硬件（PAC 无 DMA 外设、CR1 无 DMAEN 位），本路径不编译
        #[cfg(any(feature = "cw32f030", feature = "cw32f020"))]
        pub async fn read_dma(
            &mut self,
            dma: &mut crate::dma::ClassicDma,
            ch: ClassicChannel,
            sample: ClassicSampleTime,
            buf: &mut [u16],
        ) -> Result<(), ClassicAdcError> {
            let n = buf.len();
            if n == 0 || n > 256 {
                // CR2.CNT[7:0] 上限 = 256 次转换（CNT=255，§22.13.3）
                return Err(ClassicAdcError::BadConfig);
            }
            let code = ch.code().ok_or(ClassicAdcError::BadConfig)?;
            let regs = self.regs;

            // 先停住 ADC 并清残留标志，再武装 DMA（避免旧事件吃掉首格）
            regs.start.write(0);
            regs.icr.write(0);
            self.apply_channel_analog(ch);

            // DMA 通道 1：源 = RESULT0 固定（ADC_BASE+0x20）、目的自增、
            // B16、BLOCK、硬件触发 = ADC 转换完成（hs::ADC_COMPLETE=0x0A）
            let cfg = crate::dma::ClassicConfig::periph_to_mem(
                pac::ADC_BASE as u32 + 0x20,
                buf.as_mut_ptr() as u32,
                n as u16,
                crate::dma::ClassicWidth::B16,
                crate::dma::hs::ADC_COMPLETE,
            );
            let mut dch = dma.channel(1);
            dch.configure(&cfg); // EN=1，等转换完成请求

            // MODE=1 配方（同 read() 的单通道步骤 + CR2/CR1 扩展）：
            // CHMUX → MODE=001 + 采样周期 → CR2(ACCRST|ACCEN|CNT=n-1) →
            // 开 DMAEN → START
            regs.cr1.modify(|v| (v & !CR1_CHMUX_MASK) | code);
            regs.cr0.modify(|v| {
                (v & !(CR0_SAM_MASK | CR0_MODE_MASK)) | sample.code() | MODE_SINGLE_MORE
            });
            regs.cr2
                .write(CR2_ACCRST | CR2_ACCEN | ((n as u32 - 1) & CR2_CNT_MASK));
            regs.cr1.modify(|v| v | CR1_DMAEN);
            regs.start.write(START_START);

            // 时长上限：每转换 = 采样 10 + 19 = 29 ADCCLK；最低速档
            // PCLK/128@8MHz = 62.5kHz 时 ≈0.46ms/次，5ms/次 ≈10 倍裕量
            //（PCLK 降到 1MHz 仍 <1ms/次）。DMA 侧错误与超时一并归入
            // Timeout：结果没搬齐
            let r = match with_timeout(Duration::from_millis(10 + 5 * n as u64), dch.wait()).await {
                Ok(Ok(())) => Ok(()),
                _ => Err(ClassicAdcError::Timeout),
            };

            // 收尾：关 DMAEN、清 CR2、停 ADC 残留、恢复单通道单次、停 DMA
            regs.cr1.modify(|v| v & !CR1_DMAEN);
            regs.cr2.write(0);
            regs.start.write(0);
            regs.ier.modify(|v| v & !(IER_EOC | IER_EOS));
            regs.icr.write(0);
            regs.cr0.modify(|v| v & !CR0_MODE_MASK);
            dch.stop();

            r?;
            Ok(())
        }
    }

    /// 轮询等待 ISR 标志置位：等待时才开对应 IER 位，IRQ 唤醒后由本
    /// poll 复查（标志清除由调用方经 ICR 写 0 完成，中断里不动 ISR）
    async fn wait_flag(regs: &'static pac::ADC, flag: u32, ier_bit: u32) {
        poll_fn(|cx| {
            ADC_WAKER.register(cx.waker());
            if regs.isr.read() & flag != 0 {
                Poll::Ready(())
            } else {
                regs.ier.modify(|v| v | ier_bit);
                Poll::Pending
            }
        })
        .await;
    }

    // ================= 中断派发 =================

    fn on_irq(regs: &'static pac::ADC) {
        let isr = regs.isr.read();
        // 唤醒等待者并单发压掉 IER 位（下次 poll 会重新打开）；ISR 标志
        // 由驱动读结果后经 ICR 写 0 清除，此处不动
        if isr & ISR_EOC != 0 {
            regs.ier.modify(|v| v & !IER_EOC);
            ADC_WAKER.wake();
        }
        if isr & ISR_EOS != 0 {
            regs.ier.modify(|v| v & !IER_EOS);
            ADC_WAKER.wake();
        }
    }

    /// 经典组 ADC 中断标记（IRQ12，f030/f020 同号）
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub struct ClassicAdcIrq;
    impl IrqHandler<interrupt::ADC> for ClassicAdcIrq {
        fn call() {
            on_irq(unsafe { &*(pac::ADC_BASE as *const pac::ADC) });
        }
    }
}

#[cfg(any(feature = "cw32f030", feature = "cw32f020", feature = "cw32f003"))]
pub use classic::*;

// ================= 8 槽扫描组分支（l031/l052/l083/r031/w031）=================
// 经典代配方的 8 槽扩展版：单实例 @0x4001_2400（IRQ12，各家 PAC
// interrupt::ADC = 12）。寄存器面（cw32l031-pac src/lib.rs:136-180，
// 五家族逐寄存器同构——cw32l031/cw32r031/cw32w031 三份 adc 位段常量
// diff 为空，l052/l083 仅 TRIGGER 位面多 GTIM3/SPI2/I2C2/UART4-6 等
// 外设触发源，软件触发路径不涉及）：
// cr0(EN/MODE[3:1]/TSEN/REF[7:6]/CLK[10:8]/SAM[12:11]/BUF[13]/BIAS[15:14])
// /cr1(CHMUX/ALIGN/WDT*)/start(START/AUTOSTOP)/cr2/vth/vtl/trigger/
// result0..7/resultacc/ier/icr/isr(EOC..OVW+READY)/sqr0(SQR0..3+ENS[18:16]
// 3bit)/sqr1(SQR4..7)。
//
// 与 f030 classic 的差异点（同配方不同形状）：
// - 无 SQR@0x0C（空洞），序列拆 SQR0/SQR1 两寄存器、ENS 3bit（1..=8 槽）
// - CR1 无 DMAEN（bit7 无位段）；CR0 无 BGREN 位段（Vref1.2 档见
//   ClassicVref 同款 REF 编码；TS/BUF 位段在）
// - 结果 8 个；ICR 写 0 清 EOC..OVW（READY 无 ICR 位，不可清）
//
// 配方出处（与 classic 分支三源交叉核对同源）：CW32L031 用户手册 V1.6
// 表 22-5（AIN0..AIN12=PA0..PA7/PB0/PB1/PB2/PB10/PB11，通道映射经
// cw32-data af/cw32l031-analog.yaml 路由表核对）；模式编码 SINGLE=0/
// SCAN=4（cw32-hal2 cw32-data registers/adc_cw32l031_v1.yaml enum/Mode）；
// 电气上限（采样 5/6/8/10 + 19 比较周期、分电压域 0.5/2/4/12/24MHz）
// 与 classic 组同表（cw32-data electrical.yaml CW32L031/L052/L083/R031/
// W031 adc_limits——R031 供电 2.2..3.6V、W031 1.8..3.6V，分频档选择时
// 注意电压域）。R031/W031 的 PAC 复用 l031 寄存器版本
// （classic-adc-scans.yaml register_version=cw32l031_v1）。
//
// ⚠ 全部路径为语义级实现（配方逐条对照上述来源），尚未实板验证；
//   首次上板请按 examples/adc_read_l031.rs 回归。
#[cfg(any(
    feature = "cw32l031",
    feature = "cw32l052",
    feature = "cw32l083",
    feature = "cw32r031",
    feature = "cw32w031"
))]
mod scan8 {
    use core::future::poll_fn;
    use core::sync::atomic::Ordering;
    use core::task::Poll;

    use embassy_sync::waitqueue::AtomicWaker;
    use embassy_time::{Duration, with_timeout};

    use crate::interrupt::{Binding, IrqHandler, interrupt};
    use crate::pac;

    // ---- 位常量面（cw32l031-pac pub mod adc 位段常量；l052/l083/r031/
    //      w031 同名同位——四份 PAC 的 adc 模块逐常量比对，差异仅 TRIGGER
    //      的外设触发源位，下列本驱动用到的位全部同位）----
    /// CR0.EN：模块使能（EN 0→1 后 ≤40µs ISR.READY 置位，同 classic 组）
    const CR0_EN: u32 = 1 << 0;
    /// CR0.MODE[3:1]：工作模式（SINGLE=0 / CONTINUOUS=2 / SCAN=4）
    const CR0_MODE_MASK: u32 = 0xE;
    /// MODE=0：单通道单次转换（结果落 RESULT0）
    const MODE_SINGLE_ONE: u32 = 0 << 1;
    /// MODE=4：序列扫描——一次 START 按序转换 SQR0/SQR1 整段序列，逐通道
    /// 置 EOC、序列完置 EOS 且 START 自动清零（register yaml enum/Mode.SCAN）
    const MODE_SERIAL_SCAN: u32 = 4 << 1;
    /// CR0.TSEN：内置温度传感器使能（读 TS 通道必需）
    const CR0_TSEN: u32 = 1 << 5;
    /// CR0.REF[7:6]：参考电压选择（编码同 classic 组）
    const CR0_REF_MASK: u32 = 0xC0;
    /// CR0.CLK[10:8]：ADCCLK = PCLK / N（N=1..128）
    const CR0_CLK_MASK: u32 = 0x700;
    /// CR0.SAM[12:11]：采样周期 5/6/8/10 ADCCLK
    const CR0_SAM_MASK: u32 = 0x1800;
    /// CR0.BUF：输入跟随器（内部通道 VDDA/3、TS、Vref1.2 必需，同 classic）
    const CR0_BUF: u32 = 1 << 13;
    /// CR1.CHMUX[3:0]：单通道模式待转换通道（表 22-5 编码）
    const CR1_CHMUX_MASK: u32 = 0xF;
    /// CR1.ALIGN：对齐（0=右对齐，本驱动固定右对齐）
    const CR1_ALIGN: u32 = 1 << 6;
    /// START.START：软件启动。序列完成后硬件自动清零（SCAN 模式）
    const START_START: u32 = 1 << 0;
    /// SQR0.ENS[18:16]：序列长度 1..=8（00..=仅 SQR0 槽 0 … 111=SQR0..SQR1
    /// 全 8 槽；3bit——classic 组 f030 的 ENS[17:16] 只有 2bit，勿混）
    const SQR0_ENS_POS: u32 = 16;
    /// ISR/IER/ICR 公共位面：EOC=bit0（单次/每通道转换完）、EOS=bit1
    /// （整段序列完）。ICR 为 R1W0——写 0 清对应 ISR 标志（EOC..OVW），
    /// READY（bit7）只读无 ICR 位（cw32l031-pac adc::icr/isr 位段）
    const ISR_EOC: u32 = 1 << 0;
    const ISR_EOS: u32 = 1 << 1;
    const IER_EOC: u32 = 1 << 0;
    const IER_EOS: u32 = 1 << 1;
    /// TRIGGER[15:0]：外部触发源使能位面。本枚举只收五家族 PAC 同名同位
    /// 的公共源；l052/l083 额外的 GTIM3/GTIM4/SPI2/I2C2/UART4-6 与 l031
    /// 组的 DMA/GPIOA..F 位不在此暴露（各家位面不同，避免跨家族误配）
    const TRIGGER_ALL: u32 = 0xFFFF;

    /// ADC 时钟分频（ADCCLK = PCLK / N）。电压域上限同 classic 组
    /// （electrical.yaml adc_limits supply_bands：2.7V+ 档 24MHz …
    /// 1.65V 档 500kHz；R031 供电 2.2..3.6V）
    #[allow(missing_docs)]
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    #[repr(u32)]
    pub enum Scan8Clk {
        Div1 = 0,
        Div2 = 1,
        Div4 = 2,
        Div8 = 3,
        Div16 = 4,
        Div32 = 5,
        Div64 = 6,
        Div128 = 7,
    }

    impl Scan8Clk {
        fn code(self) -> u32 {
            (self as u32) << 8
        }
    }

    /// 采样周期档位（单位 ADCCLK；总转换时间 = 采样 + 19）
    #[allow(missing_docs)]
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    #[repr(u32)]
    pub enum Scan8SampleTime {
        S5 = 0,
        S6 = 1,
        S8 = 2,
        S10 = 3,
    }

    impl Scan8SampleTime {
        fn code(self) -> u32 {
            (self as u32) << 11
        }
    }

    /// 转换通道（表 22-5 编码，与 f030 classic 同映射：cw32-data
    /// af/cw32l031-analog.yaml 路由 IN0..7=PA0..PA7、IN8=PB0、IN9=PB1、
    /// IN10=PB2、IN11=PB10、IN12=PB11）
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub enum Scan8Channel {
        /// 外部引脚通道 AIN0..=AIN12
        In(u8),
        /// VDDA/3（AIN13；需输入跟随器，驱动自动开 CR0.BUF）
        VddaDiv3,
        /// 内置温度传感器（AIN14；需跟随器 + TSEN，自动开）
        TempSensor,
        /// BGR 1.2V 基准（AIN15；需跟随器，自动开——本组 CR0 无 BGREN
        /// 位段，基准使能不在 ADC 侧）
        Bgr1v2,
    }

    impl Scan8Channel {
        fn code(self) -> Option<u32> {
            match self {
                Scan8Channel::In(n) if n <= 12 => Some(n as u32),
                Scan8Channel::In(_) => None,
                Scan8Channel::VddaDiv3 => Some(13),
                Scan8Channel::TempSensor => Some(14),
                Scan8Channel::Bgr1v2 => Some(15),
            }
        }
    }

    /// 参考电压（CR0.REF[7:6]，编码同 classic 组）
    #[allow(missing_docs)]
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    #[repr(u32)]
    pub enum Scan8Vref {
        /// BGR 1.5V（VDDA 1.8~2V 档）
        Bgr1p5 = 0,
        /// BGR 2.5V
        Bgr2p5 = 1,
        /// 外部基准 ExRef（PB00 引脚，须先 `into_analog`）
        Extern1 = 2,
        /// VDDA（0..VDDA 满量程，最常用）
        Vdda = 3,
    }

    /// 硬件触发源（TRIGGER 位面；五家族公共子集，见模块注释）。
    /// 置位后对应外设的事件等效一次 START。软件触发路径之外为寄存器面
    /// 直配——待板验证。
    #[allow(missing_docs)]
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub enum Scan8Trig {
        Atim,
        Gtim1,
        Gtim2,
        Btim1,
        Btim2,
        Btim3,
        Uart1,
        Uart2,
        Uart3,
        Spi1,
        I2c1,
    }

    impl Scan8Trig {
        fn code(self) -> u32 {
            match self {
                // 位面据 cw32l031-pac adc::trigger（atim=0/gtim1=1/gtim2=2/
                // btim1=5/btim2=6/btim3=7/uart1=8/uart2=9/uart3=10/spi1=11/
                // i2c1=13）；l052/l083 的同名位经 diff 确认同位
                Scan8Trig::Atim => 1 << 0,
                Scan8Trig::Gtim1 => 1 << 1,
                Scan8Trig::Gtim2 => 1 << 2,
                Scan8Trig::Btim1 => 1 << 5,
                Scan8Trig::Btim2 => 1 << 6,
                Scan8Trig::Btim3 => 1 << 7,
                Scan8Trig::Uart1 => 1 << 8,
                Scan8Trig::Uart2 => 1 << 9,
                Scan8Trig::Uart3 => 1 << 10,
                Scan8Trig::Spi1 => 1 << 11,
                Scan8Trig::I2c1 => 1 << 13,
            }
        }
    }

    /// ADC 错误
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub enum Scan8AdcError {
        /// 转换未在 10ms 内完成（时钟门控没开 / ADCCLK 超频？）
        Timeout,
        /// 配置非法（外部通道号 >12 / 序列长度 0 或 >8 / out 缓冲不足）
        BadConfig,
    }

    /// per-实例 waker（本组单 ADC 实例）
    static ADC_WAKER: AtomicWaker = AtomicWaker::new();

    /// 异步 ADC（8 槽扫描组：l031/l052/l083/r031/w031）
    pub struct Scan8Adc {
        regs: &'static pac::ADC,
    }

    impl Scan8Adc {
        /// 初始化：开时钟 → 基础配置 → 使能等 READY → 挂 NVIC。
        ///
        /// * `clk` — ADCCLK 分频（ADCCLK=PCLK/N，注意电压域上限）
        /// * 默认参考电压 VDDA（可用 [`Self::set_vref`] 改）
        pub fn new(
            regs: &'static pac::ADC,
            clk: Scan8Clk,
            _irqs: impl Binding<interrupt::ADC>,
        ) -> Self {
            // SYSCTRL.APBEN2.ADC（bit2；cw32l031/cw32l052/cw32l083/
            // cw32r031/cw32w031 五 PAC apben2.adc 均为 bit2）。经典代
            // APBEN 无 KEY 写保护，读改写即可（同 classic 分支）
            let sysctrl = unsafe { &*(pac::SYSCTRL_BASE as *const crate::Sysctrl) };
            sysctrl.apben2.modify(|v| v | (1 << 2));

            // 基础配置（同 classic 组 ADC_Init 配方）：分频按参数、参考
            // VDDA、采样 5CLK、单通道单次模式；BUF/TSEN 关（内部通道在
            // read 时按需开）、右对齐；CR2 关累加（本组 CR2 位面同 classic）
            regs.cr1.modify(|v| v & !(CR1_ALIGN | CR1_CHMUX_MASK));
            regs.cr2.write(0);
            regs.cr0.modify(|v| {
                (v & !(CR0_CLK_MASK | CR0_REF_MASK | CR0_SAM_MASK | CR0_MODE_MASK))
                    | clk.code()
                    | (Scan8Vref::Vdda as u32) << 6
                    | Scan8SampleTime::S5.code()
            });

            // 使能 + 等 READY（同 classic 组：忙等 ISR.READY，~1ms 上限；
            // 超限不报错，首次 read 会以 Timeout 暴露）
            regs.cr0.modify(|v| v | CR0_EN);
            let n = (crate::HCLK_HZ.load(Ordering::Relaxed) / 4_000).max(1);
            for _ in 0..n {
                if regs.isr.read() & (1 << 7) != 0 {
                    break;
                }
                core::hint::spin_loop();
            }

            crate::interrupt::nvic_enable::<interrupt::ADC>();
            Scan8Adc { regs }
        }

        /// 切换参考电压（CR0.REF[7:6]）
        pub fn set_vref(&mut self, vref: Scan8Vref) {
            self.regs
                .cr0
                .modify(|v| (v & !CR0_REF_MASK) | (vref as u32) << 6);
        }

        /// 使能外部硬件触发源（TRIGGER |= bit）。见 [`Scan8Trig`] 注。
        pub fn enable_trigger(&mut self, src: Scan8Trig) {
            self.regs.trigger.modify(|v| v | src.code());
        }

        /// 关闭全部外部触发源（TRIGGER 清零，仅软件 START）
        pub fn disable_trigger(&mut self) {
            self.regs.trigger.modify(|v| v & !TRIGGER_ALL);
        }

        /// 内部通道前置条件：内部通道均需跟随器（同 classic 组
        /// ADC_BufEnSingleCh 配方），TS 另开 TSEN。置位即粘滞
        fn apply_channel_analog(&self, ch: Scan8Channel) {
            let regs = self.regs;
            match ch {
                Scan8Channel::In(_) => {}
                Scan8Channel::VddaDiv3 => regs.cr0.modify(|v| v | CR0_BUF),
                Scan8Channel::TempSensor => regs.cr0.modify(|v| v | CR0_BUF | CR0_TSEN),
                Scan8Channel::Bgr1v2 => regs.cr0.modify(|v| v | CR0_BUF),
            }
        }

        /// 单次转换一个通道（MODE=0 单通道单次 + 软件触发），等 EOC 异步
        /// 唤醒，返回 0..4095（右对齐）。
        ///
        /// 配方（同 classic 组 read）：配 CHMUX/SAM → 清标志 → START=1 →
        /// 等 EOC → 读 RESULT0。
        pub async fn read(
            &mut self,
            ch: Scan8Channel,
            sample: Scan8SampleTime,
        ) -> Result<u16, Scan8AdcError> {
            let code = ch.code().ok_or(Scan8AdcError::BadConfig)?;
            let regs = self.regs;
            regs.start.write(0); // 清残留（MODE=0 下转换完 START 自清）
            self.apply_channel_analog(ch);
            regs.cr1.modify(|v| (v & !CR1_CHMUX_MASK) | code);
            regs.cr0.modify(|v| {
                (v & !(CR0_SAM_MASK | CR0_MODE_MASK)) | sample.code() | MODE_SINGLE_ONE
            });
            regs.icr.write(0); // R1W0：写 0 清全部标志
            regs.start.write(START_START);
            // 时长上限：采样 10 + 19 = 29 ADCCLK；最低速档 PCLK/128@8MHz
            // = 62.5kHz 时 ≈0.47ms，10ms 软超时裕量充足
            match with_timeout(Duration::from_millis(10), wait_flag(regs, ISR_EOC, IER_EOC)).await {
                Ok(()) => {}
                Err(_) => {
                    regs.start.write(0);
                    regs.ier.modify(|v| v & !IER_EOC);
                    regs.icr.write(0);
                    return Err(Scan8AdcError::Timeout);
                }
            }
            let r = (regs.result0.read() & 0xFFF) as u16;
            regs.icr.write(0);
            Ok(r)
        }

        /// 序列扫描转换（MODE=4：一次 START 按序转换 SQR0/SQR1 各槽一次，
        /// 逐通道置 EOC、整段完置 EOS 且 START 自动清零；结果按传入顺序
        /// 落 RESULT0..RESULT7），1..=8 通道。
        ///
        /// * `chs` — 通道序列（1..=4 写 SQR0、5..=8 写 SQR0+SQR1）
        /// * `sample` — 采样周期（CR0.SAM 全局一档，无 per-通道位面）
        /// * `out` — 结果缓冲（长度 ≥ chs.len()，0..4095 右对齐）
        ///
        /// 配方（cw32-hal2 classic 后端同款）：SQR0.ENS+SQR0y/SQR1y →
        /// 清标志 → START=1 → 等 EOS → 读 RESULT0..n-1。
        pub async fn read_sequence(
            &mut self,
            chs: &[Scan8Channel],
            sample: Scan8SampleTime,
            out: &mut [u16],
        ) -> Result<(), Scan8AdcError> {
            let n = chs.len();
            if n == 0 || n > 8 || out.len() < n {
                return Err(Scan8AdcError::BadConfig);
            }
            // ENS = n-1；槽 0..3 → SQR0.SQRy、槽 4..7 → SQR1.SQRy（各 4bit）
            let mut sqr0 = (n as u32 - 1) << SQR0_ENS_POS;
            let mut sqr1 = 0u32;
            for (i, ch) in chs.iter().enumerate() {
                let code = ch.code().ok_or(Scan8AdcError::BadConfig)?;
                self.apply_channel_analog(*ch);
                if i < 4 {
                    sqr0 |= code << (4 * i);
                } else {
                    sqr1 |= code << (4 * (i - 4));
                }
            }
            let regs = self.regs;
            regs.start.write(0);
            regs.cr0.modify(|v| {
                (v & !(CR0_SAM_MASK | CR0_MODE_MASK)) | sample.code() | MODE_SERIAL_SCAN
            });
            regs.sqr0.write(sqr0);
            regs.sqr1.write(sqr1);
            regs.icr.write(0);
            regs.start.write(START_START);
            // 时长上限：8 通道 × 29 ADCCLK ≈ 232 ADCCLK，最低速档 ≈3.7ms
            match with_timeout(Duration::from_millis(10), wait_flag(regs, ISR_EOS, IER_EOS)).await {
                Ok(()) => {}
                Err(_) => {
                    // 同 classic 组：序列中途超时不再硬停（EOS 到来后 START
                    // 自清），只压掉中断与标志并恢复单通道模式
                    regs.ier.modify(|v| v & !IER_EOS);
                    regs.icr.write(0);
                    regs.cr0.modify(|v| v & !CR0_MODE_MASK);
                    regs.sqr0.write(0);
                    regs.sqr1.write(0);
                    return Err(Scan8AdcError::Timeout);
                }
            }
            let results = [
                regs.result0.read(),
                regs.result1.read(),
                regs.result2.read(),
                regs.result3.read(),
                regs.result4.read(),
                regs.result5.read(),
                regs.result6.read(),
                regs.result7.read(),
            ];
            for (slot, r) in out.iter_mut().zip(results.iter()) {
                *slot = (r & 0xFFF) as u16;
            }
            regs.icr.write(0);
            // 恢复单通道单次模式 + ENS=0，避免影响下次 read()
            regs.cr0.modify(|v| v & !CR0_MODE_MASK);
            regs.sqr0.write(0);
            regs.sqr1.write(0);
            Ok(())
        }
    }

    /// 轮询等待 ISR 标志置位：等待时才开对应 IER 位，IRQ 唤醒后由本
    /// poll 复查（标志清除由调用方经 ICR 写 0 完成，中断里不动 ISR）
    async fn wait_flag(regs: &'static pac::ADC, flag: u32, ier_bit: u32) {
        poll_fn(|cx| {
            ADC_WAKER.register(cx.waker());
            if regs.isr.read() & flag != 0 {
                Poll::Ready(())
            } else {
                regs.ier.modify(|v| v | ier_bit);
                Poll::Pending
            }
        })
        .await;
    }

    // ================= 中断派发 =================

    fn on_irq(regs: &'static pac::ADC) {
        let isr = regs.isr.read();
        // 唤醒等待者并单发压掉 IER 位（下次 poll 会重新打开）；ISR 标志
        // 由驱动读结果后经 ICR 写 0 清除，此处不动
        if isr & ISR_EOC != 0 {
            regs.ier.modify(|v| v & !IER_EOC);
            ADC_WAKER.wake();
        }
        if isr & ISR_EOS != 0 {
            regs.ier.modify(|v| v & !IER_EOS);
            ADC_WAKER.wake();
        }
    }

    /// 8 槽扫描组 ADC 中断标记（IRQ12，l031/l052/l083/r031/w031 同号）
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub struct Scan8AdcIrq;
    impl IrqHandler<interrupt::ADC> for Scan8AdcIrq {
        fn call() {
            on_irq(unsafe { &*(pac::ADC_BASE as *const pac::ADC) });
        }
    }
}

#[cfg(any(
    feature = "cw32l031",
    feature = "cw32l052",
    feature = "cw32l083",
    feature = "cw32r031",
    feature = "cw32w031"
))]
pub use scan8::*;

// ================= L010/L011 低配代组分支 =================
// 单实例 @0x4000_0000（IRQ12）。任务面的"L012 代组"只在门控布局意义
// 上成立（APBEN1.adc=bit0 + KEY 写保护）；ADC 本体是与 L012 不同的
// IP——厂商头文件逐寄存器比对（CW32L010 SDK V1.0.9 cw32l010.h:164-379
// ADC_TypeDef Size=128、cw32l011.h:167-388 同布局 vs cw32l012.h
// Size=124）：START@0x08（L012@0x04）、SAMPLE@0x28/SQRCFR@0x2C（L012
// @0x18/0x20）、RESULT0..7@0x40..0x5C（L012@0x30）、IER@0x74（L012
// @0x70）；CR 位序 EN/BGREN/TSEN/CONT/CLK[5:4]/ENS[8:6]（L012 为
// EN/CONT/CLK[3:2]/ENS[6:4]/SLAVE，且 BGREN/TSEN 在独立 BGR 控制器）。
// 生成 PAC 佐证：cw32l010-pac adc::cr 位段（bgren=1/tsen=2/cont=3/
// clk=4/ens=6，src/lib.rs:714-727）与 l012 手写 PAC 不同。
//
// 低配点（相对 L012）：无 BGR 外设/无出厂校准字（全头文件无校准地址，
// 原始 counts）、无 READY 位（ISR 仅 EOC/EOS/AWDL/AWDH，上电后直接
// max(2µs,15 ADCCLK) 等稳定）、无 DMA、ICR 为 R1W0 且 AWDL/AWDH 写 1
// 保留。内部通道 TS(IN14)/BGR1.2(IN15) 存在（TS/BGR 采集≥40µs——
// cw32-hal2 docs/adc-low-async.md 审计结论 + sequence 后端
// channel>=14 内部采集路径），外部通道 IN0..13（af/cw32l010-analog.yaml
// 路由 0..6=PA0..PA6、7..12=PB0..PB5、13=PB6；l011 为 0..7=PA0..PA7、
// 8=PB0、9=PB1、10..13=PA8..PA11）。采样档 6/7/9/12/18/24/30/42/54/
// 70/102/134/166/198/262/**390**（末档 390 非 L012 的 518）+ 15 比较周期
// （cw32-data electrical.yaml CW32L010/CW32L011 adc_limits）。
//
// 配方出处：START 写 0=停止+复位序列游标（L010 手册 §20.5.1/§20.12.2，
// 经 cw32-hal2 docs/adc-low-async.md 表格引用锁定）、CR 配 EN/CONT=0/
// ENS/CLK + TSEN/BGREN 粘滞置位（TS/BGR 前置）、序列 1..=8 槽逐槽
// SAMPLE/SQRCFR 4bit、等 EOC（单次）/EOS（序列）后读 RESULT0..7 & 0xFFF。
//
// ⚠ 全部路径为语义级实现，尚未实板验证；首次上板请核对 RAW 值合理性。
#[cfg(any(feature = "cw32l010", feature = "cw32l011"))]
mod low {
    use core::future::poll_fn;
    use core::sync::atomic::Ordering;
    use core::task::Poll;

    use embassy_sync::waitqueue::AtomicWaker;
    use embassy_time::{Duration, Timer, with_timeout};

    use crate::interrupt::{Binding, IrqHandler, interrupt};
    use crate::pac;

    // ---- 位常量面（cw32l010-pac / cw32l011-pac pub mod adc 位段常量，
    //      两 PAC 的 adc 模块除 TRIGGER 外逐常量一致）----
    /// CR.EN：模块使能
    const CR_EN: u32 = 1 << 0;
    /// CR.BGREN：BGR 1.2V 基准使能（CR 内位——L012 在独立 BGR 控制器）
    const CR_BGREN: u32 = 1 << 1;
    /// CR.TSEN：温度传感器使能
    const CR_TSEN: u32 = 1 << 2;
    /// CR.CONT：连续转换（本驱动恒 0，单次/单序列）
    const CR_CONT: u32 = 1 << 3;
    /// CR.CLK[5:4]：ADCCLK = PCLK / N（N=1/2/4/8，2bit——L012 为 CLK[3:2]）
    const CR_CLK_MASK: u32 = 0x30;
    /// CR.ENS[8:6]：序列长度 1..=8 槽（3bit）
    const CR_ENS_MASK: u32 = 0x1C0;
    /// START.START：软件启动；写 0 = 停止并复位序列游标（§20.12.2）
    const START_START: u32 = 1 << 0;
    /// ISR/IER 位面：EOC=bit0、EOS=bit1（另有 AWDL/AWDH bit2/3）
    const ISR_EOC: u32 = 1 << 0;
    const ISR_EOS: u32 = 1 << 1;
    const IER_EOC: u32 = 1 << 0;
    const IER_EOS: u32 = 1 << 1;
    /// ICR R1W0：写 0 清 EOC/EOS、写 1 保留 AWDL/AWDH（docs/adc-low-async.md
    /// 「Typed ICR writes clear EOC/EOS with zero and preserve AWDL/AWDH
    /// with one」）——与 classic 组「写 0 清全部」不同
    const ICR_CLEAR_EOC_EOS: u32 = !(ISR_EOC | ISR_EOS);

    /// ADC 时钟分频（ADCCLK = PCLK / N）。
    ///
    /// 电压域上限（electrical.yaml supply_bands）：L010 1.62V 档 4MHz …
    /// 2.8V 档 24MHz、3.3V 档 48MHz；L011 1.7V 档 6MHz 且 **ADCCLK 下限
    /// 4MHz**（manual 保守值，低于会返回 Timeout——降档请先降采样率预期）
    #[allow(missing_docs)]
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    #[repr(u32)]
    pub enum LowClk {
        Div1 = 0,
        Div2 = 1,
        Div4 = 2,
        Div8 = 3,
    }

    impl LowClk {
        fn code(self) -> u32 {
            (self as u32) << 4
        }
    }

    /// 采样周期档位（单位 ADCCLK；总转换时间 = 采样 + 15 比较周期）。
    /// 与 L012 同表但末档 390（非 518）
    #[allow(missing_docs)]
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    #[repr(u8)]
    pub enum LowSampleCycles {
        S6 = 0,
        S7 = 1,
        S9 = 2,
        S12 = 3,
        S18 = 4,
        S24 = 5,
        S30 = 6,
        S42 = 7,
        S54 = 8,
        S70 = 9,
        S102 = 10,
        S134 = 11,
        S166 = 12,
        S198 = 13,
        S262 = 14,
        S390 = 15,
    }

    /// 转换通道（外部 IN0..=IN13 + 内部 TS/BGR；引脚映射见模块注释，
    /// l010 与 l011 不同）
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub enum LowChannel {
        /// 外部引脚通道 IN0..=IN13（各家族引脚映射不同）
        In(u8),
        /// 内置温度传感器（IN14；原始 counts，无出厂校准字）
        TempSensor,
        /// BGR 1.2V 基准（IN15；原始 counts，仅量程校验用）
        Bgr1v2,
    }

    impl LowChannel {
        fn code(self) -> Option<u32> {
            match self {
                LowChannel::In(n) if n <= 13 => Some(n as u32),
                LowChannel::In(_) => None,
                LowChannel::TempSensor => Some(14),
                LowChannel::Bgr1v2 => Some(15),
            }
        }
    }

    /// ADC 错误
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub enum LowAdcError {
        /// 转换未在 10ms 内完成（时钟门控没开 / L011 的 ADCCLK≥4MHz
        /// 下限不满足？）
        Timeout,
        /// 配置非法（外部通道号 >13 / 序列长度 0 或 >8 / out 缓冲不足 /
        /// chs 与 samples 不等长）
        BadConfig,
    }

    /// per-实例 waker（单 ADC 实例）
    static ADC_WAKER: AtomicWaker = AtomicWaker::new();

    /// 异步 ADC（L010/L011）
    pub struct LowAdc {
        regs: &'static pac::ADC,
    }

    impl LowAdc {
        /// 初始化：开时钟（APBEN1.adc=bit0，KEY 写保护读改写）→ 基础
        /// 配置 → 上电稳定 → 挂 NVIC。
        ///
        /// * `clk` — ADCCLK 分频（ADCCLK=PCLK/N；L011 注意 ≥4MHz 下限）
        pub fn new(
            regs: &'static pac::ADC,
            clk: LowClk,
            _irqs: impl Binding<interrupt::ADC>,
        ) -> Self {
            // SYSCTRL.APBEN1.ADC（bit0；cw32l010/cw32l011 PAC apben1.adc）。
            // L012 代组 KEY 写保护：低 16 位为使能位面，写须带 0x5A5A 键
            // 且保留既有使能位（cw32l012-hal rcc.rs set_bits 实机配方：
            // reg.write(KEY | (read() & 0xFFFF) | bits)）
            let sysctrl = unsafe { &*(pac::SYSCTRL_BASE as *const crate::Sysctrl) };
            let v = sysctrl.apben1.read() & 0xFFFF;
            sysctrl.apben1.write(crate::SYSCTRL_KEY | v | (1 << 0));

            // 基础配置：EN=1、CONT=0、ENS=0（单通道）、分频按参数；
            // BGREN/TSEN 粘滞位保留（比较器等模拟块共用，见模块注释）
            regs.start.write(0); // 停止 + 复位序列游标
            regs.cr
                .modify(|v| (v & !(CR_CONT | CR_CLK_MASK | CR_ENS_MASK)) | CR_EN | clk.code());
            regs.ier.write(0);

            // 上电稳定 max(2µs, 15 ADCCLK)（datasheet tSTAB=15/fADC；手册
            // EN 后约 1µs——ADCCLK 最低 PCLK/8，15 ADCCLK ≤ 120 PCLK）。
            // 圈数按每圈 ~4 HCLK 周期估（同 l012 分支）
            let n = (crate::HCLK_HZ.load(Ordering::Relaxed) * 2 / 1_000_000).max(120) / 4;
            for _ in 0..n {
                core::hint::spin_loop();
            }

            crate::interrupt::nvic_enable::<interrupt::ADC>();
            LowAdc { regs }
        }

        /// 内部通道前置条件：按需置 CR.TSEN/CR.BGREN（粘滞；比较器共用
        /// BGREN 不回清）。BGR 通道或新开的 TS 需 50µs 稳定期（手册
        /// 30~40µs + 裕量，cw32-hal2 同款 50µs）
        async fn ensure_internal_ready(&self, want_ts: bool, want_bgr: bool) {
            let regs = self.regs;
            let cr = regs.cr.read();
            let missing_ts = want_ts && cr & CR_TSEN == 0;
            let missing_bgr = want_bgr && cr & CR_BGREN == 0;
            if missing_ts || missing_bgr {
                let mask = if want_ts { CR_TSEN } else { 0 } | if want_bgr { CR_BGREN } else { 0 };
                regs.cr.modify(|v| v | mask);
                Timer::after_micros(50).await;
            }
        }

        /// 单次转换一个通道（ENS=0 单通道 + 软件触发），等 EOC 异步唤醒，
        /// 返回 0..4095（右对齐原始 counts）。
        ///
        /// 配方：START=0（复位游标）→ 配 SQRCFR/SAMPLE 槽 0 → 清标志 →
        /// START=1 → 等 EOC → 读 RESULT0。
        pub async fn read(
            &mut self,
            ch: LowChannel,
            sample: LowSampleCycles,
        ) -> Result<u16, LowAdcError> {
            let code = ch.code().ok_or(LowAdcError::BadConfig)?;
            self.ensure_internal_ready(
                matches!(ch, LowChannel::TempSensor),
                matches!(ch, LowChannel::Bgr1v2),
            )
            .await;
            let regs = self.regs;
            regs.start.write(0); // 停止 + 复位序列游标
            regs.sqrcfr.write(code); // 槽 0
            regs.sample.write(sample as u32); // 槽 0
            regs.icr.write(ICR_CLEAR_EOC_EOS); // R1W0：写 0 清、写 1 保留
            regs.cr.modify(|v| (v & !CR_ENS_MASK) | CR_EN); // ENS=0 单通道
            regs.start.write(START_START);
            // 时长上限：采样 390 + 15 = 405 ADCCLK；最低速档 PCLK/8@8MHz
            // = 1MHz 时 ≈0.41ms，10ms 软超时裕量充足
            match with_timeout(Duration::from_millis(10), wait_flag(regs, ISR_EOC, IER_EOC)).await {
                Ok(()) => {}
                Err(_) => {
                    regs.start.write(0);
                    regs.ier.modify(|v| v & !IER_EOC);
                    regs.icr.write(ICR_CLEAR_EOC_EOS);
                    return Err(LowAdcError::Timeout);
                }
            }
            let r = (regs.result0.read() & 0xFFF) as u16;
            regs.icr.write(ICR_CLEAR_EOC_EOS);
            Ok(r)
        }

        /// 序列扫描转换（ENS 模式，1..=8 通道，一次启动按序全部转换）。
        ///
        /// * `chs` / `samples` — 通道与采样周期（等长；SAMPLE/SQRCFR 逐槽
        ///   4bit，本组支持 per-槽采样周期——classic 组没有）
        /// * `out` — 结果缓冲（长度 ≥ chs.len()，按通道顺序 0..4095）
        ///
        /// 配方：START=0 → 逐槽配 SQRCFR/SAMPLE → ENS=n-1 → 清标志 →
        /// START=1 → 等 EOS → 读 RESULT0..n-1。
        pub async fn read_sequence(
            &mut self,
            chs: &[LowChannel],
            samples: &[LowSampleCycles],
            out: &mut [u16],
        ) -> Result<(), LowAdcError> {
            let n = chs.len();
            if n == 0 || n > 8 || samples.len() != n || out.len() < n {
                return Err(LowAdcError::BadConfig);
            }
            let mut codes = 0u32;
            let mut sams = 0u32;
            let mut any_ts = false;
            let mut any_bgr = false;
            for (i, ch) in chs.iter().enumerate() {
                let code = ch.code().ok_or(LowAdcError::BadConfig)?;
                codes |= code << (4 * i);
                sams |= (samples[i] as u32) << (4 * i);
                any_ts |= matches!(ch, LowChannel::TempSensor);
                any_bgr |= matches!(ch, LowChannel::Bgr1v2);
            }
            self.ensure_internal_ready(any_ts, any_bgr).await;
            let regs = self.regs;
            regs.start.write(0); // 复位序列游标
            regs.sqrcfr.write(codes);
            regs.sample.write(sams);
            regs.icr.write(ICR_CLEAR_EOC_EOS);
            regs.cr
                .modify(|v| (v & !(CR_ENS_MASK | CR_CONT)) | CR_EN | ((n as u32 - 1) << 6));
            regs.start.write(START_START);
            // 时长上限：8 槽 × (390+15) = 3240 ADCCLK，最低速档 1MHz 时
            // ≈3.2ms，10ms 软超时贴边但够（L011 的 4MHz 下限档更快）
            match with_timeout(Duration::from_millis(10), wait_flag(regs, ISR_EOS, IER_EOS)).await {
                Ok(()) => {}
                Err(_) => {
                    regs.start.write(0);
                    regs.ier.modify(|v| v & !IER_EOS);
                    regs.icr.write(ICR_CLEAR_EOC_EOS);
                    regs.cr.modify(|v| v & !CR_ENS_MASK);
                    return Err(LowAdcError::Timeout);
                }
            }
            let results = [
                regs.result0.read(),
                regs.result1.read(),
                regs.result2.read(),
                regs.result3.read(),
                regs.result4.read(),
                regs.result5.read(),
                regs.result6.read(),
                regs.result7.read(),
            ];
            for (slot, r) in out.iter_mut().zip(results.iter()) {
                *slot = (r & 0xFFF) as u16;
            }
            regs.icr.write(ICR_CLEAR_EOC_EOS);
            regs.cr.modify(|v| v & !CR_ENS_MASK); // 恢复单通道模式
            Ok(())
        }
    }

    /// 轮询等待 ISR 标志置位：等待时才开对应 IER 位，IRQ 唤醒后由本
    /// poll 复查（标志清除由调用方经 ICR R1W0 完成，中断里不动 ISR）
    async fn wait_flag(regs: &'static pac::ADC, flag: u32, ier_bit: u32) {
        poll_fn(|cx| {
            ADC_WAKER.register(cx.waker());
            if regs.isr.read() & flag != 0 {
                Poll::Ready(())
            } else {
                regs.ier.modify(|v| v | ier_bit);
                Poll::Pending
            }
        })
        .await;
    }

    // ================= 中断派发 =================

    fn on_irq(regs: &'static pac::ADC) {
        let isr = regs.isr.read();
        // 唤醒等待者并单发压掉 IER 位（下次 poll 会重新打开）；ISR 标志
        // 由驱动读结果后经 ICR 清除，此处不动
        if isr & ISR_EOC != 0 {
            regs.ier.modify(|v| v & !IER_EOC);
            ADC_WAKER.wake();
        }
        if isr & ISR_EOS != 0 {
            regs.ier.modify(|v| v & !IER_EOS);
            ADC_WAKER.wake();
        }
    }

    /// L010/L011 ADC 中断标记（IRQ12）
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub struct LowAdcIrq;
    impl IrqHandler<interrupt::ADC> for LowAdcIrq {
        fn call() {
            on_irq(unsafe { &*(pac::ADC_BASE as *const pac::ADC) });
        }
    }
}

#[cfg(any(feature = "cw32l010", feature = "cw32l011"))]
pub use low::*;
