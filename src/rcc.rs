//! CW32F030 时钟配置（RCC/PLL）——v0.1 仅经典家族。
//!
//! ⚠ 三个 SYSCTRL 寄存器的 KEY 保护分野（2026-10-24 上板实测 + 厂商库
//! 实锤）：CR0 / CR1 / IER 带 KEY（写时高 16 位须 0x5A5A），AHBEN /
//! APBEN1 / APBEN2 无 KEY（带 KEY 写会顶掉使能位，见 uart/spi/i2c 的
//! 修复记录）。本模块所有 CR0/CR1 写都走 [`KEY`]。
//!
//! 复位默认时钟：HSI 48MHz ÷ ? = **实测 8MHz**（SysTick 计速），HSSI
//! DIV[12:11] 的确切分频表尚未从手册/实板标定——`hsi_div` 谨慎暴露。
//!
//! PLL 配方转录自厂商库 `RCC_PLL_Enable`（cw32f030_rcc.c:478-540）：
//! FREQIN 有效 4–24MHz（查表 {6,12,20,24}M 边界），MUL 2–12，
//! FREQOUT 查表 {18,24,36,48,72}M 边界。
//!
//! 典型用法（切换到 PLL，须注意 embassy 时间基准同步更新）：
//!
//! ```ignore
//! // HSI 48M → PLL: 48/2=24M × 8/3? —— 合法组合受 FREQOUT 上限约束,
//! // 例:24M × 2 = 48M(freqOutMsk 查表)。64M 需 HSE 晶振(8M×8)。
//! rcc::pll_enable(rcc::PllSource::Hsi, 24_000_000, 2)?;
//! rcc::sysclk_switch(rcc::Sysclk::Pll)?;
//! // 更新 embassy 时间基准(必须!否则 Timer 全部错拍):
//! embassy_cw32::set_frequencies(48_000_000, 48_000_000);
//! ```

use crate::pac;

/// SYSCTRL 写保护键（CR0/CR1/IER 专用）
pub const KEY: u32 = 0x5A5A << 16;

/// PLL 输入源（SYSCTRL.PLL.SOURCE[1:0]，厂商宏值）
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PllSource {
    /// HSI 经输入分频
    Hsi = 0,
    /// HSE 经输入分频
    Hse = 1,
}

/// 系统时钟源（SYSCTRL.CR0.SYSCLK[2:0]）
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Sysclk {
    /// HSI(经 HSI.DIV)
    Hsi = 0,
    /// HSE
    Hse = 1,
    /// PLL 输出
    Pll = 2,
    /// LSI
    Lsi = 3,
    /// LSE
    Lse = 4,
}

fn sysctrl() -> &'static crate::Sysctrl {
    unsafe { &*(pac::SYSCTRL_BASE as *const crate::Sysctrl) }
}

/// PLL 启动失败（STABLE 等待超时，0xFFFF 次读）
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PllTimeout;

/// 使能 PLL 并等待稳定（转录 RCC_PLL_Enable，cw32f030_rcc.c:478）。
///
/// * `freq_in` — PLL 输入频率（4–24MHz，超出查表边界取最近档）
/// * `mul` — 倍频 2–12
///
/// 返回 `Err(())` 为稳定超时（0xFFFF 次读 STABLE，厂商同款）。
pub fn pll_enable(source: PllSource, freq_in: u32, mul: u32) -> Result<(), PllTimeout> {
    assert!((2..=12).contains(&mul), "PLLMul 2..12");
    // FREQIN 查表：第一个大于 FreqIn 的边界索引（c:495-503）
    let freq_in_table = [6_000_000u32, 12_000_000, 20_000_000, 24_000_001];
    let mut freq_in_msk = None;
    for (i, edge) in freq_in_table.iter().enumerate() {
        if freq_in < *edge {
            freq_in_msk = Some(i as u32);
            break;
        }
    }
    let freq_in_msk = freq_in_msk.expect("FREQIN 超出 4–24MHz 支持范围");
    // FREQOUT 查表：FREQIN×MUL 的输出档（c:506-513）
    let freq_out_table = [
        18_000_000u32,
        24_000_000,
        36_000_000,
        48_000_000,
        72_000_001,
    ];
    let freq_out = freq_in.saturating_mul(mul);
    let mut freq_out_msk = freq_out_table.len() as u32 - 1;
    for (i, edge) in freq_out_table.iter().enumerate() {
        if freq_out < *edge {
            freq_out_msk = i as u32;
            break;
        }
    }
    // PLL 寄存器一把写：WAITCYCLE=16384 档（RCC_PLL_WAITCYCLE_16384）
    // + FREQOUT/MUL/FREQIN/SOURCE（c:515-520）
    let waitcycle_16384 = 0u32; // WAITCYCLE[14:12]=000 即 16384 周期档
    let s = sysctrl();
    s.pll.write(
        waitcycle_16384
            | (freq_out_msk << 9)
            | (mul << 4)
            | (freq_in_msk << 2)
            | ((source as u32) & 0x3),
    );
    // CR1 |= BYPASS | PLLEN（c:522）
    s.cr1.modify(|v| KEY | ((v | (1 << 2)) & !(KEY)));
    // 等 STABLE（c:524-527）
    let mut timeout = 0xffffu32;
    while s.pll.read() & (1 << 15) == 0 {
        timeout -= 1;
        if timeout == 0 {
            return Err(PllTimeout);
        }
    }
    Ok(())
}

/// 关闭 PLL（c:548-552）
pub fn pll_disable() {
    sysctrl().cr1.modify(|v| KEY | (v & !(1 << 2)));
}

/// 切换系统时钟源（转录 RCC_SysClk_Switch，c:568-576）。
///
/// 调用前须确保新时钟源已使能且稳定。返回 `false` 表示切换后回读
/// 不一致（切换失败）。
pub fn sysclk_switch(source: Sysclk) -> bool {
    let s = sysctrl();
    s.cr0
        .modify(|v| KEY | ((v & !(0x7)) | (source as u32 & 0x7)));
    (s.cr0.read() & 0x7) == (source as u32 & 0x7)
}

/// 更新 embassy 时间基准（切频后必须调用，否则 Timer 全部错拍）。
///
/// `hclk` 用于 SysTick（时间驱动按 tick↔systick 换算），`pclk` 用于
/// UART/SPI/I2C 波特率换算。
pub fn update_timebase(hclk: u32, pclk: u32) {
    crate::set_frequencies(hclk, pclk);
}
