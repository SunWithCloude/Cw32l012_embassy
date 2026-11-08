# embassy-cw32

[![license](https://img.shields.io/badge/license-MIT%20OR%20Apache--2.0-blue.svg)](#license)

**English** | [简体中文](README.zh-CN.md)

Multi-family embassy async HAL for Wuhan Xinyuan (CMSemicon) **CW32** Cortex-M0+ MCUs — **11 families**, selected by mutually exclusive Cargo features: `cw32l012` (default), `cw32l010`, `cw32l011`, `cw32f030` (CW32F030/A030), `cw32f020`, `cw32f003`, `cw32l031`, `cw32l052`, `cw32l083`, `cw32r031`, `cw32w031`.

Two hardware generations: the **L012 generation** (`cw32l010`/`cw32l011`/`cw32l012`) gates SYSCTRL writes behind a 0x5A5A key; the **classic generation** (the other 8 families) has no key (plain read-modify-write). Peripheral register sets differ between the generations too (UART error-flag bits, gate layout, and whole peripherals such as the I2C or ADC IP).

## Ecosystem

- [cw32l012-pac](https://github.com/SunWithCloude/Cw32l012_pac) — PAC for the default family, transcribed and checked line-by-line against the *CW32L012 User Manual* V1.4.
- [cw32-pacs](https://github.com/SunWithCloude/cw32-pacs) — PACs for the other 10 families, generated from the vendor SDK headers (one generator, same output style).
- [cw32-bsp](https://github.com/SunWithCloude/cw32-bsp) — generic BSPs for all 11 families: pin mapping tables plus `blink` / `uart_echo` project templates.

## What you get

| Module | Contents |
|--------|----------|
| `time_driver` | SysTick time driver for the embassy-time-driver 0.2 interface: 1 MHz tick, dynamic one-shot alarm (reprograms `RVR`), **no periodic wakeup while idle** — low-power friendly |
| `bind_interrupts!` | embassy-style compile-time interrupt binding (binding the wrong IRQ fails to compile); driver constructors enable their NVIC lines automatically |
| `gpio` | async `Input` (`wait_for_rising_edge` / `wait_for_falling_edge` / `wait_for_any_edge` / `wait_for_high` / `wait_for_low`) and `Output` (with the `embedded-hal` 1.0 traits), all families; one shared IRQ per port wakes per-pin wakers (l052/l083 shared vectors `GPIOC_GPIOD` / `GPIOE_GPIOF` dispatch the port pairs) |
| `uart` | async UART (`embedded-io-async`) on all 11 families: byte-wise `Uart` **plus `BufferedUart`** — background ring-buffered TX/RX (embassy-sync `Pipe`, 64 bytes each way), concurrent multi-task readers/writers, `flush` waits for TC. L012/F030 expose UART1/2/3, the remaining families a single UART1. L012 additionally `write_dma`/`read_dma` (semantics-level, pending on-hardware validation). Design absorbed from cw32-hal2, see `THIRD_PARTY.md` |
| `spi` | async SPI master (`SpiBus`, software chip-select) on all 11 families: L012 SPI1/2/3, F030 SPI1/2, single instance elsewhere; both generations share the R1W0 `ICR` discipline. L012 additionally `transfer_dma`/`write_dma` (semantics-level, pending on-hardware validation) |
| `i2c` | async I2C master (`embedded-hal-async::i2c::I2c`) on 10 of 11 families (L011 has no I2C hardware): L012 keeps its command-FIFO controller; the other nine use the classic 8051-style state-machine controller, whose transactions issue a single START with repeated-START between operations (F030 verified on board, remaining classic parts compile-level from per-PAC bit-plane comparison) |
| `timer` | (L012) GTIM1-4 single-channel edge-aligned PWM (`SetDutyCycle`, 0..=10000 full scale) with `wait_update()` for async update-event ticks; plain PWM needs no IRQ binding |
| `adc` | two IPs behind one module: L012 ADC1/2 async single/sequence conversion with internal channels (`vdda_mv()` via the 1.2 V BGR, `temperature_mc()`) and `read_sequence_dma()`; classic group (F030/F020, a completely different IP) separately implemented — async single/4-result sequence, internal channels VDDA/3 + temperature + 1.2 V BGR, software-triggered path |
| `rtc` | two IPs behind one module: L012 `Rtc` — BCD calendar read/write (`now`/`set_datetime`) plus async alarm (`wait_alarm`, the KEY unlock sequence wrapped in a critical section), LSE/LSI time base; classic group `ClassicRtc` (F030/F020/L031/L052/L083/R031/W031, a different IP — ACCESS/WINDOW running-access window instead of the WAIT bit, no prescaler/subsecond, wider DATE fields) with the same method surface and async alarm semantics. No RTC hardware on F003 |
| `wdt` | two generations: L012 `Iwdt`/`Wwdgt`; classic `ClassicIwdt`/`ClassicWwdgt` (F003/F020/F030/L031/L052/L083/R031/W031) — same key sequence and bit plane, but the vendor library requires start-before-config for IWDT (0xCCCC → unlock → CR/ARR → sync wait → 0xAAAA feed), the dedicated RC10K counts at a nominal 10 kHz per the vendor macro (datasheet electrical table says typical 8 kHz, 4–12 kHz over temperature), and gates live in APBEN1 without a key. Non-async thin wrappers with a feed-window solver (`WindowConfig::for_feed_interval` / `ClassicWindowConfig::for_feed_interval`) |
| `dac` | (L012) dual-channel 12-bit output (`set_ch1`/`set_ch2`/`set_both` + readback) with DMA waveform output — one-shot `write_dma` and `start_dma_loop`/`stop_dma` ring mode (GTIM TRGO as trigger); DAC→ADC internal loopback example included |
| `dma` | two generations: L012 `Dma` — 4 channels (IRQ9 `DMA12` = ch1/2, IRQ10 `DMA34` = ch3/4) with `mem_to_mem`/`periph_to_mem`/`mem_to_periph`; classic `ClassicDma` (F020/F030/L031/L052/L083/R031/W031) — 5 channels on F030/F020/L083, 4 on the L031 group, **no DMA hardware on F003/L010/L011** (F020 channels 4/5 have no NVIC line, polling only). Hardware trigger source tables (`pac::dma::hs`) per generation |
| `critical-section-impl` | PRIMASK implementation (on by default; one per firmware) |
| `defmt` | optional feature: driver error/config types (30+) derive `defmt::Format`; the `rtt_hello` example wires defmt-rtt end to end |

Twenty-nine runnable examples live in `examples/` (blink, UART echo/buffered/DMA, SPI loopback/DMA, I2C scan, ADC async/DMA/F030, DAC wave and DAC→ADC loop, RTC alarm and WDT demo on both the L012 and F030 generations, DMA mem2mem on both generations, timer PWM, RTT hello), each gated by its family feature in `Cargo.toml`.

## Getting started

This crate depends on sibling checkouts of its PAC crates: `cw32l012-pac` for the
default family, `cw32-pacs` for the other 10 families (each PAC sits at
`../cw32-pacs/crates/<family>-pac`). **Land the repos side by side**:

```
Desktop/
├── cw32l012-pac/       <- needed for the default `cw32l012` feature
├── cw32-pacs/          <- needed when building for one of the other 10 families
└── embassy-cw32/       <- this repo
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
    embassy_cw32::init(Config::default()); // reset default: 4 MHz (L012 gen) / 8 MHz (classic gen)
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

Build:

```bash
cargo build --target thumbv6m-none-eabi --release --features rt --example blink_async
```

`.cargo/config.toml` already passes `-Tlink.x`, which pulls in this crate's
`memory.x` and the `device.x` generated by the PAC's `rt` feature.

## Clock convention

The crate does not configure the clock tree. `Config { hclk_hz, pclk_hz }` only
*declares* the current frequencies so that SysTick and the peripheral baud-rate
math agree; `Config::default()` follows the reset defaults — **4 MHz** on the
L012 generation (HSI 96M/24) and **8 MHz** on the classic generation (HSI
48M/6). Firmware that changes clocks should do so with the register layer
first, then call `init` with the resulting frequencies.

## Interrupt binding

Every async peripheral asks for an `impl Binding<Irq>` proof, checked at compile
time:

```rust,ignore
embassy_cw32::bind_interrupts!(struct Irqs {
    UART1 => uart::Uart1Irq;
    GPIOA => gpio::GpioAIrq;
});

let mut u = Uart::new_uart1(p.uart1, tx, rx, 1 /* AF */, Irqs, Default::default());
```

Unbound IRQs fall back to an infinite-loop default handler through the
`PROVIDE(sym = DefaultHandler)` weak aliases in `device.x`.

## How the time driver works

SysTick on the M0+ has no compare register. This driver programs it as a
**one-shot alarm** (`RVR` = systicks until the next deadline, then write `CVR=0`
to restart the count). The interrupt dispatches the embassy-time queue and arms
the next alarm; deadlines beyond 24 bits degrade to the maximum period and are
re-checked on wrap. The 64-bit wrap arithmetic double-reads `COUNTFLAG` inside a
critical section, so there is no race.

Alarms less than ~100 systicks away may fire up to that much late — never early,
which is safe for embassy-time semantics. `now()` is monotonic-clamped
(`LAST_NOW`), removing the overshoot that the first tick after a `CVR=0`
reprogram otherwise produces.

> Hardware debugging note (2026-10-24): the first version stored the reprogram
> base as `now - (period - 1)`, assuming the counter restarts from 0 after
> `CVR=0`. In fact the next tick reloads `RVR = period - 1` and counts down, so
> `now()` oscillated inside one period and never reached the deadline — the LED
> stayed lit instead of blinking. Located by reading ODR/SysTick/RAM over
> probe-rs; fixed by storing `now` itself as the base.

## Roadmap (not yet supported)

- `timer` / `dac` on the non-L012 families (per-family register
  surfaces; classic-group ADC already landed for F030/F020, classic-group
  rtc/wdt landed for the seven classic families — semantics-level,
  pending on-hardware validation)
- ADC for the L031 group / F003 / L010 / L011 (yet another IP, not yet wired)
- I2C repeated-start on the L012 command-FIFO controller (the classic
  controller already runs single-START transactions)
- DMA-backed UART/SPI on the classic generation (L012 has it, semantics-level)
- runtime clock-tree configuration beyond the F030 `rcc` helper (async RCC API)
- `Output` settle helpers, GPIO low-power wakeup configuration

## Relationship to cw32l012-hal

Two parallel options, pick one:

| | cw32l012-hal | embassy-cw32 |
|---|---|---|
| Model | blocking + self-contained runtime | async / embassy |
| Dependencies | none | embassy ecosystem |
| Vector table | built in (`UART1_irq`, …) | cortex-m-rt + PAC `rt` (`UART1`, …) |

⚠ Do **not** enable both critical-section implementations
(`cw32l012-hal/critical-section-impl` and this crate's default) in one firmware —
the link will fail.

## Verification status

- `thumbv6m-none-eabi`: lib and examples build warning-free in CI, which checks
  all 11 family features exclusively plus a defmt-on job; the linked vector
  table was checked with `nm`/`objdump` (`__INTERRUPTS @ 0x40`,
  SysTick → time driver, UART1 → bound handler, the rest → `DefaultHandler`).
- `cargo test` passes on the host (layout / doc tests).
- **blink_async verified on a CW32L012 board (2026-10-24)** with probe-rs +
  ST-Link V2: PC13 toggles on a clean 1-second period driven by
  `Timer::after_millis(500)` (confirmed by polling ODR), and the SysTick alarm
  period measures 1,997,499 ≈ 2,000,000 systicks (500 ms at 4 MHz HCLK) — the
  async executor, time driver and task wakeup all work end to end.
- uart_echo_async was flashed and runs (no hardware fault), but the async
  loopback still needs a USB-serial adapter to measure.
- **buffered_echo verified on board (2026-10-24)**: the heartbeat LED proves
  the executor with two concurrent tasks (echo task parked on a pipe read);
  `ISER bit27 = 1` proves the v0.2 NVIC-enable fix; injecting a UART1 pend
  through `ISPR` via the debug probe clears `ICPR` with no lockup, exercising
  the full vector → `bind_interrupts!` → `buffered_irq` → critical-section
  path on hardware. Actual buffered data flow still needs a serial adapter.

## F030 family notes

- `--no-default-features --features "rt,cw32f030,critical-section-impl"` builds
  for CW32F030/A030 against the generated `cw32f030-pac` (cw32-pac-gen).
- Reset clock is **8 MHz** (HSI 48M/6) — `Config::default()` handles it.
- The SYSCTRL AHBEN/APBEN registers have **no key protection** on this
  generation (unlike L012's 0x5A5A-gated writes); gates use read-modify-write,
  while CR0/CR1/IER do carry the key. The `rcc` module (PLL setup transcribed
  from the vendor library) handles the keyed writes, and `set_frequencies`
  re-declares the clocks after a switch.
- The classic I2C is a different peripheral (8051-style state machine) with its
  own driver; `i2c_scan_f030` was verified on board (112/112 addresses NACK on
  an empty bus, register chain checked item by item).
- Verified on an LCKFB CW32F030C8T7: blink_async_f030 (500 ms heartbeat),
  UART1 init chain (PA08/PA09 AF2, BRRI=4 @115200), SPI1 (CR1=0x1E7C,
  AFRL=0x55500000).

## Related work

[fan2nd/cw32-hal2](https://github.com/fan2nd/cw32-hal2) (October 2026, earlier
than this crate) takes a whole-family approach: chiptool/metapac architecture
covering many CW32F/L/A parts, broad driver surface (UART DMA/LIN, SPI PIO,
deep timer coverage), GTIM hardware-compare time driver. This crate instead
builds on per-family PACs — the L012 one transcribed line-by-line against UM
V1.4, the rest generated from vendor SDK headers — with drivers checked against
each family's manual and on-hardware verification where a board is available.
See `THIRD_PARTY.md` for what was absorbed from it and under which license.

## License

MIT OR Apache-2.0, same as cw32l012-pac / cw32l012-hal.
