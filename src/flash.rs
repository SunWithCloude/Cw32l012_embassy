//! 片上 FLASH 自编程：读 / 页擦除 / 编程——参数存储与校准数据落盘的底层。
//!
//! **L012 家族先行**（`cw32l012` feature，同 [`crate::rtc`]/[`crate::dma`] 的
//! 分支格局）；经典家族（f030/f020/f003/l031/l052/l083/r031/w031）的
//! FLASH 是**另一套 IP**（寄存器面不同，本轮未做），后续按 classic 分支
//! 接入——届时类型对齐本分支命名。
//!
//! 寄存器配方与阻塞版 HAL **同源**（`cw32l012-hal/src/flash.rs` + 其
//! `examples/flash_store.rs`，**实机验证过**：第 127 页掉电计数器跨复位
//! 累加，见其 `TESTING.md` flash_store 条目）；与厂商库
//! `cw32l012_flash.c`（标准外设库 V1.0.5）逐函数交叉核对。
//!
//! # 硬件事实（逐条出处）
//!
//! - **页参数**：64KB / 128 页 / 页 512 字节（厂商库 `FLASH_PAGES_SIZE=128`、
//!   `FLASH_BYTES_SIZE=0x10000`，cw32l012_flash.h:48-49；PAC
//!   `FLASH_SIZE=64*1024`）。擦写锁定按"**每 8 页一个 LOCKx 位**"分组
//!   （手册表 7-2，经 hal 注释；PAC `pagelock` 字段注），128 页 → LOCK0..15。
//! - **KEY 写保护**：CR1 / CR2 / PAGELOCK 的高 16 位必须为 `0x5A5A`，否则
//!   写入被丢弃（PAC `flash::KEY` 注 + 厂商库所有 `0x5A5A0000 |` 前缀）。
//! - **工作模式**：CR1[1:0]——0=读、1=编程、2=页擦除、3=片擦除（PAC
//!   `flash::CR1_MODE_MASK` 注；厂商 `FLASH_ErasePage` 或入 `0x02`、
//!   `FLASH_WriteBytes` 或入 `0x01`）。**MODE 必须"或"进现有 CR1**，写裸
//!   `KEY|MODE` 会静默失效——hal 上板实锤（TESTING.md flash_store 坑 2，
//!   厂商库同为 `CR1BAK | mode` 配方）。
//! - **配置时钟**：擦写前必须使能 `SYSCTRL.AHBEN.FLASH`（bit1，§7.4 经
//!   hal 注释）。hal 实测：缺这个时钟时 CR1/CR2/PAGELOCK 的写**全被静默
//!   丢弃**，ISR 一片干净、擦除/编程变空操作（TESTING.md flash_store
//!   坑 1）——[`Flash::new`] 自动开。
//! - **三条硬性约束**（hal 模块头注，违反会置错误标志或静默失败）：
//!   1. 编程**只能 1→0**：目标区域必须已是 0xFF（擦除态），否则 `ISR.PROG`；
//!   2. 页必须先解锁（`PAGELOCK.LOCKx=1`），否则 `ISR.PAGELOCK`；
//!   3. **不能碰 PC 所在页**：硬件保护，`ISR.PC` 且操作失败。
//! - **CACHE/FETCH**：擦写期间必须关（否则 `ISR.CACHEON`）——本模块进入
//!   擦写前自动关、结束自动恢复。
//! - **忙等待**：`ISR.BUSY`（ISR[5]）。页擦除典型 3~5ms（hal wait_idle
//!   注释）。**BUSY 没有中断可用**——IER 只有 PROG/PAGELOCK/PC 三个错误
//!   中断位（厂商 cw32l012_flash.h:106-108），所以擦/写只能**忙等**，
//!   无法做成 embassy 异步 future。
//! - **编程宽度**：32 位字（4 字节对齐）与 16 位半字（2 字节对齐）均受
//!   支持（厂商 `FLASH_WriteWords` / `FLASH_WriteHalfWords`，两函数分别
//!   校验 `addr&3` / `addr&1`）。**字路径上板验证过**（hal flash_store
//!   编程 3 个字跨复位回读一致）；半字路径为厂商库同配方、未单独上板。
//!   厂商另有字节写入口（`FLASH_WriteBytes`，无对齐要求），本驱动不用
//!   ——统一收敛到半字粒度。
//! - **同页约束**：一次编程的起止必须落在**同一页**（厂商
//!   `FLASH_WriteBytes` 注释原文"待写入起始地址和结束地址需要位于同一个
//!   页面以内"）——[`Flash::program_slice`] 强制校验，跨页请按页分次。
//!
//! # 写 FLASH 期间取指/执行的约束（安全注释）
//!
//! 厂商库的 `FLASH_ErasePage` / `FLASH_WriteBytes` / `FLASH_WriteWords`
//! 全是**运行在 FLASH 里**的普通 C 函数：不拷 RAM、不关中断、原地
//! busy-wait `ISR.BUSY`（cw32l012_flash.c:232-255/304-329）——即擦写期间
//! 的取指/读数由硬件停顿到 BUSY 清零，不需要"RAM 执行"或手工关中断；
//! hal 的 embassy 移植亦按此上板通过（TESTING.md flash_store ✅）。
//! 硬件唯一的禁区是 **PC 所在页**（`ISR.PC`）：擦除/编程自己正取指的页
//! 会操作失败并置标志；其余代码页不在保护范围——**误擦代码区是调用方的
//! 责任**，本模块用 `unsafe` 把这层责任显式交给调用方（见下"# 为什么
//! API 是 unsafe"）。擦写操作本身经 `critical_section` 只保护 KEY 寄存器
//! 的读改写窗口（同 [`crate::rtc`] 纪律），忙等不占临界区，中断照常响应
//! （ISR 取指若落在擦写页之外不受影响；落在擦写页则停顿到操作完成）。
//!
//! # 为什么 API 是 unsafe
//!
//! 擦除会**不可恢复地**清掉一页。硬件只挡"PC 所在页"这一种自毁，挡不住
//! 擦掉别的在用页（常量表、bootloader 参数、另一任务正在读的数据）——
//! 这层检查无法在库里可靠完成（链接脚本符号 `_etext`/`__etext` 依赖
//! 用户链接布局，库层引用会让自备链接脚本的固件链接失败），所以擦/写
//! 入口一律 `unsafe`，由调用方保证目标页空闲。
//!
//! # 阻塞性披露（embassy 纪律，同 [`crate::rtc`])
//!
//! 页擦除忙等典型 3~5ms、每个编程单元（半字/字）微秒级——调用期间**当前
//! 任务阻塞**，执行器其它任务停摆（ erase 在 main 里做一次时影响有限，
//! 但别在心跳时序敏感场景擦页）。BUSY 无中断（见上）， embassy 化
//! 无从谈起；本模块按"构造短阻塞 + 操作短阻塞"披露使用。
//!
//! # 用法
//!
//! ```ignore
//! use embassy_cw32::flash::Flash;
//!
//! let mut flash = Flash::new(p.flashctrl); // 自动开 AHBEN.FLASH 时钟
//!
//! // 读：直接内存读（无需实例，FLASH 地址空间就是内存映射）
//! let magic = Flash::read_u32(0x0000_FE00)?;
//!
//! // 第 127 页（0xFE00..0xFFFF）存一条记录：擦 → 写 → 回读
//! # let buf = [0u8; 20];
//! unsafe {
//!     flash.erase_page(127)?;
//!     flash.program_slice(0x0000_FE00, &buf)?; // 4 对齐 + 4 倍数 → 纯字路径
//! }
//! ```
//!
//! ⚠ 读保护等级非 0（[`Flash::security_level`]）时 ISP/SWD 读写受限；
//! 本驱动不提供降级接口（`FLASH_SetReadOutLevel` 会触发整片擦除+自动
//! 复位，厂商库 cw32l012_flash.c:135-147，风险操作不代劳）。整片擦除
//! （CR1 MODE=3）同理未提供——量产场景才用，误触代价太大。

// ================= L012 家族分支 =================
#[cfg(feature = "cw32l012")]
mod l012 {
    //! L012 FLASH 自编程实现（§7.5 / §7.6 经 hal 注释；位定义 §7.10）。
    //!
    //! 寄存器组 `Flashctrl`（表 7-5，基地址 0x4002_2000）：
    //! CR1@0x00（KEY + MODE[1:0]）、CR2@0x04（WAIT/FETCH/CACHE）、
    //! PAGELOCK@0x08（KEY + LOCK0..15）、IER@0x20、ISR@0x24（RO）、
    //! ICR@0x28（**R1W0：写 0 清对应位**，厂商 `CW_FLASH->ICR = 0x00`
    //! 清全部 + hal `icr.write(!ISR_ERRORS)` 定向清）。

    use crate::pac;
    use crate::pac::flash as fc;

    /// FLASH 页大小（字节）：512（厂商 `FLASH_PAGES_SIZE=128` 页 × 手册
    /// 表 7-1/7-2 经 hal 注释；厂商库按 `Page_Number * 512` 寻址）
    pub const PAGE_SIZE: u32 = 512;
    /// 总页数（64KB / 512B = 128，厂商 `FLASH_PAGES_SIZE`）
    pub const PAGES: u32 = 128;
    /// FLASH 总容量（字节，厂商 `FLASH_BYTES_SIZE=0x10000`）
    pub const FLASH_SIZE: u32 = 64 * 1024;

    /// 忙等上限（spin 轮数）：页擦除典型 3~5ms，4MHz 下 hal 同款给足
    /// 200 万轮（cw32l012-hal/src/flash.rs wait_idle）；时钟拉高后该上限
    /// 只会更宽（轮数不变、周期变短）。
    const BUSY_POLL: u32 = 2_000_000;

    /// FLASH 错误
    ///
    /// 命名与阻塞版 `cw32l012-hal/src/flash.rs` 的 `FlashError` 对齐
    /// （`AddrRange`/`CrossPage` 为 embassy 版新增拆分）。
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub enum FlashError {
        /// 页号越界（>= [`PAGES`] = 128）
        BadPage,
        /// 地址区间越过 FLASH 末尾（0x0001_0000）
        AddrRange,
        /// 一次编程跨页（厂商库要求起止同页，见模块文档）
        CrossPage,
        /// 地址未按半字（2 字节）对齐
        BadAlign,
        /// 等待 `ISR.BUSY` 清零超时
        Timeout,
        /// 目标区域不是全 0xFF（`ISR.PROG`：编程只能 1→0）
        NotErased,
        /// 目标页被锁定（`ISR.PAGELOCK`）
        PageLocked,
        /// 擦写地址落在 PC 所在页（`ISR.PC`，硬件保护）
        PcPage,
        /// 擦写时 CACHE/FETCH 未关（`ISR.CACHEON`；本模块自动关，见到
        /// 即状态被外部改动）
        CacheOn,
        /// 擦写区域含安全运行库（`ISR.SDKERR`）
        SdkRegion,
    }

    /// FLASH 控制器（单实例，独占 `pac::Flashctrl`）
    ///
    /// 擦/写方法要求 `&mut self`：从 Rust 所有权层面排除同一实例的并发
    /// 擦写；读操作是关联函数（FLASH 地址空间即内存映射，无需控制器状态）。
    pub struct Flash {
        regs: &'static pac::Flashctrl,
    }

    impl Flash {
        /// 绑定 FLASH 控制器，并**使能 FLASH 配置时钟**（`AHBEN.FLASH`
        /// bit1，§7.4；阻塞版 `Flash::new(regs, &mut rcc)` 的
        /// `rcc.enable_flash_clock()` 步骤内联——本 crate 不做 Rcc 抽象）。
        ///
        /// 这一步不能省（hal 上板实锤，TESTING.md flash_store 坑 1）：没有
        /// 这个时钟时 CR1/CR2/PAGELOCK 的写入会被静默丢弃——擦除和编程
        /// 变成空操作，且 `ISR` 不报任何错误。
        pub fn new(regs: &'static pac::Flashctrl) -> Self {
            // KEY 占高 16 位，AHBEN 只有低 16 位有效——同 rtc.rs 的
            // APBEN2 写规范：读改写 + KEY，掩掉读出的高半字
            let sysctrl = unsafe { &*(pac::SYSCTRL_BASE as *const crate::Sysctrl) };
            critical_section::with(|_| {
                sysctrl
                    .ahben
                    .modify(|v| pac::sysctrl::KEY | (v & 0xFFFF) | pac::sysctrl::AHBEN_FLASH);
            });
            Flash { regs }
        }

        /// 读保护等级（CR1.SECURITY[6:5]，RO）：0=Level0 未保护 … 3=Level3
        /// （厂商 `FLASH_GetReadOutLevel`，cw32l012_flash.c:119-122）
        pub fn security_level(&self) -> u8 {
            ((self.regs.cr1.read() & fc::CR1_SECURITY_MASK) >> fc::CR1_SECURITY_POS) as u8
        }

        /// 擦写是否进行中（`ISR.BUSY`，ISR[5]）
        pub fn is_busy(&self) -> bool {
            self.regs.isr.read() & fc::ISR_BUSY != 0
        }

        /// 地址所在页号（0..=127）
        pub const fn page_of(addr: u32) -> u32 {
            addr / PAGE_SIZE
        }

        // ---- 读（关联函数：FLASH 是内存映射的，读无需解锁/无需实例）----
        //
        // 与阻塞版的差异：hal 的读入口是"安全函数内裸解引用任意整数地址"
        // （cw32l012-hal/src/flash.rs read_u32/read），这在 Rust 语义上是
        // 漏洞（未校验地址就解引用）；本驱动改为**范围校验后解引用**并
        // 返回 `Result`——0..0x10000 恒为映射的 FLASH 空间（L012 全家族
        // 64KB，厂商 FLASH_BYTES_SIZE 定值），校验通过后 volatile 读才
        // 声音安全。不 panic：越界/未对齐一律 Err。

        /// 读一个字节（任意 FLASH 地址）
        pub fn read_u8(addr: u32) -> Result<u8, FlashError> {
            if addr >= FLASH_SIZE {
                return Err(FlashError::AddrRange);
            }
            // SAFETY: 范围校验已保证地址落在恒映射的 FLASH 空间内
            Ok(unsafe { core::ptr::read_volatile(addr as *const u8) })
        }

        /// 读一个字（须 4 字节对齐，M0+ 对齐访问约束）
        pub fn read_u32(addr: u32) -> Result<u32, FlashError> {
            if addr >= FLASH_SIZE {
                return Err(FlashError::AddrRange);
            }
            if addr % 4 != 0 {
                return Err(FlashError::BadAlign);
            }
            // SAFETY: 同 read_u8；对齐已校验（volatile u32 读须自然对齐）
            Ok(unsafe { core::ptr::read_volatile(addr as *const u32) })
        }

        /// 读一片字节到缓冲（逐字节 volatile 读，不限对齐）
        pub fn read_slice(addr: u32, out: &mut [u8]) -> Result<(), FlashError> {
            if out.is_empty() {
                return Ok(());
            }
            let end = addr as u64 + out.len() as u64;
            if end > FLASH_SIZE as u64 {
                return Err(FlashError::AddrRange);
            }
            for (i, b) in out.iter_mut().enumerate() {
                // SAFETY: 先整段校验，逐地址必落在 FLASH 空间内
                unsafe { *b = core::ptr::read_volatile(addr.wrapping_add(i as u32) as *const u8) };
            }
            Ok(())
        }

        // ---- 擦写（unsafe：见模块文档"为什么 API 是 unsafe"）----

        /// 页擦除：整页（512B）恢复为 0xFF
        ///
        /// 流程（§7.5 经 hal 注释；厂商 `FLASH_ErasePage`
        /// cw32l012_flash.c:232-255 逐行同配方）：等 BUSY → 清标志 →
        /// CR1"或"入页擦模式（MODE=2）→ 解锁本页（PAGELOCK 每 8 页一位）
        /// → **向页内任意地址写任意数据触发擦除**（厂商用字节写，hal
        /// 实测字节写触发、32 位写也能触发但没必要冒险）→ 等 BUSY →
        /// 回锁 → 恢复 CR1/CR2 → 查错误标志。
        ///
        /// 解锁/回锁为**每操作自动**进行（阻塞版同款选择）；调用方无需
        /// 也不能手工管 PAGELOCK。
        ///
        /// ⚠ 阻塞忙等典型 3~5ms（BUSY_POLL 上限 ~2s@4MHz）。
        ///
        /// # Safety
        /// 该页内容会被**不可恢复地**清除。调用方必须保证：页号有效
        /// （< [`PAGES`]，越界返回 [`FlashError::BadPage`]）且该页不存放
        /// 任何在用数据（代码、常量表、其它任务正在读的记录——硬件只挡
        /// PC 所在页，`ISR.PC`）。
        pub unsafe fn erase_page(&mut self, page: u32) -> Result<(), FlashError> {
            if page >= PAGES {
                return Err(FlashError::BadPage);
            }
            self.wait_idle()?;
            let addr = page * PAGE_SIZE;
            let saved = self.begin(fc::MODE_PAGE_ERASE);
            self.set_lock(page, true);
            // 擦除触发：向页内首地址做一次字节写（厂商
            // `*((volatile uint8_t*)(Page_Number*512)) = 0x00`，hal 实测同款）
            unsafe { core::ptr::write_volatile(addr as *mut u8, 0x00) };
            let busy = self.wait_idle();
            self.set_lock(page, false);
            self.end(saved);
            busy?;
            self.check()
        }

        /// 编程：把 `data` 依半字/字粒度写入 `addr` 起的目标区
        ///
        /// - **对齐**：`addr` 必须半字（2 字节）对齐（厂商
        ///   `FLASH_WriteHalfWords` 校验 `addr&1`），否则
        ///   [`FlashError::BadAlign`]；`data` 长度任意——尾部/引子按半字
        ///   写，奇数末字节以 **0xFF 填充**凑满半字（编程只能 1→0，写 0xFF
        ///   即"不写"，对目标位无副作用）。
        /// - **同页**：写入范围（含 0xFF 填充字节）不得跨页（厂商库
        ///   `FLASH_WriteBytes` 注释），否则 [`FlashError::CrossPage`]；
        ///   整体越过 FLASH 末尾报 [`FlashError::AddrRange`]。
        /// - **前置**：目标区域必须已擦除（全 0xFF），否则 `ISR.PROG` →
        ///   [`FlashError::NotErased`]（编程只能 1→0）。
        /// - **路径选择**：`addr` 4 字节对齐且 `data` 长度为 4 的倍数时走
        ///   **纯字路径**（hal 上板验证路径）；否则半字引子/收尾 + 字主体
        ///   （半字编程为厂商库 `FLASH_WriteHalfWords` 同配方，未单独上板）。
        ///
        /// 每个编程单元写完即等 BUSY + 查错误（hal 同款），中途出错立即
        /// 停止并回锁恢复。
        ///
        /// # Safety
        /// 调用方必须保证 `addr` 起的目标区域：已擦除且可写（页已腾空）、
        /// 不含在用数据（代码/常量/他任务共享数据）。页锁定由本方法自动
        /// 解锁/回锁。
        pub unsafe fn program_slice(&mut self, addr: u32, data: &[u8]) -> Result<(), FlashError> {
            if data.is_empty() {
                return Ok(());
            }
            if addr % 2 != 0 {
                return Err(FlashError::BadAlign);
            }
            // 末端 = addr + len + (len 为奇数时的填充字节)，须同时满足：
            // 不越过 FLASH 末尾、不跨页（厂商同页约束）
            let len = data.len() as u64;
            let end = addr as u64 + len + (len & 1);
            if end > FLASH_SIZE as u64 {
                return Err(FlashError::AddrRange);
            }
            let page_end = (Self::page_of(addr) as u64 + 1) * PAGE_SIZE as u64;
            if end > page_end {
                return Err(FlashError::CrossPage);
            }

            self.wait_idle()?;
            let page = Self::page_of(addr);
            let saved = self.begin(fc::MODE_PROGRAM);
            self.set_lock(page, true);
            let mut res = Ok(());

            // 半字引子：addr 落在 4 字节边界的后半（addr%4==2）且还有 ≥2
            // 字节时，先写一个半字把主体推到字对齐
            let (lead, words, tail) = Self::split_chunks(addr, data.len());
            let mut i = 0usize;
            if lead == 1 {
                let hw = u16::from_le_bytes([data[0], data[1]]);
                unsafe { core::ptr::write_volatile(addr as *mut u16, hw) };
                if let Err(e) = self.unit_sync() {
                    res = Err(e);
                }
                i = 2;
            }
            // 字主体（4 字节对齐段，上板验证路径）
            if res.is_ok() {
                for k in 0..words {
                    let b = &data[i + k * 4..i + k * 4 + 4];
                    let w = u32::from_le_bytes([b[0], b[1], b[2], b[3]]);
                    unsafe {
                        core::ptr::write_volatile((addr + i as u32 + k as u32 * 4) as *mut u32, w)
                    };
                    if let Err(e) = self.unit_sync() {
                        res = Err(e);
                        break;
                    }
                }
                i += words * 4;
            }
            // 半字收尾（rem 1..=3 → 1..=2 个半字，末半字可能 0xFF 填充）
            if res.is_ok() {
                for k in 0..tail {
                    let base = i + k * 2;
                    let lo = data[base];
                    let hi = if base + 1 < data.len() {
                        data[base + 1]
                    } else {
                        0xFF
                    };
                    let hw = u16::from_le_bytes([lo, hi]);
                    unsafe { core::ptr::write_volatile((addr + base as u32) as *mut u16, hw) };
                    if let Err(e) = self.unit_sync() {
                        res = Err(e);
                        break;
                    }
                }
            }

            self.set_lock(page, false);
            self.end(saved);
            res
        }

        /// 擦除整页后从页首写入（"写一条记录"一次搞定；阻塞版
        /// `write_page` 同款组合）
        ///
        /// `data` 不得超过一页（[`PAGE_SIZE`]，超出报
        /// [`FlashError::BadPage`]——本方法不从页首越界写下一页）。
        ///
        /// # Safety
        /// 合并 [`erase_page`](Self::erase_page) 与
        /// [`program_slice`](Self::program_slice) 的安全约束。
        pub unsafe fn write_page(&mut self, page: u32, data: &[u8]) -> Result<(), FlashError> {
            if data.len() as u64 > PAGE_SIZE as u64 {
                return Err(FlashError::BadPage);
            }
            unsafe {
                self.erase_page(page)?;
                self.program_slice(page * PAGE_SIZE, data)
            }
        }

        // ---- 内部：寄存器配方 ----

        /// 单元写后同步：等 BUSY 清零 + 翻译 ISR 错误位（每个编程单元
        /// 一次，hal 同款）
        fn unit_sync(&self) -> Result<(), FlashError> {
            self.wait_idle()?;
            self.check()
        }

        /// 等待 `ISR.BUSY` 清零（有界忙等 → [`FlashError::Timeout`]；
        /// 厂商库无界等，这里给上限防挂死）
        fn wait_idle(&self) -> Result<(), FlashError> {
            for _ in 0..BUSY_POLL {
                if !self.is_busy() {
                    return Ok(());
                }
                core::hint::spin_loop();
            }
            Err(FlashError::Timeout)
        }

        /// 把 `ISR` 里的错误位翻译成 `Err`（优先级 PC > PAGELOCK > PROG >
        /// CACHEON > SDKERR，hal 同款；命中后经 ICR R1W0 定向清除）
        fn check(&self) -> Result<(), FlashError> {
            let isr = self.regs.isr.read();
            let e = if isr & fc::ISR_PC != 0 {
                Some(FlashError::PcPage)
            } else if isr & fc::ISR_PAGELOCK != 0 {
                Some(FlashError::PageLocked)
            } else if isr & fc::ISR_PROG != 0 {
                Some(FlashError::NotErased)
            } else if isr & fc::ISR_CACHEON != 0 {
                Some(FlashError::CacheOn)
            } else if isr & fc::ISR_SDKERR != 0 {
                Some(FlashError::SdkRegion)
            } else {
                None
            };
            match e {
                Some(err) => {
                    // ICR 是 R1W0：写 0 清对应位，其余写 1（无操作）
                    self.regs.icr.write(!fc::ISR_ERRORS);
                    Err(err)
                }
                None => Ok(()),
            }
        }

        /// 进入擦写模式（阻塞版 `begin` 同配方，hal 上板实锤两条坑都在这）：
        ///
        /// 1. 统一清标志（ICR R1W0 写 0 清全部，厂商 `CW_FLASH->ICR = 0x00`）；
        /// 2. CACHE/FETCH **只在确实开着时才关**（4MHz 复位默认即为 0，
        ///    避免多余寄存器写；关了的值由返回值带出供 [`Self::end`] 恢复）；
        /// 3. **MODE 必须"或"进现有 CR1**：写裸 `KEY|mode` 会让编程静默
        ///    失效（TESTING.md 坑 2；厂商库同为 `CR1BAK | mode`）。
        ///
        /// CR1/CR2 的读改写包进 critical-section（防 ISR 侧并发 FLASH 操作
        /// 互踩，rtc.rs 同款纪律；忙等不进临界区）。
        fn begin(&self, mode: u32) -> (u32, u32) {
            critical_section::with(|_| {
                self.regs.icr.write(0);
                let cr1 = self.regs.cr1.read() & 0xFFFF;
                let cr2 = self.regs.cr2.read() & 0xFFFF;
                if cr2 & (fc::CR2_CACHE | fc::CR2_FETCH) != 0 {
                    self.regs
                        .cr2
                        .write(fc::KEY | (cr2 & !(fc::CR2_CACHE | fc::CR2_FETCH)));
                }
                self.regs.cr1.write(fc::KEY | (cr1 | mode));
                (cr1, cr2)
            })
        }

        /// 退出擦写模式：恢复 CR1（含 MODE=读）与 CR2（CACHE/FETCH 原样
        /// 还原；厂商库收尾两写同款，cw32l012_flash.c:252-253）
        fn end(&self, saved: (u32, u32)) {
            critical_section::with(|_| {
                self.regs.cr1.write(fc::KEY | saved.0);
                self.regs.cr2.write(fc::KEY | saved.1);
            })
        }

        /// 解锁/锁定某一页（每 8 页共用一个 LOCKx 位，表 7-2；厂商
        /// `FLASH_UnlockPage`/`FLASH_LockPage` 同配方：读改写 PAGELOCK
        /// 低 16 位 + KEY）
        fn set_lock(&self, page: u32, unlock: bool) {
            let bit = 1u32 << (page / 8);
            critical_section::with(|_| {
                let cur = self.regs.pagelock.read() & 0xFFFF;
                let v = if unlock { cur | bit } else { cur & !bit };
                self.regs.pagelock.write(fc::KEY | v);
            })
        }

        /// 编程单元拆分（纯逻辑，供单元测试）：`(半字引子数, 字数,
        /// 半字收尾数)`。总覆盖 = 引子 + 主体 + 收尾，仅当 `len` 为奇数
        /// 时末半字带一个 0xFF 填充字节；`addr` 必须已半字对齐。
        fn split_chunks(addr: u32, len: usize) -> (usize, usize, usize) {
            let lead = usize::from(addr % 4 != 0 && len >= 2);
            let body = len - lead * 2;
            let words = body / 4;
            let tail = match body % 4 {
                0 => 0,
                3 => 2,
                _ => 1, // 1 或 2 字节 → 一个半字
            };
            (lead, words, tail)
        }
    }

    #[cfg(test)]
    mod tests {
        use super::*;

        // 与阻塞版 cw32l012-hal/src/flash.rs 的同名单元测试同源
        #[test]
        fn page_math() {
            assert_eq!(Flash::page_of(0x0000_0000), 0);
            assert_eq!(Flash::page_of(0x0000_01FF), 0);
            assert_eq!(Flash::page_of(0x0000_0200), 1);
            assert_eq!(Flash::page_of(0x0000_FE00), 127);
            assert_eq!(Flash::page_of(0x0000_FFFF), 127);
            assert_eq!(PAGES * PAGE_SIZE, FLASH_SIZE);
        }

        #[test]
        fn split_word_path_is_verified_default() {
            // 4 对齐 + 4 倍数 → 纯字路径（上板验证路径，hal flash_store 同款）
            assert_eq!(Flash::split_chunks(0xFE00, 12), (0, 3, 0));
            assert_eq!(Flash::split_chunks(0xFE00, 20), (0, 5, 0)); // u64 记录
            assert_eq!(Flash::split_chunks(0x0000, 4), (0, 1, 0));
        }

        #[test]
        fn split_misaligned_and_odd_tails() {
            // 半字引子：addr%4==2 且 len>=2
            assert_eq!(Flash::split_chunks(0xFE02, 6), (1, 1, 0));
            assert_eq!(Flash::split_chunks(0xFE02, 2), (1, 0, 0));
            // 奇数收尾：末半字 0xFF 填充
            assert_eq!(Flash::split_chunks(0xFE00, 1), (0, 0, 1));
            assert_eq!(Flash::split_chunks(0xFE00, 3), (0, 0, 2));
            assert_eq!(Flash::split_chunks(0xFE00, 5), (0, 1, 1));
            assert_eq!(Flash::split_chunks(0xFE02, 5), (1, 0, 2));
            // len<2 时不做引子（引子条件 len>=2），落到收尾
            assert_eq!(Flash::split_chunks(0xFE02, 1), (0, 0, 1));
        }

        #[test]
        fn split_covers_every_byte() {
            // 覆盖不变量：引子*2 + 字数*4 + 收尾*2 == len + (len 奇数 ? 1 : 0)
            let addrs = [0u32, 2, 4, 0xFE00, 0xFE02, 0xFEFC, 0xFFFE];
            for &a in &addrs {
                for len in 1usize..24 {
                    let (lead, words, tail) = Flash::split_chunks(a, len);
                    let covered = lead * 2 + words * 4 + tail * 2;
                    let expect = len + (len & 1);
                    assert_eq!(covered, expect, "addr={a:#06} len={len}");
                    // 字主体严格 4 对齐（引子补齐后；words>0 蕴含 lead 补齐）
                    if words > 0 {
                        let body_addr = a + lead as u32 * 2;
                        assert_eq!(body_addr % 4, 0, "body addr misaligned addr={a:#06}");
                    }
                }
            }
        }
    }
}

#[cfg(feature = "cw32l012")]
pub use l012::*;
