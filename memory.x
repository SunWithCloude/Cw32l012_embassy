/* CW32L012 存储器布局（64KB Flash / 8KB SRAM，带奇偶校验） */
MEMORY
{
  FLASH : ORIGIN = 0x00000000, LENGTH = 64K
  RAM   : ORIGIN = 0x20000000, LENGTH = 8K
}

/* cortex-m-rt link.x 的栈顶符号 */
_stack_start = ORIGIN(RAM) + LENGTH(RAM);
