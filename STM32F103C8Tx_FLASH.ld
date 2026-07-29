/*
******************************************************************************
* STM32F103C8Tx GCC 链接脚本
* Flash: 64K @ 0x08000000    RAM: 20K @ 0x20000000
******************************************************************************
*/

/* 入口 */
ENTRY(Reset_Handler)

/* 栈顶 = RAM 末地址 (0x20000000 + 20K) */
_estack = 0x20005000;

/* 最小堆栈需求 */
_Min_Heap_Size  = 0x200;   /* 512 字节堆  */
_Min_Stack_Size = 0x400;   /* 1024 字节栈 */

/* 存储器定义
 *
 * 最后 1K(0x0800FC00~0x0800FFFF, 中容量 F103 的一页) 划出来存 IMU 标定数据，
 * 所以给代码的 FLASH 只写 63K。这样一旦代码涨过 63K，链接器会直接报
 * "region FLASH overflowed" 而不是在运行时把标定页擦掉——这种运行期才暴露的
 * 覆盖问题极难查，宁可让它编译不过。
 * 页地址与 Storage/cali_store.h 的 CALI_STORE_ADDR 必须保持一致。
 */
MEMORY
{
  RAM   (xrw) : ORIGIN = 0x20000000, LENGTH = 20K
  FLASH (rx)  : ORIGIN = 0x08000000, LENGTH = 63K
}

SECTIONS
{
  /* 中断向量表，必须放在 Flash 起始 */
  .isr_vector :
  {
    . = ALIGN(4);
    KEEP(*(.isr_vector))
    . = ALIGN(4);
  } >FLASH

  /* 代码段 */
  .text :
  {
    . = ALIGN(4);
    *(.text)
    *(.text*)
    *(.glue_7)
    *(.glue_7t)
    *(.eh_frame)
    KEEP (*(.init))
    KEEP (*(.fini))
    . = ALIGN(4);
    _etext = .;
  } >FLASH

  /* 只读数据 */
  .rodata :
  {
    . = ALIGN(4);
    *(.rodata)
    *(.rodata*)
    . = ALIGN(4);
  } >FLASH

  /* CLI 命令自动注册表
   *
   * CLI_CMD_EXPORT 宏把每条命令的描述结构体扔进 cli_cmd 段，链接器在这里把它们
   * 收成一个连续数组，cli.c 用 _cli_cmd_start/_cli_cmd_end 遍历。这样加一条新命令
   * 只要写一个函数 + 一行宏，不用回头改任何注册列表。
   *
   * KEEP 不能省: 工程开了 -Wl,--gc-sections，没有 KEEP 的话这些结构体因为
   * "没有任何代码引用" 会被当成垃圾回收掉，表就空了。
   */
  .cli_cmd :
  {
    . = ALIGN(4);
    _cli_cmd_start = .;
    KEEP (*(cli_cmd))
    KEEP (*(SORT(cli_cmd.*)))
    _cli_cmd_end = .;
    . = ALIGN(4);
  } >FLASH

  .ARM.extab : { *(.ARM.extab* .gnu.linkonce.armextab.*) } >FLASH
  .ARM :
  {
    __exidx_start = .;
    *(.ARM.exidx*)
    __exidx_end = .;
  } >FLASH

  .preinit_array :
  {
    PROVIDE_HIDDEN (__preinit_array_start = .);
    KEEP (*(.preinit_array*))
    PROVIDE_HIDDEN (__preinit_array_end = .);
  } >FLASH

  .init_array :
  {
    PROVIDE_HIDDEN (__init_array_start = .);
    KEEP (*(SORT(.init_array.*)))
    KEEP (*(.init_array*))
    PROVIDE_HIDDEN (__init_array_end = .);
  } >FLASH

  .fini_array :
  {
    PROVIDE_HIDDEN (__fini_array_start = .);
    KEEP (*(SORT(.fini_array.*)))
    KEEP (*(.fini_array*))
    PROVIDE_HIDDEN (__fini_array_end = .);
  } >FLASH

  /* 已初始化数据: 加载到 Flash, 运行时拷到 RAM */
  _sidata = LOADADDR(.data);
  .data :
  {
    . = ALIGN(4);
    _sdata = .;
    *(.data)
    *(.data*)
    . = ALIGN(4);
    _edata = .;
  } >RAM AT> FLASH

  /* 未初始化数据 (清零) */
  . = ALIGN(4);
  .bss :
  {
    _sbss = .;
    __bss_start__ = _sbss;
    *(.bss)
    *(.bss*)
    *(COMMON)
    . = ALIGN(4);
    _ebss = .;
    __bss_end__ = _ebss;
  } >RAM

  /* 堆栈占位检查: 剩余 RAM 不够堆栈则链接报错 */
  ._user_heap_stack :
  {
    . = ALIGN(8);
    PROVIDE ( end = . );
    PROVIDE ( _end = . );
    . = . + _Min_Heap_Size;
    . = . + _Min_Stack_Size;
    . = ALIGN(8);
  } >RAM

  /* 丢弃标准库中不需要的段 */
  /DISCARD/ :
  {
    libc.a ( * )
    libm.a ( * )
    libgcc.a ( * )
  }

  .ARM.attributes 0 : { *(.ARM.attributes) }
}
