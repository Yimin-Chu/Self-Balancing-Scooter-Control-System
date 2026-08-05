/**
 * @file    cli.h
 * @brief   串口命令行(CLI)：行编辑 + 命令自动注册
 *
 * 与二进制协议共用 USART3：帧解析器不认的可见字符会被转到这里，
 * 所以手机 App 发的协议帧和串口终端敲的命令可以同时工作(见 comm.c 的路由)。
 *
 * 加一条命令只要两步，不用回头改任何注册列表：
 *
 *   static int cliCmd_foo(int argc, char *argv[]) { ... return 0; }
 *   CLI_CMD_EXPORT(foo, "一句话说明", cliCmd_foo);
 *
 * 原理是 CLI_CMD_EXPORT 把一个 const 结构体塞进名为 cli_cmd 的自定义段，
 * 链接器把所有 .o 里的同名段拼成一段连续内存，cli.c 直接当数组遍历。
 * 段的边界符号 _cli_cmd_start / _cli_cmd_end 在 STM32F103C8Tx_FLASH.ld 里定义。
 */

#ifndef __CLI_H__
#define __CLI_H__

#include "stm32f1xx_hal.h"

#define CLI_LINE_LEN_MAX  (64U)  // 单行命令最大长度
#define CLI_ARGC_MAX      (4U)   // 最大参数个数(含命令名本身)
#define CLI_PRINT_LEN_MAX (128U) // 单次 Cli_Printf 输出上限

typedef int (*FuncCliCmd)(int argc, char *argv[]);

typedef struct
{
    const char *name;
    const char *help;
    FuncCliCmd  handler;
} CliCmd_tTypeDef;

/**
 * @brief  命令自动注册
 * @param  _name 命令名，直接写标识符，宏内部会字符串化
 * @param  _help 帮助说明，help 命令会列出来
 * @param  _func 处理函数，原型 int (*)(int argc, char *argv[])，返回 0 表示成功
 * @note   used 属性防止编译器因"没人引用"而优化掉；链接脚本里的 KEEP 防止
 *         --gc-sections 把它回收掉。两个都不能少。
 *         aligned(4) 保证每个表项紧密排列，否则链接器插了填充字节就没法当数组遍历。
 */
#define CLI_CMD_EXPORT(_name, _help, _func)                     \
    static const CliCmd_tTypeDef _cli_cmd_item_##_func          \
        __attribute__((used, section("cli_cmd"), aligned(4))) = \
            {                                                   \
                #_name,                                         \
                _help,                                          \
                _func,                                          \
            }

void    Cli_Init(void);
void    Cli_Banner(void); // 开机横幅 + 提示符，由 Comm_Init() 调用
void    Cli_RxByte(uint8_t byte);
uint8_t Cli_IsTextByte(uint8_t byte);
void    Cli_Printf(const char *fmt, ...);

/* 静默命令('@' 前缀)的成败计数，由 status 打印。静默通道不回执，
 * 失败没有任何提示，这两个数是唯一的排查手段——详见 cli.c 里的说明 */
uint16_t Cli_GetQuietOk(void);
uint16_t Cli_GetQuietErr(void);

#endif
