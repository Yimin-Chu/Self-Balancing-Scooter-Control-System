/**
 * @file    cli.c
 * @brief   串口命令行(CLI)：行编辑 + 命令自动注册表遍历
 */

#include "cli.h"
#include "comm_echo.h"
#include "comm_port.h"
#include "comm_send.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

/* Define --------------------------------------------------------------------*/
#define CLI_CHAR_BS  (0X08U)
#define CLI_CHAR_LF  (0X0AU)
#define CLI_CHAR_CR  (0X0DU)
#define CLI_CHAR_DEL (0X7FU)

/* 命令表边界。GCC 的符号在链接脚本里定义；ARMCC 需要在分散加载文件里
 * 建一个名为 cli_cmd 的执行域才会有 Image$$ 符号(见 comm_protocol_readme.md)。 */
#if defined(__ARMCC_VERSION)
extern const CliCmd_tTypeDef Image$$cli_cmd$$Base[];
extern const CliCmd_tTypeDef Image$$cli_cmd$$Limit[];
#define CLI_CMD_TAB_BEGIN (Image$$cli_cmd$$Base)
#define CLI_CMD_TAB_END   (Image$$cli_cmd$$Limit)
#else
extern const CliCmd_tTypeDef _cli_cmd_start[];
extern const CliCmd_tTypeDef _cli_cmd_end[];
#define CLI_CMD_TAB_BEGIN (_cli_cmd_start)
#define CLI_CMD_TAB_END   (_cli_cmd_end)
#endif

/* Datas ---------------------------------------------------------------------*/
static char    cliLine[CLI_LINE_LEN_MAX];
static uint8_t cliLen;
static uint8_t cliActive; // 第一次收到可见字符后置 1
static uint8_t cliLastCr; // 上一个字节是 CR，用来把紧随其后的 LF 吞掉
static char    cliPrintBuf[CLI_PRINT_LEN_MAX];

static const char CLI_PROMPT[] = "car> ";

/* Function ------------------------------------------------------------------*/
static void Cli_Puts(const char *str);
static void Cli_OnFirstActivity(void);
static void Cli_Exec(char *line);
static int  cliCmd_help(int argc, char *argv[]);

CLI_CMD_EXPORT(help, "list all commands", cliCmd_help);

/**
 * @brief  初始化 CLI
 */
void Cli_Init(void)
{
    cliLen    = 0U;
    cliActive = 0U;
    cliLastCr = 0U;
}

/**
 * @brief  判断一个字节是否属于 CLI 文本
 * @retval 1:交给 CLI  0:交给旧的单字节遥控码处理
 * @note   可见字符 0x20~0x7E 加上回车/换行/退格。旧遥控码都在 0x00~0x09 区间，
 *         和这个范围不重叠，所以两套输入可以共存。
 *         注意 0x09 恰好是 TAB：在终端里按 TAB 会被当成旧的"停止"码，
 *         这个副作用无害，就不特意排除了。
 */
uint8_t Cli_IsTextByte(uint8_t byte)
{
    if ((byte >= 0X20U) && (byte <= 0X7EU))
    {
        return 1U;
    }

    return ((CLI_CHAR_CR == byte) || (CLI_CHAR_LF == byte) || (CLI_CHAR_BS == byte) ||
            (CLI_CHAR_DEL == byte))
               ? 1U
               : 0U;
}

/**
 * @brief  喂一个字符给 CLI
 */
void Cli_RxByte(uint8_t byte)
{
    uint8_t lastCr = cliLastCr;

    cliLastCr = 0U;

    if (0U == cliActive)
    {
        Cli_OnFirstActivity();
    }

    if ((CLI_CHAR_CR == byte) || (CLI_CHAR_LF == byte))
    {
        /* CRLF 只算一个行尾。终端的换行符设成 "\r\n" 时，不这么处理会多执行一次
         * 空行，屏幕上凭空多出一个提示符 */
        if ((CLI_CHAR_LF == byte) && (0U != lastCr))
        {
            return;
        }
        cliLastCr = (CLI_CHAR_CR == byte) ? 1U : 0U;

        Cli_Puts("\r\n");

        if (cliLen > 0U)
        {
            cliLine[cliLen] = '\0';
            cliLen          = 0U;
            Cli_Exec(cliLine);
        }

        CommEcho_SetInput(""); // 命令已提交，清空 OLED 上的输入行
        Cli_Puts(CLI_PROMPT);
        return;
    }

    if ((CLI_CHAR_BS == byte) || (CLI_CHAR_DEL == byte))
    {
        if (cliLen > 0U)
        {
            cliLen--;
            Cli_Puts("\b \b"); // 退格 + 空格盖掉 + 再退格，终端上才是真的删掉
            cliLine[cliLen] = '\0';
            CommEcho_SetInput(cliLine);
        }
        return;
    }

    /* 留一个字节给字符串结束符 */
    if (cliLen < (CLI_LINE_LEN_MAX - 1U))
    {
        cliLine[cliLen++] = (char)byte;
        CommPort_TxPush(&byte, 1U); // 回显，否则终端上看不见自己敲了什么

        cliLine[cliLen] = '\0';
        CommEcho_SetInput(cliLine); // OLED 上同步回显，不接终端也能确认收到了
    }
}

/**
 * @brief  格式化输出到串口
 * @note   只能在主循环调用(用了共享静态缓冲)。输出走发送环形缓冲，
 *         缓冲满时整条丢弃，不会阻塞控制周期
 */
void Cli_Printf(const char *fmt, ...)
{
    va_list args;
    int     len;

    va_start(args, fmt);
    len = vsnprintf(cliPrintBuf, sizeof(cliPrintBuf), fmt, args);
    va_end(args);

    if (len <= 0)
    {
        return;
    }

    /* vsnprintf 返回的是"本该写入的长度"，超出缓冲时会大于缓冲大小，要夹一下 */
    if ((size_t)len >= sizeof(cliPrintBuf))
    {
        len = (int)sizeof(cliPrintBuf) - 1;
    }

    CommPort_TxPush((const uint8_t *)cliPrintBuf, (uint16_t)len);
}

/**
 * @brief  输出一个字符串
 */
static void Cli_Puts(const char *str)
{
    CommPort_TxPush((const uint8_t *)str, (uint16_t)strlen(str));
}

/**
 * @brief  开机横幅
 * @note   在 Comm_Init() 里调用，是"小车活过来了"的第一条证据：
 *         终端上能看到这几行，就说明时钟、串口波特率、发送链路全都对。
 *         注意蓝牙是后连的话这几行已经发出去没人收，敲一次回车拿提示符即可。
 */
void Cli_Banner(void)
{
    Cli_Puts("\r\n=== balance car ready ===\r\n");
    Cli_Puts("uart 115200 8N1, type 'help'\r\n");
    Cli_Puts(CLI_PROMPT);
}

/**
 * @brief  首次检测到有人在敲命令时的处理
 * @note   兜底关掉二进制周期上报：开机默认是关的，但手机 App 可能已经下发
 *         REV_CMD_REPORT_CFG 打开过，那样终端上会不停刷乱码，根本没法敲命令。
 *         要恢复上报，让 App 重新下发 REV_CMD_REPORT_CFG(0x06) 即可。
 */
static void Cli_OnFirstActivity(void)
{
    cliActive = 1U;

    /* 上报本来就关着的话什么都不用做，别再刷一遍提示符 */
    if (0U != CommSend_IsReporting())
    {
        CommSend_SetReportCfg(0U, 0U);
        Cli_Puts("\r\n--- auto report muted ---\r\n");
        Cli_Puts(CLI_PROMPT);
    }

    CommEcho_SetEvent("cli ready");
}

/**
 * @brief  切分并执行一行命令
 */
static void Cli_Exec(char *line)
{
    const CliCmd_tTypeDef *pCmd;
    char                  *argv[CLI_ARGC_MAX];
    char                  *p    = line;
    int                    argc = 0;
    int                    ret;

    /* 原地切分：空格改成 '\0'，argv 直接指进 cliLine */
    while (('\0' != *p) && (argc < (int)CLI_ARGC_MAX))
    {
        while (' ' == *p)
        {
            *p++ = '\0';
        }

        if ('\0' == *p)
        {
            break;
        }

        argv[argc++] = p;

        while (('\0' != *p) && (' ' != *p))
        {
            p++;
        }
    }

    if (0 == argc)
    {
        return;
    }

    for (pCmd = CLI_CMD_TAB_BEGIN; pCmd < CLI_CMD_TAB_END; pCmd++)
    {
        if (0 == strcmp(argv[0], pCmd->name))
        {
            ret = pCmd->handler(argc, argv);

            if (0 == ret)
            {
                CommEcho_SetEvent("%s ok", argv[0]);
            }
            else
            {
                Cli_Printf("command failed (%d)\r\n", ret);
                CommEcho_SetEvent("%s err%d", argv[0], ret);
            }

            return;
        }
    }

    CommEcho_SetEvent("? %s", argv[0]);
    Cli_Printf("unknown command: %s\r\n", argv[0]);
}

/**
 * @brief  help：遍历自动注册表列出所有命令
 */
static int cliCmd_help(int argc, char *argv[])
{
    const CliCmd_tTypeDef *pCmd;

    (void)argc;
    (void)argv;

    for (pCmd = CLI_CMD_TAB_BEGIN; pCmd < CLI_CMD_TAB_END; pCmd++)
    {
        Cli_Printf("  %-8s %s\r\n", pCmd->name, pCmd->help);
    }

    return 0;
}
