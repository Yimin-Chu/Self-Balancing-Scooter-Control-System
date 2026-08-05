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

/* 静默命令前缀 '@'：一行以它开头就照常执行，但全程不往串口回一个字节。
 *
 * 是给 VOFA+ 的滑块/按钮控件准备的。拖一次滑块会连续下发几十条命令，而普通
 * CLI 每条都要回"字符回显 + 执行结果 + 提示符"共几十字节 ASCII，这些字节会插进
 * JustFloat 的二进制流中间，上位机每次都得靠帧尾重新对齐，波形就没法看了。
 *
 * 语法就是普通命令加个前缀，所以范围校验、改速度环增益时自动清积分这些
 * 都照旧生效：
 *     @pid vkp 520     滑块绑 "@pid vkp %.2f\n"
 *     @stop            按钮绑 "@stop\n"，一个不污染波形的急停 */
#define CLI_CHAR_QUIET (0X40U)

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
static uint8_t cliQuiet;  // 当前这行是静默命令，行尾清零
static char    cliPrintBuf[CLI_PRINT_LEN_MAX];

/* 静默命令的成败计数。静默通道的代价是失败也没有任何提示——滑块拖了没反应时，
 * 敲一次 status 看这两个数就能分清是"命令根本没到"(两个都不动)还是
 * "到了但被拒了"(err 在涨，多半是数值超范围或格式不对) */
static uint16_t cliQuietOk;
static uint16_t cliQuietErr;

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
    cliLen      = 0U;
    cliActive   = 0U;
    cliLastCr   = 0U;
    cliQuiet    = 0U;
    cliQuietOk  = 0U;
    cliQuietErr = 0U;
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
    uint8_t isPrefix;

    /* 只有行首那个 '@' 算前缀，行中间出现的照常当普通字符 */
    isPrefix = ((0U == cliLen) && (0U == cliQuiet) && (CLI_CHAR_QUIET == byte)) ? 1U : 0U;

    cliLastCr = 0U;

    /* 前缀必须赶在 Cli_OnFirstActivity() 之前置位：那里会打一行 "[ev] cli ready"，
     * 第一条静默命令若漏出这行，正好插进 JustFloat 数据流里 */
    if (0U != isPrefix)
    {
        cliQuiet = 1U;
    }

    if (0U == cliActive)
    {
        Cli_OnFirstActivity();
    }

    if (0U != isPrefix)
    {
        return; // 前缀本身不入行缓冲，也不回显
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

        CommEcho_SetInput(""); // 命令已提交，清空回显快照
        Cli_Puts(CLI_PROMPT);

        cliQuiet = 0U; // 静默只作用于一行，行尾恢复
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
        cliLine[cliLen]   = '\0';

        /* 回显只有这一处是绕过 Cli_Puts 直接推字节的，静默时得单独挡掉。
         * CommEcho_SetInput 只写内存快照不碰串口，照常调即可 */
        if (0U == cliQuiet)
        {
            CommPort_TxPush(&byte, 1U); // 否则终端上看不见自己敲了什么
        }

        CommEcho_SetInput(cliLine);
    }
    else if (0U != cliQuiet)
    {
        /* 静默行把缓冲填满了还没等到换行，几乎只有一个原因：VOFA+ 控件模板
         * 漏了结尾的 \n。必须就地解除静默——否则 cliQuiet 一直挂着，此后所有
         * CLI 输出都被丢弃，终端看上去和死机一模一样，极难排查。
         * 丢掉这行并记一次 err，让 status 能看出问题出在哪 */
        cliLen   = 0U;
        cliQuiet = 0U;
        cliQuietErr++;
    }
    else
    {
        /* 非静默的超长行维持原行为：多余字符丢掉，行尾照常执行已收下的部分 */
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

    /* 静默命令期间所有输出就地丢弃。挡在这里而不是挡在每个命令处理函数里，
     * 是为了让 cli_cmd.c 完全不用关心静默这回事 */
    if (0U != cliQuiet)
    {
        return;
    }

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
 * @brief  静默命令的成功/失败累计次数
 * @note   静默通道不回执，失败是"悄无声息"的。滑块拖了没反应时靠这两个数定位：
 *         两个都不涨 = 命令根本没送到(查控件模板的结尾换行、串口是否被占)；
 *         err 在涨   = 命令到了但被拒(数值超范围、格式不对、命令名拼错)
 */
uint16_t Cli_GetQuietOk(void)
{
    return cliQuietOk;
}

uint16_t Cli_GetQuietErr(void)
{
    return cliQuietErr;
}

/**
 * @brief  输出一个字符串
 */
static void Cli_Puts(const char *str)
{
    if (0U != cliQuiet)
    {
        return;
    }

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

    /* CommEcho_SetEvent 是直接往发送缓冲推的，不经过 Cli_Puts，静默时要单独挡 */
    if (0U == cliQuiet)
    {
        CommEcho_SetEvent("cli ready");
    }
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

            /* 静默命令只累加计数。CommEcho_SetEvent 绕过 Cli_Printf 直接推字节，
             * 不在这里提前返回的话回执照样会漏进波形数据流 */
            if (0U != cliQuiet)
            {
                if (0 == ret)
                {
                    cliQuietOk++;
                }
                else
                {
                    cliQuietErr++;
                }

                return;
            }

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

    if (0U != cliQuiet)
    {
        cliQuietErr++; // 命令名都不认识，多半是控件模板写错了
        return;
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
