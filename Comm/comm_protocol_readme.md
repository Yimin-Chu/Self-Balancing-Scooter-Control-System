# 平衡小车蓝牙串口通信协议

物理层：USART3(PB10=TX / PB11=RX)，115200 8N1，接蓝牙透传模块（如 HC01）。
帧格式参考追觅 P1917 协议（`universal_controller_platform/ComProtocol/com_protocol_readme.md`）。

---

## 一、帧格式

| 帧头 | 数据长度 | 命令ID | 数据 | CRC-16 校验 | 帧尾 |
| ---- | -------- | ------ | ---- | ----------- | ---- |
| 0x3C | 0~48     | 0~255  | 见协议 | crcH8-crcL8 | 0x3E |

- **数据长度(len)** 只算 `数据` 段的字节数，不含 len / 命令ID / CRC 自身。
- **CRC** 覆盖 `len + 命令ID + 数据` 三段（转义前的原始字节），**高字节先发**。
- **多字节字段一律小端**（与 STM32 本机字节序一致），例如 `uint16_t 0x0003` 发 `03 00`。

### 转义规则

帧头帧尾之间的所有字节（含 CRC），凡是等于 `0x3C` / `0x3E` / `0x3F` 的，都在其前面插入一个 `0x3F`：

```
原始:  03 00 3C 00 01 CC 61
转义:  03 00 3F 3C 00 01 CC 61
```

这样做的好处是 `0x3C` / `0x3E` 在帧内绝不可能出现，接收端任何时候看到 `0x3C`
都能立刻重新对齐帧头，不需要等超时——这是这套格式最实用的地方。

### CRC-16 参数

| 项目 | 值 |
| ---- | -- |
| 多项式 | X16+X15+X2+1 (MODBUS)，反转形式 `0xA001` |
| 初值 | `0xFFFF` |
| 输入/输出反转 | 无 |
| 上线字节序 | 高字节先发 |

实现见 `crc16.c`。追觅原版用两张 256 字节查表，本工程改成逐位移位实现以省 Flash，
**对同一数据两者返回值完全相同**，协议层完全兼容。

---

## 二、下行命令（手机/上位机 → 小车）

| 命令ID | 名称 | 数据长度 | 说明 |
| ------ | ---- | -------- | ---- |
| 0x00 | REV_CMD_MOVE | 3 | 运动控制，同时喂指令看门狗 |
| 0x01 | REV_CMD_STOP | 0 | 紧急停止：切电机 + 清速度环积分 |
| 0x02 | REV_CMD_MOTOR_EN | 1 | 电机使能总闸 |
| 0x03 | REV_CMD_PID_SET | 13 | PID 参数设置 |
| 0x04 | REV_CMD_PID_GET | 0 | 查询 PID，回 0x04 上报帧 |
| 0x05 | REV_CMD_MED_ANGLE | 4 | 平衡中值角微调 |
| 0x06 | REV_CMD_REPORT_CFG | 3 | 自动上报周期与内容 |
| 0x07 | REV_CMD_HEARTBEAT | 0 | 心跳，只喂看门狗 |
| 0x08 | REV_CMD_VERSION | 0 | 查询版本，回 0x05 上报帧 |

### 0x00 REV_CMD_MOVE

| 偏移 | 字段 | 类型 | 说明 |
| ---- | ---- | ---- | ---- |
| 0 | speed | int8 | -100~100，前进为正 |
| 1 | turn | int8 | -100~100，右转为正 |
| 2 | enable | uint8 | 0=松开(目标归零) 1=使能 |

> **注意**：当前固件只取 `speed`/`turn` 的**符号和死区**（死区 `COMM_MOVE_DEADZONE=10`），
> 幅值暂未使用。因为 `pid.c` 的 `Control()` 采用"按住方向每 10ms 累加 1、上限 ±SPEED_Y"
> 的斜坡模型，直接写 `Target_Speed` 会被下一个控制周期覆盖。等 `pid.c` 改成接收比例给定，
> 这两个字段就能直接用上，**协议不需要改动**。

### 0x02 REV_CMD_MOTOR_EN

| 偏移 | 字段 | 类型 | 说明 |
| ---- | ---- | ---- | ---- |
| 0 | enable | uint8 | 0=PWM 恒为 0 1=输出 Control() 计算值 |

开机默认为 0（只观察传感器，电机不转），必须显式下发 `enable=1` 才会驱动电机。
`REV_CMD_STOP` 会把它清回 0。

### 0x03 REV_CMD_PID_SET

| 偏移 | 字段 | 类型 | 说明 |
| ---- | ---- | ---- | ---- |
| 0 | loop | uint8 | 0=直立环 1=速度环 2=转向环 |
| 1 | kp | float | 小端 IEEE754 |
| 5 | ki | float | 直立环/转向环忽略此字段 |
| 9 | kd | float | 速度环忽略此字段 |

参数绝对值上限 `COMM_PID_GAIN_ABS_MAX = 2000`，超限或 NaN 回 `REV_ERROR_PARA`。
转向环只接受 `kp`：`Turn_Kd` 每个控制周期都会被 `Control()` 按"是否正在转向"重算，写了留不住。

### 0x05 REV_CMD_MED_ANGLE

| 偏移 | 字段 | 类型 | 说明 |
| ---- | ---- | ---- | ---- |
| 0 | med_angle | float | 平衡中值角(度)，范围 ±20 |

### 0x06 REV_CMD_REPORT_CFG

| 偏移 | 字段 | 类型 | 说明 |
| ---- | ---- | ---- | ---- |
| 0 | period_10ms | uint8 | 上报周期/10ms；0=关闭自动上报；非 0 时最小值 5(50ms) |
| 1 | mask | uint16 | bit0=姿态 bit1=运动 bit2=状态 |

---

## 三、上行上报（小车 → 手机/上位机）

| 命令ID | 名称 | 数据长度 | 触发条件 |
| ------ | ---- | -------- | -------- |
| 0x00 | SEND_ID_ACK | 2 | 命令出错时必回；配置类命令成功也回 |
| 0x01 | SEND_ID_ATTITUDE | 12 | 周期上报 |
| 0x02 | SEND_ID_MOTION | 12 | 周期上报 |
| 0x03 | SEND_ID_STATUS | 10 | 周期上报 |
| 0x04 | SEND_ID_PID | 28 | 收到 0x04 查询 |
| 0x05 | SEND_ID_VERSION | 3 | 收到 0x08 查询 |

### 0x00 SEND_ID_ACK

| 偏移 | 字段 | 类型 | 说明 |
| ---- | ---- | ---- | ---- |
| 0 | cmd | uint8 | 被应答的下行命令ID；**0xFF 表示帧层错误**(命令ID 不可信) |
| 1 | error | uint8 | 见下方错误码表 |

| 错误码 | 含义 |
| ------ | ---- |
| 0 | REV_ERROR_NONE 正常 |
| 1 | REV_ERROR_CRC 校验失败 |
| 2 | REV_ERROR_FRAME 帧结构错误（0x3F 后面跟了非法字节） |
| 3 | REV_ERROR_OVER_NUM 帧长超限 |
| 4 | REV_ERROR_INVALID 无效命令ID |
| 5 | REV_ERROR_LEN 载荷长度不对 |
| 6 | REV_ERROR_PARA 参数越界 |

### 0x01 SEND_ID_ATTITUDE

| 偏移 | 字段 | 类型 | 说明 |
| ---- | ---- | ---- | ---- |
| 0 | roll_x100 | int16 | 横滚角 ×100（平衡小车的控制角） |
| 2 | pitch_x100 | int16 | 俯仰角 ×100 |
| 4 | yaw_x100 | int16 | 偏航角 ×100 |
| 6 | gyrox | int16 | 已扣除零偏的角速度原始值 |
| 8 | gyroy | int16 | |
| 10 | gyroz | int16 | |

### 0x02 SEND_ID_MOTION

| 偏移 | 字段 | 类型 |
| ---- | ---- | ---- |
| 0 | encoder_l | int16 |
| 2 | encoder_r | int16 |
| 4 | target_speed | int16 |
| 6 | target_turn | int16 |
| 8 | moto1 | int16 |
| 10 | moto2 | int16 |

`moto1/moto2` 是限幅后的 PWM 计算值，`motor_enable=0` 时实际输出为 0。

### 0x03 SEND_ID_STATUS

| 偏移 | 字段 | 类型 | 说明 |
| ---- | ---- | ---- | ---- |
| 0 | motor_enable | uint8 | |
| 1 | cmd_timeout | uint8 | 1=超过 350ms 没收到运动指令 |
| 2 | gyrox_offset | int16 | 陀螺零偏 |
| 4 | distance_mm | uint16 | 超声波测距(mm) |
| 6 | tick_ms | uint32 | HAL_GetTick() |

### 0x04 SEND_ID_PID

7 个 float，顺序：`vertical_kp, vertical_kd, velocity_kp, velocity_ki, turn_kp, turn_kd, med_angle`。

### 0x05 SEND_ID_VERSION

3 个 uint8：`major, minor, patch`（当前 1.0.0）。

---

## 四、示例帧（十六进制，已含转义与 CRC）

下行：

| 含义 | 完整帧 |
| ---- | ------ |
| 前进（speed=100, en=1） | `3C 03 00 64 00 01 1F E0 3E` |
| 前进（speed=60=0x3C，**触发转义**） | `3C 03 00 3F 3C 00 01 CC 61 3E` |
| 松开摇杆 | `3C 03 00 00 00 00 00 60 3E` |
| 紧急停止 | `3C 00 01 70 C0 3E` |
| 电机使能 | `3C 01 02 01 A0 E0 3E` |
| 电机断使能 | `3C 01 02 00 60 21 3E` |
| 查询 PID | `3C 00 04 73 00 3E` |
| 上报配置 100ms / 姿态+运动 | `3C 03 06 0A 03 00 7A 40 3E` |
| 心跳 | `3C 00 07 72 40 3E` |
| 查询版本 | `3C 00 08 76 00 3E` |

上行：

| 含义 | 完整帧 |
| ---- | ------ |
| ACK（应答 0x02，无错误） | `3C 02 00 02 00 FC 00 3E` |

以"前进（speed=60）"为例逐段拆解：

```
3C                          帧头
03                          len = 3(数据段 3 字节)
00                          命令ID = REV_CMD_MOVE
3F 3C                       speed = 0x3C，因等于帧头值而被转义
00                          turn  = 0
01                          enable = 1
CC 61                       CRC16([03 00 3C 00 01]) = 0xCC61，高字节先发
3E                          帧尾
```

---

## 五、使用注意

1. **指令看门狗**：`pid.c` 里 `CMD_TIMEOUT_MS = 350`。超过 350ms 没收到
   `REV_CMD_MOVE` / `REV_CMD_STOP` / `REV_CMD_HEARTBEAT`，`Control()` 会强制把
   目标速度和转向归零。手机端摇杆不动时也要按 <350ms 的间隔发心跳。
2. **带宽**：115200bps 约 11.5KB/s。姿态+运动两帧约 36 字节，
   100ms 一轮占用很小；上报周期最小 50ms。过密时看 `commPortStat.txDropFrame`。
3. **旧单字节遥控码兼容**：帧外的裸字节仍按老规则处理
   （0x01 前进 / 0x05 后退 / 0x03 右转 / 0x07 左转 / 0x09 停止 / 其他松开），
   方便 App 分步升级。App 切换完成后把 `comm.h` 里的
   `COMM_LEGACY_BYTE_CMD_ENABLE` 改成 0 关掉——留着它的风险是任何噪声字节
   都可能被当成运动指令。
4. **上电默认状态**：`motor_enable = 0`，电机不转；**自动上报默认关闭**。
   二进制帧和 CLI 的文本共用一个串口，上报一开终端上就全是乱码，开机横幅和命令
   回执都看不清，所以默认交给 App 用 `REV_CMD_REPORT_CFG(0x06)` 按需打开。
   想恢复"一上电就上报"，把 `comm_send.h` 的 `COMM_REPORT_PERIOD_DEFAULT` 改回 `10U`。
5. **三种输出不能同时开**：二进制周期上报、VOFA+ 波形、CLI 文本共用一条 TX。
   前两个互斥（`vofa jf` 会自动关掉周期上报），CLI 则是随时可以插进去的——
   JustFloat 靠帧尾重新对齐，敲一条命令最多糊掉一两个采样点，不影响继续调参。

---

## 六、串口命令行（CLI）

同一个 USART3 上还挂了一个文本命令行，方便不开手机 App 也能调试。路由规则：

| 字节范围 | 去向 |
| -------- | ---- |
| `0x3C` 开头的完整帧 | 二进制协议解析器 |
| 可见字符 `0x20~0x7E` 及 CR/LF/退格 | CLI（`Cli/cli.c`） |
| 其余帧外字节（`0x00~0x09`） | 旧的单字节遥控码 |

三者取值范围不重叠，所以手机 App 和串口终端可以同时连着互不干扰。
上报默认是关的；万一 App 中途打开过，终端里第一次敲下字符时 CLI 会**兜底再关一次**，
需要恢复时让 App 重新下发 `REV_CMD_REPORT_CFG(0x06)`。

### 串口上能看到什么

开机时 `Comm_Init()` 会打一段横幅，这是"小车活过来了"的第一条证据——能看到它，
就说明时钟、波特率、发送链路全都对：

```
[ev] boot ok
=== balance car ready ===
uart 115200 8N1, type 'help'
car>
```

之后**每收到一条命令都会回一行** `[ev] ...`：

```
car> start
[ev] start ok
car> foo
unknown command: foo
[ev] ? foo
car>
```

这些行由 `CommEcho_SetEvent()` 统一输出（`comm_echo.c`），CLI 命令、二进制帧、
旧单字节码三条入口都会经过它，所以不管命令从哪来都有回执。内容和上一条相同时
也照打不误——连按两次 `start` 要看得见两行。

不需要这些回执时（比如 App 高频下发命令，不想浪费带宽），把 `comm_echo.h` 里的
`COMM_ECHO_UART_ENABLE` 改成 0。

> 蓝牙是开机之后才连上的话，横幅早就发出去没人收了。敲一次回车拿到 `car> ` 提示符
> 即可，效果一样。

### 命令

| 命令 | 说明 |
| ---- | ---- |
| `help` | 列出所有命令（遍历自动注册表生成） |
| `cali` | 静置标定 IMU（阻塞约 20s）并把结果写进 Flash |
| `start` | 使能电机，开始跑 PID |
| `stop` | 切断电机输出 |
| `status` | 打印控制与通信状态 |
| `pid` | 查看/修改各环增益，见"七、VOFA+ 波形调参" |
| `vofa` | 开关 VOFA+ 波形输出，见"七、VOFA+ 波形调参" |

典型流程：终端连上 115200 8N1 → 回车出现 `car> ` 提示符 → `cali`（首次或想重标定时）
→ `start` 起车 → 出问题随时 `stop`。

> **两个注意点**：
> `cali` 会阻塞约 20s，期间 `Control()` 完全不执行，所以它做的第一件事就是切电机；
> 随后的 Flash 擦写又要占用 20~40ms（CPU 连取指都被卡住），同样必须在电机停转时做。
> 另外 `<` 和 `>` 是帧头帧尾，在终端里输入会被帧解析器吞掉，命令里不要用这两个字符。
> 真敲进去也不会死：`CommPack_Poll()` 会在帧内静默 100ms 后放弃半截帧、退回 CLI，
> 串口上打一行 `[ev] frm timeout`。

> **命令必须以回车结尾**。串口工具里那个"换行符"要选 `\r\n` 或 `\n`，选"无"的话
> 字符收得到（能看到回显）但永远不会执行。CR、LF、CRLF 三种都认，
> CRLF 会被合并成一个行尾，不会多出空提示符行。

### 用终端软件连（Xshell / PuTTY / Tera Term）

Xshell 新建会话选 **SERIAL** 协议，端口参数：

| 项 | 值 |
| -- | -- |
| 波特率 | 115200 |
| 数据位 / 停止位 / 校验 | 8 / 1 / None |
| **流控制** | **None**（选了 RTS/CTS 但线没接，会一个字节都发不出去） |
| 本地回显 | 关（小车自己会回显，开了变双份） |
| 退格键 | 发 `0x08` 或 `0x7F` 都行，两个都认 |

走蓝牙时先在 Windows 里配对 HC-05，然后在"更多蓝牙设置 → COM 端口"里挑**传出(Outgoing)**
那个口。注意 **HC-05 模块自身的波特率也得是 115200**（`AT+UART=115200,0,0`），
模块还停在出厂 9600 的话，两头对不上，收到的只会是乱码。

连上后敲一次回车就能拿到 `car> ` 提示符（开机横幅在连上之前就发完了，看不到很正常）。

> 终端里别按方向键：`ESC [ A` 这类转义序列的 `ESC(0x1B)` 会被当成旧遥控码，
> 后面的 `[`、`A` 则会当普通字符塞进命令行。CLI 没做历史记录，按退格删掉即可。
> 要发二进制帧还是得用带 HEX 模式的串口工具，Xshell 发不了。

### 怎么确认小车真的收到了指令

三层证据，从"字节到了"一直到"命令执行了"：

| 现象 | 说明什么 |
| ---- | -------- |
| 终端里敲的字符**回显**出来了 | 字节进了 USART3 中断、被 `Comm_Poll()` 取走、又原路发了回来，收发两条链路都通 |
| 终端出现 `[ev] ...` 执行结果 | 命令查到了、处理函数跑完了 |
| `status` 里的计数在涨 | 逐项统计，见"十、调试入口" |

`[ev]` 那行的内容对照：

| 内容 | 来源 |
| ---- | ---- |
| `cli ready` | 第一次检测到有人敲命令 |
| `start ok` / `cali err-1` | CLI 命令执行结果 |
| `? foo` | 命令不存在 |
| `f00 ok n12` | 收到二进制帧：命令ID `0x00`、成功、累计第 12 帧 |
| `crc err n3` | CRC 校验失败第 3 次（**波特率配错时这个数会一路涨**） |
| `frm timeout` | 收到半截帧后静默超过 100ms，已丢弃重新对齐 |
| `byte 01` | 旧的单字节遥控码 |

带计数的几条是故意的：手机 App 反复下发同一条命令时，文字不变但数字在动，
一眼就能看出链路是活的。

回显不出来只有两种可能：波特率不对（现在是 **115200 8N1**，改过 `MX_USART3_UART_Init()`
就要同步改终端和蓝牙模块），或者 TX/RX 接反了。

### OLED 显示（默认已关闭）

`Core/Inc/oled.h` 里的 `OLED_ENABLE` 默认为 **0**，屏幕相关代码整块不参与编译，
调试反馈全部走串口（CLI 文本 + VOFA+ 波形）。

**关掉的原因是时序，不是嫌它没用。** `OLED_ShowLine()` 一次要往 I2C1 写 131 字节，
100kHz 下阻塞约 **12.3ms —— 比 10ms 的控制周期还长**。原先"每 100ms 只刷一行"的
写法已经是把伤害压到最小了（一次刷四行是 50ms 的失控窗口），但 12.3ms 依然意味着：

1. 那一拍的控制输出最迟 12ms 才打到电机，相当于 1.2 拍纯延迟，直接吃相位裕度。
   在 VOFA+ 的 `roll` 曲线上表现为 **10Hz 的周期性晃动**——正好是刷新频率。
2. 更麻烦的是 `pid.c` 的 `imu_data_ready` 是标志位不是计数器。12.3ms 的阻塞窗口
   若跨过两次 MPU 中断（约 20% 的概率），两次置位只被消费一次，DMP FIFO 就
   **永久积压一包且不会自愈**；积压满 1024 字节还会触发 `mpu_reset_fifo()` 里的
   `HAL_Delay(50)`，那 50ms 车必倒。详见 `Imu_ControlPending()` 上方的注释。

关掉之后电机运行期间的最长阻塞不足 2ms，上面两条都不再可达。

顺带省下约 **5KB FLASH**：工程开了 `-ffunction-sections` + `--gc-sections`，`main.c`
里没有调用点之后，`oled.c` 的函数体和 `F6x8`/`F8X16` 字库会被链接器自动回收；
`MX_I2C1_Init()` 也一并关掉（I2C1 全工程只有 OLED 在用），整个 HAL_I2C 驱动跟着消失。
`oled.c` 本身没有改动，随时可以把宏改回 1。

> **改回 1 之前先想清楚**：屏是唯一"不接上位机也能看见"的反馈手段，代价是上面那
> 12.3ms。真需要屏的话，比直接打开更好的两条路是——把 `MX_I2C1_Init()` 的
> `ClockSpeed` 从 100000 提到 400000（12.3ms → 3.1ms，SSD1306 支持，但屏线太长或
> 上拉太弱时可能花屏），或者把 `OLED_ShowLine()` 改成 `HAL_I2C_Mem_Write_DMA()`
> （阻塞降到几微秒，需要加一个"上一笔没发完就跳过"的状态机）。
>
> 另外别用 `OLED_ShowString()` 刷高频内容：它每个字符要发 3 次定位命令 + 6 次数据，
> 每次都是一整笔 I2C 事务，一行 16 字符要 40ms 上下。

### 自动注册（auto registration）

加一条新命令只要两步，不用回头改任何注册列表：

```c
static int cliCmd_foo(int argc, char *argv[]) { ...; return 0; }
CLI_CMD_EXPORT(foo, "一句话说明", cliCmd_foo);
```

原理是 `CLI_CMD_EXPORT` 把一个 `const` 结构体塞进名为 `cli_cmd` 的自定义段：

```c
static const CliCmd_tTypeDef _cli_cmd_item_##_func
    __attribute__((used, section("cli_cmd"), aligned(4))) = { #_name, _help, _func }
```

链接器把所有 `.o` 里的同名段拼成一段连续内存，`cli.c` 直接当数组遍历。
`STM32F103C8Tx_FLASH.ld` 里对应的收集规则：

```
.cli_cmd :
{
  . = ALIGN(4);
  _cli_cmd_start = .;
  KEEP (*(cli_cmd))
  KEEP (*(SORT(cli_cmd.*)))
  _cli_cmd_end = .;
  . = ALIGN(4);
} >FLASH
```

三处细节缺一不可：`used` 防止编译器因"没人引用"优化掉结构体；`KEEP` 防止
`--gc-sections` 把整个段当垃圾回收；`aligned(4)` 保证表项紧密排列，
否则链接器插进填充字节就没法当数组遍历了。

验证是否真的注册上了：

```
arm-none-eabi-nm build/balance_car.elf | findstr _cli_cmd
```

`_cli_cmd_end - _cli_cmd_start` 应该等于 `命令条数 × 12`。

> **Keil 用户注意**：`Image$$cli_cmd$$Base/Limit` 只有在分散加载文件里
> 建了同名执行域才会存在，用 Keil 编译需要改用 scatter file 并加上：
> `cli_cmd +0 { *(cli_cmd) }`。GCC/Makefile 这条路径不受影响。

---

## 七、VOFA+ 波形调参

`Comm/comm_vofa.c` 把控制环内部量按 [VOFA+](https://www.vofa.plus/) 能直接画的格式吐到
同一个 USART3。调 PID 靠的是看"目标值和实测值差在哪、超调多少、几个周期收敛"，
在终端里刷数字根本看不出来，画成曲线一眼就明白。

### 开关

| 命令 | 作用 |
| ---- | ---- |
| `vofa` | 打印当前模式、分频、丢帧数，并列出八个通道的名字 |
| `vofa jf` | 打开 JustFloat（二进制，推荐） |
| `vofa fw` | 打开 FireWater（文本，肉眼可读） |
| `vofa off` | 关闭 |
| `vofa rate <n>` | 每 n 个控制周期发一帧，n 取 1~50。默认 2，即 50Hz |

分频按**控制周期数**而不是 `HAL_GetTick()` 计：主循环一圈的耗时并不均匀（`Control()`
本身是事件驱动的，加上 Flash 读写、CLI 的浮点格式化都会让某几圈变长），按时间分频
会出现同一拍发两帧或者连跳两拍，波形横轴就歪了。计数器是 `pid.c` 的 `control_cycle`，
`Control()` 跑完一次自增一次。

### 通道

八个通道分成三组，每组是调一个环需要的那几条线：

| 通道 | 名称 | 含义 | 调哪个环 |
| ---- | ---- | ---- | -------- |
| 0 | `roll` | 实测倾角(度) | 直立环 |
| 1 | `roll_ref` | 目标角 = `Med_Angle + Velocity_out` | 直立环 |
| 2 | `gyrox` | 角速度(已扣零偏)，直立环 D 项输入 | 直立环 |
| 3 | `vert_out` | 直立环输出，也就是基础 PWM | 直立环 |
| 4 | `enc_sum` | 实测速度 = 左右编码器之和 | 速度环 |
| 5 | `spd_ref` | 目标速度 | 速度环 |
| 6 | `gyroz` | 转向角速度，转向环 D 项输入 | 转向环 |
| 7 | `turn_out` | 转向环输出，叠加到左右轮上的差值 | 转向环 |

速度环的输出没有单列：它本来就是叠加在直立环给定上的，即 `ch1 - Med_Angle`。

八个值全部取自**同一次** `Control()` 的计算结果（`Vofa_Poll()` 在 `Comm_Poll()`
里、`Control()` 刚跑完之后执行），不会出现"角度是这一拍、输出是上一拍"的错位。

### 两种格式

**JustFloat**：8 个小端 float + 4 字节帧尾 `00 00 80 7F`，一帧 36 字节。
STM32 本机就是小端 IEEE754，直接按字节拷出去，不用逐个转换。

> 帧尾 `00 00 80 7F` 就是 `+inf` 的字节形态。所以通道值绝不能等于 `+inf`，
> 否则上位机会在数据中间切帧，之后所有曲线永久错位。`vofa_finite()` 把
> inf/NaN 一律换成 0，宁可丢一个点也不能让整屏波形失去意义。

**FireWater**：文本 `"1.23,4.56,...\n"`，一帧 55~70 字节。带宽是 JustFloat 的两倍，
但拿任何串口助手都能肉眼核对数值——怀疑通道对错位时用它对一下。

带宽账（115200bps 约 11.5KB/s）：

| 模式 | rate=1 (100Hz) | rate=2 (50Hz，默认) |
| ---- | -------------- | ------------------- |
| JustFloat | 3.6KB/s，约 31% | 1.8KB/s，约 16% |
| FireWater | 6.5KB/s，约 56% | 3.3KB/s，约 28% |

超出带宽时发送环形缓冲会整帧丢弃，`vofa` 或 `status` 里的 `drop` 会涨，调大 rate 即可。

### VOFA+ 上位机怎么配

1. 串口选蓝牙的**传出(Outgoing)** COM 口或 USB-TTL 口，115200 8N1。
2. 数据解析协议选 **JustFloat**（对应 `vofa jf`）或 **FireWater**（对应 `vofa fw`）。
3. 通道数填 8，顺序就是上面那张表。
4. VOFA+ 的命令输入框可以直接发 CLI 命令（记得末尾带 `\n`），所以调参不用再开一个终端。

### 改增益

```
car> pid                 # 不带参数，打印全部
vert: kp=480.00 kd=1.20
velo: kp=0.000 ki=0.000
turn: kp=10.00 kd=0.60(auto)
med : 3.10

car> pid vkp 520         # 改直立环 Kp
vkp = 520.000
```

| 参数 | 对应变量 | 说明 |
| ---- | -------- | ---- |
| `vkp` / `vkd` | `Vertical_Kp` / `Vertical_Kd` | 直立环 PD |
| `skp` / `ski` | `Velocity_Kp` / `Velocity_Ki` | 速度环 PI，改完自动清积分 |
| `tkp` | `Turn_Kp` | 转向环 P |
| `med` | `Med_Angle` | 平衡中值角，范围 ±20 |

增益上限 `COMM_PID_GAIN_ABS_MAX = 2000`，和二进制命令 `REV_CMD_PID_SET` 用的是同一套校验。
`tkd` 不接受赋值：`Turn_Kd` 每个控制周期都会被 `Control()` 按"是否正在转向"重算，写了留不住。

> **改动只在 RAM 里，掉电就没了**。调出满意的一组之后，请把值写回
> `Core/Src/pid.c` 开头的初值。故意不存 Flash：标定页只有 20 字节的结构，
> 加字段要同步改 `cali_store.h` 的版本号和长度，而调参是低频动作，不值得。

### 典型调参流程

级联结构必须**从内环往外调**，内环没站稳时外环的波形没有参考意义：

1. `vofa jf` 打开波形，VOFA+ 里对上 8 个通道。
2. **直立环**：速度环增益保持 0（默认就是 0），扶着车 `start`。
   看 ch0(`roll`) 能不能跟住 ch1(`roll_ref`)：
   - `pid vkp` 从小往大加，直到车能站住；加过头会出现高频抖动。
   - `pid vkd` 加阻尼压掉抖动；`vkd` 过大会引入噪声放大，ch3(`vert_out`) 上能看到毛刺。
3. **速度环**：`pid skp` 从小往大加，看 ch4(`enc_sum`) 向 ch5(`spd_ref`) 收敛的速度。
   `ski` 一般取 `skp/200` 量级起步。速度环太强会把直立环带得前后晃 —— 表现是
   ch1(`roll_ref`) 大幅摆动。
4. **转向环**：`pid tkp`，看 ch6/ch7。
5. 随时 `stop` 切电机。

> 直立环和速度环的极性搞反时，车会朝着倒下的方向加速。波形上很好认：
> ch0 偏离 ch1 之后 ch3 反而把它推得更远。这种情况先 `stop`，检查电机接线方向。

---

## 八、标定数据的 Flash 存储

| 项目 | 值 |
| ---- | -- |
| 地址 | `0x0800FC00 ~ 0x0800FFFF`（片内 Flash 最后一页，1K） |
| 内容 | magic(4) + version(2) + length(2) + med_angle(float) + gyrox_offset(int32) + crc16(2) + 保留(2) = 20 字节 |
| 校验 | magic + version + length + CRC16-MODBUS 四重校验 |

这一页已经在链接脚本里从代码区划走（`FLASH` 只给 63K），所以代码一旦涨过 63K
会**在链接期**报 `region FLASH overflowed`，而不是运行时悄悄把标定页覆盖掉。
改地址时 `STM32F103C8Tx_FLASH.ld` 和 `Storage/cali_store.h` 的 `CALI_STORE_ADDR`
必须同步改。

开机流程变化：现在优先读 Flash 里的标定值，读到就**跳过 18~22 秒静置等待**；
Flash 里没有（首次上电或数据损坏，空白 Flash 全是 `0xFF`，magic 必然对不上）
才现场标定并保存。

> 代价是陀螺零偏本来就随温度和上电次数漂移，复用旧值不如现场标定准。
> `pid.c` 的运行时零偏跟踪器会慢慢把差值吸收掉，但如果发现小车静止时缓慢跑偏，
> 敲一次 `cali` 重新标定即可。

---

## 九、代码分层

| 文件 | 职责 | 对应追觅文件 |
| ---- | ---- | ------------ |
| `comm_protocol.h` | 帧常量、命令ID、载荷结构 | `receive.h` / `send.h` 的定义部分 |
| `crc16.c/h` | CRC16-MODBUS | `Package/CRC_16.c` |
| `comm_pack.c/h` | 组帧/解帧状态机、转义 | `Package/pack.c` + `package.c` |
| `comm_port.c/h` | USART3 收发环形缓冲、HAL 回调 | `Application/task_com.c` |
| `comm_receive.c/h` | 下行命令分发表与处理 | `ComProtocol/receive.c` |
| `comm_send.c/h` | 上行上报与周期调度 | `ComProtocol/send.c` |
| `comm_vofa.c/h` | VOFA+ 波形输出：JustFloat / FireWater | — |
| `comm.c/h` | 门面：`Comm_Init()` / `Comm_Poll()`、帧外字节路由 | — |
| `comm_echo.c/h` | 命令回执统一出口（`[ev] ...`），兼作屏显快照 | — |
| `../Cli/cli.c/h` | CLI 框架：行编辑、回显、命令自动注册表 | — |
| `../Cli/cli_cmd.c` | 具体命令：cali / start / stop / status / pid / vofa | — |
| `../Storage/cali_store.c/h` | 标定数据 Flash 读写 | — |

数据流向：

```
接收: USART3 中断 -> HAL_UART_RxCpltCallback -> rxRing 环形缓冲
      -> [主循环] Comm_Poll -> CommPack_RxByte 解帧 -> cmdReceiveTab[cmd] -> 控制变量
发送: CommSend_xxx -> send2app -> CommPack_Send 组帧 -> txRing 环形缓冲
      -> HAL_UART_Transmit_IT -> HAL_UART_TxCpltCallback 续发
```

**中断里只搬字节，协议解析全部在主循环（线程态）完成**，因此命令处理函数可以
放心读写 PID 参数、`Target_Speed` 等控制变量，不会和 `Control()` 产生竞态。
发送也必须走中断：阻塞发送会挤占平衡控制周期，
会直接挤掉 10ms 的平衡控制周期，小车必倒。

---

## 十、调试入口

1. **看解帧统计**：在调试器里 watch `commPackStat`。
   `rxFrame` 不涨说明帧根本没收到（查波特率/接线/蓝牙配对）；
   `crcError` 涨说明上位机 CRC 算错（重点查是否漏算了 len 和 cmd 字节、是否对转义后的字节算了 CRC）；
   `frameError` 涨说明转义规则实现有问题。
2. **看端口统计**：watch `commPortStat`。
   `uartError` 涨=接收溢出（主循环被什么东西阻塞太久）；
   `rxDrop` 涨=接收缓冲 256 字节不够，说明 `Comm_Poll()` 调用间隔太长
   （115200 下每毫秒进 11.5 字节，256 字节只兜得住约 22ms 的阻塞）；
   `txDropFrame` 涨=上报太密超出发送带宽，调大 `period_10ms`；
   开着 VOFA+ 波形时先看 `vofa` 里的 `drop`，是它的话调大 `vofa rate`。
3. **不接手机也能测**：用 USB-TTL 直连 PB10/PB11，115200 8N1。
   最快的办法是切到文本模式敲 `help`：能列出命令说明收发两条链路都通了，
   再敲 `status` 就能直接看到上面那两组统计值，不用连调试器。
   要验二进制链路，按十六进制发"查询版本" `3C 00 08 76 00 3E`，
   收到 `SEND_ID_VERSION` 即可。
4. **开机横幅是第一条证据**：`OLED_ENABLE` 默认为 0，小车没有任何本地显示，所以
   "固件跑起来了没有"只能看串口。开机横幅（见"六、串口上能看到什么"）能打出来，
   就说明时钟、波特率、发送链路全都对；蓝牙是后连的话敲一次回车拿 `car> ` 提示符，
   效果一样。敲字符没有回显 = 字节压根没进来，查波特率和 TX/RX；有回显但不执行
   = 收得到字符但没收到回车，查终端的换行设置；`[ev] crc err n` 一路涨 = 波特率
   对不上或上位机 CRC 算错。
5. **首次上电要静置 20s**：Flash 里没有有效标定时会现场标定，而 `Comm_Init()` 排在
   标定之后，这 20 秒里串口是哑的，属正常现象。等横幅出来再敲命令。
