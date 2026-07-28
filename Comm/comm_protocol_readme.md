# 平衡小车蓝牙串口通信协议

物理层：USART3(PB10=TX / PB11=RX)，9600 8N1，接 JDY-31 蓝牙模块。
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
2. **带宽**：9600bps 实际只有约 960 字节/秒。姿态+运动两帧约 36 字节，
   100ms 一轮占用 360 字节/秒，比较合适。上报周期最小 50ms，再快会挤爆发送缓冲
   （表现为 `commPortStat.txDropFrame` 递增，整帧被丢弃而不会发半帧）。
3. **旧单字节遥控码兼容**：帧外的裸字节仍按老规则处理
   （0x01 前进 / 0x05 后退 / 0x03 右转 / 0x07 左转 / 0x09 停止 / 其他松开），
   方便 App 分步升级。App 切换完成后把 `comm.h` 里的
   `COMM_LEGACY_BYTE_CMD_ENABLE` 改成 0 关掉——留着它的风险是任何噪声字节
   都可能被当成运动指令。
4. **上电默认状态**：`motor_enable = 0`，电机不转；自动上报 100ms 一次姿态+运动。

---

## 六、代码分层

| 文件 | 职责 | 对应追觅文件 |
| ---- | ---- | ------------ |
| `comm_protocol.h` | 帧常量、命令ID、载荷结构 | `receive.h` / `send.h` 的定义部分 |
| `crc16.c/h` | CRC16-MODBUS | `Package/CRC_16.c` |
| `comm_pack.c/h` | 组帧/解帧状态机、转义 | `Package/pack.c` + `package.c` |
| `comm_port.c/h` | USART3 收发环形缓冲、HAL 回调 | `Application/task_com.c` |
| `comm_receive.c/h` | 下行命令分发表与处理 | `ComProtocol/receive.c` |
| `comm_send.c/h` | 上行上报与周期调度 | `ComProtocol/send.c` |
| `comm.c/h` | 门面：`Comm_Init()` / `Comm_Poll()` | — |

数据流向：

```
接收: USART3 中断 -> HAL_UART_RxCpltCallback -> rxRing 环形缓冲
      -> [主循环] Comm_Poll -> CommPack_RxByte 解帧 -> cmdReceiveTab[cmd] -> 控制变量
发送: CommSend_xxx -> send2app -> CommPack_Send 组帧 -> txRing 环形缓冲
      -> HAL_UART_Transmit_IT -> HAL_UART_TxCpltCallback 续发
```

**中断里只搬字节，协议解析全部在主循环（线程态）完成**，因此命令处理函数可以
放心读写 PID 参数、`Target_Speed` 等控制变量，不会和 `Control()` 产生竞态。
发送也必须走中断：9600bps 下 1 字节约 1.04ms，一帧 20 字节阻塞发送要 20ms，
会直接挤掉 10ms 的平衡控制周期，小车必倒。

---

## 七、调试入口

1. **看解帧统计**：在调试器里 watch `commPackStat`。
   `rxFrame` 不涨说明帧根本没收到（查波特率/接线/蓝牙配对）；
   `crcError` 涨说明上位机 CRC 算错（重点查是否漏算了 len 和 cmd 字节、是否对转义后的字节算了 CRC）；
   `frameError` 涨说明转义规则实现有问题。
2. **看端口统计**：watch `commPortStat`。
   `uartError` 涨=接收溢出（主循环被什么东西阻塞太久）；
   `rxDrop` 涨=接收缓冲 64 字节不够，说明 `Comm_Poll()` 调用间隔太长；
   `txDropFrame` 涨=上报太密超出 9600bps 带宽，调大 `period_10ms`。
3. **不接手机也能测**：用 USB-TTL 直连 PB10/PB11，串口工具按十六进制发上面的示例帧，
   看能否收到 ACK / 上报帧。先发"查询版本" `3C 00 08 76 00 3E`，
   收到 `SEND_ID_VERSION` 就说明收发两条链路都通了。
