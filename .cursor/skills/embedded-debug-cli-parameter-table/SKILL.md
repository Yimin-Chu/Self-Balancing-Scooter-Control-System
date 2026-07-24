---
name: embedded-debug-cli-parameter-table
description: >-
  Build a data-driven debug system for firmware: a parameter table
  {name,addr,min,max,type} that a generic CLI can get/set/monitor/save without
  per-variable code, RELEASE-time name stripping to save flash, and a single
  UART shared by multiple protocols (CLI/log/factory-test) via a dispatch
  table. Use when adding runtime tuning, a debug shell, telemetry, or NVM save.
---

# 数据驱动调试与参数表 / Data-Driven Debug & Parameter Table

## 1. 参数表 / Parameter table

把"可调/可观测变量"登记成表，CLI 全泛型操作，加变量=加一行，不改 CLI。

```c
enum paraType { PT_U8, PT_S8, PT_H8, PT_B8, PT_U16, /* ... */ PT_FLOAT, /* ... */ };

struct Setting_Data_Type {           // 可写：带范围
    const char *name; uint8_t *dataAdd;
    const int16_t min, max; const enum paraType type;
};
struct Status_Data_Type {            // 只读：观测
    const char *name; uint8_t *dataAdd; const enum paraType type;
};

const struct Setting_Data_Type Setting_Parameter[] = {
    SETP("FanVolt",  (uint8_t*)&fan.pwmVolt, 0, FAN_MAX_VOLTAGE, PT_S16),
    SETP("MopPTCKp", (uint8_t*)&PidMopPTC.Kp, 0, 30000, PT_U16),
};
```

CLI 靠 `{addr,type}` 泛型读写：`num2Str()` 按 type 格式化，`Cli_Set()` 按 type 做范围校验后写回地址。命令：`info -a` 列表、`get /name`、`set /name val`、`mon -f ms /name...` 周期监控、`save -a` 存 Flash。

## 2. RELEASE 剥离省 Flash / Strip on release

```c
#ifndef RELEASE
  #define SETP(a,b,c,d,e) { a,b,c,d,e }     // 调试版：名字+范围
#else
  #define SETP(a,b,c,d)   { b }             // 量产版：只留地址，省字符串/范围
#endif
```

## 3. 单串口多协议分发 / One UART, many protocols

一路调试串口被 CLI / 日志上传 / 产测 复用，用分发表切换"当前解析器"：

```c
typedef struct { uint32_t (*analysisByteFunc)(uint8_t); void (*overTimeFunc)(void); }
        DebugMultiProcessFunc_sTypeDef;

DebugMultiProcessFunc_sTypeDef MultiRxProFuncTab[DEBUG_FUNC_TOTAL] = {
    {Cli_AnalysisInByte,   cliVoidFun},
    {LogUp_AnalysisInByte, LogUp_OverTime},
    {fct_receiveByte,      fct_receiveTimeout},
};
// 收到字节： MultiRxProFuncTab[debugFuncUse].analysisByteFunc(byte);
```

## 准则 / Rules

- 变量地址用 `(uint8_t*)&var` + type 枚举做"手写反射"，避免每个变量写 get/set。
- `set` 必须先范围校验再落值；写 Flash 前先 diff（脏才写）延长寿命。
- 监控 (`mon`) 用软件定时器周期打印，配合 VT100 光标控制做原地刷新。
- 量产固件用 `RELEASE` 剥离名字与命令，既省 Flash 又防止现场被随意改参数。
