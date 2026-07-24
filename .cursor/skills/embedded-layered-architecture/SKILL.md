---
name: embedded-layered-architecture
description: >-
  Scaffold and review embedded firmware directory structure and layering:
  driver/BSP vs application vs algorithm isolation, dependency direction, and
  where each file belongs. Use when starting a new MCU/RTOS project, adding a
  module, or reviewing whether code respects layer boundaries.
---

# 嵌入式分层架构 / Embedded Layered Architecture

参考「追觅一体化基站平台」验证过的分层法。目标：**上层依赖下层的抽象接口，下层永不依赖上层；驱动、应用、算法互不渗透。**

## 目录分层 / Layout（推荐）

| 层 | 目录 | 只允许依赖 |
|---|---|---|
| 入口 Entry | `user/` | bsp, application |
| 驱动/HAL Driver | `bsp/<board>/` | 厂商 HAL only |
| 应用胶水 App glue | `application/` | bsp, rtos |
| 执行器 Actuator | `motor/` | bsp |
| 传感器 Sensor | `device&sensor/` | bsp |
| 功能模块 Module | `module/` | motor, sensor, hmi |
| 人机 HMI | `hmi/` | bsp |
| 控制流 Control | `state_machine/` | module |
| 数据 Data | `parameter_list/` | (被 CLI 引用) |
| 调试 Debug | `cli_tool/ cm_backtrace/ segger/` | 任意（只读观测） |

## 依赖铁律 / Dependency Rules

1. 应用/模块层**禁止**直接调用厂商 HAL 或写寄存器，只经 `bsp_*` 接口。
2. 驱动层**禁止** `#include` 任何 `module/state_machine/` 头文件（无反向依赖）。
3. 算法（PID、滤波、状态机逻辑）**不碰硬件**：入参给数据、出参给控制量，便于单测。
4. 硬件差异统一收敛到 `bsp/<board>/bsp_config.h` 与 `project_config.h`，业务代码里不出现具体外设名。

## 评审清单 / Review Checklist

- [ ] 新文件放在正确的层，未跨层 include
- [ ] 未在 `motor/`、`module/` 里出现 `xxx_hal.h` / 寄存器地址
- [ ] 算法函数无副作用、可脱离硬件调用
- [ ] 硬件相关常量落在 config，而非散落在逻辑里

## 判层口诀 / Where does this file go?

「碰寄存器→bsp；驱动一个器件→motor/device；编排多个器件完成一个功能→module；只做数学→algorithm；只做观测→debug。」
