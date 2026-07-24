---
name: embedded-hw-portability-config
description: >-
  Make one firmware codebase support many hardware variants using compile-time
  Config-Wizard headers (project_config.h + IS_CONFIG_ENABLE), a per-board BSP
  folder with identical interfaces, and the strategy pattern for device
  variants. Use when porting firmware to a new board, adding a hardware SKU, or
  abstracting a peripheral that has multiple vendor implementations.
---

# 一套应用适配多硬件 / One App, Many Boards

三根支柱：**编译期配置裁剪 + 每板 BSP 目录 + 器件策略模式**。

## 1. 编译期特性开关 / Config-Wizard

`project_config.h` 每板一份，用 Keil Configuration Wizard 注释可 GUI 编辑：

```c
#define CONFIG_ENABLE  (1)
#define CONFIG_DISABLE (2)
#define IS_CONFIG_ENABLE(m) ((m) == CONFIG_ENABLE)

// <e> Use Heat Module
#define MODULE_USE_HEAT_MODULE 1
// <o> Select the type of heat module
//  <1=> Tyrek  <2=> SaiNa  <3=> Dreame
#define MODULE_HEAT_MODULE_TYPE 3
```

业务代码用 `#if IS_CONFIG_ENABLE(MODULE_USE_HEAT_MODULE)` 包裹；用**双值（1/2）而非 0/1**，可捕获"忘了定义"的笔误。

## 2. 每板一个 BSP 目录 / Per-board BSP

```
bsp/<board>/
  bsp_uart.c/h  bsp_i2c  bsp_pwm  bsp_adc  bsp_dma  bsp_gpio
  bsp_time  bsp_rtc  bsp_wdgt  bsp_exit
  bsp_config.h      # 抽象名 -> 具体外设 + 中断优先级枚举
  project_config.h  # 该板的特性开关
  rtconfig.h        # 该板 RTOS 裁剪
```

`bsp_config.h` 把**抽象角色**映射到**具体外设**，业务只认抽象名：

```c
#define CLI_UART      USART0
#define CLI_TX_DMA    DMA0
#define CLI_RX_BYTE_IRQ USART0_IRQHandler
typedef enum { ISR_PRIORITY_TIMER4=0, ISR_PRIORITY_USART0, /* ... */ ISR_PRIORITY_TOTAL } IsrPri_eTypeDef;
```

换板 = 换一个 `bsp/<board>/`，应用代码零改动。构建系统按目标板选择对应 BSP 目录进编译。

## 3. 器件多实现的策略模式 / Strategy Pattern

同类器件有多个厂商实现时：公共头声明统一接口，变体各自 `.c/.h`，配置选型。

```c
// heat_module.h
#if (/* ... */ MODULE_HEAT_MODULE_TYPE == HEAT_MODULE_TYREK)
#include "heat_module_tyrek.h"
#elif (/* ... */ == HEAT_MODULE_SAINA)
#include "heat_module_saina.h"
#elif (/* ... */ == HEAT_MODULE_DREAME)
#include "heat_module_dreame.h"
#endif
```

同款套路也用于霍尔检测（binary/analog/voltage/block）、电机控制（PWM/IO）。

## 移植新板清单 / Port Checklist

- [ ] 复制最接近的 `bsp/<board>/` 为新目录
- [ ] 改 `bsp_config.h` 的外设映射 + 中断优先级枚举
- [ ] 改 `project_config.h` 开关与器件选型
- [ ] 改 `rtconfig.h` 的 RTOS 裁剪
- [ ] 构建脚本加入新板 target；应用层不动
