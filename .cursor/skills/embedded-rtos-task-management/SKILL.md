---
name: embedded-rtos-task-management
description: >-
  Organize RTOS threads with a single centralized priority/stack table, self-
  registering tasks via auto-init, and a cooperative event-flag watchdog that
  pinpoints which task hung. Use when creating RTOS tasks, assigning priorities
  or stack sizes, designing a software watchdog, or reviewing thread layout on
  RT-Thread/FreeRTOS.
---

# RTOS 线程管理 / RTOS Task Management

## 1. 集中式优先级与栈表 / Central priority & stack table

所有任务优先级放在**一个枚举**里，**声明顺序即优先级**（0 最高），一眼看清抢占关系；栈深用宏集中定义，便于统一调栈。

```c
// task_manage.h —— 唯一真源
typedef enum {
    IR_TASK_PRIORITY = TASK_PRIORITY_HIGHEST,
    KEY_TASK_PRIORITY, LED_TASK_PRIORITY, MOTOR_TASK_PRIORITY,
    STATE_TASK_PRIORITY, CLI_TASK_PRIORITY, WATCHDOG_TASK_PRIORITY,
    TOTAL_PRIORITY,
} TaskPriority_eTypeDef;

#define MOTOR_TASK_STACK_DEPTH (512)
#define CLI_TASK_STACK_DEPTH   (1024)
#define TASK_THREAD_TIMESLICE  (5)
```

好处：改优先级只动一处、不冲突；`TOTAL_PRIORITY` 自动等于任务数。

## 2. 任务自注册 / Self-registering tasks

每个 `task_*.c` 自带 init，用自动初始化导出，`main` 不用手动逐个 create：

```c
static int TaskXxxInit(void) {
    rt_thread_t t = rt_thread_create("xxx", TaskXxx, RT_NULL,
                     XXX_TASK_STACK_DEPTH, XXX_TASK_PRIORITY, TASK_THREAD_TIMESLICE);
    if (t) rt_thread_startup(t);
    return t != RT_NULL;
}
INIT_APP_EXPORT(TaskXxxInit);      // 资源(信号量/邮箱)用 INIT_PREV_EXPORT 先建
```

## 3. 协作式事件标志看门狗 / Cooperative watchdog

硬件狗只在**所有关键任务都活着**时才喂；任一任务卡死 → 缺它的 bit → 不喂 → 复位，且日志能报出缺哪个 bit。

```c
#define WATCHDOG_BIT_ALL (BIT(LED_TASK_PRIORITY)|BIT(MOTOR_TASK_PRIORITY)/* ... */)

// 各任务循环末尾： rt_event_send(watch_dog_event, BIT(SELF_PRIORITY));
// 看门狗任务：
rt_event_recv(watch_dog_event, WATCHDOG_BIT_ALL,
              RT_EVENT_FLAG_AND | RT_EVENT_FLAG_CLEAR, 500, &recved);
if ((recved & WATCHDOG_BIT_ALL) == WATCHDOG_BIT_ALL) fwdgt_counter_reload();
else RLOG_E("task stuck, event=%d", recved);   // recved 缺的位=卡死任务
```

## 设计准则 / Rules

- 优先级：中断代理/时间敏感(IR/KEY) > 电机/控制 > 状态机 > CLI/日志 > 看门狗兜底。
- 用 `RT_ASSERT` 校验 create 结果（debug 版）；release 版判空跳过。
- 栈深靠 `cpu -a` / 高水位(`max used`) 实测回调，不拍脑袋。
- 只有"活着=有意义地跑完一轮"的任务才纳入看门狗位图；纯等待型任务别混进去。
