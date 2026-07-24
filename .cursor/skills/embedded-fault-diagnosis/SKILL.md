---
name: embedded-fault-diagnosis
description: >-
  Add production-grade fault diagnosis to firmware: CmBacktrace-style HardFault
  call-stack capture, SEGGER RTT/SystemView low-overhead tracing, and Git
  version/build-time embedding into the binary for field triage. Use when
  setting up crash diagnostics, tracing RTOS timing, or making firmware report
  its exact source version.
---

# 故障诊断基建 / Fault Diagnosis Infrastructure

三件套，开机即挂载，现场问题可回溯。

## 1. HardFault 调用栈回溯 / CmBacktrace

开机在 board-init 阶段挂上，崩溃时打印出错的函数调用栈地址，配 `addr2line` 还原到源码行。

```c
int rtt_backtrace_init(void) {
    SEGGER_RTT_Init();
#ifdef USE_SYSTEMVIEW
    SEGGER_SYSVIEW_Conf();
#endif
    cm_backtrace_init("Universal_Dock", CodeVersion, PUBLIC_VERSION_NUMBER);
    rt_cm_backtrace_init();          // 接管 HardFault_Handler
    return 0;
}
INIT_BOARD_EXPORT(rtt_backtrace_init);
```

## 2. RTT / SystemView 低开销追踪 / Tracing

- RTT：不占 UART，J-Link 直读 RAM 环形缓冲，`printf` 级开销极低。
- SystemView：可视化任务切换/中断时序，抓竞态、优先级反转、时序抖动。

## 3. 版本内嵌 / Version stamping

把 Git 信息编进固件，现场一条命令确认版本，杜绝"装错固件"：

```c
CLIecho("Project:%s GitVer:%s CodeVer:\"%s\" Build@%s-%s\r\n",
        ProjectName, PUBLIC_VERSION_NUMBER, CodeVersion, __DATE__, __TIME__);
// ver -c/-p/-t/-b/-d 分别查 code/public/project/branch/buildtime
```

`public_version.h`（含 `PUBLIC_VERSION_NUMBER`、`GIT_BRANCH_NAME`、`BUILD_TIME`）由构建脚本生成。

## 上线清单 / Checklist

- [ ] `cm_backtrace` 接管 HardFault，量产版也保留（只读、无副作用）
- [ ] 崩溃栈能被 CI 里的 `addr2line` 脚本还原
- [ ] 版本号由构建自动注入，禁止手改
- [ ] RTT/SystemView 仅调试态编译进去，量产可宏关闭
