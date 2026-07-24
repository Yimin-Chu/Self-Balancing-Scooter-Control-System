---
name: embedded-linker-auto-registration
description: >-
  Implement decoupled auto-registration in C firmware using linker sections so
  that CLI commands, init routines, or plugin tables register themselves by
  declaration (no central table edits). Use when building a shell/CLI, an
  auto-init/component framework, test-case tables, or any "add a file and it
  just works" registry. Also use to understand RT-Thread INIT_*_EXPORT / FinSH.
---

# 链接器段自动注册 / Linker-Section Auto-Registration

核心思想：把同类条目放进**同名 section**，链接器聚成连续数组；再放两个**哨兵 (sentinel)** 标出首尾，运行时遍历。新增条目只需声明一个变量，无需改任何中央表。

## 模式 / Pattern

```c
struct CliCmdList { const char *name; void (*func)(void); };

// section 名带排序级别：链接器按名字排序 -> START(0.end) < 命令(1) < END(1.end)
#define CLI_SECTION(level)  __attribute__((used, __section__(".cli_cmd." level)))
#define CLI_CMD_START(n,f)  const struct CliCmdList cmd_##f CLI_SECTION("0.end") = {n,f}
#define CLI_CMD_FUNC(n,f)   const struct CliCmdList cmd_##f CLI_SECTION("1")     = {n,f}
#define CLI_CMD_END(n,f)    const struct CliCmdList cmd_##f CLI_SECTION("1.end") = {n,f}
```

定义首尾哨兵各一次，命令随处声明：

```c
CLI_CMD_START("start", cliStart);   // 只出现一次
CLI_CMD_END  ("end",   cliEnd);     // 只出现一次
CLI_CMD_FUNC ("reset", Cli_Reset);  // 任意文件里，随手加
```

遍历执行：

```c
const struct CliCmdList *p;
for (p = &cmd_cliStart + 1; p < &cmd_cliEnd; p++)
    if (strcmp(name, p->name) == 0) { p->func(); break; }
```

## 要点 / Notes

- `used` 属性防止 `-O` 把"没人引用"的注册项优化掉。
- 用 `.0.end / .1 / .1.end` 级别名保证哨兵排在两端（依赖链接器按 section 名排序；GNU ld / armlink 均支持）。
- RT-Thread 的 `INIT_BOARD_EXPORT / INIT_PREV_EXPORT / INIT_APP_EXPORT` 是同一原理：分级 section 决定**初始化顺序**，`rt_components_init()` 自动遍历，故 `main()` 可为空。
- 通用性：CLI 命令、开机自检项、单元测试用例表、协议处理器表都适用。

## 陷阱 / Pitfalls

- LTO 或裁剪链接可能吃掉注册项 → 保留 `used`，必要时链接脚本 `KEEP()`。
- 跨编译器移植：IAR 用 `@ "section"` 语法、`__root`；需在链接脚本/icf 里声明段。
