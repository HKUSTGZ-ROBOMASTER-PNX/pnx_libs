# pnx_libraries

通用算法与工具库（`numeric` / `matrix` / `crc` / `pid` / `filter` / `tools`）。

## 依赖

- `pnx_platform`、`pnx_os`（头路径与 CMSIS-DSP 链接由集成仓注入）

## CMake 目标

`pnx_libraries`（静态库）

## Submodule

```bash
git submodule add <url> pnx_libraries
```

## 约定

- 头文件：`pnx_libraries/<module>/include/`
- 源文件：`pnx_libraries/<module>/src/`
