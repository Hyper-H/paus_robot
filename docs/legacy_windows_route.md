# Legacy Windows Route

这个文档用于记录项目早期采用的 `Windows + WSL` 混合路线。

## 包含内容

- Windows 相机桥接
- Windows FAIRINO 执行桥
- WSL 与 Windows 之间的 TCP/JSON 桥接

对应仓库内保留位置：

- [legacy/hybrid_stack/docs/fairino_windows_bridge.md](/root/projects/ag-repro/legacy/hybrid_stack/docs/fairino_windows_bridge.md)
- [legacy/hybrid_stack/scripts/test_fairino_exec_bridge.py](/root/projects/ag-repro/legacy/hybrid_stack/scripts/test_fairino_exec_bridge.py)
- [legacy/hybrid_stack/scripts/restart_marker_stack.sh](/root/projects/ag-repro/legacy/hybrid_stack/scripts/restart_marker_stack.sh)
- [legacy/hybrid_stack/src/ag_repro/execution_bridge.py](/root/projects/ag-repro/legacy/hybrid_stack/src/ag_repro/execution_bridge.py)

## 为什么降级为 legacy

- 环境复杂度高
- Windows 与 WSL 双向桥接调试成本高
- 执行层和感知层分散在两个系统里，维护成本高
- 已决定后续切换到纯 Linux / 原生 Ubuntu 主线

## 当前定位

这些内容：

- 不删除
- 不继续作为默认主线维护
- 仅用于回溯、参考和必要时的临时排查
