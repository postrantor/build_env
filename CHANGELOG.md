---
date: 2026-08-21
update: 2026-08-21 09:56:27
---

# Changelog

面向 ROS 2 Humble 的多工程元工作区变更记录。版本号按 SemVer；tag 带工程前缀 (`hexa/vX.Y.Z`)。

## [Unreleased]

- 根 `deploy.bash --project build_env` 透传到 `common/setup/deploy.bash`：按 URL / 版本 clone，未给 `--path` 时浅克隆到 `/tmp/build_env` 并写入 `BUILD_ENV_ROOT` 与 `PATH`。
- 各工程 `.deploy/hostrc.env` 只向当前 shell 提供环境 (`source deploy.bash --project <name> hostrc`), 不改系统文件。
- `docs/README.md` 重命名为 `docs/index.md`；用法见 `docs/usage/deploy.md`。
- 版本与发布约定从 `ai2-robot/.deploy/docs/design/` 迁到 [`docs/design/`](docs/design/index.md)；配图在 [`docs/media/design/`](docs/media/design/version-management/)。

## [hexa/v0.1.0] - 2026-08-21

首个可标记快照。当前仓库已从单工程骨架扩成并列工作区，宿主机 setup 与工程部署入口分开。

### 当前状态

- **并列工程**：`ai2-robot`、`hexafuture`、`real_cap-x`、`robocasa`。身份由各工程 `.config/project.env` 决定 (`AI2` / `HEXA` / `RCX` / `CASA`)。
- **共享 vs 私有**：`common/` 提供 env、setup、格式化与 `keybindings.json`；`.config/`、`.devcontainer/`、其余 VSCode 文件为各工程副本。业务源码不入库，由 `vcs import` 写入 `<project>/src/`。
- **`setup.bash`**：只改宿主机 (用户、hosts、ssh、docker daemon、网络、桌面)。机型 `orin` / `woosh` 在 `common/setup/machines/` 自动扫描。
- **`RUNME.bash`**：各工程工作区初始化 (`.env`、vcs、`COLCON_IGNORE`、可选预编译 deb)。`COLCON_HOME` 按 `cpu|gpu` × `arm|x86` 选择。
- **`deploy.bash`**：按 `--project` 透传到 `<project>/.deploy/{install,dump}.bash`。目前仅 `ai2-robot` 具备 `.deploy/`。
- **文档**：仓库级说明在 `docs/` (架构 + `usage/`)；深度安装与排障仍在 `ai2-robot/docs/`。
