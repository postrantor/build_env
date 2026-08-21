---
tip: build_env 元工作区总览；细节见 docs/
date: 2026-04-16
update: 2026-08-21
---

# build_env

面向 **ROS 2 Humble** 的多工程元工作区：集中提供 Shell 模板、colcon 与 vcs 配置、Dev Container 与编辑器相关文件。当前并列工程为 **ai2-robot**、**hexafuture**、**real_cap-x**、**robocasa**；业务源码在各自目录 `vcs import` 之后位于 `<project>/src/`，本仓库不跟踪完整源码树。

共享逻辑在 `common/`，各工程通过符号链接复用 env、格式化配置与 `keybindings.json`；colcon / vcs / Dev Container / `entrypoint_additions` 以及其余 VSCode 文件在各工程自己的 `.config/`、`.devcontainer/`、`.vscode/`。

上文即本仓库的**目标**。工程文档聚在 [`docs/`](docs/index.md) (布局同功能包 `docs/`：架构 + `design/` + `usage/`)；本 README 只做入口。

## 从哪里开始

| 你想做的事                                   | 建议入口                                                                                     |
| -------------------------------------------- | -------------------------------------------------------------------------------------------- |
| 文档索引                                     | [docs/index.md](docs/index.md)                                                               |
| 了解工作区结构、环境分层、配置索引与排障脉络 | [docs/architecture.md](docs/architecture.md)                                                 |
| 初始化 ai2-robot                             | [ai2-robot/RUNME.bash](ai2-robot/RUNME.bash)、[ai2-robot/README.md](ai2-robot/README.md)     |
| 初始化 hexafuture                            | [hexafuture/RUNME.bash](hexafuture/RUNME.bash)、[hexafuture/README.md](hexafuture/README.md) |
| 初始化 real_cap-x                            | [real_cap-x/RUNME.bash](real_cap-x/RUNME.bash)、[real_cap-x/README.md](real_cap-x/README.md) |
| 初始化 robocasa                              | [robocasa/RUNME.bash](robocasa/RUNME.bash)、[robocasa/README.md](robocasa/README.md)         |
| 配置宿主机 (用户、hosts、docker、网络)       | [setup.bash](setup.bash)、[docs/usage/setup.md](docs/usage/setup.md)                         |
| 部署本仓库或工程 `.deploy/`                  | [deploy.bash](deploy.bash)、[docs/usage/deploy.md](docs/usage/deploy.md)                     |
| Docker 开发容器                              | [docs/usage/docker.md](docs/usage/docker.md)                                                 |

## 文档

| 文档                                             | 说明                                      |
| ------------------------------------------------ | ----------------------------------------- |
| [docs/architecture.md](docs/architecture.md)     | 工作区架构与排障                          |
| [docs/design/index.md](docs/design/index.md)     | 分支 / 标签 / 发布产物与 deploy 版本语义  |
| [docs/usage/setup.md](docs/usage/setup.md)       | 宿主机 setup                              |
| [docs/usage/deploy.md](docs/usage/deploy.md)     | 根 `deploy.bash` 与 `--project build_env` |
| [docs/usage/machines.md](docs/usage/machines.md) | 机型自动扫描                              |
| [docs/usage/desktop.md](docs/usage/desktop.md)   | 物理机桌面配置                            |
| [docs/usage/docker.md](docs/usage/docker.md)     | `container-create` / bind-mount           |
| [docs/usage/media.md](docs/usage/media.md)       | `media` / `media-compress` 批量视频压缩   |

完整索引见 [docs/index.md](docs/index.md)。

## 顶层目录 (一句话)

- **`ai2-robot/`** / **`hexafuture/`** / **`real_cap-x/`** / **`robocasa/`** — 各 ROS 2 工作区根：`RUNME.bash`、`src/` (由 vcs 填充) 等。
- **`common/`** — 共享 env、宿主机 setup、格式化配置、VSCode `keybindings.json`。
- **`setup.bash`** — 宿主机部署入口 (`common/setup/`)。
- **`deploy.bash`** — 按 `--project` 透传到 `common/setup/deploy.bash` 或 `<name>/.deploy/`。
- **`docs/`** — 本仓库文档；索引 [`docs/index.md`](docs/index.md)。
