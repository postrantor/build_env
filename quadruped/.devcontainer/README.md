---
date: 2025-01-23 19:12:21
---

根据提供的 `devcontainer.json` 和 `Dockerfile`，以下是手动构建镜像和运行容器的等效命令行操作：

---

### 1. 构建 Docker 镜像

```bash
# 进入项目根目录（假设 .devcontainer 在项目根目录下）
cd /home/trantor/build_env/quadruped

# 构建镜像
docker build -t quadruped-dev \
  --build-arg WORKSPACE=/home/ros/quadruped \
  --build-arg REMOTE_USER=ros \
  -f .devcontainer/Dockerfile \
  .
```

#### 参数说明：

- `-t quadruped-dev`：镜像标签名称。
- `--build-arg WORKSPACE=...`：对应 `devcontainer.json` 中的 `containerWorkspaceFolder`（默认为容器内工作区路径）。
- `--build-arg REMOTE_USER=ros`：定义 Dockerfile 中的用户名为 `ros`。
- `-f .devcontainer/Dockerfile`：指定 Dockerfile 路径。

---

### 2. 运行 Docker 容器

```bash
docker run -it --rm \
  --name quadruped-container \
  --network=host \
  --cap-add=SYS_PTRACE \
  --security-opt seccomp:unconfined \
  --security-opt apparmor:unconfined \
  --volume=/tmp/.X11-unix:/tmp/.X11-unix \
  --volume=/mnt/wslg:/mnt/wslg \
  --ipc=host \
  -e DISPLAY="$DISPLAY" \
  -e WAYLAND_DISPLAY="$WAYLAND_DISPLAY" \
  -e XDG_RUNTIME_DIR="$XDG_RUNTIME_DIR" \
  -e PULSE_SERVER="$PULSE_SERVER" \
  -e LIBGL_ALWAYS_SOFTWARE=1 \
  --user ros \
  -v "$(pwd)":/home/ros/quadruped \
  quadruped-dev \
  /bin/bash
```

#### 参数说明：

- `--network=host`：使用主机网络模式。
- `--cap-add=SYS_PTRACE`：允许调试。
- `--security-opt`：禁用安全配置。
- `--volume`：挂载 X11/WSLG 相关路径以实现 GUI 支持。
- `-e`：传递环境变量（确保主机已定义 `DISPLAY` 等变量）。
- `--user ros`：以 `ros` 用户身份运行。
- `-v "$(pwd)":/home/ros/quadruped`：将当前目录挂载到容器内工作区。

---

### 3. 验证环境

进入容器后，检查以下内容：

```bash
# 检查用户
whoami  # 应输出 `ros`

# 检查工作区
ls ~/quadruped  # 应看到挂载的项目文件

# 检查 GUI 环境（例如运行 rviz2）
ros2 run rviz2 rviz2
```

---

### 补充说明

1. **GUI 支持**：

   - 如果使用 Linux，确保主机已安装 X Server 并运行 `xhost +local:docker`。
   - 如果使用 WSL2，需要额外配置 Windows 的 X Server（如 VcXsrv）。

2. **挂载路径**：
   `-v "$(pwd)":/home/ros/quadruped` 将主机项目目录映射到容器内，确保与 `WORKSPACE` 参数一致。

3. **依赖项安装**：
   如果需要额外软件包，取消 Dockerfile 中 `RUN apt-get update...` 部分的注释并添加包名。

4. **硬件加速**：
   若需使用 GPU，取消 `devcontainer.json` 中的 `--device=/dev/dri` 并在运行时添加 `--device=/dev/dri`。
