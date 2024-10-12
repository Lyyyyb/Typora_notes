# 在 Ubuntu 20.04 上使用 Lux 下载 Bilibili 视频的详细教程

在 Ubuntu 20.04 上使用 Lux 下载 Bilibili（哔哩哔哩）视频的完整和详细步骤如下，包括使用预编译二进制文件的安装方法：

### 1. 安装依赖
确保你的系统已安装 FFmpeg，这是因为 Lux 在合并下载的视频文件时需要用到 FFmpeg。

```bash
sudo apt update
sudo apt install ffmpeg
```

### 2. 安装 Lux
您有两种安装 Lux 的方法：使用 Go 语言环境安装或者使用预编译的二进制文件。

#### 使用 Go 安装：
首先，确保您已经安装了 Go 语言环境：

```bash
sudo apt install golang-go
```

然后，通过以下命令安装 Lux：

```bash
go install github.com/iawia002/lux@latest
```

#### 使用预编译的二进制文件：
直接从 [Lux Releases](https://github.com/iawia002/lux/releases) 页面下载适用于 Linux 的二进制文件，并将其放置在系统的可执行路径下。例如，如果你下载的是 `lux_linux_amd64`，你可以使用以下命令：

```bash
sudo wget https://github.com/iawia002/lux/releases/download/v0.12.0/lux_linux_amd64 -O /usr/local/bin/lux
sudo chmod +x /usr/local/bin/lux
```

### 3. 使用 Lux 下载视频
在安装 Lux 后，使用以下命令格式来下载视频：

```bash
lux [OPTIONS] URL
```

#### 示例：下载单个视频

假设您想下载以下 Bilibili 视频 URL：`https://www.bilibili.com/video/BV1v4411B7Gv`。

首先，查看所有可用的视频质量：

```bash
lux -i "https://www.bilibili.com/video/BV1v4411B7Gv"
```

这将列出所有可用的视频格式和质量。选择一个特定格式的 ID 来下载最高质量的视频（通常标识为 `最高清晰度`）：

```bash
lux -f <stream_id> "https://www.bilibili.com/video/BV1v4411B7Gv"
```

替换 `<stream_id>` 为您从输出中选择的流 ID。

### 4. 下载整个播放列表
要下载整个播放列表，请使用 `-p` 参数：

```bash
lux -p "https://www.bilibili.com/bangumi/play/ep198061"
```

### 5. 高级选项
- **多线程下载**：使用 `-n` 参数指定下载线程数：
  ```bash
  lux -n 10 "https://www.bilibili.com/video/BV1v4411B7Gv"
  ```

- **指定输出文件夹和文件名**：使用 `-o` 指定输出目录，`-O` 指定输出文件名：
  ```bash
  lux -o /path/to/directory -O filename "https://www.bilibili.com/video/BV1v4411B7Gv"
  ```

### 6. 调试和错误处理
遇到下载问题时，开启调试模式以打印出详细的网络请求信息，帮助诊断问题：

```bash
lux -d "https://www.bilibili.com/video/BV1v4411B7Gv"
```

确保在使用 Lux 时遵守 Bilibili 的使用条款和版权政策，合法使用下载工具。