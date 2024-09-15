# ubuntu 20.04添加ros官方的软件源（解决下载ros软件包出现的E: 无法定位软件包的问题）

在 Ubuntu 20.04 上添加 ROS 官方软件源可以解决使用其他镜像源时遇到的 "E: 无法定位软件包" 的问题，主要是因为这些镜像源可能没有同步所有官方提供的包，或者同步延迟，导致一些特定或新发布的包在镜像源中不可用。使用官方软件源确保了你能直接访问到 ROS 的最新和完整的软件包库。

## 为什么添加 ROS 官方软件源解决问题？

使用官方源的好处包括：

1. **完整性和及时性**：ROS 官方源保持最新状态，含有所有最新发布的 ROS 软件包。镜像源可能因为更新策略或同步频率的问题，没有及时更新这些包。
2. **官方支持**：官方源包含的软件包都是由 ROS 维护团队支持的，确保了软件包的兼容性和安全性。
3. **可靠性**：官方源通常有更好的可靠性和稳定性保证。

例如，如果你想安装 `ros-noetic-desktop-full`，这是一个包含完整 ROS 桌面环境、所有 ROS 核心组件以及二维/三维模拟器的软件包。如果使用未及时同步的镜像源，可能会出现无法找到该包的情况。使用官方源则可以避免这类问题。

### 镜像源不包含 ROS 软件包

- 许多镜像源仅包含 Ubuntu 官方的软件包，而不包含 ROS（Robot Operating System）等第三方软件包。ROS 软件包由 Open Robotics 和社区维护，通常托管在专门的 ROS 软件源中，而非 Ubuntu 官方的镜像源中。因此，如果你使用的镜像源不包含 ROS 软件包，就会出现 "E: 无法定位软件包" 的错误。

## 如何添加 ROS 官方软件源到 Ubuntu 20.04

以下是详细步骤：

### 步骤 1: 打开终端

可以通过快捷键 `Ctrl + Alt + T` 打开终端窗口。

### 步骤 2: 安装软件包管理工具

确保 `curl` 和 `lsb-release` 被安装，这些工具将帮助添加软件源和密钥。
```bash
sudo apt update
sudo apt install curl lsb-release
```

### 步骤 3: 添加 ROS 官方软件源

使用以下命令添加 ROS 的官方 APT 仓库到你的系统。这个命令首先获取你的 Ubuntu 版本号，并将 ROS 的源地址添加到 `sources.list.d` 目录下的 `ros-latest.list` 文件中。
```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```
这里使用 `$(lsb_release -sc)` 命令自动替换为当前系统的版本代号，如 Ubuntu 20.04 的 `focal`。

#### 命令详解

1. **`sudo`**:

   - `sudo` 是 “superuser do” 的缩写，它允许普通用户以超级用户（管理员）权限执行命令，用于执行需要更高权限的操作。

2. **`sh -c`**:

   - `sh` 是一个命令行解释器，用于执行命令。选项 `-c` 允许你传递一个完整的命令给 shell 来执行。这在需要一次执行多个命令或处理复杂的命令行时非常有用。

3. **`echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main"`**:

   - `echo` 命令用于在终端输出其后面的字符串。

   - ```
     deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main
     ```

      是一个软件源地址，其中：

     - `deb` 表示这是一个 Debian 软件包的源。
     - `http://packages.ros.org/ros/ubuntu` 是 ROS 软件包存储的 URL。
     - `$(lsb_release -sc)` 是一个命令替换，执行 `lsb_release -sc` 命令并将输出替换在此位置。`lsb_release -sc` 输出当前操作系统的代号（如 Ubuntu 20.04 的 `focal`），确保添加的源与系统版本兼容。
     - `main` 指的是该源中主要部分的分类，通常包含软件包的主要和稳定版本。

4. **`> /etc/apt/sources.list.d/ros-latest.list`**:

   - `>` 是重定向操作符，用于将前面命令的输出写入到指定的文件中，而不是显示到终端。
   - `/etc/apt/sources.list.d/ros-latest.list` 指定了输出文件的路径。这里的路径表示将软件源地址写入到 `ros-latest.list` 文件中。这个文件位于 `/etc/apt/sources.list.d/` 目录下，这是一个专门用来存放额外 apt 源列表文件的目录。

### 步骤 4: 导入密钥

为了验证从 ROS 仓库下载的包的真实性，需要导入 ROS 项目的 GPG 密钥：
```bash
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
```
这个命令使用 `curl` 下载密钥，并通过 `apt-key add` 添加到系统的密钥库中。

### 步骤 5: 更新软件包列表

完成软件源添加后，需要更新本地的包数据库，以包括新添加的源：
```bash
sudo apt update
```

### 步骤 6: 安装 ROS 包
现在你可以安装所需的 ROS 包了。例如，安装 ROS Noetic 的完整桌面版：
```bash
sudo apt install ros-noetic-desktop-full
```

### 总结

通过以上步骤，你已经成功将 ROS 的官方软件源添加到 Ubuntu 20.04 系统中。这不仅可以解决由于软件源问题导致的 "E: 无法定位软件包" 错误，还能确保你安装的 ROS 软件包是最新和最完整的。这对于依赖最新功能和修复的用户尤其重要。