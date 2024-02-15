# ROS 应用商店APT源

## APT

在 Ubuntu 和其他基于 Debian 的 Linux 发行版中，APT（Advanced Package Tool）是一个用于包管理的命令行工具。APT 使得安装、更新、配置和移除软件包变得简单高效。以下是关于 APT 的详细解释：

### 1. **APT的功能**
- **软件包安装**：APT 允许用户轻松安装软件包及其依赖。
- **软件包更新**：提供了一种机制来更新系统中安装的软件包。
- **依赖性管理**：自动处理软件包的依赖关系，确保软件包之间的兼容性。
- **系统升级**：支持整个系统的升级，包括核心系统组件和应用程序。
- **软件源管理**：通过 `/etc/apt/sources.list` 文件和 `/etc/apt/sources.list.d/` 目录管理软件源。

### 2. **常用APT命令**
- **更新软件包列表**：
  ```bash
  sudo apt update
  ```
  更新本地软件包索引，以便获得最新软件包信息。
  
- **安装软件包**：
  ```bash
  sudo apt install package_name
  ```
  安装指定的软件包及其依赖。
  
- **升级软件包**：
  ```bash
  sudo apt upgrade
  ```
  升级所有已安装的软件包到最新版本。
  
- **全系统升级**：
  ```bash
  sudo apt dist-upgrade
  ```
  升级系统到最新版本，包括那些可能改变依赖关系的包。

- **删除软件包**：
  ```bash
  sudo apt remove package_name
  ```
  删除指定的软件包，但保留配置文件和数据。
  
- **清除软件包**：
  ```bash
  sudo apt purge package_name
  ```
  删除软件包及其配置文件。

- **清理无用的软件包**：
  ```bash
  sudo apt autoremove
  ```
  移除未使用的软件包和依赖。

### 3. **APT与DPKG的关系**
- **dpkg** 是 Debian 包管理的底层工具。APT 在 dpkg 之上提供了一个更高层次的界面，用于处理复杂的依赖关系和批量操作。

### 4. **APT源**
- APT 获取软件包信息和软件包文件的地方称为“源”（Source），通常指向 Internet 上的服务器或本地镜像。
- 源的信息存储在 `/etc/apt/sources.list` 文件和 `/etc/apt/sources.list.d/` 目录中的文件里。

### 5. **安全性和信任**
- APT 使用密钥和签名来验证软件包的完整性和真实性，确保安全性。

### 注意事项
- **使用权限**：大多数 APT 操作需要超级用户权限，因此需要使用 `sudo`。
- **网络连接**：更新软件包列表和安装软件包通常需要可靠的网络连接。
- **软件源的选择**：选择可靠和更新及时的软件源可以提高软件安装和更新的速度和安全性。
- **系统兼容性**：在安装软件包时，确保它们与你的 Ubuntu 版本兼容。

APT 是 Ubuntu 系统软件管理的核心，提供了一种方便、高效的方式来维护系统的完整性和最新状态。

## ROS软件包下载

- 网址：ROS Index（https://index.ros.org/）
- 点击PACKAGE LIST，进入软件包列表

![2024-02-15 21-10-16 的屏幕截图](/home/lyb/github/Typora_notes/2024-02-15 21-10-16 的屏幕截图.png)

- 闪电图标表示的是这个软件包是否已经发布，也就是我们是不是可以通过APT去下载它，只有拥有闪电图标的软件包，我们才能用APT去下载安装，同一个软件包，在Noetic中没有发布，并不代表在其他版本的ROS里边没有发布。

![2024-02-15 21-10-30 的屏幕截图](/home/lyb/github/Typora_notes/2024-02-15 21-10-30 的屏幕截图.png)

- 随便选择一个软件包，里面是关于这个软件包的一些信息，如github仓库地址等

![2024-02-15 21-17-29 的屏幕截图](/home/lyb/github/Typora_notes/2024-02-15 21-17-29 的屏幕截图.png)

### 具体下载方法

下载和安装 ROS（Robot Operating System）软件包可以通过几种不同的方法进行，取决于包是否在 ROS 的官方存储库中，以及你的系统设置。以下是详细的步骤和方法：

#### 方法 1: 使用 APT（适用于官方存储库中的包）

如果软件包位于 ROS 官方存储库中，可以使用 Ubuntu 的包管理器 APT 进行安装。

1. **打开终端**
2. **更新软件源列表**（确保最新）：
   ```bash
   sudo apt update
   ```
3. **安装软件包**：
   使用 `sudo apt install` 命令加上软件包名称。例如，如果你想要安装名为 `ros-noetic-turtlesim` 的包（对于 ROS Noetic 版本）：
   ```bash
   sudo apt install ros-noetic-turtlesim
   ```
   将 `noetic` 替换为你安装的 ROS 版本（如 `melodic`、`kinetic` 等），`turtlesim` 替换为你想要安装的软件包名称。

#### 方法 2: 从源代码编译（适用于非官方存储库或需要最新版本的包）

如果软件包不在官方存储库中，或者你需要安装的是开发版本，可以从源代码编译。

1. **创建或定位到你的 ROS 工作空间**（通常名为 `catkin_ws`）：
   ```bash
   mkdir -p ~/catkin_ws/src
   cd ~/catkin_ws/src
   ```

2. **克隆软件包的源代码仓库**：
   使用 `git clone` 命令加上仓库的 URL。例如：
   ```bash
   git clone https://github.com/ros/turtlesim.git
   ```
   将 URL 替换为你需要的软件包的仓库地址。

3. **安装依赖**：
   切换到你的工作空间根目录并使用 `rosdep` 安装所有依赖：
   ```bash
   cd ~/catkin_ws
   rosdep install --from-paths src --ignore-src -r -y
   ```

4. **编译工作空间**：
   在工作空间的根目录下使用 `catkin_make`：
   ```bash
   catkin_make
   ```

5. **源工作空间设置**：
   每次打开新终端时，你需要源工作空间的环境设置：
   ```bash
   source ~/catkin_ws/devel/setup.bash
   ```

### 注意事项
- **ROS 版本**：确保下载的软件包与你的 ROS 版本兼容。
- **依赖关系**：确保安装所有必要的依赖项，特别是从源代码编译时。
- **环境设置**：在使用新安装的包之前，确保正确设置你的环境。

通过上述任一方法，你可以下载和安装所需的 ROS 软件包。使用 APT 是最简单直接的方式，但是从源代码编译提供了更多的灵活性，特别是对于开发人员或需要最新版本的用户。