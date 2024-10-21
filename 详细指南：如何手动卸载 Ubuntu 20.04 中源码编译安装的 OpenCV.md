# 详细指南：如何手动卸载 Ubuntu 20.04 中源码编译安装的 OpenCV

在 Ubuntu 20.04 系统中，如果您已通过源代码编译和安装了 OpenCV，卸载过程需要手动执行，因为编译安装通常不会像包管理器安装的软件那样注册一个卸载脚本。此过程包括识别和删除所有由 OpenCV 安装过程创建的文件和目录。以下是详细的卸载步骤及示例：

### 1. 确定安装位置
在源码编译时，OpenCV 默认安装在 `/usr/local` 目录。这可以通过检查编译时 CMake 配置的 `CMAKE_INSTALL_PREFIX` 参数确定。如果未修改，默认路径为 `/usr/local`。

### 2. 手动删除文件

因为没有自动的卸载程序，您需要手动删除所有相关的库文件、头文件、可执行文件、配置文件和其他安装的资源。

#### a. 删除库文件
库文件（通常是 `.so` 文件）位于 `/usr/local/lib` 或 `/usr/local/lib64`，具体路径取决于系统架构。删除这些库文件：
```bash
sudo rm /usr/local/lib/libopencv*
```

#### b. 删除 CMake 文件
CMake 配置文件位于 `/usr/local/lib/cmake/opencv4`。这些文件用于项目配置时查找和链接 OpenCV。
```bash
sudo rm -r /usr/local/lib/cmake/opencv4
```

#### c. 删除头文件
头文件位于 `/usr/local/include/opencv4`。这些文件包含了库的接口定义。
```bash
sudo rm -r /usr/local/include/opencv4
```

#### d. 删除二进制文件和脚本
二进制文件和一些辅助脚本位于 `/usr/local/bin`。这些包括实用程序和配置脚本。
```bash
sudo rm /usr/local/bin/opencv_*
sudo rm /usr/local/bin/setup_vars_opencv4.sh
```

#### e. 删除共享资源和许可证
如果安装了包含共享资源和许可证的文件，也应将其删除。
```bash
sudo rm -r /usr/local/share/licenses/opencv4
sudo rm -r /usr/local/share/opencv4  # 如果存在
```

### 3. 更新动态链接库缓存
删除库文件后，运行 `ldconfig` 命令来更新系统的动态链接库缓存，确保系统不会在寻找已删除的库。
```bash
sudo ldconfig
```

### 4. 验证卸载
使用 `find` 命令确认所有相关文件都已被正确删除。如果命令没有输出任何路径，表明卸载成功。
```bash
find /usr/local -name "*opencv*"
```

### 实例解释
假设您之前安装了 OpenCV 并希望现在将其完全卸载。您打开终端，执行上述步骤中的命令，逐一删除所有 OpenCV 相关的文件和目录。这些命令涵盖了从库文件到配置文件的所有内容，确保系统中不留下任何残余文件。卸载后，您还通过运行 `find` 命令确认卸载操作的彻底性。

这种手动卸载方法虽然繁琐，但是对于从源代码安装的库而言是必要的，因为它们不是通过标准的包管理器安装，因此不支持自动卸载。正确且完全地卸载 OpenCV 可以避免将来升级或安装其他版本时可能出现的冲突。