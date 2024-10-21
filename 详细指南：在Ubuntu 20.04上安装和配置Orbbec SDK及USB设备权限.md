# 详细指南：在Ubuntu 20.04上安装和配置Orbbec SDK及USB设备权限

在Ubuntu 20.04上安装和配置Orbbec SDK以及进行USB设备的权限配置和调整USBFS缓存大小，涉及到一系列系统配置和环境准备步骤。以下是详细的步骤说明，以确保准确和高效地设置开发环境。

### 1. 系统环境配置

在开始安装Orbbec SDK之前，需要确保系统具备必要的依赖库，以支持SDK的功能。

#### 安装依赖库

- **libudev-dev**：用于管理插拔硬件设备，非常重要用于设备节点的动态管理。
- **libusb-dev**：提供对USB设备的直接控制和通信能力。

执行以下命令来安装这些库：
```bash
sudo apt update
sudo apt install libudev-dev libusb-dev
```

### 2. USB访问权限配置

由于Linux系统默认限制非root用户直接访问USB设备，需要通过udev规则来修改这一权限设置。

#### 安装udev规则
1. **下载Orbbec SDK**：从Orbbec官方网站下载适用于Linux或ARM的SDK包，并解压到本地目录。
2. **配置udev规则**：
   - 导航到SDK解压后的`Script`目录。
   - 找到名为`99-obsensor-libusb.rules`的文件，这个文件包含允许非root用户访问USB设备的规则。
   - 执行安装脚本来应用这些规则：
     ```bash
     sudo chmod +x ./install.sh
     sudo ./install.sh
     ```
   - 重新插拔设备或重启系统以使新的udev规则生效。

### 3. USBFS 缓存大小配置

对于高分辨率图像处理或多设备连接，可能需要更大的USBFS缓存。

#### 调整USBFS缓存大小
- **检查当前缓存大小**：
  ```bash
  cat /sys/module/usbcore/parameters/usbfs_memory_mb
  ```
- **临时增加缓存大小**：
  ```bash
  sudo sh -c 'echo 128 > /sys/module/usbcore/parameters/usbfs_memory_mb'
  ```
- **永久增加缓存大小**：
  - 修改GRUB配置文件以添加内核参数：
    ```bash
    sudo nanso /etc/default/grub
    # 将GRUB_CMDLINE_LINUX_DEFAULT行修改为：
    GRUB_CMDLINE_LINUX_DEFAULT="quiet splash usbcore.usbfs_memory_mb=128"
    ```
  - 更新GRUB配置并重启：
    ```bash
    sudo update-grub
    sudo reboot
    ```

### 4. 验证设备状态

确保设备被系统正确识别是关键的一步。

- 使用USB 3.0 Type-C数据线连接Orbbec的Astra+相机。
- 执行以下命令来检查设备识别情况：
  ```bash
  lsusb
  # 查找VID为2bc5，PID为0536或0636的设备
  ```

### 5. 编译Linux示例

最后，编译SDK提供的示例程序，验证整个配置的有效性。

- 在SDK的`Examples`目录下创建一个`build`目录，并导航到此目录：b
  ```bash
  mkdir build && cd build
  ```
- 使用CMake来配置项目，并编译：
  ```bash
  cmake ..
  make
  ```

以上步骤详细描述了在Ubuntu 20.04上安装和配置Orbbec SDK的全过程，包括如何设置USB访问权限和调整USBFS缓存大小。这些步骤确保了系统的正确配置，使开发者能够高效地进行开发和测试。