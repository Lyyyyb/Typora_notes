# CANable USB转CAN适配器固件和驱动的安装(以candlelight和pcan为例)ubuntu20.04

## candlelight

### candlelight是什么

- Candlelight是CANable设备的固件之一，用于将CANable设备转换为本机CAN设备，无需使用slcand。它允许CANable设备直接在Linux系统上以本机CAN设备的形式出现，并且与SocketCAN一起使用，无需额外的驱动程序或转换工具。
- 这意味着你可以直接使用Linux上的标准CAN工具（如can-utils）和Wireshark等来与CAN总线进行交互。

- 以下是Candlelight固件的主要特点和功能：

1. **本机CAN设备支持**：Candlelight固件使得CANable设备可以在Linux系统上直接以本机CAN设备的形式出现，而无需使用slcand来创建虚拟串口。

2. **高性能**：与使用串口线固件相比，使用Candlelight固件可以获得更高的性能，因为slcand被完全绕过。

3. **与SocketCAN兼容**：Candlelight固件与SocketCAN兼容，因此可以与Linux系统上的SocketCAN框架集成，并且可以使用所有标准的can-utils命令行工具进行通信和分析。

4. **Wireshark集成**：由于Candlelight固件与SocketCAN兼容，因此你可以使用Wireshark等网络分析工具直接捕获和分析CAN总线上的数据流。

5. **不支持FD帧**：目前，CANable 2.0的Candlelight固件不支持FD（Flexible Data-Rate）帧，这意味着它只能处理标准的CAN帧，无法处理高速数据传输的FD帧。

### 固件安装与更新

- 用镊子短接boot引脚，上点后进入bootloader

#### 使用网页应用程序更新candlelight

- 以访问CANable更新程序站点，通过简单的步骤来更新CANable设备。网页应用程序提供了一个易于使用的界面，指导用户完成更新过程。

- 地址：https://canable.io/updater/canable1.html

![image-20240506120815794](/home/lyb/github/Typora_notes/image-20240506120815794.png)

1. **准备CANable设备**：
   - 首先，将CANable设备上的"Boot"跳线移动到引导位置，或者在CANable Pro上按住"Boot"按钮，然后将设备插入计算机。
   
2. **选择并更新固件**：
   - 接下来，用户需要在网页中选择要刷新的固件版本，并点击下方的"Connect and Update"按钮。
   
   ![image-20240506143300619](/home/lyb/github/Typora_notes/image-20240506143300619.png)
   
   - 这将触发与CANable设备的连接，并开始更新固件。
   
   ![image-20240506143237066](/home/lyb/github/Typora_notes/image-20240506143237066.png)
   
   ![image-20240506144122099](/home/lyb/github/Typora_notes/image-20240506144122099.png)
   
   ![image-20240506144257068](/home/lyb/github/Typora_notes/image-20240506144257068.png)
   
   - 固件安装更新完成，取下镊子，不再短接boot0，重新插拔设备，打开终端键入ifconfig -a,识别到can0，则固件安装成功。
   
   ![image-20240506144511938](/home/lyb/github/Typora_notes/image-20240506144511938.png)

#### 使用linux下的dfu-util安装更新固件candlelight

在Linux系统下更新和安装固件可以通过以下步骤完成：

##### 1. 安装dfu-util

```bash
sudo apt-get update
sudo apt-get install dfu-util
```

##### 2.安装gcc-arm-none-eabi和CMake

```bash
sudo apt-get update
sudo apt-get install gcc-arm-none-eabi cmake
```

##### 3.获取用于烧录的二进制镜像文件

- 解压candlelight源码文件

```bash
tar xvf candleLight_fw.tar.bz2
```

- 进入candleLight_fw目录

```bash
cd candleLight_fw
```

- 创建bulid目录并进入（如果本来就有build要先删除）

```bash
mkdir build
cd build
```

- 使用 CMake 生成 Makefile，指定交叉编译的工具链文件

```bash
cmake .. -DCMAKE_TOOLCHAIN_FILE=../cmake/gcc-arm-none-eabi-8-2019-q3-update.cmake
```

- 编译

```bash
make
```

- 编译完之后，可看到镜像candleLight_fw\build\candleLight_fw.bin

![image-20240506150246260](/home/lyb/github/Typora_notes/image-20240506150246260.png)

![image-20240506150200435](/home/lyb/github/Typora_notes/image-20240506150200435.png)

##### 4. 刷新设备固件

一旦dfu-util安装完成，你可以使用以下命令来刷新CANable设备的固件：

```bash
sudo dfu-util -d 0483:df11 -c 1 -i 0 -a 0 -s 0x08000000 -D candleLight_fw.bin
```

这个命令会将指定的固件文件（candleLight_fw.bin）传输到设备的固件存储位置，并刷新设备的固件。

![image-20240506150458096](/home/lyb/github/Typora_notes/image-20240506150458096.png)

##### 5. 完成更新

刷新固件后，确保将CANable设备上的"启动"跳线移回到原始位置，并重新插入或拔出设备。这样你就成功地更新了设备的固件。

![image-20240506150718868](/home/lyb/github/Typora_notes/image-20240506150718868.png)

- 固件安装更新完成，取下镊子，不再短接boot0，重新插拔设备，打开终端键入ifconfig -a,识别到can0，则固件安装成功。

![image-20240506150843040](/home/lyb/github/Typora_notes/image-20240506150843040.png)

## pcan

### pcan是什么

PCAN 是 PEAK-System Technik GmbH（PEAK 系统技术有限公司）开发的一系列产品的名称，专门用于控制器局域网络（Controller Area Network，简称 CAN）的应用。CAN 是一种串行通信协议，最初设计用于车辆中的实时控制系统，但现在已被广泛用于工业控制、自动化、航空航天等领域。

PCAN 产品系列包括各种硬件和软件工具，用于支持 CAN 总线的开发和应用。其中一些产品和工具包括：

1. **PCAN 接口卡**：这些是用于连接计算机和 CAN 总线的硬件接口，可通过 USB、PCI、PCI Express 或 PC/104 接口连接到计算机，使计算机能够与 CAN 总线通信。

2. **PCAN 分析器**：用于分析 CAN 总线通信的硬件和软件工具，可以捕获和解码 CAN 总线上的消息，帮助诊断和调试 CAN 总线网络。

3. **PCAN 开发工具包（SDK）**：用于开发 CAN 总线应用程序的软件开发工具包，提供了一系列 API 和示例代码，帮助开发者在各种操作系统（如 Windows、Linux）上编写自己的 CAN 应用程序。

4. **PCAN 总线分析软件**：用于监视、记录和分析 CAN 总线通信的软件工具，提供实时图形显示和日志记录功能，帮助用户分析 CAN 总线上的数据流。

总的来说，PCAN 是一个成熟的产品系列，提供了丰富的硬件和软件工具，用于支持 CAN 总线的开发、调试和应用。这些工具被广泛应用于汽车行业、工业控制、航空航天等领域，为 CAN 总线应用提供了强大的支持和解决方案。

### 固件安装与更新

- 以访问CANable更新程序站点，通过简单的步骤来更新CANable设备。网页应用程序提供了一个易于使用的界面，指导用户完成更新过程。

- 地址：https://canable.io/updater/canable1.html

![image-20240506120815794](/home/lyb/github/Typora_notes/image-20240506120815794.png)

1. **准备CANable设备**：

   - 首先，将CANable设备上的"Boot"跳线移动到引导位置，或者在CANable Pro上按住"Boot"按钮，然后将设备插入计算机。

2. **选择并更新固件**：

   - 接下来，用户需要在网页中选择要刷新的固件版本，并点击下方的"Connect and Update"按钮。

   ![image-20240506141800240](/home/lyb/github/Typora_notes/image-20240506141800240.png)

   - 这将触发与CANable设备的连接，并开始更新固件。

   ![image-20240506141628191](/home/lyb/github/Typora_notes/image-20240506141628191.png)

   

   ![image-20240506141728385](/home/lyb/github/Typora_notes/image-20240506141728385.png)

   

   - 固件安装更新完成，取下镊子，不再短接boot0，重新插拔设备，打开终端（驱动安装完成后）键入pcaninfo识别到设备，则固件安装成功。

   ![image-20240506152223171](/home/lyb/github/Typora_notes/image-20240506152223171.png)

#### 使用linux下的dfu-util安装更新固件pcan

在Linux系统下更新和安装固件可以通过以下步骤完成：

##### 1. 安装dfu-util

```bash
sudo apt-get update
sudo apt-get install dfu-util
```

##### 2.安装gcc-arm-none-eabi和CMake

```bash
sudo apt-get update
sudo apt-get install gcc-arm-none-eabi cmake
```

##### 3.获取用于烧录的二进制镜像文件

- 解压pcan源码文件

```bash
tar xvf pcan_cantact.tar.bz2
```

- 进入pcan_cantact目录

```bash
cd pcan/pcan_cantact/build-canable
```

- 编译

```bash
make
```

- 编译完得到：pcan_cantact/build-canable/pcan_canable_hw.bin    pcan_canable_hw.hex

##### 4. 刷新设备固件

一旦dfu-util安装完成，你可以使用以下命令来刷新CANable设备的固件：

```bash
sudo dfu-util -d 0483:df11 -c 1 -i 0 -a 0 -s 0x08000000 -D pcan_canable_hw.bin
```

这个命令会将指定的固件文件（pcan_canable_hw.bin ）传输到设备的固件存储位置，并刷新设备的固件。

![image-20240506151747158](/home/lyb/github/Typora_notes/image-20240506151747158.png)

##### 5. 完成更新

- 刷新固件后，确保将CANable设备上的"启动"跳线移回到原始位置，并重新插入或拔出设备。这样你就成功地更新了设备的固件。

![image-20240506152343221](/home/lyb/github/Typora_notes/image-20240506152343221.png)

- 固件安装更新完成，取下镊子，不再短接boot0，重新插拔设备，打开终端（驱动安装完成后）键入pcaninfo识别到设备，则固件安装成功。

![image-20240506152223171](/home/lyb/github/Typora_notes/image-20240506152223171.png)

### 驱动安装

- 解压pcan驱动源码包

```bash
tar xvf peak-linux-driver-8.15.2.tar.gz
```

- 进入peak-linux-driver-8.15.2目录

```bash
cd peak-linux-driver-8.15.2/
```

- 清理之前的构建结果

```bash
make clean
```

- 编译

```bash
make
```

- 安装编译后的软件

```bash
sudo make install 
```

#### 问题解决

- 执行完上述步骤后，终端键入pcaninfo，出现如下错误

![image-20240506153128248](/home/lyb/github/Typora_notes/image-20240506153128248.png)

- 即使驱动程序已经安装到系统中，但如果它没有正确加载到内核中，系统也无法识别它。
- 在终端键入以下指令，手动加载模块，或者手动加载pcan.ko文件（执行 `sudo insmod pcan.ko` 命令以加载 `pcan.ko` 文件到内核中。）

```bash
sudo modprobe pcan
```

- 之后在终端键入以下指令，确认PCAN相关的模块是否已经加载

```bash
lsmod | grep pcan
```

![image-20240506154230612](/home/lyb/github/Typora_notes/image-20240506154230612.png)

- 之后键入pcaninfo，检查系统中是否存在 PCAN 驱动，并显示有关 PCAN 驱动版本、PCAN-Basic 版本以及已连接 PCAN 设备的信息。正常输出如下

![image-20240506154322345](/home/lyb/github/Typora_notes/image-20240506154322345.png)

### pcaniew的安装

- 下载软件源列表

```bash
wget -q http://www.peak-system.com/debian/dists/`lsb_release -cs`/peak-system.list -O- | sudo tee /etc/apt/sources.list.d/peak-system.list
```

- 下载软件源公钥

```bash
wget -q http://www.peak-system.com/debian/dists/wheezy/peak-system.list -O- | sudo tee /etc/apt/sources.list.d/peak-system.list
```

- 添加软件源公钥

```bash
wget -q http://www.peak-system.com/debian/peak-system-public-key.asc -O- | sudo apt-key add -
```

- 更新软件包列表

```bash
sudo apt-get update
```

- 安装pcanview-ncurses 应用程序

```bash
sudo apt-get install pcanview-ncurses
```

- 查看是否安装成功

```bash
ls /usr/bin/pcanview
```

- 运行

```bash
pcanview
```

![image-20240506161530362](/home/lyb/github/Typora_notes/image-20240506161530362.png)

![image-20240506161650835](/home/lyb/github/Typora_notes/image-20240506161650835.png)
