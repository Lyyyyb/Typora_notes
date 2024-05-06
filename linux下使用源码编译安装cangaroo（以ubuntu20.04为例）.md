# linux下使用源码编译安装cangaroo（以ubuntu20.04为例）

## cangaroo是什么

Cangaroo是一个开源的CAN总线分析器，用于监控和分析CAN总线上的通信。它是一个软件项目，提供了一个用户界面，可以实时捕获、解析和显示CAN消息的内容，并提供各种分析工具和功能，帮助用户理解和调试CAN总线上的通信。以下是Cangaroo的一些主要特点和功能：

### 1. **实时监控CAN通信**：

- Cangaroo能够连接到CAN总线，并实时捕获CAN消息。它可以持续监视CAN总线上的数据流动，并将捕获的CAN消息显示在用户界面上，使用户能够即时了解CAN总线上的通信活动。

### 2. **CAN消息解析**：

- Cangaroo能够解析捕获的CAN消息，提取出CAN帧的标识符（ID）、数据、数据长度码（DLC）等重要信息。这使得用户能够准确地了解每个CAN消息的含义和结构。

### 3. **可视化显示**：

- Cangaroo提供了图形化的用户界面，用于直观地显示捕获的CAN消息。用户可以通过图表、列表、图形等方式查看CAN消息，以便更好地理解和分析CAN总线上的通信。

### 4. **过滤和筛选**：
- Cangaroo支持根据特定条件对捕获的CAN消息进行过滤和筛选。用户可以设置过滤条件，只显示满足条件的CAN消息，这有助于集中注意力在感兴趣的消息上，并忽略其他无关的消息。

### 5. **数据记录和导出**：

- Cangaroo可以将捕获的CAN消息保存到文件中，以便后续分析和回放。用户可以选择将数据记录到文件，以便在需要时进行进一步的研究或分享给其他人。

### 6. **开源和可定制**：
- Cangaroo是一个开源项目，用户可以查看和修改其源代码。这使得它成为一个灵活和可定制的解决方案，可以根据用户的特定需求进行定制和扩展。

## 通过源码编译安装cangaroo

- 编译CANable Cangaroo for Linux需要一些步骤，主要涉及获取源代码、安装依赖项、编译和安装。以下是详细的步骤：

### 1. 获取源代码

- 首先，你需要获取Cangaroo的源代码。你可以从它的GitHub仓库中克隆源代码：

```bash
git clone https://github.com/HubertD/cangaroo.git
```

### 2. 安装依赖项

- 在编译Cangaroo之前，确保你的系统安装了必要的依赖项。这些依赖项通常包括C++编译器、CMake和Qt开发工具包。

- 在Ubuntu上，你可以使用以下命令安装这些依赖项：

```bash
sudo apt-get update
sudo apt-get install build-essential git qt5-qmake qtbase5-dev libnl-3-dev libnl-route-3-dev
```

### 3. 进入cangaroo目录：

```bash
cd cangaroo
```

### 4. 使用qmake生成Makefile：

- 运行CMake来配置项目，生成Makefile：

```bash
qmake -qt=qt5
```

### 5. 编译

- 运行Make命令来编译Cangaroo：

```bash
make
```

### 6. 解决可能的编译错误：

![image-20240506114523995](/home/lyb/github/Typora_notes/image-20240506114523995.png)

- 如果在编译过程中遇到 `SIOCGSTAMPNS` 和 `SIOCGSTAMP` 相关的错误，需要手动修复。

- 打开文件 `src/driver/SocketCanDriver/SocketCanInterface.cpp`，并在文件开头添加以下行：

```bash
vim src/driver/SocketCanDriver/SocketCanInterface.cpp
```

```bash
#include <linux/sockios.h>
```

![image-20240506114920973](/home/lyb/github/Typora_notes/image-20240506114920973.png)

- 然后再次执行 `make` 命令：

```bash
make
```

### 7. 运行Cangaroo：

完成安装后，你可以运行Cangaroo：

```bash
cd./bin/cangaroo
```

![image-20240506115300098](/home/lyb/github/Typora_notes/image-20240506115300098.png)

### 8.将可执行文件复制到系统路径中，以便全局调用：

```bash
sudo cp ./bin/cangaroo /usr/local/bin/
```

