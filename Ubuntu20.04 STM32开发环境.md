# Ubuntu20.04 STM32开发环境

## CubeMX

在Ubuntu 20.04上搭建STM32开发环境主要涉及安装必要的工具和配置环境。以下是详细步骤：

### 1. 安装基本依赖
首先，打开终端并安装一些基本的依赖：
```bash
sudo apt update
sudo apt install build-essential git
```

### 2. 安装 ARM GCC 编译器
安装用于ARM Cortex-M微控制器的交叉编译器：
```bash
sudo apt install gcc-arm-none-eabi
```

### 3. 安装 OpenOCD
OpenOCD用于调试和编程：
```bash
sudo apt install openocd
```

### 4. 安装 STM32CubeMX（可选）
STM32CubeMX是一个图形化工具，可帮助配置STM32微控制器和初始化代码。下载STM32CubeMX（从ST官网获取），然后解压并安装。

### 5. 安装 STM32CubeIDE（可选）
STM32CubeIDE整合了编码、编译、调试等功能。从ST官网下载STM32CubeIDE，然后按照指南进行安装。

### 6. 配置 udev 规则（用于调试）
为了不用sudo权限进行调试，需要配置udev规则：
1. 创建文件 `/etc/udev/rules.d/49-stlink.rules`：
   ```bash
   sudo nano /etc/udev/rules.d/49-stlink.rules
   ```
2. 粘贴以下内容：
   ```
   # STLink/V2
   SUBSYSTEM=="usb", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="3748", MODE="0666"
   # STLink/V2-1
   SUBSYSTEM=="usb", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="374b", MODE="0666"
   # STLink/V3
   SUBSYSTEM=="usb", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="374e", MODE="0666"
   SUBSYSTEM=="usb", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="374f", MODE="0666"
   SUBSYSTEM=="usb", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="3753", MODE="0666"
   ```
3. 重新加载 udev 规则：
   ```bash
   sudo udevadm control --reload
   sudo udevadm trigger
   ```

### 7. 创建工程
使用STM32CubeMX或直接在STM32CubeIDE中创建新项目。选择合适的MCU，配置所需的外设，并生成初始化代码。

### 8. 编写代码
在STM32CubeIDE或你选择的编辑器中编写代码。

### 9. 编译和上传
使用STM32CubeIDE或命令行工具来编译和上传代码到你的STM32板子。

### 10. 调试
使用OpenOCD和GDB进行调试。

这些步骤涵盖了在Ubuntu 20.04上为STM32开发环境的基本设置。某些步骤，如安装STM32CubeMX和STM32CubeIDE，是可选的，但强烈建议，特别是对于新手。

## VScode

使用Visual Studio Code (VSCode) 来开发STM32项目，你需要配置一些扩展和工具链。下面是详细的步骤：

### 1. 安装 VSCode
如果还没有安装VSCode，可以从[Visual Studio Code官网](https://code.visualstudio.com/)下载并安装。

### 2. 安装必要的扩展
在VSCode中，安装以下扩展：
- **C/C++**：由Microsoft提供，用于C/C++语言支持。
- **Cortex-Debug**：用于调试ARM Cortex-M微控制器。
- **PlatformIO**（可选）：一个集成开发环境，支持多种微控制器。

可以在VSCode的扩展市场搜索并安装这些扩展。

### 3. 安装 ARM GCC 编译器
如果之前没有安装，可以通过以下命令安装ARM GCC编译器：
```bash
sudo apt install gcc-arm-none-eabi
```

### 4. 配置项目
#### 使用现有项目
- 打开你的STM32项目文件夹。
- 根据需要修改`Makefile`或其他项目配置文件。

#### 创建新项目
- 如果你使用PlatformIO，可以直接在VSCode中创建新的STM32项目。
- 否则，你需要手动创建项目结构，包括源代码文件、头文件、Makefile等。

### 5. 编写代码
使用VSCode编写你的STM32项目代码。

### 6. 构建项目
- 如果使用PlatformIO，可以直接在VSCode中构建项目。
- 如果手动管理项目，可以在VSCode的终端中运行Makefile来构建项目。

### 7. 调试配置
要在VSCode中调试STM32项目，需要创建一个`.vscode/launch.json`文件，并配置调试器。这通常涉及设置GDB路径、指定OpenOCD配置文件等。

一个基本的`launch.json`配置可能如下所示：
```json
{
    "version": "0.2.0",
    "configurations": [
        {
            "name": "Debug (OpenOCD)",
            "type": "cortex-debug",
            "request": "launch",
            "servertype": "openocd",
            "cwd": "${workspaceRoot}",
            "executable": "./build/your_project.elf",
            "configFiles": [
                "interface/stlink.cfg",
                "target/stm32f1x.cfg"
            ]
        }
    ]
}
```
请根据你的开发板和调试器进行相应的修改。

### 8. 调试
- 使用配置好的调试配置开始调试。
- 通过设置断点、单步执行、检查变量等来调试代码。

### 9. 其他工具和扩展
- **Git**：源代码版本控制。
- **其他有用的扩展**，比如Bracket Pair Colorizer, Code Spell Checker等。

使用VSCode进行STM32开发涉及多个步骤和配置，但它提供了一个强大而灵活的开发环境。上述步骤提供了一个基本的框架，你可以根据自己的项目需求进一步定制开发环境。

https://stm32world.com/wiki/STM32_development_and_debugging_using_VSCode

https://bacelarhenrique.me/2020/03/29/complete-guide-to-stm32-arm-development-on-visual-studio-code.html#:~:text=,MCUs%20on%20Visual%20Studio%20Code

https://www.cnblogs.com/Foriver/p/15772071.html#:~:text=EIDE%EF%BC%88%E5%85%A8%E7%A7%B0Embedded%20IDE%EF%BC%89%E6%98%AF%E5%9B%BD%E5%86%85%E7%9A%84%E4%B8%80%E4%B8%AA%E5%A4%A7%E4%BD%AC%E5%BC%80%E5%8F%91%E7%9A%84vscode%E6%8F%92%E4%BB%B6%EF%BC%8C%E6%9C%AC%E4%BA%BA%E7%94%A8%E4%B8%8B%E6%9D%A5%E4%BD%93%E9%AA%8C%E9%9D%9E%E5%B8%B8%E5%A5%BD%EF%BC%8C%E5%9B%A0%E6%AD%A4%E5%9C%A8%E8%BF%99%E9%87%8C%E5%88%86%E4%BA%AB%E7%BB%99%E5%A4%A7%E5%AE%B6%EF%BC%8C%E8%BF%99%E6%98%AF%E8%BF%99%E4%B8%AA%E6%8F%92%E4%BB%B6%E7%9A%84%E3%80%9019%E2%80%A0%E5%AE%98%E6%96%B9%E8%AE%BA%E5%9D%9B%E2%80%A0discuss.em

https://blog.csdn.net/ben_black/article/details/109906781#:~:text=

https://www.cnblogs.com/slike/articles/17415879.html#:~:text=%E4%B8%80%E3%80%81%E7%8E%AF%E5%A2%83%E6%90%AD%E5%BB%BA%0A%0A1%EF%BC%8E%E4%B8%8B%E8%BD%BD%E8%BD%AF%E4%BB%B6%0A%0A%EF%BC%881%EF%BC%89VS%20Code%20V1.78.2%0A%0A%E3%80%9020%E2%80%A0https%3A%2F%2Fcode.visualstudio.com%E2%80%A0code.visualstudio.com%E3%80%91%0A%0A%EF%BC%882%EF%BC%89STM32CubeMX%20V1.12.1%0A%0A%E3%80%9021%E2%80%A0https%3A%2F%2Fwww.st.com%2Fen%2Fdevelopment,scm.com%E3%80%91%0A%0A2%EF%BC%8E%E8%BD%AF%E4%BB%B6%E5%AE%89%E8%A3%85