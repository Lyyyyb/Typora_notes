# ubuntu 更换软件源

在Ubuntu 20.04 LTS中更换软件源可以通过图形界面或命令行进行。更换软件源通常是为了加快软件包的下载速度或解决软件源不可用的问题。这里我将分别说明如何通过图形界面和命令行更换软件源。

### 通过图形界面更换软件源：

1. **打开“软件和更新”设置**：
   - 你可以通过点击系统的“Show Applications”（显示应用程序）按钮（通常位于Dock的底部），然后搜索“Software & Updates”（软件和更新）来找到并打开它。

2. **选择“Ubuntu软件”标签**：
   - 在打开的窗口中，点击“Ubuntu软件”标签页。

3. **选择新的下载服务器**：
   - 在“下载自”一栏，点击下拉菜单。你可以选择“其他...”，然后选择一个镜像服务器。系统会提供一个列表，包括各地的镜像服务器。
   - 选择“选择最佳服务器”（Select Best Server），系统将测试并选择响应时间最快的服务器。

4. **更新更改**：
   - 选择好服务器后，点击“选择服务器”（Choose Server），然后关闭所有“软件和更新”窗口。
   - 系统可能会提示你重新加载软件包信息，点击“重新加载”（Reload）。

### 通过命令行更换软件源：

1. **打开终端**：
   - 可以通过快捷键`Ctrl+Alt+T`打开终端。

2. **备份当前的sources.list文件**：
   - 在做任何修改之前，先备份你当前的`sources.list`文件以防万一：
     ```bash
     sudo cp /etc/apt/sources.list /etc/apt/sources.list.backup
     ```

3. **编辑sources.list文件**：
   - 使用文本编辑器编辑`sources.list`文件：
     ```bash
     sudo nano /etc/apt/sources.list
     ```
   - 你可以注释掉（在行首添加`#`）当前的软件源，并在文件底部添加新的软件源。例如，要更换为清华大学的源，添加以下行：
     ```
     deb http://mirrors.tuna.tsinghua.edu.cn/ubuntu/ focal main restricted universe multiverse
     deb http://mirrors.tuna.tsinghua.edu.cn/ubuntu/ focal-updates main restricted universe multiverse
     deb http://mirrors.tuna.tsinghua.edu.cn/ubuntu/ focal-backports main restricted universe multiverse
     deb http://mirrors.tuna.tsinghua.edu.cn/ubuntu/ focal-security main restricted universe multiverse
     ```
   - 按`Ctrl+O`保存文件，然后`Ctrl+X`退出nano编辑器。

4. **更新软件包列表**：
   - 更换源之后，更新软件包列表以确保你的软件包信息是最新的：
     ```bash
     sudo apt update
     ```

这些步骤可以帮助你在Ubuntu 20.04中更换软件源，无论是通过图形界面还是命令行。选择一个离你较近的镜像服务器通常可以提高下载和更新的速度。