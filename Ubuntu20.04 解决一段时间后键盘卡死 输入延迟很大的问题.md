# Ubuntu20.04 解决一段时间后键盘卡死 输入延迟很大的问题

为了确保您能顺利通过双击快捷方式来重启 IBus，下面详细描述了从脚本创建到快捷方式设置的每一步，包括具体的命令行操作和必要的说明，以确保您能够按步骤成功执行。

### 步骤 1: 创建并配置脚本

1. **打开终端**：
   使用快捷键 `Ctrl+Alt+T` 打开终端。

2. **创建脚本存放的目录**：
   输入以下命令来创建一个存放脚本的文件夹（如果该文件夹尚不存在的话）：
   ```bash
   mkdir -p /home/your_username/scripts/
   ```
   替换 `your_username` 为您的实际用户名。

3. **创建脚本文件**：
   使用 `nano` 或您喜欢的任何文本编辑器来创建脚本：
   ```bash
   nano /home/your_username/scripts/restart_ibus.sh
   ```
   替换 `your_username` 为您的实际用户名。

4. **输入脚本内容**：
   在打开的编辑器中，输入以下内容：
   ```bash
   #!/bin/bash
   # 重启 IBus 服务的脚本
   ibus restart
   ```
   然后保存并关闭编辑器。在 `nano` 中，您可以通过按 `Ctrl+O` 保存更改，然后按 `Ctrl+X` 退出。

5. **设置脚本的执行权限**：
   使脚本可执行，输入以下命令：
   ```bash
   chmod +x /home/your_username/scripts/restart_ibus.sh
   ```

### 步骤 2: 创建桌面快捷方式

1. **切换到桌面目录**：
   ```bash
   cd ~/Desktop
   ```

2. **创建快捷方式文件**：
   使用文本编辑器创建 `.desktop` 文件：
   ```bash
   nano Restart_IBus.desktop
   ```

3. **编辑快捷方式内容**：
   在打开的编辑器里，输入以下内容：
   ```ini
   [Desktop Entry]
   Encoding=UTF-8
   Name=Restart IBus
   Exec=terminator -e /home/lyb/scripts/restart_ibus.sh
   Icon=preferences-desktop-keyboard-shortcuts
   Terminal=false
   Type=Application
   StartupNotify=true
   ```
   请确保替换 `your_username` 为您的实际用户名。保存并退出编辑器（在 `nano` 中按 `Ctrl+O`，然后 `Ctrl+X`）。
   
4. **给快捷方式文件设置执行权限**：
   设置桌面快捷方式的执行权限：
   
   ```bash
   chmod +x ~/Desktop/Restart_IBus.desktop
   ```

### 步骤 3: 测试快捷方式

- 在桌面找到 `Restart_IBus` 图标，双击它以测试是否能够成功重启 IBus。
- 如果快捷方式不起作用，请确认脚本路径和文件权限是否正确设置。

这些详细步骤应确保您能够通过一个双击的桌面快捷方式来控制和重启 IBus 输入法，为在输入设备失灵时提供快速的解决方案。

