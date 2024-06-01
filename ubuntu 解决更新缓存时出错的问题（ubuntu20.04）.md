# ubuntu 解决更新缓存时出错的问题（ubuntu20.04）

![](/home/lyb/github/Typora_notes/微信图片_20240601225325.jpg)

![image-20240601230023372](/home/lyb/github/Typora_notes/image-20240601230023372.png)

这张图片显示了Ubuntu系统在使用APT工具更新软件包时遇到的一系列错误，主要与软件源的可用性和代理配置问题有关。这里的错误提示指出，“Unsupported proxy configured: 127.0.0.1:7890”，即配置了不被支持的代理设置。解决这些问题的关键在于检查和更正系统的代理设置，以及确保软件源配置的正确性。

### 详细解决方案和步骤

#### 步骤 1: 检查并更正代理设置

1. **检查环境变量代理配置**：
   打开终端（可以使用快捷键 `Ctrl+Alt+T`）并输入以下命令查看当前的代理设置：
   ```bash
   echo $http_proxy
   echo $https_proxy
   echo $ftp_proxy
   ```

2. **取消设置代理**：
   如果上述命令返回了代理设置，使用以下命令取消这些代理：
   ```bash
   unset http_proxy
   unset https_proxy
   unset ftp_proxy
   ```

3. **检查APT配置文件中的代理设置**：
   查看APT的代理配置文件是否包含代理设置，特别是检查 `/etc/apt/apt.conf` 或 `/etc/apt/apt.conf.d/` 目录下的文件：
   ```bash
   sudo nano /etc/apt/apt.conf
   ```
   如果存在类似 `Acquire::http::Proxy "http://127.0.0.1:7890";` 的行，请将其删除或注释掉。

#### 步骤 2: 更新软件源列表

1. **运行APT更新**：
   更新软件包列表，确认没有代理错误阻碍更新：
   ```bash
   sudo apt update
   ```

2. **修复可能的软件源错误**：
   如果更新过程中提示某些软件源无法访问或存在错误（如“没有Release文件”），需要检查并编辑 `/etc/apt/sources.list` 文件及 `/etc/apt/sources.list.d/` 目录下的文件：
   ```bash
   sudo nano /etc/apt/sources.list
   ```
   对于每个出错的源，可以尝试注释掉（在行首添加`#`）或更换为其他镜像源。

#### 步骤 3: 清除和自动修复

1. **清理无用的软件包**：
   ```bash
   sudo apt autoremove
   ```

2. **尝试自动修复损坏的包**：
   ```bash
   sudo apt -f install
   ```

#### 步骤 4: 重新尝试更新和升级

1. **再次更新软件包列表**：
   ```bash
   sudo apt update
   ```

2. **升级所有已安装的软件包**：
   ```bash
   sudo apt upgrade
   ```

#### 步骤 5: 重启计算机

完成上述步骤后，重启计算机确保所有更改生效：
```bash
sudo reboot
```

这些步骤应该能够解决你面临的关于APT软件源和代理设置的问题。如果问题仍然存在，可能需要进一步检查网络连接设置或联系网络管理员确认是否有网络策略或防火墙规则影响到了代理或HTTP连接。

