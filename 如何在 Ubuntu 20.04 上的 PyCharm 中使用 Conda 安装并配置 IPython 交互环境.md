![image-20241004155306508](/home/lyb/github/Typora_notes/image-20241004155306508.png)

# 如何在 Ubuntu 20.04 上的 PyCharm 中使用 Conda 安装并配置 IPython 交互环境

要在 Ubuntu 20.04 上的 PyCharm 中配置 IPython 交互环境，并使用 Conda 作为包管理器进行安装，你需要遵循一系列明确的步骤。这些步骤将确保你可以在 PyCharm 中使用 Conda 环境，并且利用 IPython 的高级交互功能，从而优化你的开发体验。

### 步骤 1: 安装 Anaconda 或 Miniconda

首先，确保你的系统中已安装 Anaconda 或 Miniconda。如果尚未安装，可以从 [Anaconda](https://www.anaconda.com/products/distribution) 或 [Miniconda](https://docs.conda.io/en/latest/miniconda.html) 的官网下载安装包，并按照提供的指南进行安装。

### 步骤 2: 创建 Conda 环境并安装 IPython

1. **打开终端**：
   - 使用快捷键 `Ctrl+Alt+T` 打开 Ubuntu 终端。

2. **创建新的 Conda 环境**：
   - 输入以下命令创建一个新的 Conda 环境，这里命名为 `myenv`，你可以根据需要更改环境名：
     ```bash
     conda create --name myenv python=3.8
     ```
   - 指定 Python 版本为 3.8，根据你的需求调整。

3. **激活 Conda 环境**：
   - 激活你刚创建的环境：
     ```bash
     conda activate myenv
     ```

4. **安装 IPython**：
   - 在激活的环境中安装 IPython：
     ```bash
     conda install ipython
     ```

### 步骤 3: 配置 PyCharm 使用 Conda 环境

1. **启动 PyCharm**：
   - 打开 PyCharm IDE。

2. **配置 Python 解释器**：
   - 在 PyCharm 中，进入 `File` > `Settings`（对于 macOS 是 `PyCharm` > `Preferences`）。
   - 在设置窗口中选择 `Project` > `Python Interpreter`。
   - 点击右上角的齿轮图标，选择 `Add`。
   - 在弹出的窗口中，选择 `Conda Environment`。
   - 选择 `Existing environment`，然后点击 `...` 浏览并选择你的 Conda 环境目录中的 Python 解释器（通常位于 `anaconda3/envs/myenv/bin/python`）。
   - 点击 `OK` 保存设置。

### 步骤 4: 配置 IPython 作为默认控制台

1. **配置 IPython 控制台**：
   - 在 `Settings` 或 `Preferences` 窗口中，依次选择 `Build, Execution, Deployment` > `Console` > `Python Console`。
   - 勾选 `Use IPython if available` 选项。
   - 点击 `Apply` 然后 `OK` 以保存更改。

### 步骤 5: 验证设置

1. **打开 Python 控制台**：
   - 在 PyCharm 的底部工具栏中找到并点击 `Python Console`。
   - 检查新打开的控制台是否显示 IPython 的特征提示符（如 `In [1]:`）。

通过这些步骤，你已成功在 PyCharm 中设置了 Conda 环境，并且配置了 IPython 作为 Python 控制台。这样的配置可以充分利用 IPython 的交互式功能，提升你的代码开发和调试效率。



![image-20241004160027467](/home/lyb/github/Typora_notes/image-20241004160027467.png)