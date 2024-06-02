# ubuntu 安装Anaconda （ubuntu20.04）

## anaconda简介

Anaconda 是一个非常流行的开源分发版，主要用于科学计算、数据分析和机器学习。它集成了 Python 和 R 语言，并提供了大量预装的科学计算库和工具。Anaconda 的主要目的是简化包管理和环境管理，尤其是对于需要大量依赖包的科学计算项目。下面详细解释 Anaconda 的组成部分和作用。

### Anaconda 的组成部分

1. **Conda**：
   - **作用**：Conda 是 Anaconda 的核心包管理和环境管理工具。它不仅可以用于安装和管理 Python 包，还可以用于其他语言的包，如 R。
   - **功能**：Conda 可以创建独立的环境，每个环境可以有不同的 Python 版本和包集合，避免了包依赖冲突。

2. **Python 和 R**：
   - **作用**：Anaconda 默认包含 Python 和 R 语言的解释器。
   - **功能**：这使得用户可以直接开始使用这些语言进行数据分析和科学计算。

3. **Anaconda Navigator**：
   - **作用**：这是一个图形用户界面应用，用于管理包和环境，启动应用程序如 Jupyter Notebook 和 Spyder。
   - **功能**：通过 Navigator，用户可以直观地管理和安装包、创建和切换环境。

4. **Jupyter Notebook**：
   - **作用**：Jupyter Notebook 是一个交互式笔记本环境，允许用户编写和运行代码、可视化数据并添加注释。
   - **功能**：它非常适合用于数据分析和机器学习实验。

5. **Spyder**：
   - **作用**：Spyder 是一个集成开发环境（IDE），专为科学计算设计。
   - **功能**：它提供了强大的编辑、调试和数据探索工具，非常适合数据科学家和工程师。

6. **预装的科学库**：
   - **作用**：Anaconda 预装了大量常用的科学计算库，如 NumPy、Pandas、SciPy、Matplotlib、Scikit-learn 等。
   - **功能**：这些库为用户提供了进行数据处理、统计分析、机器学习和数据可视化的基本工具。

### Anaconda 的作用

1. **简化包管理**：
   - 使用 Conda，用户可以轻松地安装、更新和卸载包。Conda 能够自动处理包依赖关系，避免了手动管理依赖的复杂性。

2. **环境隔离**：
   - Conda 允许创建独立的环境，每个环境可以有不同的包和 Python 版本。这种隔离避免了不同项目之间的依赖冲突，确保项目的稳定性和可重复性。

3. **科学计算和数据分析**：
   - Anaconda 提供了一整套用于科学计算和数据分析的工具和库，使得数据科学家和研究人员可以快速上手进行数据处理和分析。

4. **集成开发环境**：
   - Anaconda 包含的 Spyder 和 Jupyter Notebook 提供了强大的开发和实验平台，适合编写、调试和分享代码。

5. **跨平台支持**：
   - Anaconda 支持 Windows、MacOS 和 Linux 操作系统，使得用户可以在不同的平台上获得一致的开发体验。

6. **社区和企业支持**：
   - Anaconda 有广泛的社区支持，提供大量的教程和资源。此外，Anaconda, Inc. 还提供企业级的支持和服务，适合企业使用。

### 总结

Anaconda 是一个功能强大且易于使用的分发版，适合数据科学、机器学习和科学计算。它集成了 Python 和 R 语言，以及大量预装的科学库和工具，通过 Conda 实现高效的包管理和环境管理。Anaconda 使得用户可以快速部署和管理复杂的计算环境，专注于数据分析和研究工作。

## 下载anaconda安装脚本

- 官网地址：https://www.anaconda.com/download/success
- 直接点击Download下载，或者右键复制链接，使用wget下载，命令如下

```bash
wget https://repo.anaconda.com/archive/Anaconda3-2024.02-1-Linux-x86_64.sh
```

![image-20240602123248298](/home/lyb/github/Typora_notes/image-20240602123248298.png)

## 运行安装脚本

- 找到安装脚本所在目录，使用bash命令运行安装脚本以安装Anaconda

![image-20240602123528324](/home/lyb/github/Typora_notes/image-20240602123528324.png)

- 根据提示，这里需要一直按着enter，安装协议有点长，当然可以直接Ctrl+C跳过条款的阅读

![image-20240602123708952](/home/lyb/github/Typora_notes/image-20240602123708952.png)

![image-20240602123818191](/home/lyb/github/Typora_notes/image-20240602123818191.png)

- 最后输入yes即可

![image-20240602124056346](/home/lyb/github/Typora_notes/image-20240602124056346.png)

- 继续enter，安装目录默认即可

![image-20240602124222673](/home/lyb/github/Typora_notes/image-20240602124222673.png)

- 提示是否希望更新shell配置文件，以便在启动终端时自动初始化Conda，选择yes

![image-20240602124545633](/home/lyb/github/Typora_notes/image-20240602124545633.png)

- 安装成功

![image-20240602124642057](/home/lyb/github/Typora_notes/image-20240602124642057.png)

## 基本设置

### 重新打开终端后直接进入了conda的base环境

- 这是因为在上文的安装过程中，在提示是否希望更新shell配置文件，以便在启动终端时自动初始化Conda，选择了yes

![image-20240602124545633](/home/lyb/github/Typora_notes/image-20240602124545633.png)

![image-20240602124941893](/home/lyb/github/Typora_notes/image-20240602124941893.png)

- 如果你不希望在终端启动时自动激活 Conda 的基本环境，那么在 Conda 环境激活后运行以下命令：

```bash
conda config --set auto_activate_base false
```

- 之后再次打开终端就不会自动激活Conda的基本环境了

![image-20240602125724682](/home/lyb/github/Typora_notes/image-20240602125724682.png)

- 如需再次进入base环境，则在终端输入,即可再次激活conda环境

```bash
conda activate base
```

