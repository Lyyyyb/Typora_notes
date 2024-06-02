# ubuntu python虚拟环境（ubuntu20.04）







![image-20240602135342571](/home/lyb/github/Typora_notes/image-20240602135342571.png)

### 真实环境结构

- **Python38**:
  - 这是安装在系统中的 Python 解释器目录。
  - 包含了所有的标准库和可执行文件。
- **Lib**:
  - 包含 Python 的标准库，这是 Python 提供的内置模块和库。
- **site-packages**:
  - 这是用于存放通过 `pip` 安装的第三方库的目录。
- **Scripts**:
  - 包含可执行文件，如 `python.exe` 和 `pip.exe`。
- **python.exe**:
  - Python 解释器的可执行文件，用于运行 Python 程序。
- **pip.exe**:
  - pip 包管理工具的可执行文件，用于安装和管理 Python 包。

### 虚拟环境结构

![image-20240602135057126](/home/lyb/github/Typora_notes/image-20240602135057126.png)

![image-20240602134551530](/home/lyb/github/Typora_notes/image-20240602134551530.png)

- **venv1 / venv2**:
  - 这些是创建的虚拟环境，每个虚拟环境都有一个独立的目录。
  - 每个虚拟环境都有独立的 `Lib` 和 `Scripts` 目录。
- **Lib**:
  - 仅包含 `site-packages` 目录，而不包含标准库。
  - 标准库在虚拟环境中不会被复制，而是引用真实环境中的标准库。
- **site-packages**:
  - 存放通过 `pip` 安装的第三方库。
- **Scripts**:
  - 包含虚拟环境的可执行文件，如 `python.exe` 和 `pip.exe`，这些文件统一放在 `Scripts` 目录下。
  - 这样做简化了路径设置，只需设置一个 `Scripts` 目录的路径。

### 归纳总结

1. **虚拟环境是真实环境的副本**：
   - 虚拟环境基本上是对真实环境的一个独立副本，可以有多个虚拟环境，每个虚拟环境都是相对独立的。
2. **标准库的引用和site-packages**：
   - 虚拟环境不会复制真实环境中的标准库，而是引用它们。这样可以节省空间和时间。
   - 虚拟环境中的 `Lib` 目录只包含 `site-packages`，即用户安装的第三方库。
3. **可执行文件的组织**：
   - 虚拟环境中的所有可执行文件（如 `python.exe` 和 `pip.exe`）统一放在 `Scripts` 目录下。这简化了环境变量 `PATH` 的配置，因为只需设置一个目录路径。
4. **隔离性和灵活性**：
   - 虚拟环境允许不同的项目使用不同版本的库和 Python 解释器，避免依赖冲突。
   - 每个虚拟环境都有独立的目录和配置，可以根据需要创建和删除。