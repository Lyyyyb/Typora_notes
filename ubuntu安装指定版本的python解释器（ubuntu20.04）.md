# ubuntu安装指定版本的python解释器（ubuntu20.

## 下载python解释器源码包

- 官网地址：https://www.python.org/downloads/
- 选择自己需要的版本，点击download（以Python 3.10.14为例）

![image-20240601143634554](/home/lyb/github/Typora_notes/image-20240601143634554.png)

- 往下滑，根据自己的需要，选择要下载的python解释器源码包，我这里选的是gzip压缩格式
  - **Gzipped source tarball**：使用 `gzip` 压缩格式，文件大小为 24.7 MB。
  - **XZ compressed source tarball**：使用 `xz` 压缩格式，文件大小为 18.7 MB。

![image-20240601143745341](/home/lyb/github/Typora_notes/image-20240601143745341.png)

- 可以直接点击Gzipped source tarball下载，也可以右击，复制下载链接后，使用wget下载，在终端输入：（使用 `wget` 下载文件时，默认情况下文件会保存到当前终端所在的目录。如果你没有指定其他路径，下载的文件会出现在你运行 `wget` 命令的那个目录里。）

```bash
sudo wget https://www.python.org/ftp/python/3.10.14/Python-3.10.14.tgz
```

## 解压源码包

- 在终端输入：解压源码包

```bash
sudo tar -vxf Python-3.10.14.tgz
```

- 进入到解压后的目录，在终端输入

```bash
cd Python-3.10.14/
```

## 编译安装源码包

### 默认安装目录 

- 运行配置脚本，检查系统环境和依赖项，生成编译所需的 `Makefile`

```bash
sudo ./configure
```

- 调用 `make` 工具，根据 `Makefile` 中的指令编译软件。

```bash
sudo make
```

- 调用 `make` 工具的 `install` 目标，将已经编译完成的软件安装到系统中。（安装过程通常会将可执行文件、库文件、头文件等复制到系统指定的目录中，以便其他程序可以使用该软件。）

```bash
sudo make install
```

- 这时候**Python**已经安装完成，可执行文件在/usr/local/bin下，库文件在/usr/local/lib下，配置文件在/usr/local/include下，其他资源文件在/usr/local/share下，大家用**Pycharm**等编辑器使用**Python**时就用这些路径

### 自定义安装目录

- 运行配置脚本，检查系统环境和依赖项，生成编译所需的 `Makefile`，并利用–prefix=指定安装路径为/usr/local/python3.10.14，在终端输入

```bash
sudo ./configure --prefix=/usr/local/python3.10.14
```

- 调用 `make` 工具，根据 `Makefile` 中的指令编译软件。

```bash
sudo make
```

- 调用 `make` 工具的 `install` 目标，将已经编译完成的软件安装到系统中。（安装过程通常会将可执行文件、库文件、头文件等复制到系统指定的目录中，以便其他程序可以使用该软件。）

```bash
sudo make install
```

- 添加环境变量

```bash
PATH=$PATH:$HOME/bin:/usr/local/python3.10.14/bin
```

## 软链接新版的python解释器

### 删除原有链接

- 删除原有链接，这样做会删除这两个文件，而不会删除任何其他文件或数据。通常情况下，这样的操作是为了删除旧的 Python 符号链接，以便重新创建指向新安装 Python 解释器的符号链接。

```bash
sudo rm /usr/bin/python
sudo rm /usr/bin/python3
```

### 创建新的软链接

#### 默认路径安装

- 创建符号链接（symbolic link），将 `/usr/local/bin/python3.10` 这个文件链接到系统默认的 Python 解释器路径（`/usr/bin/python` 和 `/usr/bin/python3`）。这样做的目的是将系统默认的 `python` 和 `python3` 命令指向 `/usr/local/bin/python3.10`，使系统在运行 `python` 或 `python3` 命令时使用 `/usr/local/bin/python3` 这个解释器版本。

```bash
sudo ln -s /usr/local/bin/python3.10 /usr/bin/python
sudo ln -s /usr/local/bin/python3.10 /usr/bin/python3 
```

#### 自定义路径安装

- 将我们刚装的python3.10.14指定运行命令为python与python3，创建符号链接（symbolic link），将新安装的 Python 3.10.14 解释器的可执行文件（通常位于 `/usr/local/python3.10.14/bin/python3.10`）链接到系统默认的 Python 解释器路径（`/usr/bin/python` 和 `/usr/bin/python3`）。这样做的目的是将系统默认的 `python` 和 `python3` 命令指向新安装的 Python 3.10.14 解释器，使系统在运行 `python` 或 `python3` 命令时使用新安装的解释器版本。

```bash
sudo ln -s /usr/local/python3.10.14/bin/python3.10 /usr/bin/python
sudo ln -s /usr/local/python3.10.14/bin/python3.10 /usr/bin/python3.10
```

#### 出现的问题

- 经过测试如果执行一下命令

```bash
sudo ln -s /usr/local/python3.10.14/bin/python3.10 /usr/bin/python3.10
```

- 将导致终端无法打开，还没搞清楚原因，但是可以通过下面这段指令解决，

```bash
sudo rm /usr/bin/python3
sudo ln -s /usr/bin/python3.8 /usr/bin/python3
```

- 安装完成后，在终端输入python，显示如下，安装完成

![image-20240601221613318](/home/lyb/github/Typora_notes/image-20240601221613318.png)

- 在终端输入如下指令，查看可执行文件的路径

```bash
which python
```

- 在终端输入如下指令，查看软链接指向是否正确

```
ls -l /usr/bin/python
```

![image-20240601221838898](/home/lyb/github/Typora_notes/image-20240601221838898.png)

- 显示如上图，软链接指向正确





### 1. `which python3`

命令`which`是一个在系统的PATH环境变量指定的目录中搜索给定程序名的可执行文件的工具。如果找到了这个可执行文件，它会输出该文件的完整路径。

- **命令格式**：`which [程序名]`
- **作用**：
  - 确定一个程序的可执行文件的位置。
  - 验证是否安装了某个程序，以及它的安装位置。
- **示例**：`which python3`
  - 这个命令查找`python3`这一命令的位置。如果在PATH指定的目录中找到了`python3`的可执行文件，它会输出这个文件的完整路径。这可以帮助你确定系统中使用的默认Python版本的安装位置。

### 2. `ls -l /usr/local/bin/python3`

`ls`是一个列出目录内容的命令，`-l`参数表示以“长列表格式”显示信息，它提供了文件的详细信息，如权限、链接数、所有者、组、大小、时间戳和名称。

- **命令格式**：`ls -l [文件或目录路径]`
- **作用**：
  - 查看指定文件或目录的详细信息。
  - 对于文件，显示包括文件类型、权限、硬链接数、所有者、组、文件大小、最后修改日期等信息。
  - 对于链接，显示链接本身的信息及其指向的目标。
- **示例**：`ls -l /usr/local/bin/python3`
  - 这个命令显示`/usr/local/bin/python3`的详细信息。如果`python3`是一个软链接，这个命令还会显示它指向的目标路径。这有助于了解`/usr/local/bin`中的`python3`是实体文件还是指向其他位置的链接。
