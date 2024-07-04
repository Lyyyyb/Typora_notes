# 在ubuntu下使用socat创建虚拟串口并使用虚拟串口进行调试

在Ubuntu 20.04下使用虚拟串口进行调试，可以通过创建虚拟串口对来模拟串行通信。这是非常有用的，尤其是在开发和测试涉及串行通信的软件时。下面将提供详细步骤，如何在Ubuntu 20.04中设置虚拟串口，以及如何使用这些串口进行数据发送和接收的测试。

### 步骤 1: 安装必要工具

首先，你需要安装`socat`，它是一个多功能的命令行工具，可以用来创建连接虚拟串口的通道。

打开一个终端窗口，执行以下命令安装`socat`：

```bash
sudo apt update
sudo apt install socat
```

### 步骤 2: 创建虚拟串口对

使用`socat`来创建一对虚拟串口。在终端中执行以下命令：

```bash
socat -d -d pty,raw,echo=0,link=/tmp/virtual-ttyS0 pty,raw,echo=0,link=/tmp/virtual-ttyS1
```

这个命令将创建两个虚拟串口，并为它们创建符号链接`/tmp/virtual-ttyS0`和`/tmp/virtual-ttyS1`，便于记忆和访问。

- **`-d -d`** 提高调试信息的输出，帮助诊断问题。
- **`pty`** 表示创建一个伪终端设备。
- **`raw`** 表示数据传输不进行任何处理。
- **`echo=0`** 表示关闭回显，发送到这个端口的数据不会在同一个端口显示。
- **`link=/tmp/virtual-ttyS0`** 创建一个符号链接到具体的伪终端设备，便于使用。

或者用以下指令也可以，不额外创建符号链接：

```bash
socat -d -d pty,raw,echo=0 pty,raw,echo=0
```

这条命令将输出如下内容：

![image-20240704153840850](/home/lyb/github/Typora_notes/image-20240704153840850.png)

### 步骤 3: 使用 `screen` 或其他工具进行交互(以screen为例)

你可以使用 `screen` 或其他终端仿真程序来与这些虚拟串口进行交互。安装 `screen` 并连接到其中一个虚拟串口：

```bash
sudo apt install screen
```

现在，你可以使用任何支持串口的程序或工具来访问这些虚拟串口。例如，你可以使用`screen`，`minicom`或任何自定义开发的程序来发送和接收数据。

#### 打开两个终端窗口，分别连接这两个虚拟串口：

在第一个终端，运行：

```bash
screen /tmp/virtual-ttyS0 115200
```

在第二个终端，运行：

```bash
screen /tmp/virtual-ttyS1 115200
```

这里，`115200`是设置的波特率，虽然对于虚拟串口来说这不是必需的，但可以指定以模仿真实设备的设置。

#### 在实际代码中的使用

在实际代码中，你需要修改对应的串口的端口下，例如：

```C++
SerialHandler serialHandler("/tmp/virtual-ttyS0", 115200);
```

之后，打开终端运行：

```bash
screen /tmp/virtual-ttyS1 115200
```

如果串口发送的代码正确，就可以看到输出内容了，如下:

![image-20240704154424865](/home/lyb/github/Typora_notes/image-20240704154424865.png)

### 步骤 4: 测试数据传输

在`screen`的一个会话中输入字符，这些字符应该会出现在另一个`screen`会话中。这证明了数据可以通过虚拟串口从一个端点发送到另一个端点。如下：

![image-20240704154700023](/home/lyb/github/Typora_notes/image-20240704154700023.png)

### 步骤 5: 清理

在测试完成后，你可能需要关闭`socat`进程。找出`socat`的进程ID并杀掉它：

```bash
ps aux | grep socat
kill -9 [PID]
```

