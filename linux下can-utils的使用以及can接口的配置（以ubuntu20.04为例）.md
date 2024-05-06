# linux下can-utils的使用以及can接口的配置（以ubuntu20.04为例）

## can-utils是什么

`can-utils` 是一套用于Linux操作系统的开源工具，专门用来处理与CAN（Controller Area Network）总线相关的任务。CAN总线广泛应用于汽车和工业自动化中，用于设备之间的通讯。

这个工具集提供了多种命令行工具，用于发送、接收和处理CAN网络上的数据。比如：

1. **cansend**：发送单个CAN帧。
2. **candump**：捕获并显示经过CAN接口的数据。
3. **canplayer**：重放candump记录的数据。
4. **cansniffer**：显示CAN数据的变化。

`can-utils` 还包括了一些用于高级功能的工具，比如设置CAN硬件过滤器，或者调试CAN设备和网络的工具。这些工具通过命令行界面提供，可以灵活地集成到脚本和自动化系统中。

对于从事与汽车电子、嵌入式系统或工业网络相关工作的开发者和工程师来说，`can-utils` 是一个非常实用的资源。

## can-utils的基本使用

### 1. **candump** - 显示、过滤并记录CAN数据
`candump` 工具用于捕捉和显示通过CAN接口的数据。它还可以将数据记录到文件中，便于后续分析。

**基本用法**：
```bash
candump can0
```
这个命令将显示通过 `can0` 接口的所有CAN数据。

**过滤特定ID**：
```bash
candump can0,123:7FF
```
这个命令只显示ID为123的CAN帧。

**记录数据到文件**：
```bash
candump -l can0
```
这将把通过 `can0` 的数据记录到文件中，默认文件名格式为 `candump-日期.log`。

### 2. **canplayer** - 回放CAN日志文件
`canplayer` 用于回放用 `candump` 录制的CAN数据日志。

**基本用法**：
```bash
canplayer -I candump-2023-05-06.log
```
这个命令将回放文件 `candump-2023-05-06.log` 中记录的CAN数据。

### 3. **cansend** - 发送单个CAN帧
`cansend` 用来发送指定的CAN帧。

**基本用法**：
```bash
cansend can0 123#1122334455667788
```
这个命令向 `can0` 接口发送一个ID为123的CAN帧，数据内容为 `1122334455667788`。

### 4. **cangen** - 生成随机CAN流量
`cangen` 用于生成随机或特定规则的CAN流量，用于测试或模拟。

**基本用法**：
```bash
cangen can0 -I 1A -L 8 -D i -g 10 -n 100
```
这个命令在 `can0` 上生成100个ID为1A，长度为8字节的递增数据包，每个包之间间隔10毫秒。

### 5. **cansequence** - 发送并检查具有递增载荷的一系列CAN帧
`cansequence` 用于发送一系列具有递增载荷的CAN帧，并检查是否有丢帧现象。

**基本用法**：
```bash
cansequence can0
```
这将在 `can0` 上发送并检查一系列递增载荷的CAN帧。

### 6. **cansniffer** - 显示CAN数据内容差异
`cansniffer` 用于显示CAN数据的变化，这对于调试和理解数据流非常有帮助。

**基本用法**：
```bash
cansniffer can0
```
这个命令将监控并显示 `can0` 接口上CAN数据的任何变化。

## can接口的配置

### 虚拟can

配置虚拟CAN（vCAN）涉及在Linux系统上创建虚拟CAN接口，并将其配置为模拟实际CAN总线。以下是配置vCAN的基本步骤：

### 1. 加载vcan模块
首先，确保你的Linux内核支持vCAN，并加载vcan内核模块。通常情况下，vcan模块已经包含在Linux内核中，你只需要加载它。

```bash
sudo modprobe vcan
```

### 2. 创建vCAN接口
一旦vcan模块加载成功，你就可以使用 `ip` 命令创建虚拟CAN接口。通常，你会创建多个vCAN接口，以便模拟多个CAN总线。

```bash
sudo ip link add dev vcan0 type vcan
```

这个命令创建了一个名为 `vcan0` 的虚拟CAN接口。你可以根据需要创建更多的接口，例如 `vcan1`、`vcan2` 等。

### 3. 配置vCAN接口
一旦接口创建完成，你可以像配置物理CAN接口一样配置vCAN接口。通常，你需要设置接口的波特率和其他参数。

```bash
sudo ip link set vcan0 up
```

这个命令将激活 `vcan0` 接口，使其准备好接收和发送CAN数据。

### 4. 验证配置
你可以使用 `ip` 命令来验证vCAN接口的配置是否正确：

```bash
ip -details link show vcan0
```

这将显示 `vcan0` 接口的详细配置信息，包括状态、波特率等。

### 5. 使用vCAN接口
一旦vCAN接口配置完成，你就可以像使用实际CAN总线一样使用它了。你可以使用 `cansend`、`candump` 和其他 `can-utils` 工具来与vCAN接口进行交互，发送和接收CAN数据。

```bash
cansend vcan0 123#1122334455667788
```

这个命令将向 `vcan0` 接口发送一个ID为123的CAN帧，数据为 `1122334455667788`。

通过这些步骤，你可以在Linux系统中轻松地配置和使用虚拟CAN接口，以模拟CAN总线上的通信。这对于开发和测试CAN应用程序非常有用。

### 实体can

修改CAN接口的配置主要涉及两个步骤：首先将接口关闭，然后重新配置所需的参数，并重新激活接口。下面详细说明这一过程：

### 1. 关闭CAN接口
在修改CAN接口的配置之前，你需要先将接口关闭。使用以下命令来停用接口：

```bash
sudo ip link set can0 down
```
这里 `can0` 是你想要修改的CAN接口的名称。确保替换成你实际使用的接口名。

### 2. 修改配置
一旦接口被关闭，你就可以自由地修改其配置了。这可以通过再次使用 `ip link set` 命令来完成，添加你需要改变的参数。例如，如果你想改变波特率或设置其他模式（如环回模式或只监听模式），可以使用以下命令：

- **修改波特率**：
```bash
sudo ip link set can0 type can bitrate 250000
```
这个命令将 `can0` 的波特率修改为250 Kbps。

- **开启环回模式**：
```bash
sudo ip link set can0 type can loopback on
```
这将启用环回模式。

- **开启只监听模式**：
```bash
sudo ip link set can0 type can listen-only on
```
这将启用只监听模式。

### 3. 重新激活CAN接口
修改完配置后，你需要重新激活CAN接口。使用以下命令：

```bash
sudo ip link set can0 up
```

这个命令将重新启用 `can0` 接口，现在它将按照新的配置运行。

### 4. 验证配置
最后，为了确认配置是否正确应用，可以使用以下命令来查看接口的详细信息：

```bash
ip -details link show can0
```

这将显示 `can0` 的所有配置详情，包括波特率和其他设置的状态。
