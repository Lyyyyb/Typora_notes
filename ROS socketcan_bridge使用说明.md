# ROS socketcan_bridge使用说明（以ubuntu20.04为例）

## socketcan_bridge是什么

- ROS针对socketcan提供了三个层次的驱动库，分别是`ros_canopen`，`socketcan_bridge`和`socketcan_interface`。

  1. **`socketcan_interface`**：
     - **功能**：这是最底层的包，直接与Linux的SocketCAN库交互。它提供了基础的API来发送和接收CAN帧，抽象化了对硬件的直接操作。
     - **作用**：它作为基础层，被其他高级的ROS CAN包使用，来进行更复杂的数据处理和通信逻辑。

  2. **`socketcan_bridge`**：
     - **功能**：这个包是用于将ROS消息（topics）与CAN帧之间进行转换的中间件。它监听ROS系统中的特定topics，将消息转换为CAN帧并发送到CAN网络；同时，它也接收来自CAN网络的帧，转换成ROS消息并发布到ROS网络。
     - **作用**：`socketcan_bridge`是ROS中最常用的包之一，因为它桥接了ROS系统与实际的硬件设备之间的通信，是实现两者间互操作性的关键组件。

  3. **`ros_canopen`**：
     - **功能**：这个包是基于CANopen协议的实现，它利用`socketcan_interface`提供的接口，实现了CANopen标准的高级功能，如节点管理、数据对象传输等。
     - **作用**：`ros_canopen`用于那些需要符合CANopen通信协议的复杂工业应用，提供了一套完整的解决方案，使得ROS能够更好地集成进这些环境中。

  这三个包的关系非常明确：`socketcan_interface`作为基础，提供了与SocketCAN的直接交互；`socketcan_bridge`使用这些接口将ROS系统与CAN网络连接起来；而`ros_canopen`则在这些基础上实现了符合CANopen协议的高级功能。这样的分层设计使得每个组件都可以专注于其核心功能，同时保持了系统的模块化和可扩展性。

## socketcan_bridge的使用

### 安装

- 终端键入以下命令

```bash
sudo apt-get install ros-noetic-socketcan-bridge
```

- 在安装`ros-noetic-socketcan-bridge`包时，Ubuntu的包管理器会自动安装以下三个相关的ROS包，以确保`socketcan_bridge`的功能完整性和依赖关系满足：
  - **ros-noetic-can-msgs**：这个包定义了在ROS topics中用于CAN通信的消息类型。它提供了标准的消息定义，例如CAN帧，这些定义对于在ROS中处理CAN数据是必需的。
  - **ros-noetic-socketcan-bridge**：这是核心的转换包，负责将ROS messages（topics）与CAN帧之间进行双向转换。这允许ROS节点能够发送和接收CAN网络上的数据。
  - **ros-noetic-socketcan-interface**：这个包提供了与底层SocketCAN通信所需的接口。`socketcan-bridge`依赖此包来实现与Linux系统中SocketCAN驱动的交互。、

### 组成

- `socketcan_bridge` 包提供了三个主要的节点，以支持不同的通信需求：

  - **socketcan_bridge_node**：这是一个集成节点，能同时处理从CAN接收的数据和发送到CAN的数据。使用这个节点可以防止发送的每个消息被回显到接收主题，即避免了发送和接收的数据混淆。


  - **socketcan_to_topic_node**：这个节点专门负责将从CAN接收的数据转换成ROS topics。它只处理接收操作。


  - **topic_to_socketcan_node**：与`socketcan_to_topic_node`相对，这个节点负责将ROS topics中的消息转换成CAN帧，并发送到CAN网络。


通过这样的设计，`socketcan_bridge` 提供了灵活的配置选项，以适应不同的应用场景，无论是单向还是双向通信。这使得用户可以根据具体的需求选择合适的节点，以实现高效且准确的数据交换。

### 工作流程

- ROS上位机装载CAN报文 ，并作为Publisher将CAN报文发布到“sent_messages”话题。
- topic_to_socketcan_node节点监听“sent_messages”话题，是否有消息发布，如果有，则会将发布的ROS消息转换为CAN报文，并发布到CAN总线上。
- 驱动器根据CAN ID接收CAN报文，并向ROS上位机返回一个应答CAN报文。
- socketcan_to_topic_node节点监听CAN总线，是否有CAN报文发布，如果有，则接收CAN报文，并将CAN报文转换为ROS消息然后发布到“received_messages”话题。
- ROS上位机订阅“received_messages”话题，通过回调函数处理接收到的CAN报文。

### socketcan_bridge_node节点

#### 功能描述

1. **接收并发布**：节点从SocketCAN设备接收CAN帧，并将这些帧发布到一个名为`received_messages`的ROS主题上。这使得其他ROS节点可以订阅这个主题以获取来自CAN网络的数据。
   
2. **监听并发送**：同时，该节点订阅一个名为`sent_messages`的ROS主题，该主题中的消息会被转换为CAN帧并发送到SocketCAN设备。这允许ROS系统内的其他部分能够通过CAN网络与外部设备进行通信。

3. **防止回显**：发送到CAN设备的帧不会被作为接收消息再次发布到ROS中。这防止了可能的数据重复和混淆。

#### ROS节点参数

- **can_device**：这是一个节点参数，用于指定SocketCAN设备的名称。默认情况下，设备名为`can0`，但可以通过设置参数来更改。

#### 修改配置

- 如果你需要根据特定的应用需求修改话题名称或CAN设备的参数，可以通过修改launch文件来实现。例如，你可以使用`remap`标签改变话题的名称，或者通过`param`标签设置不同的CAN设备名。

#### Launch文件示例

下面是一个ROS launch文件的例子，它展示了如何配置`socketcan_bridge_node`：

```xml
<launch>
    <node pkg="socketcan_bridge" type="socketcan_bridge_node" name="socketcan_bridge_node" output="screen">
        <param name="can_device" value="can0"/>
        <remap from="sent_messages" to="your_topic_name"/>
        <remap from="received_messages" to="your_topic_name"/>
    </node>
</launch>
```

这个配置允许用户自定义CAN设备的名称和与节点交互的ROS话题名称，使得`socketcan_bridge_node`能够灵活地适应不同的使用场景和系统架构。

### 系统架构及数据流

#### 主要组件

1. **socketcan_interface包**
   - 这是一个ROS包，提供了与Linux的SocketCAN驱动程序进行接口的基础设施。它允许ROS节点直接与CAN设备进行低层次的通信。

2. **socketcan_bridge节点**
   - 这个节点是`socketcan_bridge`包的一部分，它桥接了ROS系统和CAN网络。该节点负责将ROS消息转换为CAN帧，以及将接收到的CAN帧转换为ROS消息。

3. **socketcan_bridge包**
   - 这个包包含了实现桥接功能的节点，包括`socketcan_bridge_node`，它处理实际的消息转换和通信。

#### 数据流向

- **从ROS到CAN设备**
  - ROS节点发布的消息通过`/sent_messages`话题发送。这些消息是`can_msgs/Frame`类型，包含了CAN帧的信息。
  - `socketcan_bridge_node`接收这些消息，并通过`socketcan_interface`将它们转换为SocketCAN可以处理的格式。
  - 消息经由连接到计算机的USB接口和USB转SocketCAN的硬件接口被发送到CAN网络。

- **从CAN设备到ROS**
  - CAN设备发送的CAN帧通过USB转SocketCAN硬件接口和USB接口进入计算机。
  - 这些帧被`socketcan_interface`捕获并转换为ROS能够理解的`can_msgs/Frame`消息格式。
  - `socketcan_bridge_node`然后将这些消息发布到`/received_messages`话题，供其他ROS节点使用。

#### 硬件接口

- **USB转SocketCAN设备**
  - 这是一种硬件设备，能够将USB接口转换为CAN通信接口。它使得没有原生CAN接口的计算机可以通过USB端口与CAN网络连接和通信。

#### 操作系统和驱动

- **Linux SocketCAN驱动**
  - SocketCAN是Linux内核的一部分，提供了对CAN设备的支持。它使得用户空间程序可以通过标准的套接字接口与CAN硬件交互。

### CAN数据帧

- socketcan_bridge_node的话题的消息类型都为**ROS内置消息类型**`can_msgs/Frame`：

```go
Header header
uint32 id
bool is_rtr
bool is_extended
bool is_error
uint8 dlc
uint8[8] data
```

### 消息结构 `can_msgs/Frame`

1. **Header header**
   - 这是一个标准的ROS消息头，包含时间戳和帧序列号。时间戳用于记录消息的接收或发送时间，而帧序列号用于跟踪消息的顺序。

2. **uint32 id**
   - 这是CAN帧的标识符。在CAN网络中，ID用于标识不同的消息类型和优先级。ID的值和大小直接影响到消息的处理优先级，值越小优先级越高。

3. **bool is_rtr**
   - 这是一个布尔值，表示这是否为一个远程传输请求（Remote Transmission Request）帧。RTR帧是一种特殊类型的CAN帧，用于请求发送某个特定ID的数据。

4. **bool is_extended**
   - 这个布尔值指示ID是否为扩展格式。CAN协议支持标准ID（11位）和扩展ID（29位）。扩展ID允许更多的ID数值和更复杂的网络结构。

5. **bool is_error**
   - 这是一个布尔值，表示该帧是否为错误帧。错误帧用于在CAN网络上报告错误状态。

6. **uint8 dlc**
   - 这表示数据长度代码（Data Length Code），它指定`data`字段中的字节数。在CAN协议中，dlc可以是0到8的任何值，表示帧数据字段的实际字节大小。

7. **uint8[8] data**
   - 这是一个具有8个字节的数组，包含实际的数据负载。根据`dlc`的值，数组中的0到8个元素将被使用。

### 数据填充

在使用`can_msgs/Frame`类型发送CAN帧时，需要正确填充`id`、`dlc`（数据段长度），以及`data`（实际的数据内容）。例如，如果你需要发送一个帧，其中包含实际的传感器读数或控制命令，你将根据需要的数据格式来设置这三个字段。正确设置这些字段是确保CAN帧正确被网络上的其他设备解读的关键。



## 遇到的问题

### 关键点总结
1. **初始化顺序的重要性**：
   - 在ROS中，发布器（`can_pub`）和订阅器（`cmd_vel_sub`）的初始化顺序至关重要，特别是当订阅器的回调函数依赖于发布器时。
   - 如果发布器在订阅器之前被初始化，那么回调函数在调用时可以安全地使用发布器，因为它已经被正确地设置和配置。

2. **回调函数中的变量引用**：
   - 使用`boost::bind`绑定回调函数时，确保所有引用的变量（如`can_pub`）在回调函数被调用之前已经被初始化。
   - 任何在回调函数中使用的变量都必须在该函数触发之前处于有效和稳定的状态。

3. **错误和行为未定义的风险**：
   - 如果在变量完全初始化之前设置了订阅器，那么当回调函数触发并尝试使用这些未初始化的变量时，可能会导致运行时错误或行为未定义。
   - 这种情况下，程序可能会崩溃或产生不可预料的结果，因为它尝试访问未正确初始化的资源。

### 解决方案
- **先初始化发布器，再设置订阅器**：
   - 为了避免初始化顺序问题，应先初始化所有需要在回调函数中使用的发布器或服务服务器，然后再创建任何使用这些对象的订阅器。
   - 这样做可以确保当回调函数被调用时，所有需要的资源都已经处于可用状态。

通过遵循这些指导原则，你可以确保你的ROS节点在处理发布和订阅操作时更加稳定和安全，从而避免因初始化顺序不当而导致的错误。