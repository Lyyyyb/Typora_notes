# ROS快速入门

## ROS是什么

ROS（Robot Operating System）是一个用于机器人开发的灵活框架。虽然它被称作“操作系统”，但更准确地说，它是一系列软件库和工具，旨在帮助软件开发人员创建复杂而强大的机器人应用程序。ROS 提供的功能包括：

1. **硬件抽象**：ROS 提供一种统一的方式来访问不同的机器人硬件，使开发人员能够编写与具体硬件无关的代码。

2. **设备驱动程序**：ROS 包括各种机器人硬件的驱动程序，如摄像头、传感器和执行器。

3. **通用功能库**：它包含一系列重用的功能和算法，如路径规划、导航、映射和机器人臂的运动控制。

4. **消息传递系统**：ROS 使用一种发布/订阅的消息传递模型，允许不同的进程（称为“节点”）之间进行通信。这种模型有助于创建模块化和可扩展的机器人应用程序。

5. **工具和可视化**：ROS 提供各种工具来帮助开发和调试机器人程序，例如RViz（用于3D可视化）和Gazebo（用于模拟）。

6. **社区和支持**：由于ROS是开源的，它有一个活跃的社区，提供广泛的文档、教程和支持。此外，社区成员贡献了大量的ROS软件包，这些软件包扩展了ROS的功能。

ROS的设计哲学强调模块化和可重用性。在ROS中，复杂的机器人系统可以分解为小的、相互协作的部分（节点）。这种方法使得开发、测试和维护变得更加容易，也促进了代码的共享和重用。

ROS有两个主要的版本线：ROS 1和ROS 2。ROS 2是最新的版本，它解决了ROS 1的一些限制，如实时处理能力和跨平台支持（例如与Windows的兼容性）。

总的来说，ROS是一种强大的工具，极大地促进了机器人研究和开发的速度和效率。

## ROS的分布式和模块化

在 ROS（Robot Operating System）中，分布式和模块化是两个核心概念，它们使得机器人系统的设计和实现更加灵活和强大。

### 分布式系统

ROS 的分布式特性指的是系统中不同部分可以在不同的物理设备上运行，并通过网络进行通信。这意味着一个机器人系统可以由多台计算机协同工作构成，每台计算机负责系统的不同部分。

#### 实例：多传感器集成
假设有一个机器人，它配备了多种传感器，如摄像头、激光雷达和超声波传感器。在分布式ROS系统中，每个传感器可以通过一个独立的ROS节点来控制，这些节点可以分布在不同的计算机上。例如，摄像头数据处理可能在一个高性能的计算机上，而导航和控制算法可能在机器人的主板上。这些节点通过ROS的通信机制（如话题、服务和动作）交换信息。

### 模块化

模块化是指在 ROS 中，复杂的功能被分解成小的、独立的模块，称为“节点”。每个节点执行特定的任务，例如传感器数据处理、决策制定、运动规划等。这种模块化设计使得系统易于管理、维护和扩展。

#### 实例：自主导航系统
在一个自主导航系统中，系统可以分为多个模块，如地图构建、定位、路径规划和障碍物避让。每个模块由一个或多个ROS节点实现。

1. **地图构建节点**：处理来自传感器（如激光雷达）的数据，构建环境地图。
2. **定位节点**：使用地图和传感器数据确定机器人在环境中的位置。
3. **路径规划节点**：根据目标位置和当前位置，计算到达目的地的最佳路径。
4. **障碍物避让节点**：在机器人移动过程中实时检测和规避障碍物。

在这个例子中，每个节点都独立运行，并通过ROS的通信机制相互交换信息。例如，定位节点会将机器人的位置信息发送给路径规划节点，而路径规划节点则会将计算出的路径发送给控制节点，以引导机器人移动。

总结来说，ROS的分布式和模块化设计允许复杂的机器人系统被分解成更小、更易于管理的部分。这种设计不仅提高了系统的灵活性和可扩展性，还使得机器人开发更加高效和易于维护。

## ROS和ubuntu的版本对应关系

ROS（Robot Operating System）的不同版本通常与特定的Ubuntu版本相关联。这种关联是因为ROS的某些版本是针对Ubuntu的特定版本开发和优化的，以确保最佳的兼容性和稳定性。下面是一些常见的ROS版本及其对应的Ubuntu版本：

1. **ROS 1**
   - **Noetic Ninjemys**（最新的ROS 1版本，截至2023年）: 支持Ubuntu 20.04 (Focal Fossa)。
   - **Melodic Morenia**: 支持Ubuntu 18.04 (Bionic Beaver)。
   - **Kinetic Kame**: 支持Ubuntu 16.04 (Xenial Xerus)。

2. **ROS 2**
   - **Humble Hawksbill**（截至2023年的最新版本）: 支持Ubuntu 22.04 (Jammy Jellyfish)。
   - **Galactic Geochelone**: 支持Ubuntu 20.04 (Focal Fossa)。
   - **Foxy Fitzroy**: 也支持Ubuntu 20.04 (Focal Fossa)。
   - **Eloquent Elusor**: 支持Ubuntu 18.04 (Bionic Beaver)。

请注意，随着时间的推移，ROS版本和Ubuntu版本都会更新，所以这个列表可能不包括最新的版本。为了获取最新的信息，建议访问ROS的官方网站或相关社区资源。

另外，某些ROS版本可能在其主要支持的Ubuntu版本之外的其他版本上也能运行，但可能需要额外的配置或可能存在兼容性问题。因此，为了确保最佳体验，建议使用ROS版本官方推荐的Ubuntu版本。

## 如何安装ubuntu

### 虚拟机

在虚拟机上安装 Ubuntu 是一个比较直接的过程，但它需要一些步骤来确保一切顺利进行。以下是安装 Ubuntu 到虚拟机的详细指南：

#### 1. 选择并安装虚拟机软件

- **选择虚拟机软件**：首先，你需要选择一个虚拟机软件。流行的选择包括 VMware Workstation（或 VMware Player）、Oracle VM VirtualBox，以及在Mac上的Parallels Desktop。
- **安装虚拟机软件**：下载并安装你选择的虚拟机软件。大多数虚拟机软件都有详细的安装指南。

#### 2. 下载 Ubuntu ISO 文件

- 访问 [Ubuntu官网](https://ubuntu.com/download/desktop) 并下载你选择的Ubuntu版本的ISO文件。

#### 3. 创建新的虚拟机

- 打开虚拟机软件并选择创建新的虚拟机。
- 通常会有一个向导帮助你完成设置。在被要求选择安装媒体时，选择你刚刚下载的 Ubuntu ISO 文件。
- 配置虚拟机的硬件设置，如分配给虚拟机的内存大小和硬盘空间。Ubuntu官方推荐至少4GB内存和25GB硬盘空间。

#### 4. 配置虚拟机设置

- 在虚拟机设置中，你可以调整各种选项，比如网络设置、显示设置和存储选项。
- 对于初学者来说，默认设置通常就足够了，但如果你有特定需求，如需要更多的处理器或显存，可以在这里进行调整。

#### 5. 启动虚拟机并安装 Ubuntu

- 启动你创建的虚拟机。它会从ISO文件引导，并启动Ubuntu安装程序。
- 跟随安装程序的指示进行。你将需要选择语言、键盘布局、时区，以及创建用户账户和密码。
- 安装程序会询问你是否要将Ubuntu安装在整个虚拟硬盘上。因为这是一个虚拟环境，通常可以安全地选择“是”。

#### 6. 完成安装并重启

- 一旦安装完成，安装程序通常会提示你重启虚拟机。
- 在重启过程中，确保ISO文件不再被虚拟机用作引导媒体。在大多数虚拟机软件中，它会自动取消挂载ISO文件，但如果没有，你可能需要手动操作。

#### 7. 启动 Ubuntu

- 重启后，虚拟机应该会引导进入新安装的 Ubuntu 系统。
- 使用你在安装过程中设置的用户名和密码登录。

#### 8. 安装虚拟机增强功能（可选）

- 大多数虚拟机软件都提供了增强功能，如更好的图形性能、无缝鼠标移动和共享文件夹。
- 这些通常需要在Ubuntu系统内安装额外的软件或驱动程序。例如，在VirtualBox中，这被称为"Guest Additions"。

#### 9. 更新和安装额外软件

- 启动Ubuntu后，可能会提示你安装更新。建议执行这些更新以确保系统安全和稳定。
- 你可以通过Ubuntu的软件中心安装所需的额外软件。

通过以上步骤，你可以在虚拟机中顺利安装并运行 Ubuntu，从而在不影响主操作系统的情况下体验和学习Linux。记得，虚拟机中的操作不会影响你的主操作系统，所以这是一个尝试新软件和设置的安全环境。

### 双系统

安装 Ubuntu 是一个相对直接的过程，但它确实需要一些准备工作和步骤的遵循。以下是一个详细的指南，解释如何安装 Ubuntu 操作系统：

#### 1. 准备工作

- **备份数据**：在安装新操作系统之前，确保备份任何重要的数据。
- **选择Ubuntu版本**：访问 [Ubuntu官网](https://ubuntu.com/)，选择适合你需求的Ubuntu版本。通常，你可以在“Desktop”（桌面版）和“Server”（服务器版）之间进行选择。
- **下载Ubuntu ISO文件**：下载你选择的Ubuntu版本的ISO镜像文件。

#### 2. 创建启动盘

- **准备USB驱动器**：准备一个至少4GB的USB闪存驱动器用于创建启动盘。
- **制作启动盘**：使用软件如Rufus（Windows）、Etcher（跨平台）或Ubuntu自带的启动盘创建器（如果你已经在使用Linux），将下载的ISO文件烧录到USB驱动器上。

#### 3. 准备安装

- **确保Internet连接**：尽管不是必需的，但建议在安装过程中连接到Internet。
- **连接电源**：如果你在笔记本电脑上安装Ubuntu，请连接电源适配器。
- **配置BIOS/UEFI**：重启电脑，并进入BIOS/UEFI设置。确保设置从USB驱动器启动。这通常涉及到改变启动顺序，让USB设备排在硬盘之前。

#### 4. 安装Ubuntu

- **从USB启动**：插入你的USB启动盘，重启电脑，从USB设备启动。
- **启动或安装Ubuntu**：你将被迎接以Ubuntu的启动界面，可以选择“试用Ubuntu”或“安装Ubuntu”。
- **选择安装类型**：如果你的电脑上有其他操作系统，你可以选择安装Ubuntu与之并存，或者替换现有操作系统。如果你选择并存安装，安装程序将引导你调整分区大小。
- **跟随安装向导**：选择语言、键盘布局、时区，以及输入其他必要信息，如用户名和密码。
- **分区**（如果需要）：高级用户可以手动分区硬盘。大多数用户可以选择自动分区，并让安装程序为你处理。
- **开始安装**：一旦完成上述步骤，点击“安装”开始安装过程。

#### 5. 完成安装

- **重启电脑**：安装完成后，系统会提示你移除安装介质（USB驱动器），然后重启电脑。
- **登录**：重启后，你将被引导到登录界面。使用你在安装过程中创建的用户名和密码登录。

#### 6. 安装更新和软件

- **更新系统**：登录后，可能会提示你安装更新。建议立即执行这些更新。
- **安装额外软件**：通过Ubuntu软件中心，你可以安装需要的额外软件。

#### 提示

- 在进行任何安装之前，确保阅读每个步骤的说明。
- 如果你不熟悉分区或有任何疑问，最好选择自动分区选项。
- 安装过程中，请确保不要断电或关闭电脑。

安装Ubuntu相对简单，但如果你是首次进行这类操作，可能需要一些时间来熟悉过程。如果你遇到任何问题，Ubuntu社区和各种在线论坛是获取帮助的好地方。

### 虚拟机和双系统各自的优缺点

在考虑安装Ubuntu时，你可以选择使用虚拟机或设置双系统。这两种方法各有其优缺点，取决于你的需求和设备的配置。

#### 使用虚拟机安装Ubuntu

**优点**:

1. **无风险**：在虚拟机中安装Ubuntu不会影响主操作系统，因此风险较低。
2. **易于设置和管理**：虚拟机软件通常很容易使用，可以方便地管理和配置虚拟机。
3. **并行运行系统**：可以同时运行Windows和Ubuntu，方便在两个系统之间切换。
4. **快速测试和实验**：虚拟机是测试新软件、开发环境或系统更新的理想选择，不会影响主系统。
5. **简单的备份和恢复**：可以轻松备份整个虚拟机，并在必要时恢复。

**缺点**:
1. **性能开销**：虚拟机会消耗额外的资源，如CPU和内存，可能导致性能下降，尤其是在资源有限的系统上。
2. **不适合高性能任务**：对于要求高性能的任务（如大型游戏、高端图形处理），虚拟机可能不是最佳选择。
3. **硬件直接访问限制**：虚拟机可能无法直接访问某些硬件组件，如GPU加速。

#### 设置双系统安装Ubuntu

**优点**:
1. **性能**：Ubuntu直接在硬件上运行，提供了更好的性能，尤其是对于CPU和内存密集型的应用。
2. **硬件访问**：可以直接访问所有硬件资源，包括GPU，适合高性能计算任务。
3. **真实体验**：提供了一个没有任何虚拟化层的真实Linux体验，这对于某些开发任务很重要。

**缺点**:
1. **风险较高**：安装过程中可能存在损坏现有操作系统的风险，尤其是在分区过程中。
2. **重启切换**：需要在Ubuntu和另一个操作系统之间重启以切换，这可能比较麻烦。
3. **磁盘空间分配**：需要在安装时分配硬盘空间给Ubuntu，稍后修改这些分区可能比较复杂。
4. **系统维护**：需要独立维护两个操作系统，包括更新和安全补丁。

#### 结论

选择哪种方法取决于你的具体需求：
- 如果你需要一个低风险、易于管理的环境，主要用于学习和测试目的，选择**虚拟机**。
- 如果你需要最佳性能，尤其是对于资源密集型的应用，或者想要一个更真实的Linux体验，选择**双系统安装**。

无论哪种方式，都确保你有适当的备份，并在开始之前了解所有必要的步骤和考虑。

## ubuntu文件系统

Ubuntu 的文件系统遵循 Linux 标准文件系统层次结构（FHS, Filesystem Hierarchy Standard），这是一种用于组织文件和目录的结构。理解这个层次结构对于有效地使用 Linux 非常重要。以下是 Ubuntu 文件系统的一些主要组成部分及其用途：

### 1. **/ (根)**
- 这是文件系统的最高层级。所有其他目录都从这里开始。

### 2. **/bin**
- 存放基本用户二进制文件。这些命令是系统启动和运行所必需的，例如 `ls`, `cp`, `mv`, `cat` 等。

### 3. **/boot**
- 包含启动 Linux 时所需的文件，比如 Linux 内核 (vmlinuz)，启动加载器（如 GRUB）配置文件，以及在系统启动过程中使用的其他文件。

### 4. **/dev**
- 包含设备文件。Linux 将硬件设备视为文件，例如 `/dev/sda`（第一个硬盘驱动器），`/dev/usb`（USB设备），`/dev/cdrom`（光驱）等。

### 5. **/etc**
- 存放系统配置文件。这里的文件通常由管理员编辑，用于配置系统和应用程序的设置。

### 6. **/home**
- 用户的个人目录位于此处。每个用户都有一个与其用户名相对应的目录，例如 `/home/username`。

### 7. **/lib**
- 包含系统和应用程序运行所需的库文件。这些文件类似于 Windows 中的 DLL 文件。

### 8. **/media**
- 用于挂载可移动媒体设备，如 CD-ROMs、USB驱动器和外置硬盘。

### 9. **/mnt**
- 传统上用于临时挂载文件系统，如网络文件系统（NFS）。

### 10. **/opt**
- 用于安装“可选”的软件。通常是由第三方开发的独立软件包。

### 11. **/proc**
- 一个虚拟文件系统，包含运行内核及其进程的信息。例如，`/proc/cpuinfo` 包含 CPU 信息。

### 12. **/root**
- 系统管理员（root 用户）的家目录。这不是 `/home/root`。

### 13. **/sbin**
- 存放系统二进制文件。这些文件通常由系统管理员用于系统管理，例如 `fdisk`, `ifconfig`, `init` 等。

### 14. **/srv**
- 包含一些服务特定的数据，如 Web 服务器和 FTP 服务器的数据。

### 15. **/tmp**
- 用于存储临时文件。系统重启时，这个目录下的文件通常会被删除。

### 16. **/usr**
- 包含用户程序和数据。它是 UNIX System Resources 的缩写，包含许多子目录，如 `/usr/bin`（用户二进制文件），`/usr/lib`（用户库文件），`/usr/local`（本地安装的软件和数据）等。

### 17. **/var**
- 包含变化的数据，如日志文件 `/var/log`，邮件 `/var/mail`，打印队列 `/var/spool`，以及其他动态文件。

Ubuntu 的文件系统设计旨在确保系统的有序、标准化和高效运行。理解这些目录及其用途对于管理 Ubuntu 系统非常重要。

## 终端常用指令

ls （list） 列出当前文件夹下的内容

ls -a 显示所有文件

mkdir （make directory）创建新的目录

cd (change directory) 改变当前目录

cd ..回到上一级目录

cd ~回到主文件夹（～=/home/用户名/ 当前用户中的主文件夹）

Tab键 自动补全

gedit

source

~/.bashrc = 终端程序启动脚本

sudo 以管理员权限执行本条指令

catkin_make

## ros安装

 

## .bashrc

把设置工作空间环境参数的source指令添加到终端程序初始化的脚本

## ros应用商店APT源

## ros工作空间

## ROS Index 和 Github



Ctrl+Shift+B 编译软件包

编译快捷键的默认操作

```
{
	"version": "2.0.0",
	"tasks": [
		{
			"type": "catkin_make",
			"args": [
				"--directory",
				"/home/lyb/catkin_ws",
				"-DCMAKE_BUILD_TYPE=RelWithDebInfo"
			],
			"problemMatcher": [
				"$catkin-gcc"
			],
			"group": {"kind":"build","isDefault": true},
			"label": "catkin_make: build"
		}
	]
}
```

Ctrl+Shift+P 弹出设置搜索栏 error squiggles 禁用错误波形曲线