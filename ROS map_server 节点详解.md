# ROS `map_server` 节点详解

`map_server` 是 ROS（机器人操作系统）中的一个关键节点，负责提供地图数据作为 ROS 服务，并通过命令行工具 `map_saver` 动态保存生成的地图文件。本文将详细介绍 `map_server` 的功能、地图格式、命令行工具及其使用方法。

## 目录

1. [地图格式](#地图格式)
    - [图像格式](#图像格式)
    - [YAML 格式](#yaml-格式)
    - [值的解释](#值的解释)
        - [三值法](#三值法)
        - [比例法](#比例法)
        - [原始法](#原始法)
2. [命令行工具](#命令行工具)
    - [`map_server`](#map_server)
        - [用法](#用法)
        - [示例](#示例)
        - [发布的主题](#发布的主题)
        - [服务](#服务)
        - [参数](#参数)
    - [`map_saver`](#map_saver)
        - [用法](#用法-1)
        - [示例](#示例-1)
        - [订阅的主题](#订阅的主题)

## 地图格式

`map_server` 操作的地图由一对文件组成。YAML 文件描述了地图的元数据，并指定图像文件的名称。图像文件则编码了占据数据。

### 图像格式

图像文件通过每个像素的颜色描述世界中每个单元格的占据状态。在标准配置下，较白的像素表示自由区域，较黑的像素表示被占据区域，介于两者之间的像素表示未知区域。支持彩色图像，但颜色值会被平均为灰度值。

图像数据通过 SDL_Image 读取；支持的格式取决于特定平台上 SDL_Image 提供的功能。一般来说，大多数流行的图像格式都被广泛支持，但需要注意的是，PNG 格式在 OS X 上不被支持。

### YAML 格式

YAML 格式通过一个简单完整的示例来说明：

```yaml
image: testmap.png
resolution: 0.1
origin: [0.0, 0.0, 0.0]
occupied_thresh: 0.65
free_thresh: 0.196
negate: 0
```

#### 必填字段

- **image**：包含占据数据的图像文件路径，可以是绝对路径，也可以是相对于 YAML 文件位置的相对路径。
- **resolution**：地图的分辨率，单位为米/像素。
- **origin**：地图左下角像素的二维姿态，格式为 (x, y, yaw)，其中 yaw 为逆时针旋转角度（yaw=0 表示无旋转）。系统的许多部分目前忽略 yaw。
- **occupied_thresh**：占据概率大于此阈值的像素被认为是完全占据。
- **free_thresh**：占据概率小于此阈值的像素被认为是完全自由。
- **negate**：是否应反转白色/黑色的自由/占据语义（阈值的解释不受影响）。

#### 可选参数

- **mode**：可以取三种值之一：`trinary`、`scale` 或 `raw`。默认值为 `trinary`。关于不同模式下值的解释将在下一节详细说明。

### 值的解释

对于颜色值在 [0, 256) 范围内的每个像素，将其转换为 ROS 消息中的浮点数 `p`，具体取决于 YAML 中 `negate` 标志的设置。

- **如果 `negate` 为 `false`**，则 `p = (255 - x) / 255.0`。这意味着黑色（0）现在具有最高值（1.0），白色（255）具有最低值（0.0）。
- **如果 `negate` 为 `true`**，则 `p = x / 255.0`。这是图像的非标准解释，因此称为 `negate`，尽管数学上并未对 `x` 进行反转。

#### 三值法（Trinary）

标准解释方法，将所有值解释为三个值之一：

- **如果 `p > occupied_thresh`**，输出值 `100`，表示该单元格被占据。
- **如果 `p < free_thresh`**，输出值 `0`，表示该单元格是自由的。
- **否则**，输出值 `-1`（即无符号字符的 `255`），表示该单元格状态未知。

#### 比例法（Scale）

调整上述解释方法，以允许输出比三值更多的值：

- **如果 `p > occupied_thresh`**，输出值 `100`。
- **如果 `p < free_thresh`**，输出值 `0`。
- **否则**，输出值 `99 * (p - free_thresh) / (occupied_thresh - free_thresh)`。

这允许输出范围从 `[0, 100]` 的全梯度值。要输出 `-1`，只需使用 PNG 的 alpha 通道，任何透明度将被解释为未知。

#### 原始法（Raw）

此模式下，每个像素直接输出其原始值 `x`，即输出值范围为 `[0, 255]`。

## 命令行工具

### `map_server`

`map_server` 是一个 ROS 节点，读取磁盘上的地图并通过 ROS 服务提供地图数据。

当前 `map_server` 的实现将地图图像数据中的颜色值转换为三值占据值：自由（0）、占据（100）和未知（-1）。未来版本可能会利用 0 到 100 之间的值来传达更细微的占据程度。

#### 用法

```bash
map_server <map.yaml>
```

#### 示例

```bash
rosrun map_server map_server mymap.yaml
```

注意，地图数据可以通过 latched 主题（意味着每个新订阅者会收到一次数据）或通过服务获取。服务可能会在未来被淘汰。

#### 发布的主题

- **map_metadata (nav_msgs/MapMetaData)**
  
  通过此 latched 主题接收地图的元数据。

- **map (nav_msgs/OccupancyGrid)**
  
  通过此 latched 主题接收地图数据。

#### 服务

- **static_map (nav_msgs/GetMap)**
  
  通过此服务检索地图数据。

#### 参数

- **~frame_id** (string, 默认值: "map")
  
  设置发布地图头部的参考框架。

### `map_saver`

`map_saver` 是一个命令行工具，用于将地图数据保存到磁盘，例如从 SLAM（同步定位与地图构建）映射服务中获取的地图。

#### 用法

```bash
rosrun map_server map_saver [--occ <threshold_occupied>] [--free <threshold_free>] [-f <mapname>] map:=/your/costmap/topic
```

`map_saver` 从指定的主题获取地图数据，并将其写入 `map.pgm` 和 `map.yaml` 文件。使用 `-f` 选项可以为输出文件提供不同的基础名称。`--occ` 和 `--free` 选项接受 0 到 100 之间的值。要保存不同的地图主题，请将 `map` 设置为您的 costmap 主题。

#### 示例

```bash
rosrun map_server map_saver -f mymap
```

```bash
rosrun map_server map_saver --occ 90 --free 10 -f mymap map:=/move_base/global_costmap/costmap
```

#### 订阅的主题

- **map (nav_msgs/OccupancyGrid)**
  
  通过此 latched 主题检索地图数据。

## 总结

`map_server` 提供了强大的工具集，用于在 ROS 中管理和处理地图数据。通过灵活的地图格式支持和命令行工具，用户可以轻松地加载、发布和保存地图，支持机器人导航和路径规划等关键功能。理解其地图格式、值的解释方式以及命令行工具的使用方法，对于有效利用 `map_server` 至关重要。