# Ubuntu20.04 由源码编译安装opencv3.2.0

## 获取 opencv 及opencv_contrib源代码

### 创建目录以存放opencv及opencv_contrib源代码

```bash
mkdir ~/opencv3.2.0
cd ~/opencv3.2.0
```

### 获取opencv源代码并切换到对应tag

```bash
git clone https://github.com/opencv/opencv.git
cd opencv
git checkout 3.2.0
cd ..
```

### 获取opencv_contrib源代码并切换到对应tag

```bash
git clone https://github.com/opencv/opencv_contrib.git
cd opencv_contrib
git checkout 3.2.0
cd ..
```

## 编译安装

### 创建构建目录

```bash
cd ~/opencv3.2.0/opencv
mkdir build && cd build
```

### 配置 CMake

```bash
cmake -D CMAKE_BUILD_TYPE=Release \
      -D CMAKE_INSTALL_PREFIX=/usr/local/opencv3.2.0 \
      -D OPENCV_EXTRA_MODULES_PATH=~/opencv_builds/opencv_contrib-3.2.0/modules \
      -D ENABLE_PRECOMPILED_HEADERS=OFF .. #不添加这一句编译时会出现/usr/include/c++/9/cstdlib:75:15: fatal error: stdlib.h: 没有那个文件或目录的报错信息
```

**参数说明：**

1. **CMAKE_BUILD_TYPE=Release**
   - 这个参数指定构建类型。`Release` 意味着编译器将优化生成的代码，去除调试信息，使得最终的程序运行得更快。这是生产环境中推荐的设置，相对于 `Debug`，`Release` 编译的程序执行效率更高，但不利于调试。
2. **CMAKE_INSTALL_PREFIX=/usr/local/opencv3.2.0**
   - 这个参数指定安装路径。`/usr/local/opencv3.2.0` 表示 OpenCV 将被安装在这个目录下。通常默认安装在 `/usr/local`，但指定一个包含版本号的子目录可以帮助管理不同版本的 OpenCV。
3. **OPENCV_EXTRA_MODULES_PATH=~/opencv_builds/opencv_contrib-3.2.0/modules**
   - 此参数指定额外模块的路径，这里是 OpenCV 的贡献模块（contrib modules）。这些模块不包含在主 OpenCV 仓库中，提供了额外的功能和实验性算法。通过提供这个路径，你可以在构建 OpenCV 时包含这些额外的模块。
4. **ENABLE_PRECOMPILED_HEADERS=OFF**
   - 这个选项用于控制是否使用预编译头文件。预编译头文件可以加快编译过程，但在某些情况下可能会引起问题，比如在不同的构建环境间迁移代码时。设置为 `OFF` 可以提高代码的可移植性，尤其是在有多个不同编译环境或者不同的编译器版本时。
5. **..**
   - 这表示 `cmake` 将在当前目录的上一级目录中查找 `CMakeLists.txt` 文件。这是启动构建过程的配置脚本，包含了所有必要的构建信息。

### 编译并安装

```bash
make -j$(nproc)
sudo make install
```

- 安装成功，如下：安装路径为 CMake配置时设置的目录

![image-20241023140108598](/home/lyb/github/Typora_notes/image-20241023140108598.png)

## 编译报错解决

### /usr/include/c++/9/cstdlib:75:15: fatal error: stdlib.h: 没有那个文件或目录

![image-20241023124115139](/home/lyb/github/Typora_notes/image-20241023124115139.png)

#### 报错原因

这个编译错误通常表明在你的系统中找不到标准库文件 `stdlib.h`。这种情况可能是因为系统缺少某些必要的开发包或者环境变量配置不当，导致编译器无法定位到这些标准头文件。此外，由于你使用的是较新的 Ubuntu 版本（20.04）和可能较新的编译器，而 OpenCV 3.2.0 是一个较旧的版本，它可能不完全兼容最新的编译器或其默认配置，这也可能是导致找不到头文件的原因之一。

#### 解决方法

- 禁用预编译头文件

- 在配置CMake的时候添加如下指令：`-D ENABLE_PRECOMPILED_HEADERS=OFF`
- 例如：

```bash
cmake -D CMAKE_BUILD_TYPE=Release \
      -D CMAKE_INSTALL_PREFIX=/usr/local/opencv3.2.0 \
      -D OPENCV_EXTRA_MODULES_PATH=~/opencv_builds/opencv_contrib-3.2.0/modules \
      -D ENABLE_PRECOMPILED_HEADERS=OFF .. #不添加这一句编译时会出现/usr/include/c++/9/cstdlib:75:15: fatal error: stdlib.h: 没有那个文件或目录的报错信息
```

### OpenCV 代码中使用的 FFmpeg 库宏定义与最新版 FFmpeg 库中的定义不一致

![image-20241023124944917](/home/lyb/github/Typora_notes/image-20241023124944917.png)

#### 报错原因

在安装并编译 OpenCV 3.2.0 版本时，出现的编译错误主要是由于 OpenCV 代码中使用的 FFmpeg 库宏定义与最新版 FFmpeg 库中的定义不一致所致。具体来说，原有的宏 `CODEC_FLAG_GLOBAL_HEADER` 和 `AVFMT_RAWPICTURE` 在较新的 FFmpeg 版本中已被重命名或废弃，从而导致编译失败。这些错误表明存在 API 不兼容的问题，即 OpenCV 的旧代码与新版本 FFmpeg 的接口不匹配。

1. **`CODEC_FLAG_GLOBAL_HEADER` 未声明**:
   - 在旧版本的 FFmpeg 中，`CODEC_FLAG_GLOBAL_HEADER` 用于设置编码器标志，指示编码器在输出文件中生成全局头部信息，而不是每个输出帧中都包含头部信息。
   - 新版本的 FFmpeg 将此宏重命名为 `AV_CODEC_FLAG_GLOBAL_HEADER`。由于 OpenCV 源码没有跟进这一变更，因此在编译时未找到原有宏名，导致编译错误。
2. **`AVFMT_RAWPICTURE` 未声明**:
   - `AVFMT_RAWPICTURE` 在旧版 FFmpeg 中用于标识输出格式（封装格式），允许直接将未压缩的视频帧写入媒体文件，这在新版本中已被废弃或功能已改变。
   - 由于 FFmpeg 更新后移除了此宏，当 OpenCV 代码尝试使用它时，同样导致了编译错误。

#### 解决方法

1. **打开源代码文件**：

   - 找到并打开 `cap_ffmpeg_impl.hpp` 文件。
   - 路径如下：
   - `/home/lyb/opencv3.2.0/opencv/modules/videoio/src/cap_ffmpeg_impl.hpp`

2. **添加宏定义**：

   - 在文件最开始的部分（通常在包含头文件之后）添加以下宏定义：

     ```cpp
     #define AV_CODEC_FLAG_GLOBAL_HEADER (1 << 22)
     #define CODEC_FLAG_GLOBAL_HEADER AV_CODEC_FLAG_GLOBAL_HEADER
     #define AVFMT_RAWPICTURE 0x0020
     ```

   - 这些定义将重新引入旧宏，以确保代码兼容当前的 FFmpeg 版本。

### C++ 和 Python 接口间的类型不匹配

![image-20241023130150271](/home/lyb/github/Typora_notes/image-20241023130150271.png)

#### 报错原因

当在 Ubuntu 20.04 上从源代码编译安装 OpenCV 3.2 时，遇到的编译错误表明 `cv2.cpp` 文件在编译过程中出现问题。特别是，错误提示集中在 `cv2.cpp.o` 的生成上，这是 Python 接口文件的编译输出对象。此错误可能是由于 C++ 和 Python 接口间的类型不匹配或者 API 已经更新而代码未跟进引起的。

- `make[2]: *** [modules/python3/CMakeFiles/opencv_python3.dir/build.make:329: modules/python3/CMakeFiles/opencv_python3.dir/__/src2/cv2.cpp.o] Error 1`
  - 这一行表示在执行编译命令时，处理 `cv2.cpp` 文件时遇到错误，导致无法成功生成对象文件 `.o`。
- `make[1]: *** [CMakeFiles/Makefile2:11856: modules/python3/CMakeFiles/opencv_python3.dir/all] Error 2`
  - 此行表明编译整个 Python 模块时遇到问题，且因为依赖的部分编译失败，整个模块编译无法继续。
- `make: *** [Makefile:163: all] Error 2`
  - 最顶层的 Makefile 在尝试完成所有编译任务时遇到了阻碍，整个构建进程因此终止。

#### 解决方法

- 在cv2.cpp中找到对应代码行（/home/lyb/opencv3.2.0/opencv/modules/python/src2），如下：

```C++
char* str = PyString_AsString(obj);
```

- 将其修改为：

```C++
char* str = (char *)PyString_AsString(obj);
```

## 系统环境变量配置

### 编辑.bashrc文件

```bash
sudo vim .bashrc
```

### 环境变量设置

```bash
export PATH=/usr/local/opencv3.2.0/bin:$PATH
export LD_LIBRARY_PATH=/usr/local/opencv3.2.0/lib:$LD_LIBRARY_PATH
export PKG_CONFIG_PATH=/usr/local/opencv3.2.0/lib/pkgconfig #opencv4默认不启用pkgconfig，需要在CMake配置时开启
```

#### 参数含义解释：

1. **`export PATH=/usr/local/opencv3.2.0/bin:$PATH`**:
   - `PATH` 是一个环境变量，用于定义操作系统搜索可执行文件的目录列表。当你在命令行中输入一个命令时，系统会按照 `PATH` 环境变量定义的顺序来搜索这些目录，以找到可执行文件。
   - 此命令将 OpenCV 的 `bin` 目录 `/usr/local/opencv3.2.0/bin` 添加到现有的 `PATH` 变量之前。这意味着系统会首先在此目录中查找可执行文件，这对于使用 OpenCV 提供的各种工具和示例程序非常重要。
   - 通过将 OpenCV 的 `bin` 目录添加到 `PATH`，你可以从任何位置启动终端并直接运行 OpenCV 的可执行文件，而无需输入完整的路径。

2. **`export LD_LIBRARY_PATH=/usr/local/opencv3.2.0/lib:$LD_LIBRARY_PATH`**:
   - `LD_LIBRARY_PATH` 是一个环境变量，用于定义动态链接器搜索共享库时的目录列表。
   - 此命令将 OpenCV 的 `lib` 目录 `/usr/local/opencv3.2.0/lib` 添加到现有的 `LD_LIBRARY_PATH` 环境变量之前。这确保了在运行依赖 OpenCV 库的程序时，系统能够在这个目录中优先查找所需的动态库（.so 文件）。
   - 设置 `LD_LIBRARY_PATH` 对于确保程序能找到正确版本的动态库非常重要，尤其是在系统中存在多个版本的库时。

3. **`export PKG_CONFIG_PATH=/usr/local/opencv3.2.0/lib/pkgconfig`**:
   - `PKG_CONFIG_PATH` 是一个环境变量，用于指定 `pkg-config` 工具搜索其 `.pc` 文件的路径。`.pc` 文件包含了使用库（如编译器和链接器标志）所需的元数据。
   - 通过设置 `PKG_CONFIG_PATH` 环境变量为 `/usr/local/opencv3.2.0/lib/pkgconfig`，你告诉 `pkg-config` 工具在编译依赖 OpenCV 的应用程序时，首先在此目录中查找 OpenCV 的配置文件。
   - 这对于开发和编译依赖 OpenCV 的第三方应用程序非常重要，因为它简化了编译和链接过程，确保了使用正确的编译器标志和链接选项。

### 刷新环境变量

```bash
source ~/.bashrc
```

### 验证是否设置成功

```bash
pkg-config --modversion opencv
```

- 应输出3.2.0

![image-20241023140810480](/home/lyb/github/Typora_notes/image-20241023140810480.png)

- 自此 ubuntu20.04 opencv3.2安装成功