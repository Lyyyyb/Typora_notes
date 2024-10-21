# 解决 OpenCV 与 FFmpeg 版本不兼容导致的编译错误

在安装并编译 OpenCV 3.2.0 版本时，出现的编译错误主要是由于 OpenCV 代码中使用的 FFmpeg 库宏定义与最新版 FFmpeg 库中的定义不一致所致。具体来说，原有的宏 `CODEC_FLAG_GLOBAL_HEADER` 和 `AVFMT_RAWPICTURE` 在较新的 FFmpeg 版本中已被重命名或废弃，从而导致编译失败。这些错误表明存在 API 不兼容的问题，即 OpenCV 的旧代码与新版本 FFmpeg 的接口不匹配。

### 错误原因分析：

1. **`CODEC_FLAG_GLOBAL_HEADER` 未声明**:
   - 在旧版本的 FFmpeg 中，`CODEC_FLAG_GLOBAL_HEADER` 用于设置编码器标志，指示编码器在输出文件中生成全局头部信息，而不是每个输出帧中都包含头部信息。
   - 新版本的 FFmpeg 将此宏重命名为 `AV_CODEC_FLAG_GLOBAL_HEADER`。由于 OpenCV 源码没有跟进这一变更，因此在编译时未找到原有宏名，导致编译错误。

2. **`AVFMT_RAWPICTURE` 未声明**:
   - `AVFMT_RAWPICTURE` 在旧版 FFmpeg 中用于标识输出格式（封装格式），允许直接将未压缩的视频帧写入媒体文件，这在新版本中已被废弃或功能已改变。
   - 由于 FFmpeg 更新后移除了此宏，当 OpenCV 代码尝试使用它时，同样导致了编译错误。

### 解决方案：

为了解决这些问题，可以通过添加宏定义到 OpenCV 的源码中，从而桥接旧代码与新版 FFmpeg 库之间的差异。具体操作步骤如下：

1. **打开源代码文件**：
   - 找到并打开 `cap_ffmpeg_impl.hpp` 文件。

2. **添加宏定义**：
   - 在文件最开始的部分（通常在包含头文件之后）添加以下宏定义：
     ```cpp
     #define AV_CODEC_FLAG_GLOBAL_HEADER (1 << 22)
     #define CODEC_FLAG_GLOBAL_HEADER AV_CODEC_FLAG_GLOBAL_HEADER
     #define AVFMT_RAWPICTURE 0x0020
     ```
   - 这些定义将重新引入旧宏，以确保代码兼容当前的 FFmpeg 版本。

3. **保存并重新编译 OpenCV**：
   - 保存修改后的文件。
   - 返回到 OpenCV 的编译目录并执行编译命令（例如使用 `make`）。
   - 确保编译过程中无错误发生。

### 长期解决方案：

虽然以上步骤提供了一个快速修复的方法，但更稳定和可持续的解决方案是更新 OpenCV 的代码，使其与最新的 FFmpeg 库完全兼容。这可能涉及到更广泛的代码审核和修改，以及可能的 API 调整。另外，也可以考虑升级到最新的 OpenCV 版本，这通常会包含对最新外部库版本的支持。

通过上述方法，不仅可以解决当前的编译问题，还可以为未来的开发和维护奠定基础，确保与外部依赖库的兼容性和前瞻性。