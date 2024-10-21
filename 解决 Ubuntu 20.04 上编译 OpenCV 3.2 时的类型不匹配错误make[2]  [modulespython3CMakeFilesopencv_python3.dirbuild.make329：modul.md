# 解决 Ubuntu 20.04 上编译 OpenCV 3.2 时的类型不匹配错误

```bash
make[2]: *** [modules/python3/CMakeFiles/opencv_python3.dir/build.make:329：modules/python3/CMakeFiles/opencv_python3.dir/__/src2/cv2.cpp.o] 错误 1
make[1]: *** [CMakeFiles/Makefile2:11856：modules/python3/CMakeFiles/opencv_python3.dir/all] 错误 2
make: *** [Makefile:163：all] 错误 2
```

### 分析编译错误原因：

当在 Ubuntu 20.04 上从源代码编译安装 OpenCV 3.2 时，遇到的编译错误表明 `cv2.cpp` 文件在编译过程中出现问题。特别是，错误提示集中在 `cv2.cpp.o` 的生成上，这是 Python 接口文件的编译输出对象。此错误可能是由于 C++ 和 Python 接口间的类型不匹配或者 API 已经更新而代码未跟进引起的。

### 具体错误分析：
- `make[2]: *** [modules/python3/CMakeFiles/opencv_python3.dir/build.make:329: modules/python3/CMakeFiles/opencv_python3.dir/__/src2/cv2.cpp.o] Error 1`
   - 这一行表示在执行编译命令时，处理 `cv2.cpp` 文件时遇到错误，导致无法成功生成对象文件 `.o`。
- `make[1]: *** [CMakeFiles/Makefile2:11856: modules/python3/CMakeFiles/opencv_python3.dir/all] Error 2`
   - 此行表明编译整个 Python 模块时遇到问题，且因为依赖的部分编译失败，整个模块编译无法继续。
- `make: *** [Makefile:163: all] Error 2`
   - 最顶层的 Makefile 在尝试完成所有编译任务时遇到了阻碍，整个构建进程因此终止。

### 解决方案与实施步骤：

#### 1. 诊断错误具体原因：
   - 在编译时添加 `VERBOSE=1` 参数，重新运行 `make` 命令以获取更详细的错误输出：
     ```bash
     make VERBOSE=1
     ```
   - 仔细检查编译错误指向的代码行以及周围的上下文。

#### 2. 应用代码修改：
   - 仿照在树莓派上成功修改 OpenCV 源代码的案例，考虑对 `cv2.cpp` 的相似行进行类型转换处理。找到对应的代码行（在错误输出中应有指示或者近似行数）：
     ```cpp
     char* str = (char *)PyString_AsString(obj); // 添加 (char *) 转换
     ```
   - 此修改强制将 `PyString_AsString` 的返回值转换为 `char*` 类型，这可能是必要的，特别是如果 Python API 的返回类型与期待的 C++ 类型不一致时。

#### 3. 修改并重新编译 OpenCV：
   - 修改后保存 `cv2.cpp` 文件。
   - 清理之前的编译结果并重新编译：
     ```bash
     make clean
     cmake .
     make -j$(nproc)
     ```
   - 这里使用 `-j$(nproc)` 以利用所有可用的处理器核心来加速编译过程。

#### 4. 安装和验证：
   - 如果编译无误，运行安装命令：
     ```bash
     sudo make install
     ```
   - 安装后，检验 OpenCV 是否能在 Python 中正确加载：
     ```python
     python -c "import cv2; print(cv2.__version__)"
     ```

### 解释为何此解决方案有效：

此解决方案通过对可能出现类型不匹配的代码行添加显式类型转换，直接解决了 C++ 和 Python API 之间的兼容性问题。这种类型转换确保了编译器可以正确理解和处理 Python 返回的数据类型，使得代码能够顺利编译并生成正确的二进制文件。这种做法虽然简单，但针对具体的编译错误非常有效，尤其是在跨语言接口中常见的类型不匹配问题上。