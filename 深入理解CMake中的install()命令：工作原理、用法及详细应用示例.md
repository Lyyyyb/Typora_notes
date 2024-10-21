# 深入理解CMake中的install()命令：工作原理、用法及详细应用示例

在CMake中，`install()` 函数是用来指定如何将编译生成的目标（如可执行文件、库）、文件和目录等安装到预定义的位置。这个功能对于确保软件包的正确部署至关重要，尤其在将软件从开发环境迁移到生产环境时。`install()` 通过详细的参数允许开发者精确控制每个组件的安装过程，包括目标的安装路径、所需权限和针对特定构建配置的安装。

### 工作原理

`install()` 命令在CMake的构建脚本中定义安装规则，但这些规则仅在执行安装步骤（如通过运行 `make install` 或在Visual Studio中使用INSTALL项目）时应用。它可以根据不同的参数安装不同类型的内容，如TARGETS（目标文件）、FILES（文件）、PROGRAMS（程序）、LIBRARIES（库文件）、DIRECTORY（目录）等。

### 安装规则参数

- **DESTINATION**：指定安装位置的路径。
- **PERMISSIONS**：设置安装文件或目录的权限。
- **CONFIGURATIONS**：定义哪些构建类型（如Debug或Release）应应用此安装规则。
- **COMPONENT**：指定安装组件，用于在安装时进行细分。
- **OPTIONAL**：标记文件或目标为可选，如果不存在也不会导致安装失败。
- **RENAME**：安装时重命名文件。
- **PATTERN** / **REGEX**：指定只安装符合某种模式或正则表达式的文件。

### 实际示例

#### 示例 1：安装可执行文件和库

```cmake
add_executable(myApp src/main.cpp)
add_library(myLib SHARED src/myLib.cpp)

install(TARGETS myApp myLib
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib/static)
```

- `myApp` 是一个可执行文件，安装到 `bin` 目录。
- `myLib` 是一个共享库，其运行时部分安装到 `lib` 目录，而归档文件（如静态库）则安装到 `lib/static` 目录。

#### 示例 2：安装头文件

```cmake
install(DIRECTORY include/
    DESTINATION include
    FILES_MATCHING PATTERN "*.h")
```

- 这将 `include` 目录中的所有头文件（扩展名为 `.h`）安装到安装前缀下的 `include` 目录。

#### 示例 3：安装配置文件和脚本

```cmake
install(FILES settings.conf
    DESTINATION etc)
install(PROGRAMS scripts/start.sh
    DESTINATION bin)
```

- 将配置文件 `settings.conf` 安装到 `etc` 目录。
- 将脚本 `start.sh` 安装到 `bin` 目录，并确保其在安装后具有执行权限。

#### 示例 4：为特定构建配置安装

```cmake
install(TARGETS myApp
    RUNTIME DESTINATION bin
    CONFIGURATIONS Release)
```

- 仅在构建配置为Release时，将 `myApp` 安装到 `bin` 目录。

### 总结

`install()` 命令是CMake中非常强大的一个特性，使得软件的部署变得规范化和自动化。通过使用这一命令，开发者可以详细定义软件在系统中的安装布局，保证软件组件能被正确地部署到预期的位置，同时也支持对安装过程的细粒度控制，如仅在特定的构建配置下进行安装。这样的机制不仅优化了软件的安装过程，还有助于软件的标准化分发和维护。