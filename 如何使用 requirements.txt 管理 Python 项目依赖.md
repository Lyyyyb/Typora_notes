# 如何使用 `requirements.txt` 管理 Python 项目依赖

在 Python 项目开发中，确保环境的一致性和依赖的正确管理对于项目的稳定性和可移植性至关重要。`requirements.txt` 文件是一个文本文件，用于详细列出项目所需的所有 Python 库及其对应的版本号。通过这种方式，可以确保项目在不同开发、测试和生产环境中的一致运行。以下是使用 `requirements.txt` 管理 Python 项目依赖的详细步骤及其逻辑清晰的解释：

### 创建 `requirements.txt` 文件

1. **确定依赖**：首先，开发者需要确定项目运行所需的所有外部库。这包括直接使用的库以及这些库可能依赖的其他库。

2. **指定版本**：为了避免未来库更新可能引入的兼容性问题，应明确指定库的版本号。可以通过 `pip freeze` 命令查看当前环境中所有库的版本，并选择性地添加到 `requirements.txt` 文件中。

### `requirements.txt` 文件的结构

文件中每一行代表一个依赖项，格式通常为 `库名==版本号`。例如：

```
Flask==1.1.2
requests==2.24.0
numpy==1.18.5
```

这种格式确保了无论在何种环境中部署，都能获得相同版本的依赖库。

### 使用 `requirements.txt` 安装依赖

1. **安装依赖**：使用 pip 安装 `requirements.txt` 中列出的依赖，命令如下：

   ```bash
   pip install -r requirements.txt
   ```

   这条命令告诉 pip 按照文件中指定的版本安装每个库，从而创建一个与 `requirements.txt` 文件匹配的 Python 环境。

### 示例：开发一个简单的 Web 应用

假设您正在开发一个使用 Flask 和 Requests 库的简单 Web 应用。项目目录中的 `requirements.txt` 可能如下所示：

```
Flask==1.1.2
requests==2.24.0
```

在这个示例中，您的应用依赖于 Flask 作为 web 框架和 Requests 库来处理 HTTP 请求。通过在开发环境中运行 `pip install -r requirements.txt`，您可以确保所有开发者和部署环境都安装了正确版本的依赖。

### 维护 `requirements.txt` 文件

随着项目的发展和库版本的更新，可能需要更新 `requirements.txt` 文件。这应该通过仔细测试新版本的库在项目中的兼容性后进行，确保更新不会破坏现有功能。

通过这些步骤，`requirements.txt` 提供了一个系统化和可靠的方法来管理和维护 Python 项目的外部依赖，从而降低了环境差异带来的风险，提高了项目的可移植性和可维护性。