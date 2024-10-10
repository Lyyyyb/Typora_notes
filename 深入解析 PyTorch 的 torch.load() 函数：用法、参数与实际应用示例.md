深入解析 PyTorch 的 torch.load() 函数：用法、参数与实际应用示例

函数 `torch.load()` 是一个在PyTorch中用于加载通过 `torch.save()` 保存的序列化对象的核心功能。这个函数广泛应用于加载预训练模型、模型的状态字典（state dictionaries）、优化器状态以及其他PyTorch对象。它利用Python的反序列化能力，特别地对张量的底层存储（storages）进行了特殊处理，以支持跨设备加载和内存效率。

### 基本语法和参数详解

```python
torch.load(f, map_location=None, pickle_module=pickle, *, weights_only=False, mmap=None, **pickle_load_args)
```

#### 参数详细说明

- **`f` (Union[str, PathLike, BinaryIO, IO[bytes]])**：
  - **类型**：可以是字符串、路径对象或文件对象。
  - **含义**：指定要加载的文件的路径或文件对象。如果是文件对象，它必须实现基本的文件读取方法，如 `read()` 和 `seek()`。

- **`map_location` (Optional[Union[Callable[[Storage, str], Storage], torch.device, str, Dict[str, str]])**：
  - **类型**：可选，可以是函数、设备对象、字符串或字典。
  - **含义**：用于指定存储设备的重新映射策略。
    - **函数**：如果提供了函数，它应该接受存储和位置标签作为参数，并返回新的存储位置。
    - **设备或字符串**：可以直接指定所有张量应该被加载到的设备，如 `'cpu'` 或 `'cuda:0'`。
    - **字典**：将文件中的位置标签映射到新的存储位置。

- **`pickle_module` (Optional[Any])**：
  - **类型**：模块。
  - **含义**：用于反序列化的模块，默认为Python的 `pickle` 模块。如果序列化时使用了特定的模块，则加载时也必须使用相同的模块。

- **`weights_only` (Optional[bool])**：
  - **类型**：布尔值。
  - **含义**：如果设置为 `True`，则加载过程将限制为仅加载张量、基本数据类型、字典和通过 `torch.serialization.add_safe_globals()` 添加的安全类型。

- **`mmap` (Optional[bool])**：
  - **类型**：布尔值。
  - **含义**：如果设置为 `True`，则文件将通过内存映射的方式访问，而不是完全加载到内存中。这对处理大型数据文件特别有用，因为它减少了内存使用并可能提高访问速度。

- **`pickle_load_args` (Any)**：
  - **类型**：关键字参数。
  - **含义**：传递给 `pickle_module.load()` 和 `pickle_module.Unpickler()` 的附加参数，例如 `encoding`。

### 实际使用示例

#### 示例 1: 基础加载模型

加载一个在GPU上训练并保存的模型到CPU上进行推理：

```python
import torch

# 设置加载路径
model_path = 'gpu_trained_model.pth'

# 加载模型到CPU
model = torch.load(model_path, map_location='cpu')

# 打印模型结构确认加载无误
print(model)
```

#### 示例 2: 使用内存映射和仅加载权重

对于大型模型文件，使用内存映射加载权重，减少内存占用：

```python
import torch

# 模型文件路径
large_model_path = 'large_model_weights.pth'

# 使用内存映射方式加载模型权重到CPU，限制为仅加载权重
model_weights = torch.load(large_model_path, map_location='cpu', mmap=True, weights_only=True)

# 假设 MyModel 是模型的架构类
model = MyModel()
model.load_state_dict(model_weights)

# 输出模型确保权重被正确加载
print(model)
```

这些示例清楚地展示了如何灵活使用 `torch.load()` 的不同参数来优化模型的加载策略，适应不同的硬件环境和内存限制，从而实现高效的模型部署。