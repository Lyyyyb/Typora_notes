# 解决 PyTorch 模型加载中的类定义缺失问题，抛出类似于 `AttributeError: Can't get attribute 'My_Network' on <module '__main__'...`

在尝试加载一个完整的 PyTorch 模型时，如遇到无法反序列化并重建模型对象的问题，通常是因为当前运行环境缺少必要的模型类定义。以下是对这一问题的详细、严谨且逻辑清晰的分析：

### 序列化与反序列化的工作原理

**序列化**过程中，`torch.save` 使用 Python 的 `pickle` 库将模型对象（包括其结构和参数）转换成一个二进制格式的文件。这不仅涉及模型的参数（如权重和偏差）的存储，也涉及对模型类本身的引用，包括类的名称和所在的模块路径。

**反序列化**过程中，`torch.load` 调用 `pickle` 来从保存的二进制文件中读取数据，并尝试重建原始的 Python 对象。此过程要求 `pickle` 能够在当前环境中找到并访问到保存时所用的类定义，以便正确实例化该模型。

### 问题的具体表现

当环境中缺失对应的模型类 `My_Network` 的定义时，`pickle` 在尝试实例化模型时会因为找不到相应的类定义而失败，通常会抛出一个 `AttributeError`。该错误指出 `My_Network` 类在当前模块或其它被导入的模块中未被定义。

### 详细示例与解决方案

假设在一个 Python 文件 `model_def.py` 中定义了 `My_Network` 类：

```python
# model_def.py
import torch.nn as nn

class My_Network(nn.Module):
    def __init__(self):
        super(My_Network, self).__init__()
        # 模型结构定义
        self.layers = nn.Sequential(
            nn.Conv2d(3, 32, 5, 1, 2),
            nn.MaxPool2d(2),
            nn.Conv2d(32, 32, 5, 1, 2),
            nn.MaxPool2d(2),
            nn.Conv2d(32, 64, 5, 1, 2),
            nn.MaxPool2d(2),
            nn.Flatten(),
            nn.Linear(64*4*4, 64),
            nn.Linear(64, 10)
        )

    def forward(self, x):
        return self.layers(x)
```

并在另一个文件 `save_model.py` 中创建并保存了这个模型：

```python
# save_model.py
from model_def import My_Network
import torch

model = My_Network()
# 进行训练等操作（省略）
torch.save(model, 'my_network.pth')
```

如果在第三个文件 `load_model.py` 中尝试加载这个模型，而没有导入 `My_Network` 类：

```python
# load_model.py
import torch

model = torch.load('my_network.pth')
```

这将导致 `AttributeError`，因为反序列化过程找不到 `My_Network` 类。正确的做法是在 `load_model.py` 文件中添加类的导入：

```python
# load_model.py
from model_def import My_Network
import torch

model = torch.load('my_network.pth')
```

### 结论

确保在模型加载环境中导入模型定义是反序列化成功的关键。模型类的缺失会中断 `pickle` 的反序列化流程，并导致加载失败。因此，持续维护模型定义的可访问性对于模型的成功加载至关重要。