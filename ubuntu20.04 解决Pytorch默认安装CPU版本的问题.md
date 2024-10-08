# ubuntu20.04 解决Pytorch默认安装CPU版本的问题

![image-20241003221043695](/home/lyb/github/Typora_notes/image-20241003221043695.png)

在使用Anaconda安装支持CUDA的PyTorch版本时，遇到只能安装CPU版本的PyTorch是一个常见问题。这通常由于Anaconda环境配置、镜像源设置不当或版本匹配问题导致。以下是详尽的解决方案和步骤，以确保能够正确配置和使用镜像源安装正确的PyTorch版本。

### 问题分析
1. **镜像源的优先级问题**：当存在多个同名包时，Conda会根据配置的镜像源优先级决定下载哪一个版本。如果GPU支持的版本和CPU版本同时存在，没有正确设置优先级，可能导致安装了不支持CUDA的版本。
   
2. **版本匹配问题**：指定的PyTorch版本和cudatoolkit版本可能在所选的镜像源中无法找到匹配的组合，导致自动回退到只包含CPU支持的版本。

### 解决方案和步骤

#### 步骤1: 正确设置镜像源
- **添加PyTorch专用镜像源**：
  
  ```bash
  conda config --add channels https://mirrors.tuna.tsinghua.edu.cn/anaconda/cloud/pytorch/
  ```
  这个源专门为PyTorch及其依赖库提供服务，包括支持不同CUDA版本的PyTorch安装包。
  
- **设置显示频道URLs**：
  ```bash
  conda config --set show_channel_urls yes
  ```
  这一设置可以帮助您在安装过程中查看包的具体来源，有助于诊断问题。

- **编辑`.condarc`文件**：
  确保`~/.condarc`文件中PyTorch的链接优先级最高。可以使用文本编辑器直接编辑这个文件，或使用以下命令查看当前配置：
  ```bash
  cat ~/.condarc
  ```

#### 步骤2: 选择性添加和精简其他镜像源
- **深度学习源**：
  ```bash
  conda config --add channels https://mirrors.tuna.tsinghua.edu.cn/anaconda/pkgs/free/
  ```
  该源包含一些旧版本的深度学习库。

- **主镜像源**：
  ```bash
  conda config --add channels https://mirrors.tuna.tsinghua.edu.cn/anaconda/pkgs/main/
  ```
  包含大量的通用库，但可能包括CPU版本的PyTorch和旧的CUDA版本。

- **其他镜像源的选择性添加**：
  - Conda-forge源：
    ```bash
    conda config --add channels https://mirrors.tuna.tsinghua.edu.cn/anaconda/cloud/conda-forge/
    ```
    包含大量第三方库，应谨慎添加，以避免潜在的版本冲突。

```bash
auto_activate_base: false
channels:
  - https://mirrors.tuna.tsinghua.edu.cn/anaconda/cloud/pytorch/
  - https://mirrors.sustech.edu.cn/anaconda-extra/cloud/nvidia/
  - https://mirrors.tuna.tsinghua.edu.cn/anaconda/pkgs/free/
  - https://mirrors.tuna.tsinghua.edu.cn/anaconda/pkgs/main/
  - https://mirrors.tuna.tsinghua.edu.cn/anaconda/cloud/msys2/
  - https://mirrors.tuna.tsinghua.edu.cn/anaconda/cloud/conda-forge/
  - defaults
show_channel_urls: true


```



#### 步骤3: 安装指定版本的PyTorch和CUDA Toolkit

- 根据CUDA版本选择对应的PyTorch版本。您可以在PyTorch官网或清华源网站上找到版本兼容表。

- 执行安装命令：
  ```bash
  conda install pytorch torchvision torchaudio cudatoolkit=11.8 -c https://mirrors.tuna.tsinghua.edu.cn/anaconda/cloud/pytorch/
  ```

#### 步骤4: 验证安装

- 检查已安装的PyTorch版本：
  ```bash
  conda list pytorch
  ```
- 在Python中验证CUDA支持：
  ```python
  import torch
  print(torch.cuda.is_available())  # 应返回True
  ```

通过以上详细步骤，您可以确保从Anaconda的清华源正确安装支持CUDA的PyTorch版本，避免因配置不当而下载到CPU版本。这个过程不仅确保了软件的正确安装，也提高了安装过程的透明度和可控性。

![image-20241003224530315](/home/lyb/github/Typora_notes/image-20241003224530315.png)