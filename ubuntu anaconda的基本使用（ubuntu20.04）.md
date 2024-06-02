# ubuntu anaconda的基本使用（ubuntu20.04）

## Anaconda Navigator的使用

### 创建Anaconda Navigator快捷方式

- 进入应用程序目录

```bash
cd /usr/share/applications/
```

- 创建`anaconda.desktop`文件

```bash
sudo touch anaconda.desktop
```

- 编辑`anaconda.desktop`文件

```bash
sudo vim anaconda.desktop
```

- 添加如下内容
  - 可使用`anaconda -V`获取到你的anaconda版本号

```bash
[Desktop Entry]
Name=Anaconda
Version=1.7.2
Type=Application
Exec=/home/lyb/anaconda3/bin/anaconda-navigator
Icon=/home/lyb/anaconda3/pkgs/anaconda-navigator-2.5.2-py311h06a4308_0/lib/python3.11/site-packages/anaconda_navigator/app/icons/Icon1024.png
Terminal=false
StartupNotify=true
```

>`Name`填写应用名称；
> `Exec`可执行文件位置；
> `Icon`桌面显示的logo的位置，应用一般自带了，也可以自己建立一个文件夹；

- 为`anaconda.desktop`文件添加可执行权限

```bash
sudo chmod a+x anaconda.desktop
```

- 之后就可以直接使用桌面快捷方式打开了

### 管理当前环境

![image-20240602152332654](/home/lyb/github/Typora_notes/image-20240602152332654.png)

## 环境管理

### 激活base环境

- 激活base环境：

  ```bash
  conda activate base
  ```

### 创建新环境

- 使用以下命令创建一个新的环境，指定 Python 版本：

```bash
conda create -n py39 python=3.9
```

- `py39` 是环境名，可以根据需要更改。
- `python=3.9` 指定了环境中的 Python 版本，也可根据需求指定其他版本。

![image-20240602143955708](/home/lyb/github/Typora_notes/image-20240602143955708.png)

- 将会开始安装 Python 3.9 及相关的一系列基础和辅助包。这个环境是完全独立的，可以用来开发特定于 Python 3.9 的应用，而不会干扰到系统中的其他 Python 环境。

![image-20240602144111578](/home/lyb/github/Typora_notes/image-20240602144111578.png)

- 成功创建新的虚拟环境，可以在anaconda3/envs目录下找到

![image-20240602144328546](/home/lyb/github/Typora_notes/image-20240602144328546.png)

### 激活新环境

- 激活刚创建的环境,可以发现就从base切换到了刚刚创建的py39

```bash
conda activate py39
```

![image-20240602144537721](/home/lyb/github/Typora_notes/image-20240602144537721.png)

- 可以用python指令，测试一下当前环境下的python版本号，可以看到确实是python3.9

```bash
python
```

![image-20240602144711306](/home/lyb/github/Typora_notes/image-20240602144711306.png)

### 退出环境

- 从当前激活的 Anaconda 环境中退出，在终端输入以下指令

```bash
conda deactivate
```

![image-20240602145355111](/home/lyb/github/Typora_notes/image-20240602145355111.png)

### 切换环境

- 从当前py39环境切换到base环境

```bash
conda activate base
```

![image-20240602145007911](/home/lyb/github/Typora_notes/image-20240602145007911.png)

### 查看环境列表

- 查看当前所有的环境列表

```bash
conda env list
或者
conda info --envs
```

![image-20240602145121377](/home/lyb/github/Typora_notes/image-20240602145121377.png)

### 删除环境

- 删除不再需要的环境

```bash
conda remove -n py38 --all
```

![image-20240602151957065](/home/lyb/github/Typora_notes/image-20240602151957065.png)

## 包管理

### 查看已安装的包

- 查看环境中已安装的所有包：

```bash
conda list
```

![image-20240602150146673](/home/lyb/github/Typora_notes/image-20240602150146673.png)

### 安装包

- 在激活的环境中安装包：以numpy为例

  ```bash
  conda install numpy
  ```

## 将创建好的虚拟环境导入IDE（以pycharm为例）

- 进入pycharm设置界面，找到Python解释器设置一栏，点击右上角的Add Interpreter添加解释器。

![image-20240602150641663](/home/lyb/github/Typora_notes/image-20240602150641663.png)

- 选择Conda Environment，勾选Use existing environment，使用现有的环境，之后在下面选择刚刚创建好的py39虚拟环境，这里conda的可执行文件会自己识别，如果识别不到则手动载入一下即可，点击确定。

![image-20240602150941452](/home/lyb/github/Typora_notes/image-20240602150941452.png)

- 选择Python Interpreter（python解释器）为刚刚创建的py39，点击应用，点击确定，虚拟环境就应用完成了，里面的宏包、软件包和其他环境是完全不冲突的，所以可以根据项目的需要构建符合自己需求的python虚拟环境

![image-20240602151106816](/home/lyb/github/Typora_notes/image-20240602151106816.png)