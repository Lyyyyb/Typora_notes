# ubuntu安装pycharm（ubuntu20.04）

## 安装包下载（以社区版为例）

- 官网地址：https://www.jetbrains.com/pycharm/download/?section=linux

![image-20240601135736256](/home/lyb/github/Typora_notes/image-20240601135736256.png)

- 由于使用的处理器为x86_64架构，故这里使用Linux版本，点击download，等待下载完成。

## 解压安装包

- 在终端输入以下指令，解压并移动到`/opt（一般是放在opt）`目录：

```bash
sudo tar -xzf pycharm-community-2024.1.2.tar.gz -C /opt
```

## 运行pycharm

![image-20240601142503357](/home/lyb/github/Typora_notes/image-20240601142503357.png)

- 进入/opt/pycharm-community-2024.1.2/bin目录，并打开终端，输入

```bash
sudo ./pycharm.sh
```

- 运行脚本文件，启动pycharm



## 一些常规设置，可忽略

![image-20240601142734600](/home/lyb/github/Typora_notes/image-20240601142734600.png)

![image-20240601142803719](/home/lyb/github/Typora_notes/image-20240601142803719.png)

## 创建桌面快捷方式

![image-20240601142934777](/home/lyb/github/Typora_notes/image-20240601142934777.png)

- 点击左下角小齿轮，选择Create Desktop Entry创建桌面快捷方式

![image-20240601143009837](/home/lyb/github/Typora_notes/image-20240601143009837.png)

- 勾选Create the entry for all users，然后点击OK

![image-20240601143029255](/home/lyb/github/Typora_notes/image-20240601143029255.png)

- 快捷方式创建成功



## 创建工程

- 打开pycharm，点击New Project

![image-20240602115141590](/home/lyb/github/Typora_notes/image-20240602115141590.png)

- 选择自己需要的python解释器版本，我这里选择python3.10

![image-20240602115405287](/home/lyb/github/Typora_notes/image-20240602115405287.png)

- 工程创建完成

![image-20240602115957425](/home/lyb/github/Typora_notes/image-20240602115957425.png)

## 安装汉化包

- 点击右上角齿轮，选择Plugins（插件）

![image-20240602120143793](/home/lyb/github/Typora_notes/image-20240602120143793.png)

- 在搜索框中输入chinese，点击Install

![image-20240602120230329](/home/lyb/github/Typora_notes/image-20240602120230329.png)

- 下载完成后重启IDE即可

![image-20240602120251888](/home/lyb/github/Typora_notes/image-20240602120251888.png)

- 汉化成功

![image-20240602120427781](/home/lyb/github/Typora_notes/image-20240602120427781.png)

## 解释器版本选择及切换

- 直接点击右下角的python解释器版本，即可选择

![image-20240602121049658](/home/lyb/github/Typora_notes/image-20240602121049658.png)

- 如果想要添加新的本地python解释器，则点击添加新的解释器

![image-20240602121218703](/home/lyb/github/Typora_notes/image-20240602121218703.png)