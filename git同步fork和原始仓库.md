# git同步fork和原始仓库

在使用Fork的情况下，保持你的Fork与原始仓库（上游仓库）同步是一项重要的维护任务，特别是当你想要持续贡献或保持你Fork中的项目更新时。以下是详细的步骤，指导你如何将Fork与上游仓库同步：

### 步骤 1: 配置上游仓库

首先，你需要配置上游仓库，这样Git就知道从哪里拉取必要的更新。这通常只需要设置一次。

1. **打开终端并导航到你的项目目录**：
   确保你在Fork的本地副本的根目录。

2. **添加上游仓库**：
   使用`git remote add`命令添加原始仓库作为远程仓库。这里假设原始仓库的URL为`https://github.com/original-owner/repository-name.git`：
   ```bash
   git remote add upstream https://github.com/original-owner/repository-name.git
   ```

   这个命令将原始仓库添加为名为“upstream”的远程仓库，你将通过这个名称引用原始仓库。

### 步骤 2: 拉取上游的更改

1. **抓取上游仓库的更改**：
   使用`git fetch`命令从上游仓库抓取最新的更改：
   ```bash
   git fetch upstream
   ```

   这个命令将所有的更改下载到你的本地机器，但不会自动合并到你的工作分支。

### 步骤 3: 合并更改到你的主分支

1. **切换到你的主分支**：
   通常，你会想要更新你的`main`或`master`分支：
   ```bash
   git checkout main
   ```

2. **合并上游的更改**：
   现在将上游的更改合并到你的主分支。如果上游主分支是`main`，使用：
   ```bash
   git merge upstream/main
   ```

   这个命令会将拉取的更改合并到你的本地`main`分支。

### 步骤 4: 解决可能的冲突

如果合并过程中出现冲突，Git将停止合并并要求你手动解决这些冲突。你需要编辑冲突的文件，并决定如何合并变更。完成后，使用`git add`标记解决了冲突的文件，并提交更改。

### 步骤 5: 推送更改到你的Fork

1. **推送更新到GitHub**：
   一旦本地分支与上游同步，推送更改到你的GitHub Fork：
   ```bash
   git push origin main
   ```

这样，你的Fork就保持了与原始仓库的同步。定期执行这些步骤可以确保你在一个最新的基础上进行开发或者贡献。