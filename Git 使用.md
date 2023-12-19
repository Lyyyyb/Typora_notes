# Git 使用

## 基本使用

`Git` 是一种广泛使用的分布式版本控制系统，用于跟踪在计算机文件中的更改并协调多人之间的工作。以下是 Git 的一些基本用法，涵盖了常见的 Git 命令和工作流程：

### 安装 Git

在不同操作系统上安装 Git 的方式可能有所不同。在 Linux 系统上，你通常可以使用包管理器安装 Git，例如在 Ubuntu 上，可以使用以下命令：

```bash
sudo apt-get update
sudo apt-get install git
```

### 配置 Git

在首次使用 Git 之前，建议设置你的用户名和电子邮件地址，因为每个 Git 提交都会使用这些信息：

```bash
git config --global user.name "Your Name"
git config --global user.email "your.email@example.com"
```

### 初始化仓库

要开始使用 Git 跟踪某个项目，你需要初始化一个仓库。这可以在项目的根目录下进行：

```bash
git init
```

这将创建一个新的 `.git` 目录（如果尚未存在）。

### 跟踪文件和暂存更改

当你有了文件需要被 Git 跟踪时，你可以使用 `git add` 命令将它们添加到暂存区：

```bash
git add <filename>
```

或者，要一次性添加当前目录下的所有更改（包括新文件和修改的文件），可以使用：

```bash
git add .
```

### 提交更改

一旦你的更改被暂存，你可以使用 `git commit` 命令来提交它们：

```bash
git commit -m "Commit message"
```

其中 "Commit message" 应该是一个简洁明了的说明，描述了提交中的更改。

### 查看状态和日志

要查看仓库的当前状态，可以使用：

```bash
git status
```

要查看提交历史：

```bash
git log
```

### 分支管理

分支是处理不同功能的强大方法。创建新分支：

```bash
git branch <branchname>
```

切换分支：

```bash
git checkout <branchname>
```

合并分支：

```bash
git merge <branchname>
```

### 远程仓库

要将本地仓库与远程仓库关联，可以使用：

```bash
git remote add origin <remote-repository-URL>
```

上传本地提交到远程仓库：

```bash
git push origin <branchname>
```

从远程仓库获取更新：

```bash
git pull
```

### 克隆现有仓库

如果你想获取一个已经存在的 Git 仓库的副本，可以使用 `git clone`：

```bash
git clone <repository-URL>
```

### 总结

这些是 Git 的一些基本用法，涵盖了日常工作中的常见场景。Git 有许多高级特性和技巧，随着你逐渐深入学习，你会发现它有助于提高你的代码管理效率。记住，实践是学习 Git 的最佳方式。尝试在你的项目中使用 Git，并逐步探索更多的功能。

## 创建远程仓库

使用 GitHub 创建远程仓库是一个简单的过程，可以帮助你在云端存储、共享和管理代码项目。以下是详细步骤：

### 1. 创建 GitHub 账号

如果你还没有 GitHub 账号，你需要先访问 [GitHub](https://github.com/) 并注册一个账号。

### 2. 创建新仓库

1. **登录 GitHub**：使用你的账号登录 GitHub。

2. **创建仓库**：
   - 在 GitHub 的主页右上角，点击你的用户头像，然后选择 “Your repositories”。
   - 点击 “New” 按钮，这通常位于页面右上角。
   - 填写仓库信息：
     - **Repository name**：给你的仓库起一个名字。
     - **Description**（可选）：为你的仓库提供一个简短描述。
     - **Public/Private**：选择仓库是公开的还是私有的。公开仓库对所有人都是可见的，而私有仓库只对你和你授权的用户可见。
     - **Initialize this repository with**（可选）：你可以选择初始化仓库时创建一个 README 文件、添加 .gitignore 文件或选择许可证。
   - 点击 “Create repository” 按钮创建仓库。

### 3. 克隆仓库到本地

一旦你的远程仓库创建好了，你可能想在本地工作。这需要克隆仓库：

1. **获取仓库链接**：在仓库页面，点击 “Code” 按钮，复制提供的 URL（HTTPS 或 SSH）。

2. **使用 Git 克隆仓库**：在本地计算机上，打开命令行或终端，然后使用以下命令克隆仓库：

   ```bash
   git clone <repository-URL>
   ```

   替换 `<repository-URL>` 为你刚刚复制的链接。

### 4. 添加和提交更改

在本地进行更改后，你可以将这些更改提交到 GitHub 仓库：

1. **添加更改**：在仓库的本地目录中，使用以下命令将更改添加到暂存区：

   ```bash
   git add .
   ```

2. **提交更改**：提交你的更改并附上提交信息：

   ```bash
   git commit -m "Your commit message"
   ```

### 5. 推送更改到 GitHub

将本地的提交推送到远程 GitHub 仓库：

```bash
git push origin master
```

这里 `master` 是你推送到的分支的名称。如果你使用的是其他分支，需要替换为相应的分支名称。

### 6. 拉取远程更改

为了保持本地仓库与远程仓库同步，你可以拉取远程更改：

```bash
git pull origin master
```

### 总结

通过这些步骤，你可以在 GitHub 创建一个新的远程仓库，克隆它到本地，进行更改，并将这些更改推送回 GitHub。这是使用 GitHub 管理项目的基本流程，允许你在云端备份代码、协作和版本控制。随着经验的积累，你可以探索 GitHub 的更多高级功能，如分支管理、Pull Requests 和 Issues 等。

## 克隆代码到本地以及更新代码

```c
# 克隆 GitHub 仓库到本地
# 替换 <repository-URL> 为你要克隆的仓库的 URL
git clone <repository-URL>

# 进入仓库目录
# 替换 <repository-name> 为克隆下来的仓库名称
cd <repository-name>

# 更新本地代码
# 如果你的主分支是 master：
git checkout master
git pull origin master

# 或者，如果你的主分支是 main：
git checkout main
git pull origin main

# 以后，每次想要更新本地代码时，只需运行：
git pull

```

将 GitHub 上的代码克隆到本地并在后续进行更新是常见的 Git 工作流程。以下是详细步骤：

### 克隆 GitHub 仓库到本地

1. **找到仓库**：首先，在 GitHub 上找到你想克隆的仓库。

2. **复制仓库的 URL**：点击仓库页面上的 “Clone or download” 按钮，然后复制显示的 URL。这个 URL 可以是 HTTPS 格式的，也可以是 SSH 格式的。

3. **克隆仓库**：打开你的本地终端或命令行界面，切换到你想存放仓库的目录，然后使用以下命令：

   ```bash
   git clone <repository-URL>
   ```

   替换 `<repository-URL>` 为你刚刚复制的仓库 URL。

4. **进入仓库目录**：克隆完成后，进入该仓库的目录：

   ```bash
   cd <repository-name>
   ```

   替换 `<repository-name>` 为仓库的名称。

此时，你就已经将 GitHub 上的仓库成功克隆到了本地。

### 更新本地代码

如果你已经有了仓库的本地副本，并且想要获取 GitHub 上的最新更改，你可以执行以下操作：

1. **切换到正确的分支**：确保你在正确的分支上，通常是 `master` 或 `main` 分支：

   ```bash
   git checkout master
   ```

   或者，如果你的主分支是 `main`：

   ```bash
   git checkout main
   ```

2. **拉取最新更改**：使用以下命令拉取远程仓库的最新更改：

   ```bash
   git pull origin master
   ```

   或者，如果你的主分支是 `main`：

   ```bash
   git pull origin main
   ```

   这将会抓取（fetch）远程仓库的最新更改并合并（merge）到你的本地分支。

### 确保代码是最新的

在开始工作之前，始终确保你的本地代码是最新的。这有助于减少合并冲突的可能性，特别是当你在一个团队中工作时。

### 总结

通过上述步骤，你可以轻松地将 GitHub 上的代码克隆到本地，并定期更新以保持同步。这是维护和协作开发项目的基本流程，对于使用 Git 和 GitHub 是非常重要的技能。

## 将本地代码发送到远程库

```c
# 导航到项目目录
cd path/to/your/project

# 初始化本地 Git 仓库（如果尚未初始化）
git init

# 连接本地仓库到 GitHub 远程仓库
# 替换以下 URL 为你的 GitHub 仓库 URL
git remote add origin https://github.com/username/repository.git

# 添加文件到 Git 暂存区
# 添加特定文件：git add <filename>
# 或者添加所有更改：git add .
git add .

# 提交更改到本地仓库
# 替换 "Initial commit" 为你的提交信息
git commit -m "Initial commit"

# 首次推送到远程仓库，并设置本地分支跟踪远程分支
# 如果你的主分支是 master：git push -u origin master
# 如果你的主分支是 main：git push -u origin main
git push -u origin master

# 或者
# git push -u origin main

# 之后，每当有新的更改需要推送，只需：
git push

```

将本地代码上传到远程仓库是 Git 工作流程的一个重要部分，尤其是在与团队合作或将项目托管在像 GitHub 这样的平台时。以下是详细的步骤：

### 1. 初始化本地仓库（如果还未初始化）

如果你的项目还不是一个 Git 仓库，首先需要初始化它。

1. **打开终端**：打开你的命令行工具。
   
2. **导航到项目目录**：使用 `cd` 命令进入你的项目文件夹。

   ```bash
   cd path/to/your/project
   ```

3. **初始化 Git 仓库**：

   ```bash
   git init
   ```

   这将在你的项目目录中创建一个新的子目录 `.git`，其中包含所有必需的 Git 仓库文件，但目前还没有任何代码被跟踪。

### 2. 添加远程仓库

1. **在 GitHub 创建远程仓库**：在 GitHub 上创建一个新的仓库。记下仓库的 URL，它通常是这样的格式：`https://github.com/username/repository.git`。

2. **连接本地仓库到远程仓库**：

   ```bash
   git remote add origin https://github.com/username/repository.git
   ```

   替换 URL 为你的 GitHub 仓库 URL。`origin` 是远程仓库的默认名称。

### 3. 添加文件到暂存区

1. **添加文件**：将你的文件添加到 Git 暂存区。

   - 添加特定文件：

     ```bash
     git add <filename>
     ```

   - 添加所有文件：

     ```bash
     git add .
     ```

### 4. 提交更改

1. **提交到本地仓库**：将暂存的更改提交到本地仓库。

   ```bash
   git commit -m "Initial commit"
   ```

   `Initial commit` 是提交信息，应描述你所做的更改。

### 5. 推送到远程仓库

1. **首次推送**：使用以下命令将本地更改推送到远程仓库。如果你正在使用 `master` 分支：

   ```bash
   git push -u origin master
   ```

   如果你的主分支是 `main`：

   ```bash
   git push -u origin main
   ```

   这个命令不仅推送代码，而且设置了本地分支跟踪远程分支。

### 6. 后续推送

1. **后续提交**：在之后的工作中，每当你完成更改并想要将它们推送到 GitHub 时，只需要简单地提交你的更改，然后使用 `git push` 命令。

   ```bash
   git push
   ```

### 总结

通过遵循上述步骤，你可以将本地代码库上传到 GitHub 或其他远程 Git 仓库。这个流程包括初始化本地仓库（如果尚未初始化）、添加远程仓库、暂存更改、提交更改到本地仓库，以及将这些更改推送到远程仓库。这是 Git 的基本工作流程，对于代码版本控制和协作至关重要。

## 私有仓库配置

Git 在2021年8月13日停止支持使用用户名和密码进行身份验证的方式来访问 GitHub 仓库。为了克隆 GitHub 仓库，你需要使用基于令牌（Token）的身份验证方式。以下是使用基于令牌的方式来进行克隆的步骤：

1. 在 GitHub 上生成个人访问令牌（Personal Access Token）：
   - 登录到 GitHub 帐户。
   - 点击右上角的用户头像，选择 "Settings"（设置）。
   - 在左侧边栏中，选择 "Developer settings"（开发者设置）。
   - 在下拉菜单中选择 "Personal access tokens"（个人访问令牌）。
   - 点击 "Generate token"（生成令牌）按钮。
   - 给令牌一个名称，选择需要的权限（通常只需选择 "repo" 权限以克隆和访问仓库），然后点击生成令牌。
   - 复制生成的个人访问令牌。

2. 使用生成的个人访问令牌进行克隆：
   - 在终端或命令提示符中，执行以下克隆命令，并将 `<Your-Token>` 替换为你生成的个人访问令牌：
   
     ```
     git clone https://<Your-Token>@github.com/Lyyyyb/Typora_notes.git 
     <Your-Token>=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
     ```
   
   - 执行该命令时，会要求你输入 GitHub 用户名，但在密码字段中，你应该粘贴生成的个人访问令牌而不是密码。

这样，你就可以使用个人访问令牌来成功克隆 GitHub 仓库，而不再需要用户名和密码。请确保不要分享或泄露你的个人访问令牌，因为它相当于你的 GitHub 帐户的密码。如果你认为令牌不再安全，可以在 GitHub 上生成新的令牌来替换旧的令牌。