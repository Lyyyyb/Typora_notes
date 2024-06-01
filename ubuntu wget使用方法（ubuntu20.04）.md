# ubuntu wget使用方法（ubuntu20.04）

`wget` 是一个命令行工具，用于从网络上下载文件。它支持 HTTP、HTTPS 和 FTP 协议，并具有许多强大的功能，例如断点续传、递归下载和限速下载。以下是对 `wget` 的详细解释、作用和使用方法。

### `wget` 是什么？

`wget` 是一个非交互式的网络下载工具，旨在自动化下载任务。它可以在后台运行，无需用户交互，适合用于脚本和批处理任务。`wget` 名字来源于 "World Wide Web" 和 "get" 的组合。

### `wget` 的作用

1. **下载文件**：可以从指定的 URL 下载文件，并将其保存到本地。
2. **断点续传**：支持从上次中断的地方继续下载，节省时间和带宽。
3. **递归下载**：可以下载整个网站或目录，保持文件的原始结构。
4. **限速下载**：允许用户限制下载速度，防止占用过多带宽。
5. **代理支持**：可以通过代理服务器下载文件。
6. **身份验证**：支持基本的 HTTP 和 FTP 验证，适用于需要认证的下载。

### 安装 `wget`

在大多数 Linux 发行版中，`wget` 通常已经预装。如果没有，可以使用包管理器安装：

```bash
sudo apt update
sudo apt install wget
```

### 基本使用方法

#### 下载单个文件

```bash
wget https://example.com/file.zip
```

#### 指定文件保存路径

使用 `-O` 选项指定保存的文件名和路径：

```bash
wget -O /path/to/destination/file.zip https://example.com/file.zip
```

#### 断点续传

如果下载中断，可以使用 `-c` 选项继续下载：

```bash
wget -c https://example.com/file.zip
```

#### 限制下载速度

使用 `--limit-rate` 选项限制下载速度，例如限制为 200KB/s：

```bash
wget --limit-rate=200k https://example.com/file.zip
```

### 高级使用方法

#### 递归下载整个网站

使用 `-r` 选项递归下载网站，`-np` 选项防止下载父目录：

```bash
wget -r -np https://example.com/
```

#### 镜像网站

使用 `-m` 选项创建网站镜像，`-k` 选项将链接转换为本地链接，`-E` 选项将 HTML 文件保存为 `.html` 扩展名：

```bash
wget -m -k -E https://example.com/
```

#### 设置用户代理

使用 `--user-agent` 选项设置用户代理字符串，模拟浏览器下载：

```bash
wget --user-agent="Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/91.0.4472.124 Safari/537.36" https://example.com/file.zip
```

#### 使用代理服务器

如果需要通过代理服务器下载，可以使用 `-e` 选项设置代理：

```bash
wget -e use_proxy=yes -e http_proxy=proxy.example.com:8080 https://example.com/file.zip
```

#### 下载包含认证信息的文件

对于需要 HTTP 基本认证的网站，可以使用 `--user` 和 `--password` 选项提供用户名和密码：

```bash
wget --user=username --password=password https://example.com/securefile.zip
```

#### 从文件中读取 URL 列表

如果需要下载多个文件，可以将 URL 列表保存到一个文本文件中，然后使用 `-i` 选项读取文件中的 URL：

```bash
wget -i urls.txt
```

### 常用选项总结

- `-O [file]`：将下载的内容保存到指定文件。
- `-c`：断点续传。
- `-r`：递归下载。
- `-np`：不下载父目录。
- `--limit-rate=[rate]`：限制下载速度。
- `-m`：镜像网站。
- `-k`：转换链接为本地链接。
- `-E`：将 HTML 文件保存为 `.html` 扩展名。
- `--user-agent=[agent]`：设置用户代理字符串。
- `--user=[user] --password=[pass]`：提供用户名和密码进行认证。
- `-i [file]`：从文件中读取 URL 列表。

### 示例

下载一个文件并保存到指定目录：

```bash
wget -O ~/Downloads/example.zip https://example.com/file.zip
```

断点续传下载文件：

```bash
wget -c https://example.com/file.zip
```

镜像整个网站：

```bash
wget -m -k -E https://example.com/
```

使用代理服务器下载文件：

```bash
wget -e use_proxy=yes -e http_proxy=proxy.example.com:8080 https://example.com/file.zip
```

提供认证信息下载文件：

```bash
wget --user=myusername --password=mypassword https://example.com/securefile.zip
```

从文件中读取 URL 列表并下载：

```bash
wget -i urls.txt
```

通过掌握以上这些命令和选项，您可以使用 `wget` 在 Ubuntu 上高效地下载文件和网站内容。