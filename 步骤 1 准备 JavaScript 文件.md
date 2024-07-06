# 使用 HTML 和 JavaScript 实现与 ROS 的 WebSocket 通信

为确保你能成功使用 HTML 和 JavaScript 实现与 ROS 的 WebSocket 通信，我将详细地一步步指导你。以下是设置和运行整个系统的详细步骤：

### 步骤 1: 准备 JavaScript 文件

1. **下载所需的 JavaScript 库**：
   - 确保你有 `roslib.min.js`（ROS JavaScript 库）和 `eventemitter2.min.js`（事件发射器库，`roslib` 依赖此库）。
   - 如果这些文件还未在你的计算机上，可以从 [roslibjs GitHub](https://github.com/RobotWebTools/roslibjs) 和 [EventEmitter2 GitHub](https://github.com/EventEmitter2/EventEmitter2) 下载。

2. **放置 JavaScript 文件**：
   - 创建一个新目录（例如，名为 `web_interface`）在你的电脑上，将这些 `.js` 文件放入这个目录中。

### 步骤 2: 创建 HTML 文件

在同一目录 (`web_interface`) 下创建一个新的 HTML 文件，命名为 `index.html`。将以下内容粘贴进这个文件中：

```html
<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8" />
<title>ROS Web Interaction Example</title>
<script type="text/javascript" src="eventemitter2.min.js"></script>
<script type="text/javascript" src="roslib.min.js"></script>
<script type="text/javascript">
window.onload = function() {
  if (typeof ROSLIB === 'undefined') {
    console.error('ROSLIB is not loaded!');
    return;
  }

  var ros = new ROSLIB.Ros({
    url : 'ws://localhost:9090'
  });

  ros.on('connection', function() {
    console.log('Connected to websocket server.');
  });

  ros.on('error', function(error) {
    console.error('Error connecting to websocket server: ', error);
  });

  ros.on('close', function() {
    console.log('Connection to websocket server closed.');
  });

  var cmdVel = new ROSLIB.Topic({
    ros : ros,
    name : '/cmd_vel',
    messageType : 'geometry_msgs/Twist'
  });

  var twist = new ROSLIB.Message({
    linear : { x : 0.1, y : 0, z : 0 },
    angular : { x : 0, y : 0, z : 0.1 }
  });

  window.publishMessage = function() {
    cmdVel.publish(twist);
    console.log("Message published:", twist);
  }

  var listener = new ROSLIB.Topic({
    ros : ros,
    name : '/chatter',
    messageType : 'std_msgs/String'
  });

  window.subscribe = function() {
    listener.subscribe(function(message) {
      console.log('Received message on /chatter: ' + message.data);
      document.getElementById("output").innerHTML = 'Received message on /chatter: ' + message.data;
    });
  }

  window.unsubscribe = function() {
    listener.unsubscribe();
    console.log('Unsubscribed from /chatter.');
  }
}
</script>
</head>
<body>
<h1>ROS Web Interaction Example</h1>
<p>Check your Web Console for output.</p>
<p id="output"></p>
<button onclick="publishMessage()">Publish Message</button>
<button onclick="subscribe()">Subscribe</button>
<button onclick="unsubscribe()">Unsubscribe</button>
</body>
</html>
```

### 步骤 3: 启动本地 HTTP 服务器

1. **启动服务器**：
   - 打开命令行窗口。
   - 导航到你的 `web_interface` 目录。
   - 如果你使用的是 Python，运行以下命令：
     ```bash
     python -m http.server 8000
     ```
   - 如果出现权限问题，确保你的端口（此处为 8000）未被其他应用占用。

### 步骤 4: 访问 HTML 页面

1. **打开浏览器**：
   - 输入 URL `http://localhost:8000/index.html`。
   - 页面应该会加载，你可以通过开发者控制台（F12 -> Console）查看输出。

### 步骤 5: 确保 ROS 环境配置正确

1. **运行 ROSBridge**：
   - 确保你已安装 `rosbridge_suite`。
   - 在 ROS 环境中运行：
     ```bash
     roslaunch rosbridge_server rosbridge_websocket.launch
     ```
   - 这将在端口 9090 上开启 WebSocket 服务。

按照这些步骤操作，你应该能够设置一个简单的 Web 与 ROS 的交互界面。如果遇到任何问题，可以检查各步骤中的错误信息，并根据需要调整。