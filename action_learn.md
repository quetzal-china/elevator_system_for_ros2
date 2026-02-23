# Action学习笔记
---
## 1.Action的三大组成部分目标、反馈和结果
- 目标：即Action客户端告诉服务端要做什么，服务端针对该目标要有响应。解决了不能确认服务端接收并处理目标问题
- 反馈：即Action服务端告诉客户端此时做的进度如何（类似于工作汇报）。解决执行过程中没有反馈问题
- 结果：即Action服务端最终告诉客户端其执行结果，结果最后返回，用于表示任务最终执行情况。
> 一个Action = 三个服务+两个话题
> 三个服务分别是：1.目标传递服务 2.结果传递服务 3.取消执行服务 
> 两个话题：1.反馈话题（服务发布，客户端订阅） 2.状态话题（服务端发布，客户端订阅）

## 2.Action CLI
Action CLI是Action的命令行工具，用于与Action服务端进行交互。

### action list
该命令用于获取目前系统中的action列表。
```bash
ros2 action list
```
如果在list后加入-t参数，即可看到action的类型
```bash
ros2 action list -t
```
知道了接口类型之后，可以使用接口相关CLI指令获取接口的信息
```bash
ros2 interface show .../action/...
```

### action info
查看action信息，在终端中输入下面的指令。
```bash
ros2 action info /turtle1/rotate_absolute
```
返回action客户端和服务段的数量以及名字
```bash
Action: /turtle1/rotate_absolute
Action clients: 1
    /teleop_turtle
Action servers: 1
    /turtlesim
```

### action send_goal
```bash
ros2 action send_goal /turtle1/rotate_absolute turtlesim/action/RotateAbsolute "{theta: 1.6}" --feedback
```
日志反馈:
```bash
Waiting for an action server to become available...
Sending goal:
     theta: 1.5

Feedback:
    remaining: -0.0840003490447998

Goal accepted with ID: b368de0ed1a54e00890f1b078f4671c8

Feedback:
    remaining: -0.06800031661987305

Feedback:
    remaining: -0.05200028419494629

Feedback:
    remaining: -0.03600025177001953

Feedback:
    remaining: -0.020000219345092773

Feedback:
    remaining: -0.004000186920166016

Result:
    delta: 0.08000016212463379

Goal finished with status: SUCCEEDED
```
