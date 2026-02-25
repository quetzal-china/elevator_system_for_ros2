# ROS2 Action Client 异步编程笔记

## 1. 核心概念：同步 vs 异步

今天重构电梯代码最大的收获就是搞懂了为什么不能在节点内部用 `spin_until_future_complete`。

### 1.1 同步- 以前的错误写法
*   **逻辑**：我发请求 -> 我死等结果 -> 拿到结果 -> 继续跑。
*   **代码特征**：`spin_until_future_complete(...)`。
*   **问题**：
    *   如果在节点成员函数里用这个，外面的 `main` 函数里的 `spin` 就被挂起了。
    *   节点内部又想 `spin`，外面也在 `spin`，ROS2 的执行器就乱了，容易卡死或者重入。
    *   **就像**：去餐厅点餐，付完钱死站在出餐口不动，直到拿到餐。这期间有朋友给我发消息我也不回（处理不了其他回调），整个人“阻塞”在那了。

### 1.2 异步- 正确的写法
*   **逻辑**：我发请求 -> **立刻返回** -> 该干嘛干嘛 -> 结果到了自然会有人通知我（回调函数）。
*   **代码特征**：`async_send_goal(...)` + 回调函数注册。
*   **关键点**：发送函数不等待，把控制权交还给主循环。
*   **就像**：点餐拿个震动盘，直接找座位玩手机。盘震了（回调触发）再去取餐。这期间我还能聊天、看视频（处理其他任务）。

---

## 2. 核心机制：`rclcpp::spin()`

初学觉得 `spin` 只是个挂起函数，现在理解它其实是**“事件监听器”**。

*   **本质**：一个死循环 `while(true)`。
*   **职责**：不断检查网卡/消息队列里有没有新数据。
    *   有“服务器接受了请求”的消息 -> 触发 `goal_response_callback`。
    *   有“电梯到了某层”的消息 -> 触发 `feedback_callback`。
    *   有“任务结束”的消息 -> 触发 `result_callback`。
*   **我的误区**：以前总想在 `send_goal` 函数里等结果。
*   **正确理解**：`send_goal` 只负责“把信扔进邮箱”，`spin` 负责在家等回信。如果我在 `send_goal` 里死等，那谁来收信？这就矛盾了。

---

## 3. 实战重构：`SendGoalOptions` 的作用

重构前，我是先发请求，拿到 `goal_handle` 后再去订阅反馈。ROS2 Foxy 推荐把所有回调都在发送前“打包”好。

### 3.1 代码对比

**旧代码（混乱）：**
```cpp
// 1. 发送
auto future = client->async_send_goal(...);
// 2. 阻塞等待 handle (容易死锁)
spin_until_future_complete(...); 
auto handle = future.get();
// 3. 手动订阅反馈
handle.subscribe_feedback(...);
// 4. 阻塞等待结果
spin_until_future_complete(...);
```

**新代码（清晰）：**
```cpp
auto goal_options = SendGoalOptions();

// 【打包】把所有“后事”都在这里安排好
goal_options.goal_response_callback = ...; // 回信地址
goal_options.feedback_callback = ...;      // 进度汇报
goal_options.result_callback = ...;        // 最终结果

// 发送！发完即忘，后续交给 spin 和 回调
client->async_send_goal(goal_msg, goal_options);
```

### 3.2 三个回调的执行时机

我在代码里打了断点/日志，流程是这样的：

1.  **调用 `send_goal`**：
    *   日志：`正在发送请求...`
    *   动作：数据通过网络发出。
    *   **函数立即结束**，回到 `main` 的 `spin` 循环中。

2.  **服务器收到请求并决定接受/拒绝**：
    *   网络：收到数据包。
    *   `spin` 捕获 -> 触发 `goal_response_callback`。
    *   日志：`请求已被服务器接受`。
    *   *注意：此时 `future.get()` 在回调内部被自动调用，拿到 `goal_handle`。*

3.  **电梯运行中**：
    *   网络：服务器不断发 Feedback。
    *   `spin` 捕获 -> 触发 `feedback_callback`。
    *   日志：`[Feedback] Floor: 4 ...`。

4.  **电梯到达**：
    *   网络：服务器发 Result。
    *   `spin` 捕获 -> 触发 `result_callback`。
    *   日志：`任务成功完成`。

---

## 4. 避坑指南

### 4.1 回调函数签名必须匹配
`goal_response_callback` 的参数必须是 `std::shared_future<GoalHandleSharedPtr>`，不能直接是 `GoalHandleSharedPtr`。
*   **原因**：因为发送是异步的，发出的一瞬间没有 Handle，只有一个“未来的承诺”。只有当回调被触发时，这个 Future 才能 `.get()` 到真实的 Handle。

### 4.2 不要在回调里做耗时操作
回调是运行在 `spin` 线程里的。如果我在 `feedback_callback` 里写一个 `sleep(10)`，那整个节点就会卡顿 10 秒，收不到任何新消息。
*   **原则**：回调函数要快进快出，只做状态更新，别做重计算。

### 4.3 `main` 函数的职责
既然 `send_goal` 不阻塞了，`main` 函数必须负责维持节点的生命，即调用 `rclcpp::spin(node)`。否则程序跑完 `send_goal` 直接 `shutdown` 退出了，还没等到服务器回复呢。

---

## 总结

异步编程的核心思想就是**“控制反转”**：
*   **以前**：我主动去等结果（阻塞）。
*   **现在**：我注册好回调，等结果来找我（非阻塞）。

这次重构把“点餐、等餐、取餐”的流程拆解到了不同的回调里，代码虽然没有以前那种“线性直观”，但程序的健壮性和并发能力大大提升了，彻底解决了 `spin` 重入的问题。