# Action学习笔记
---
## 1.Action的三大组成部分目标、反馈和结果
- 目标：即Action客户端告诉服务端要做什么，服务端针对该目标要有响应。解决了不能确认服务端接收并处理目标问题
- 反馈：即Action服务端告诉客户端此时做的进度如何（类似于工作汇报）。解决执行过程中没有反馈问题
- 结果：即Action服务端最终告诉客户端其执行结果，结果最后返回，用于表示任务最终执行情况。
> 一个Action = 三个服务+两个话题
> 三个服务分别是：1.目标传递服务 2.结果传递服务 3.取消执行服务 
> 两个话题：1.反馈话题（服务发布，客户端订阅） 2.状态话题（服务端发布，客户端订阅）

以**电梯调度**为例：
- **目标(Goal)**：乘客告诉电梯"我要从3楼到7楼"
- **反馈(Feedback)**：电梯告诉乘客"现在到4楼了"、"到5楼了"...
- **结果(Result)**：电梯告诉乘客"已到达7楼，任务完成"
> **Goal的状态转换**
> 待处理(pending) → 执行中(executing) → 已完成(succeeded)/已取消(canceled)/已中止(aborted)

> **为什么要用Action而不是Service？**
> - Service是同步的，客户端必须等待结果
> - Action是异步的，客户端可以继续做其他事情
> - Action有中间状态的反馈，Service没有
> - Action可以被取消，Service不行

| 特性 | Topic | Service | Action |
|------|-------|---------|--------|
| 通信模式 | 发布/订阅 | 请求/响应 | 目标/反馈/结果 |
| 执行时间 | 短时 | 短时 | 长时 |
| 是否有反馈 | 无 | 无 | 有 |
| 是否可取消 | - | 不可 | 可 |
| 同步/异步 | 异步 | 同步 | 异步 |

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

## 3. rclcpp_action C++库使用

### 3.1 创建Action服务器

```cpp
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

// 创建Action服务器
this->action_server_ = rclcpp_action::create_server<ActionType>(
    this,
    "action_name",
    std::bind(&ClassName::handle_goal, this, _1, _2),      // 处理新目标
    std::bind(&ClassName::handle_cancel, this, _1),        // 处理取消请求
    std::bind(&ClassName::handle_accepted, this, _1)        // 接受目标
);
```

### 3.2 常用函数

| 函数 | 作用 |
|------|------|
| `goal_handle->publish_feedback(feedback_msg)` | 发布反馈信息 |
| `goal_handle->succeed(result)` | 标记任务成功完成 |
| `goal_handle->canceled(result)` | 标记任务被取消 |
| `goal_handle->abort(result)` | 标记任务中止/失败 |
| `goal_handle->is_canceling()` | 检查是否收到取消请求 |
| `goal_handle->get_goal()` | 获取目标信息 |

### 3.3 三个回调函数

```cpp
// 1. 处理新目标
rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &uuid,
    std::shared_ptr<const GoalType> goal)
{
    // 验证目标参数，返回 ACCEPT_AND_EXECUTE 或 REJECT
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

// 2. 处理取消请求
rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleType> goal_handle)
{
    // 决定是否允许取消，返回 ACCEPT 或 REJECT
    return rclcpp_action::CancelResponse::ACCEPT;
}

// 3. 接受目标并开始执行
void handle_accepted(const std::shared_ptr<GoalHandleType> goal_handle)
{
    // 启动新线程执行任务（重要：不能阻塞主线程）
    std::thread{std::bind(&ClassName::execute, this, _1), goal_handle}.detach();
}
```

---

## 4. C++ std::thread 多线程

### 4.1 为什么要用多线程？

在Action服务器中，执行任务可能是耗时的操作（如电梯移动）。如果直接在回调函数中执行，会阻塞主线程，导致其他请求无法处理。因此需要启动新线程来执行任务。

### 4.2 核心概念详解

#### 4.2.1 std::shared_ptr - 智能指针

**什么是智能指针？**
- 普通指针（如 `int*`）需要手动管理内存，容易造成内存泄漏
- `shared_ptr` 是"智能指针"，会自动管理内存，当没人用时自动释放

**电梯调度中的使用：**
```cpp
// 普通方式（不推荐）
GoalHandleType* goal_handle = new GoalHandleType();

// 智能指针方式（推荐）
auto goal_handle = std::make_shared<GoalHandleType>();
// 自动管理内存，不用自己delete
```

#### 4.2.2 std::bind - 绑定函数

**什么是bind？**
- 把函数和它的参数"绑定"在一起，创建一个新的可调用对象

**简单比喻：**
```cpp
// 假设有一个函数
int add(int a, int b) { return a + b; }

// bind就像"封装"这个函数
auto add_5 = std::bind(add, 5, std::placeholders::_1);

// 现在调用 add_5(3) 就等于 add(5, 3)，结果是8
```

**在Action服务器中：**
```cpp
// 把成员函数和目标绑定在一起
std::bind(&ElevatorServer::execute, this, goal_handle)
// 等价于创建一个函数对象，调用时会执行 this->execute(goal_handle)
```

#### 4.2.3 std::thread - 线程

**什么是线程？**
- 线程就像"同时做两件事"
- 主线程负责监听请求，工作线程负责执行任务

**比喻：**
```
主线程 = 接待员（只负责接收请求）
工作线程 = 电梯（真正执行任务）
```

#### 4.2.4 .detach() - 分离线程

**什么是detach？**
- `detach()` 把线程从主线程"分离"出去
- 分离后的线程在后台独立运行
- 主线程不需要等待它结束

**比喻：**
```
join() = 等电梯运行完才能做其他事
detach() = 派电梯出去运行，我可以继续接其他请求
```

#### 4.2.5 execute - 执行函数

**execute是什么？**
- 这只是一个普通的成员函数名，可以随便起名
- 它里面包含电梯移动的实际逻辑

```cpp
void execute(const std::shared_ptr<GoalHandleType> goal_handle)
{
    // 这里写电梯如何移动的逻辑
    // 1. 从当前楼层移动到乘客楼层
    // 2. 开关门
    // 3. 移动到目标楼层
    // 4. 开关门
    // 5. 发送结果
}
```

#### 4.2.6 instance - 实例

**instance就是"对象"**
- 类就像"图纸"，实例就像"造出来的房子"
- `std::make_shared<MyClass>()` 就是"按图纸造一个房子"

```cpp
class ElevatorServer { };

// 创建实例（对象）
auto server = std::make_shared<ElevatorServer>();
// server就是一个"实例"
```

### 4.3 基本用法

```cpp
#include <thread>

// 创建并启动线程
std::thread thread_object(function_to_execute, arg1, arg2);

// 分离线程（让线程在后台运行）
thread_object.detach();

// 或者等待线程结束
thread_object.join();
```

### 4.4 在Action服务器中的使用

```cpp
// 使用lamdba表达式
void handle_accepted(const std::shared_ptr<GoalHandleType> goal_handle)
{
    // 使用lambda启动新线程执行任务
    std::thread{[this, goal_handle]() {
        this->execute(goal_handle);
    }}.detach();
}
// 使用std::bind()函数
void handle_accepted(const std::shared_ptr<GoalHandleType> goal_handle)
{
    std::thread(std::bind(&ActionServerClass::execute, this, goal_handle)).detach();
}
// 使用包装函数
void handle_accepted(const std::shared_ptr<GoalHandleType> goal_handle)
{
    std::thread(execute_wrapper, this, goal_handle).detach();
}

private:
    static void execute_wrapper(
        ActionServerClass* instance, 
        std::shared_ptr<GoalHandleType> handle)
    {
        instance->execute(handle);
    }
```

### 4.5 注意事项

- `detach()` 会使线程在后台运行，不需要等待其结束
- 使用 `std::bind` 或 lambda 表达式传递成员函数
- 确保在节点销毁前线程执行完毕

---

## 5. C++ std::future 异步操作

### 5.1 什么是 std::future

C++11 引入了 `<future>` 头文件，提供了一种异步编程机制，允许程序在等待某个操作完成时继续执行其他任务。

**核心概念**：
- `std::future`：表示异步操作的结果，可以查询状态、获取结果或等待完成
- `std::promise`：与 `std::future` 配对使用，用于设置异步操作的结果
- `std::packaged_task`：封装函数或可调用对象，使其可以作为异步任务执行
- `std::async`：便捷的函数，用于启动异步任务并返回 `std::future`

### 5.2 std::promise 和 std::future 配对使用

```cpp
#include <iostream>
#include <future>
#include <thread>

int main() {
    std::promise<int> prom;          // 创建 promise
    std::future<int> fut = prom.get_future();  // 获取对应的 future
    
    // 在另一个线程中设置结果
    std::thread t([&prom]() {
        prom.set_value(10);  // 设置结果
    });
    
    // 等待并获取结果
    std::cout << "Future value: " << fut.get() << std::endl;
    
    t.join();
    return 0;
}
```

### 5.3 std::async 启动异步任务

```cpp
#include <iostream>
#include <future>

int main() {
    // 启动异步任务，立即返回 future
    std::future<int> fut = std::async(std::launch::async, [](int x) {
        return x * x;
    }, 5);
    
    // 获取结果（会等待任务完成）
    std::cout << "Result: " << fut.get() << std::endl;
    return 0;
}
```

**参数说明**：
- `std::launch::async`：立即在新线程中执行
- `std::launch::deferred`：延迟到调用 `get()` 时才执行
- `std::launch::async | std::launch::deferred`：默认值，由系统决定

### 5.4 在 ROS2 Action 客户端中的应用

在 ROS2 Action 客户端中，`std::future` 用于等待异步操作的结果：

```cpp
// 1. 发送目标后获取 future
auto send_goal_future = action_client->send_goal_async(goal_msg);

// 2. 等待目标被接受
send_goal_future.wait();  // 等待
auto goal_handle = send_goal_future.get();  // 获取结果

// 3. 等待最终结果
auto result_future = goal_handle->get_result_async();
result_future.wait();  // 等待结果
auto result = result_future.get();  // 获取结果
```

### 5.5 std::future 常用方法

| 方法 | 作用 |
|------|------|
| `get()` | 获取结果并等待完成（只能调用一次） |
| `wait()` | 等待完成，不获取结果 |
| `wait_for(duration)` | 等待指定时间 |
| `wait_until(timepoint)` | 等待到指定时间点 |
| `valid()` | 检查是否包含有效结果 |

### 5.6 异常处理

异步操作抛出的异常会被 `std::future` 捕获，通过 `.get()` 重新抛出。

```cpp
try {
    fut.get();  // 异常在这里重新抛出
} catch (const std::exception& e) {
    std::cout << "Caught exception: " << e.what() << std::endl;
}
```

---

## 小结
Action 通信适用于**长时间执行**且需要**中间反馈**的任务，如：
- 机器人导航
- 机械臂运动
- 电梯调度（本项目）

**常用异步机制对比**：
| 机制 | 适用场景 | 特点 |
|------|----------|------|
| `std::thread` | 简单并行任务 | 直接创建线程，手动管理 |
| `std::future` | 异步操作结果 | 等待结果，获取返回值 |
| `std::async` | 简单异步任务 | 自动管理线程，返回 future |