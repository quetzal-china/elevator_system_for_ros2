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

## 5. C++ 多线程同步机制（mutex、lock、condition_variable）

### 5.1 为什么需要线程同步？

在多线程程序中，多个线程可能同时访问同一资源，导致**数据竞争**问题：

```cpp
// 错误示例：没有同步机制
int shared_data = 0;

void increment() {
    for (int i = 0; i < 10000; i++) {
        shared_data++;  // 多个线程同时修改，结果不可预测
    }
}

int main() {
    std::thread t1(increment);
    std::thread t2(increment);
    t1.join();
    t2.join();
    std::cout << shared_data << std::endl;  // 期望20000，实际可能小于20000
}
```

**问题原因**：`shared_data++` 不是原子操作，包含三个步骤：
1. 读取 `shared_data` 的值
2. 将值加1
3. 将新值写回 `shared_data`

多线程同时执行这三个步骤会导致数据丢失。

### 5.2 std::mutex - 互斥锁

#### 5.2.1 基本概念

`std::mutex`（互斥锁）是一种同步机制，用于保护共享资源：
- 同一时刻只允许一个线程访问临界区
- 线程访问前必须先锁定（lock）
- 线程访问后必须解锁（unlock）

**比喻**：
```
mutex 就像卫生间门锁
- 进入卫生间前必须先锁门（lock）
- 使用完毕后必须开门（unlock）
- 其他人想用必须等门打开
```

#### 5.2.2 基本用法

```cpp
#include <mutex>

std::mutex mtx;  // 创建互斥锁
int shared_data = 0;

void safe_increment() {
    mtx.lock();           // 加锁
    shared_data++;        // 安全访问共享资源
    mtx.unlock();         // 解锁
}
```

**问题**：如果忘记 `unlock()`，会导致**死锁**！

#### 5.2.3 死锁问题

```cpp
// 危险示例：可能死锁
void unsafe_function() {
    mtx.lock();
    if (some_condition) {
        return;  // 忘记 unlock！其他线程永远无法获取锁
    }
    mtx.unlock();
}
```

**解决方案**：使用 RAII 风格的锁管理器（`std::lock_guard` 或 `std::unique_lock`）。

### 5.3 std::lock_guard - 自动管理锁

#### 5.3.1 基本概念

`std::lock_guard` 是一个 RAII 风格的锁管理器：
- 构造时自动加锁
- 析构时自动解锁（即使发生异常）
- 不需要手动调用 `lock()` 和 `unlock()`

**比喻**：
```
lock_guard 就像自动门锁
- 进入房间自动锁门（构造函数加锁）
- 离开房间自动开门（析构函数解锁）
- 即使紧急撤离（异常）也能自动开门
```

#### 5.3.2 基本用法

```cpp
#include <mutex>

std::mutex mtx;
int shared_data = 0;

void safe_function() {
    std::lock_guard<std::mutex> lock(mtx);  // 构造时自动加锁
    shared_data++;
    // 函数结束，lock 析构，自动解锁
}
```

#### 5.3.3 在电梯系统中的应用

```cpp
// 电梯系统中保护请求队列
class ElevatorActionServer {
private:
    std::queue<Request> request_queue_;
    std::mutex queue_mutex_;

public:
    // 添加请求（生产者）
    void add_request(const Request& req) {
        std::lock_guard<std::mutex> lock(queue_mutex_);  // 自动加锁
        request_queue_.push(req);
        // 自动解锁
    }
    
    // 取出请求（消费者）
    Request get_request() {
        std::lock_guard<std::mutex> lock(queue_mutex_);  // 自动加锁
        if (request_queue_.empty()) {
            return Request();  // 返回空请求
        }
        Request req = request_queue_.front();
        request_queue_.pop();
        return req;
        // 自动解锁
    }
};
```

### 5.4 std::unique_lock - 灵活的锁管理

#### 5.4.1 基本概念

`std::unique_lock` 比 `std::lock_guard` 更灵活：
- 可以手动加锁和解锁
- 支持条件变量（`std::condition_variable`）
- 支持延迟加锁、尝试加锁

#### 5.4.2 常用操作

```cpp
#include <mutex>

std::mutex mtx;

void flexible_locking() {
    // 方式1：立即加锁（和 lock_guard 类似）
    std::unique_lock<std::mutex> lock1(mtx);
    // 自动解锁
    
    // 方式2：延迟加锁
    std::unique_lock<std::mutex> lock2(mtx, std::defer_lock);
    lock2.lock();    // 手动加锁
    lock2.unlock();  // 手动解锁
    
    // 方式3：尝试加锁
    std::unique_lock<std::mutex> lock3(mtx, std::try_to_lock);
    if (lock3.owns_lock()) {
        // 成功获取锁
    } else {
        // 获取锁失败
    }
}
```

#### 5.4.3 lock_guard vs unique_lock 对比

| 特性 | lock_guard | unique_lock |
|------|-----------|-------------|
| 自动加锁 | ✓ | ✓ |
| 自动解锁 | ✓ | ✓ |
| 手动加锁/解锁 | ✗ | ✓ |
| 配合 condition_variable | ✗ | ✓ |
| 性能 | 更高 | 稍低 |
| 适用场景 | 简单临界区 | 复杂同步逻辑 |

**选择建议**：
- 简单场景：使用 `std::lock_guard`
- 需要条件变量：使用 `std::unique_lock`
- 需要手动控制：使用 `std::unique_lock`

### 5.5 std::condition_variable - 条件变量

#### 5.5.1 基本概念

`std::condition_variable` 用于线程间的通知机制：
- 允许线程等待某个条件成立
- 另一个线程可以通知等待的线程

**比喻**：
```
condition_variable 就像餐厅的叫号系统
- 顾客（消费者线程）没有号时在休息区等待
- 店员（生产者线程）叫号后通知顾客
- 顾客被叫到号后继续取餐
```

#### 5.5.2 核心方法

| 方法 | 作用 |
|------|------|
| `wait(lock)` | 阻塞等待通知 |
| `wait(lock, predicate)` | 等待条件成立（推荐） |
| `notify_one()` | 唤醒一个等待线程 |
| `notify_all()` | 唤醒所有等待线程 |

#### 5.5.3 基本用法

```cpp
#include <mutex>
#include <condition_variable>
#include <queue>

std::mutex mtx;
std::condition_variable cv;
std::queue<int> data_queue;
bool finished = false;

// 生产者线程
void producer() {
    for (int i = 0; i < 10; i++) {
        {
            std::lock_guard<std::mutex> lock(mtx);
            data_queue.push(i);
            std::cout << "生产数据: " << i << std::endl;
        }
        cv.notify_one();  // 通知消费者
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    {
        std::lock_guard<std::mutex> lock(mtx);
        finished = true;
    }
    cv.notify_one();  // 通知消费者结束
}

// 消费者线程
void consumer() {
    while (true) {
        std::unique_lock<std::mutex> lock(mtx);
        
        // 等待条件：队列不为空 或 生产结束
        cv.wait(lock, []{ return !data_queue.empty() || finished; });
        
        if (data_queue.empty() && finished) {
            break;  // 退出循环
        }
        
        int data = data_queue.front();
        data_queue.pop();
        std::cout << "消费数据: " << data << std::endl;
    }
}
```

#### 5.5.4 wait(lock, predicate) 工作原理

```cpp
cv.wait(lock, predicate);

// 等价于：
while (!predicate()) {
    lock.unlock();     // 解锁，让其他线程访问
    // 等待通知...
    // 被唤醒后
    lock.lock();       // 重新加锁
}
```

**为什么用 unique_lock 而不是 lock_guard？**
- `wait()` 内部需要解锁和重新加锁
- `lock_guard` 不支持手动解锁
- `unique_lock` 支持手动控制

### 5.6 生产者-消费者模式完整示例

#### 5.6.1 在电梯系统中的应用

```cpp
#include <mutex>
#include <condition_variable>
#include <queue>

class ElevatorActionServer {
private:
    // 共享数据
    std::queue<Request> request_queue_;
    std::mutex queue_mutex_;
    std::condition_variable cv_;
    bool running_;
    
public:
    // 生产者：接收请求
    void handle_accepted(const GoalHandle& goal_handle) {
        // 1. 创建请求
        Request req;
        req.initial_floor = goal_handle->get_goal()->initial_floor;
        req.target_floor = goal_handle->get_goal()->target_floor;
        req.direction = static_cast<Direction>(goal_handle->get_goal()->direction_to_go);
        req.goal_handle = goal_handle;
        
        // 2. 加锁并添加到队列
        {
            std::lock_guard<std::mutex> lock(queue_mutex_);
            request_queue_.push(req);
            std::cout << "新请求加入队列: " << req.initial_floor 
                      << "楼 -> " << req.target_floor << "楼" << std::endl;
        }  // 自动解锁
        
        // 3. 通知调度线程
        cv_.notify_one();
    }
    
    // 消费者：处理请求
    void schedule_loop() {
        std::cout << "调度线程已启动" << std::endl;
        
        while (running_) {
            Request req;
            
            // 1. 等待请求
            {
                std::unique_lock<std::mutex> lock(queue_mutex_);
                
                // 等待条件：队列不为空 或 停止运行
                cv_.wait(lock, [this]() {
                    return !request_queue_.empty() || !running_;
                });
                
                // 检查是否退出
                if (!running_ && request_queue_.empty()) {
                    break;
                }
                
                // 取出请求
                req = request_queue_.front();
                request_queue_.pop();
            }  // 自动解锁
            
            // 2. 处理请求（不需要锁）
            std::cout << "处理请求: " << req.initial_floor 
                      << "楼 -> " << req.target_floor << "楼" << std::endl;
            process_request(req);
        }
        
        std::cout << "调度线程已退出" << std::endl;
    }
    
    // 停止调度线程
    void stop() {
        {
            std::lock_guard<std::mutex> lock(queue_mutex_);
            running_ = false;
        }
        cv_.notify_all();  // 唤醒所有等待线程
    }
    
private:
    void process_request(const Request& req) {
        // 处理电梯请求...
    }
};
```

#### 5.6.2 工作流程图

```
handle_accepted()                    schedule_loop()
     |                                      |
     ↓                                      |
创建Request对象                             |
     |                                      |
     ↓                                      |
加锁（lock_guard）                          |
     |                                      |
添加到队列                                  |
     |                                      |
解锁（自动）                                |
     |                                      |
notify_one() ────────────────────→    wait(lock, predicate)
     |                                      |
     |                                 队列为空，等待中...
     |                                      |
     |                                  被唤醒
     |                                      |
     |                                 检查条件（队列不为空）
     |                                      |
     |                                 取出请求
     |                                      |
     |                                 解锁（自动）
     |                                      |
     |                                 处理请求
     |                                      |
     ↓                                      ↓
继续接收新请求                           循环等待下一个请求
```

### 5.7 注意事项和最佳实践

#### 5.7.1 避免死锁

```cpp
// 错误示例：可能导致死锁
std::mutex mtx1, mtx2;

void thread1() {
    std::lock_guard<std::mutex> lock1(mtx1);
    // ... 做一些操作
    std::lock_guard<std::mutex> lock2(mtx2);  // 可能死锁！
}

void thread2() {
    std::lock_guard<std::mutex> lock2(mtx2);
    // ... 做一些操作
    std::lock_guard<std::mutex> lock1(mtx1);  // 可能死锁！
}

// 正确方式：使用 std::lock 同时锁定多个互斥量
void safe_thread() {
    std::unique_lock<std::mutex> lock1(mtx1, std::defer_lock);
    std::unique_lock<std::mutex> lock2(mtx2, std::defer_lock);
    std::lock(lock1, lock2);  // 原子性地锁定两个互斥量
    // 自动解锁
}
```

#### 5.7.2 锁的粒度

```cpp
// 粒度过大（影响性能）
void process() {
    std::lock_guard<std::mutex> lock(mtx);
    // ... 大量计算
    save_to_database();  // 不需要锁
    // ... 更多计算
}

// 粒度合适
void process() {
    {
        std::lock_guard<std::mutex> lock(mtx);
        // 只保护共享资源访问
        shared_data++;
    }
    // 不需要锁的计算
    save_to_database();
}
```

#### 5.7.3 初始化顺序

```cpp
class ElevatorActionServer {
private:
    std::queue<Request> request_queue_;    // 1. 先声明队列
    std::mutex queue_mutex_;                 // 2. 再声明互斥锁
    std::condition_variable cv_;             // 3. 最后声明条件变量
    bool running_;                            // 4. 控制标志
    
public:
    ElevatorActionServer() 
        : running_(true)  // 初始化 running_ 为 true
    {
        // 启动调度线程
        std::thread t(&ElevatorActionServer::schedule_loop, this);
        t.detach();
    }
};
```

### 5.8 常见问题

#### Q1: 为什么 wait 需要放在循环中？

```cpp
// 错误：直接用 if
if (!condition) {
    cv.wait(lock);
}
// 问题：虚假唤醒，条件可能仍不成立

// 正确：用 while 或 wait(lock, predicate)
while (!condition) {
    cv.wait(lock);
}
// 或者
cv.wait(lock, []{ return condition; });
```

#### Q2: notify_one 还是 notify_all？

| 场景 | 选择 |
|------|------|
| 单个消费者 | `notify_one()` |
| 多个消费者，只唤醒一个 | `notify_one()` |
| 所有消费者都需要被唤醒 | `notify_all()` |
| 不确定 | `notify_all()`（更安全） |

#### Q3: 什么时候用 lock_guard，什么时候用 unique_lock？

```cpp
// 简单场景：使用 lock_guard
void simple_function() {
    std::lock_guard<std::mutex> lock(mtx);
    shared_data++;
}

// 复杂场景：使用 unique_lock
void complex_function() {
    std::unique_lock<std::mutex> lock(mtx);
    
    // 访问共享资源
    shared_data++;
    
    // 释放锁，做其他事情
    lock.unlock();
    do_something_else();
    
    // 重新加锁
    lock.lock();
    shared_data++;
}
```

### 5.9 多线程同步机制对比

| 机制 | 适用场景 | 特点 |
|------|----------|------|
| `std::mutex` | 保护临界区 | 最基础的锁 |
| `std::lock_guard` | 简单临界区 | 自动加锁解锁，推荐使用 |
| `std::unique_lock` | 复杂同步逻辑 | 手动控制，支持条件变量 |
| `std::condition_variable` | 线程间通信 | 等待/通知机制 |

---

## 6. C++ std::future 异步操作

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