# 题目二-Action通信与电梯调度系统

> Author: liuyuxin
> Ubuntu: 20.04
> ROS2: Foxy

---

## 目录

- [一、项目概述](#一项目概述)
- [二、Action接口设计](#二action接口设计)
- [三、任务点实现说明](#三任务点实现说明)
  - [任务点一：单次电梯接送客](#任务点一单次电梯接送客)
  - [任务点二：多次顺序接送客](#任务点二多次顺序接送客)
  - [任务点三：LOOK算法+并发请求](#任务点三look算法并发请求)
- [四、项目结构说明](#四项目结构说明)
- [五、文档阅读指南](#五文档阅读指南)
- [六、心路历程与总结](#六心路历程与总结)

---

## 一、项目概述

本项目使用ROS2 Action通信机制实现电梯调度系统，包含三个任务点的完整实现：

| 任务点 | 功能 | 调度策略 | 亮点 |
|--------|------|----------|------------|
| 任务点一 | 单次电梯接送客 | 简单直达 | 完善的错误处理、日志系统 |
| 任务点二 | 8次请求顺序处理 | 先来先服务(FCFS) | 随机楼层生成、数据合理性验证 |
| 任务点三 | 8次请求并发处理 | LOOK算法 | LOOK算法、异步架构、随机间隔时间、边界保护 |

### 系统基本参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `ground_floor` | 1 | 底层楼层, 可在config中修改 |
| `top_floor` | 10 | 顶层楼层, 可在config中修改 |
| `move_delay_ms` | 500 | 电梯移动一层所需时间(ms), 可在config中修改 |
| `open_door_time` | 1s | 开关门时间 |
| 请求间隔时间(V2) | 固定15秒 | 可在config中修改 |
| 请求间隔时间(V3) | 随机1-5秒 | 模拟真实乘客请求, 可在client中修改 |

---

## 二、Action接口设计

### Goal定义（客户端发送给服务端）

```action
# Goal - 乘客请求电梯的参数
int32 initial_floor    # 起始楼层
int32 target_floor     # 目标楼层
uint32 direction_to_go # 电梯运行方向(0=UP, 1=DOWN)
uint32 passenger_count # 乘客人数
```

### Result定义（任务完成后的结果）

```action
# Result - 任务完成后的结果
bool success           # 是否成功完成
int32 final_floor      # 最终所在楼层
string message         # 完成信息
```

### Feedback定义（执行过程中的实时反馈）

```action
# Feedback - 执行过程中的反馈信息
int32 current_floor    # 当前楼层
uint32 status          # 当前状态
uint32 direction       # 电梯运行方向
uint32 current_load    # 当前人数
```

---

## 三、任务点实现说明

### 任务点一：单次电梯接送客

#### 功能描述
模拟一次电梯接送客过程（1-10层），客户端发送固定请求，服务端处理单次接送任务。

#### 运行逻辑
1. 服务端启动，等待客户端请求
2. 客户端发送请求（固定：3楼→8楼）
3. 服务端接收请求，验证数据合理性
4. 电梯从1楼移动到3楼接乘客
5. 电梯从3楼移动到8楼送乘客
6. 全程实时反馈当前位置和状态

#### 验证命令

```bash
# 终端1 - 编译并启动服务端
colcon build --packages-select elevator_system
source ~/ros2_ws/install/setup.bash
ros2 run elevator_system elevator_server --ros-args --params-file config/elevator_params.yaml

# 终端2 - 启动客户端
source ~/ros2_ws/install/setup.bash
ros2 run elevator_system elevator_client
```

#### 亮点展示
- **完善的错误处理**：验证楼层合法性、方向合理性
- **类型安全**：使用枚举类`Direction`和`ElevatorStatus`
- **友元函数机制**：解决类内类型别名可见性问题
- **详细日志输出**：每一步操作都有清晰的日志记录

---

### 任务点二：多次顺序接送客

#### 功能描述
模拟电梯多次接送客过程（8次），乘客初始楼层和目标楼层由随机数生成，电梯按顺序处理请求。

#### 运行逻辑
1. 客户端每15秒发送一次请求，共8次
2. 服务端顺序处理每个请求，一个完成再处理下一个
3. 每次请求的楼层随机生成，保证数据合理性

#### 验证命令

```bash
# 终端1 - 服务端
source ~/ros2_ws/install/setup.bash
ros2 run elevator_system elevator_server_v2 --ros-args --params-file config/elevator_params.yaml

# 终端2 - 客户端
source ~/ros2_ws/install/setup.bash
ros2 run elevator_system elevator_client_v2
```

#### 亮点展示
- **随机楼层生成**：使用`std::mt19937`和`std::uniform_int_distribution`生成高质量随机数
- **数据合理性验证**：
  - 起始楼层 ≠ 目标楼层
  - 方向与楼层关系一致（UP时目标>起始，DOWN时目标<起始）
  - 楼层在合法范围内（1-10）
- **定时器机制**：使用`create_wall_timer`实现固定间隔请求

---

### 任务点三：LOOK算法+并发请求

#### 功能描述
模拟真实电梯调度场景，实现LOOK算法处理并发请求，乘客只提供起始楼层和方向，目标楼层在接客后随机生成。

#### 运行逻辑
1. 客户端随机间隔（1-5秒）发送请求(高并发请求模拟)，只发送起始楼层和方向
2. 服务端采用LOOK算法动态调度：
   - 电梯沿当前方向运行，顺路响应请求
   - 当前方向无任务时切换方向
   - 支持运行中接收新请求
3. 接到乘客后随机生成目标楼层（符合方向要求）
4. 支持多乘客同时在电梯内

#### LOOK算法核心
```
数据结构：
- up_stops_: 电梯往上走时需要停靠的楼层（按升序）
- down_stops_: 电梯往下走时需要停靠的楼层（按降序）

调度规则：
1. 添加请求：根据电梯位置与乘客位置关系决定加入哪个队列
2. 方向切换：当前方向无任务时检查反向队列
3. 停靠处理：pick up时生成目标楼层，drop off时发送result
```

#### 验证命令

```bash
# 终端1 - 服务端
source ~/ros2_ws/install/setup.bash
ros2 run elevator_system elevator_server_v3 --ros-args --params-file config/elevator_params.yaml

# 终端2 - 客户端
source ~/ros2_ws/install/setup.bash
ros2 run elevator_system elevator_client_v3
```

#### 亮点展示
- **LOOK算法实现**：贴近真实电梯调度，高效处理并发请求
- **随机间隔时间**：模拟真实乘客请求模式（额外加分项）
- **生产者-消费者模式**：使用`mutex`、`condition_variable`实现线程安全
- **边界保护机制**：双重保护防止电梯超出楼层范围
- **迭代器安全**：避免遍历容器时修改导致的崩溃

---

## 四、项目结构说明

### 目录结构

```
elevator_system/
├── action/
│   └── Elevator.action              # Action接口定义
├── config/
│   └── elevator_params.yaml          # 参数配置文件
├── include/elevator_system/
│   └── elevator_status.hpp           # 状态枚举和数据结构
├── src/
│   ├── elevator_server.cpp           # 任务点一服务端
│   ├── elevator_client.cpp           # 任务点一客户端
│   ├── elevator_server_v2.cpp        # 任务点二服务端
│   ├── elevator_client_v2.cpp        # 任务点二客户端
│   ├── elevator_server_v3.cpp        # 任务点三服务端（LOOK算法）
│   └── elevator_client_v3.cpp        # 任务点三客户端
├── action_learn.md                   # Action通信学习笔记
├── 任务规划.md                        # 项目问题记录与解决方案
├── CMakeLists.txt                    # 构建配置
├── package.xml                       # 包依赖配置
└── README.md                         # 项目说明文档
```

### CMakeLists.txt 关键配置

```cmake
# 依赖项
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Action接口生成
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Elevator.action"
)

# 链接自动生成的Action接口
target_link_libraries(elevator_server
  ${PROJECT_NAME}__rosidl_typesupport_cpp
)
```

### package.xml 依赖配置

```xml
<buildtool_depend>ament_cmake</buildtool_depend>
<buildtool_depend>rosidl_default_generators</buildtool_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
<depend>rclcpp_action</depend>
```

### 参数配置文件 (config/elevator_params.yaml)

```yaml
elevator_action_server:
  ros__parameters:
    ground_floor: 1
    top_floor: 10
    move_delay_ms: 500
```

---

## 五、文档阅读指南

### action_learn.md - Action通信学习笔记

本文档记录了Action通信的学习过程，包含：

| 章节 | 内容 |
|------|------|
| 第一节 | Action通信基础概念 |
| 第二节 | Goal、Feedback、Result的作用 |
| 第三节 | ROS2 Action编程实现 |
| 第四节 | 回调函数详解 |
| **第五节** | **C++多线程同步机制**（重点） |
| 第六节 | C++ std::future异步操作 |

**重点推荐**：第五节详细介绍了`mutex`、`lock_guard`、`unique_lock`、`condition_variable`的用法，包含生产者-消费者模式完整示例，是实现任务点三的必备知识。

### 任务规划.md - 项目问题记录

本文档记录了项目开发过程中遇到的大部分问题和解决方案（其实只记录了发生错误的40%， 都是比较典型和有学习价值的）：

- **第一至四章**：任务点一、二的问题记录
- **第五章**：任务点三 + LOOK算法问题记录（重点）

每个问题按以下格式记录：
- 问题描述
- 知识点
- 解决步骤
- 代码示例
- 最终结果

**推荐阅读顺序**：
1. 先阅读README了解项目整体结构
2. 遇到具体问题时查阅任务规划.md
3. 需要深入理解多线程时阅读action_learn.md第五节

---

## 六、心路历程与总结

### 开发过程回顾

#### 第一阶段：入门学习
- 学习Action通信的基本概念
- 理解Goal、Feedback、Result的作用
- 掌握ROS2 Action编程接口

#### 第二阶段：任务点一实现
- 实现单次电梯接送客
- 解决10+个编译问题（友元函数、类型别名、API兼容性等）
- 建立完善的日志和错误处理机制

#### 第三阶段：任务点二实现
- 实现多次请求顺序处理
- 添加随机楼层生成
- 使用定时器实现固定间隔请求

#### 第四阶段：任务点三挑战
- 学习C++多线程同步机制
- 实现LOOK算法核心逻辑
- 解决迭代器失效Bug（最严重）
- 添加边界保护机制
- 实现随机间隔时间

### 关键经验总结

1. **理解需求**：仔细阅读题目，区分不同任务点的差异
2. **数据结构先行**：先设计好数据结构，再实现逻辑
3. **迭代器安全**：遍历STL容器时避免修改容器
4. **日志调试**：使用RCLCPP_INFO帮助定位问题
5. **增量开发**：一步一步实现，每次修改后测试

### 未完成的想法

1. **更智能的调度算法**：实现C-SCAN、SCAN-EDF等算法
2. **多电梯协同**：支持多部电梯联合调度
3. **GUI可视化**：添加图形界面展示电梯运行状态
4. **泊松过程模拟**：更真实的请求间隔时间生成
5. **代码复用**：将三个版本共同部分提取为基类

### 技术收获

- 深入理解ROS2 Action通信机制
- 掌握C++多线程编程（mutex、condition_variable）
- 学习LOOK电梯调度算法
- 提升调试和问题解决能力
- 培养增量开发习惯

---

## 附录：常见问题排查

### Q1: 编译时找不到rclcpp_action
**解决方案**：确保package.xml中有`<depend>rclcpp_action</depend>`

### Q2: Action接口未定义
**解决方案**：先编译Action接口：`colcon build --packages-select elevator_system`

### Q3: 服务端无响应
**解决方案**：
1. 检查是否source环境：`source install/setup.bash`
2. 检查配置文件路径是否正确

### Q4: 电梯楼层变成负数
**解决方案**：已在V3版本添加边界保护机制

---

**项目完成时间**：2025年
**开发环境**：Ubuntu 20.04 + ROS2 Foxy
*补充*：该任务全开发流程记录存储在https://github.com/quetzal-china/elevator_system_for_ros2, 可以进行验证.