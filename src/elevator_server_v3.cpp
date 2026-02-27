#include <rclcpp/rclcpp.hpp>                    // ROS2核心库
#include <rclcpp_action/rclcpp_action.hpp>      // Action通信库
#include <thread>                               // 线程库
#include <chrono>                               // 时间库
#include "elevator_system/action/elevator.hpp"  // 自定义Action接口
#include "elevator_system/elevator_status.hpp"  // 电梯状态枚举
// LOOK算法所需头文件
#include <queue>
#include <mutex>
#include <condition_variable>
#include <vector>
#include <algorithm>
#include <cstdlib> 
#include <memory>

/* #define GROUND_FLOOR 1
#define TOP_FLOOR 10
#define MOVE_DELAY_MS 500 */

class ElevatorActionServer : public rclcpp::Node    // 继承自ROS2节点
{
public:

    int ground_floor_;
    int top_floor_;
    int move_delay_ms_;

    // 定义类型别名
    using Elevator = elevator_system::action::Elevator;
    using GoalHandleElevator = rclcpp_action::ServerGoalHandle<Elevator>;

    // 构造函数
    ElevatorActionServer();

private:
    // Action 服务器
    rclcpp_action::Server<Elevator>::SharedPtr action_server_;

    // 电梯状态变量
    int current_floor_;          // 当前楼层
    ElevatorStatus status_;      // 电梯状态
    int passenger_count_;        // 乘客数量

    // 三个回调函数
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const Elevator::Goal> goal);

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleElevator> goal_handle);

    void handle_accepted(
        const std::shared_ptr<GoalHandleElevator> goal_handle);

    // 添加包装函数的友元声明, 用于在execute中调用, 否则会报错!
    friend void execute_wrapper(
        ElevatorActionServer* instance, 
        std::shared_ptr<rclcpp_action::ServerGoalHandle<elevator_system::action::Elevator>> handle);

    // 执行任务的函数 处理核心逻辑
    void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<elevator_system::action::Elevator>> goal_handle);

    // LOOK算法相关成员变量
    // 请求队列与同步
    std::queue<Request> request_queue_;         // 存放待处理的电梯请求队列
    std::mutex queue_mutex_;                    // 互斥锁，防止多个线程同时访问队列导致数据竞争
    std::condition_variable cv_;                // 条件变量，让消费者线程在队列为空时休眠等待，有新请求时唤醒
    bool running_;                              // 控制调度循环是否继续运行，用于优雅退出

    // 停靠算法
    std::vector<StopInfo> up_stops_;  // 上行停靠队列
    std::vector<StopInfo> down_stops_;  // 下行停靠队列
    Direction current_direction_;  // 当前电梯运动方向

    //电梯内乘客
    std::vector<Request> onboard_passengers_;

    // LOOK算法所需要的核心调度函数, 打 * 号完成了
    void schedule_loop();// *
    void run_elevator();
    void add_request_to_queue(const Request& request);
    void process_new_requests();// *
    void add_to_stop_plan(const Request& req);// *
    void move_one_floor();
    bool need_stop(int floor);
    void handle_stop(int floor);
    bool has_pending_stops();// *
    Direction get_initial_direction();// *
};

// 实现handle_goal函数 - 验证并接受请求
// ElevatorActionServer类的handle_goal函数，返回一个GoalResponse类型的结果
// 注意: v3版本在接受请求时不会收到目标楼层!
rclcpp_action::GoalResponse ElevatorActionServer::handle_goal(
    const rclcpp_action::GoalUUID &uuid,
    std::shared_ptr<const Elevator::Goal> goal)
{
    RCLCPP_INFO(this->get_logger(), "Passenger at Floor %d, pressing %s", 
        goal->initial_floor,
        goal->direction_to_go == static_cast<uint32_t>(Direction::DIRECTION_UP) ? "🔼 UP" : "🔽 DOWN");
    // 验证楼层是否合法 (1-10)
    if (goal->initial_floor < ground_floor_ || 
        goal->initial_floor > top_floor_)
    {
        RCLCPP_WARN(this->get_logger(), "Invalid floor!");
        return rclcpp_action::GoalResponse::REJECT;
    }
    // 验证方向是否合法
    if (goal->direction_to_go != static_cast<uint32_t>(Direction::DIRECTION_UP) &&
        goal->direction_to_go != static_cast<uint32_t>(Direction::DIRECTION_DOWN))
    {
        RCLCPP_WARN(this->get_logger(), "Invalid direction!");
        return rclcpp_action::GoalResponse::REJECT;
    }
    // 特殊情况：在顶层按UP或在底层按DOWN
    if ((goal->initial_floor == top_floor_ && goal->direction_to_go == static_cast<uint32_t>(Direction::DIRECTION_UP)) ||
        (goal->initial_floor == ground_floor_ && goal->direction_to_go == static_cast<uint32_t>(Direction::DIRECTION_DOWN)))
    {
        RCLCPP_WARN(this->get_logger(), "Invalid request: cannot go that direction from this floor!");
        return rclcpp_action::GoalResponse::REJECT;
    }
    RCLCPP_INFO(this->get_logger(), "Goal accepted by server...");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

// 实现handle_cancel函数 - 处理取消请求
// ElevatorActionServer类的成员函数handle_cancel，返回CancelResponse类型"
rclcpp_action::CancelResponse ElevatorActionServer::handle_cancel(
    const std::shared_ptr<GoalHandleElevator> goal_handle
)
{
    RCLCPP_INFO(this->get_logger(), "Cancel request received");
    return rclcpp_action::CancelResponse::ACCEPT;
}

/* // 使用包装函数(因为本人觉得包装函数的方法比lambda表达式和std::bind()函数更清晰)
void execute_wrapper(
    ElevatorActionServer* instance, 
    std::shared_ptr<rclcpp_action::ServerGoalHandle<elevator_system::action::Elevator>> handle)
    {
        instance->execute(handle);
    } */


// ==================== 生产者：接收请求 ====================
// 实现handle_accepted函数
// NOTICE! V3版本的此处逻辑发生变化 不再直接启动execute线程, 而是把请求加入队列, 这是LOOK算法核心
void ElevatorActionServer::handle_accepted(
    const std::shared_ptr<GoalHandleElevator> goal_handle)
{
    // 1. 创建Request对象
    Request req;
    req.initial_floor = goal_handle->get_goal()->initial_floor;
    req.direction = static_cast<Direction>(goal_handle->get_goal()->direction_to_go);
    req.goal_handle = goal_handle;
    
    // 2. 加锁，添加到队列
    {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        request_queue_.push(req);
        RCLCPP_INFO(this->get_logger(), "新请求已加入队列: %d楼, 方向: %s", 
                    req.initial_floor,
                    req.direction == Direction::DIRECTION_UP ? "🔼 UP" : "🔽 DOWN");
    }  // 自动解锁
    
    // 3. 通知调度线程有新请求
    cv_.notify_one();
}

// ==================== 消费者：调度循环 ====================
void ElevatorActionServer::schedule_loop()
{
    RCLCPP_INFO(this->get_logger(), "调度线程已启动");
    
    while (running_) {
        // 1. 处理队列中所有新请求(非阻塞)
        process_new_requests();
        
        // 2. 如果有任务, 执行一步
        if (has_pending_stops()) 
        {
            // 边界保护
            if (current_floor_ >= top_floor_ && current_direction_ == Direction::DIRECTION_UP) {
                current_direction_ = Direction::DIRECTION_DOWN;
                RCLCPP_WARN(this->get_logger(), "到达顶层，强制切换方向为 DOWN");
            }
            if (current_floor_ <= ground_floor_ && current_direction_ == Direction::DIRECTION_DOWN) {
                current_direction_ = Direction::DIRECTION_UP;
                RCLCPP_WARN(this->get_logger(), "到达底层，强制切换方向为 UP");
            }

            // 确定方向
            if (current_direction_ == Direction::DIRECTION_IDLE) {
                current_direction_ = get_initial_direction();
            }
            
            // 移动一层
            move_one_floor();
            // 检查是否需要停靠
            if (need_stop(current_floor_))
            {
                handle_stop(current_floor_);
            }
            // LOOK 算法: 判断是否需要改变方向
            auto& current_stops = (current_direction_ == Direction::DIRECTION_UP) ? up_stops_ : down_stops_;
            if (current_stops.empty())
            {   
                // 当前方向无任务，检查反向
                auto& opposite_stops = (current_direction_ == Direction::DIRECTION_UP) ? down_stops_ : up_stops_;
                if (!opposite_stops.empty())
                {
                    // 有反向任务，改变方向
                    current_direction_ = (current_direction_ == Direction::DIRECTION_UP) ? 
                                            Direction::DIRECTION_DOWN : Direction::DIRECTION_UP;
                    RCLCPP_INFO(this->get_logger(), "方向切换: %s", 
                                current_direction_ == Direction::DIRECTION_UP ? "🔼 UP" : "🔽 DOWN");
                }
                else
                {
                    // 正向反向都没有请求了, 空闲
                    current_direction_ = Direction::DIRECTION_IDLE;
                    status_ = ElevatorStatus::STATUS_IDLE;
                    RCLCPP_INFO(this->get_logger(), "电梯空闲，等待新请求...");
                }
            }
            
            // 控制电梯移动速度
            std::this_thread::sleep_for(std::chrono::milliseconds(move_delay_ms_));
        }
        else
        {
            // 无任务时, 短暂休眠0.1s, 释放CPU
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
    
    RCLCPP_INFO(this->get_logger(), "调度线程已退出");
}

// 构造函数实现
ElevatorActionServer::ElevatorActionServer() : Node("elevator_action_server_v3")
{
    RCLCPP_INFO(this->get_logger(), "电梯 Action 服务器 V3 已启动");

    // 从参数服务器获取参数
    ground_floor_ = this->declare_parameter<int>("ground_floor", 1);
    top_floor_ = this->declare_parameter<int>("top_floor", 10);
    move_delay_ms_ = this->declare_parameter<int>("move_delay_ms", 500);

    // 初始化电梯状态
    current_floor_ = ground_floor_;  // 电梯初始在1楼
    passenger_count_ = 0;  // 初始没有乘客
    status_ = ElevatorStatus::STATUS_IDLE;  // 空闲状态

    // 初始化 look 算法相关成员
    current_direction_ = Direction::DIRECTION_IDLE;     // 当前电梯运动方向初始为空闲
    running_ = true;                                    // 调度线程运行标志

    // 创建Action服务器
    this->action_server_ = rclcpp_action::create_server<Elevator>(
        this,                    // 当前节点
        "elevator",              // Action名称
        std::bind(&ElevatorActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&ElevatorActionServer::handle_cancel, this, std::placeholders::_1),
        std::bind(&ElevatorActionServer::handle_accepted, this, std::placeholders::_1));

    // 只启动一次的调度线程!!!
    std::thread schedule_thread(&ElevatorActionServer::schedule_loop, this);
    schedule_thread.detach();

    RCLCPP_INFO(this->get_logger(), "电梯Action服务器已启动");
}

// 处理队列中所有新请求
void ElevatorActionServer::process_new_requests()
{
    // 上锁
    std::lock_guard<std::mutex> lock(queue_mutex_);

    while (!request_queue_.empty())
    {
        Request req = request_queue_.front();
        request_queue_.pop();

        RCLCPP_INFO(this->get_logger(), "处理新请求: %d楼, 方向: %s", 
                    req.initial_floor,
                    req.direction == Direction::DIRECTION_UP ? "🔼 UP" : "🔽 DOWN");

        add_to_stop_plan(req);
    }


}   // 结束自动解锁 RALL

// 将请求添加到停靠计划
void ElevatorActionServer::add_to_stop_plan(const Request& req)
{
    // 创建 pick up 停靠信息
    StopInfo stop(req.initial_floor, true, -1, -1, req.direction, req.goal_handle);
    
    // 关键改变：根据电梯当前位置与乘客位置的关系决定加入哪个队列
    // 而不是根据乘客想去的方向！
    
    if (req.initial_floor > current_floor_) {
        // 乘客在电梯上方，电梯需要往上走才能接到
        up_stops_.push_back(stop);
        std::sort(up_stops_.begin(), up_stops_.end(), 
            [](const StopInfo& a, const StopInfo& b) { return a.floor < b.floor; });
        RCLCPP_INFO(this->get_logger(), "添加上行停靠(接客): %d楼", req.initial_floor);
    } else if (req.initial_floor < current_floor_) {
        // 乘客在电梯下方，电梯需要往下走才能接到
        down_stops_.push_back(stop);
        std::sort(down_stops_.begin(), down_stops_.end(), 
            [](const StopInfo& a, const StopInfo& b) { return a.floor > b.floor; });
        RCLCPP_INFO(this->get_logger(), "添加下行停靠(接客): %d楼", req.initial_floor);
    } else {
        // 乘客就在当前楼层，根据乘客想去的方向决定
        if (req.direction == Direction::DIRECTION_UP) {
            up_stops_.push_back(stop);
            std::sort(up_stops_.begin(), up_stops_.end(), 
                [](const StopInfo& a, const StopInfo& b) { return a.floor < b.floor; });
        } else {
            down_stops_.push_back(stop);
            std::sort(down_stops_.begin(), down_stops_.end(), 
                [](const StopInfo& a, const StopInfo& b) { return a.floor > b.floor; });
        }
        RCLCPP_INFO(this->get_logger(), "电梯当前楼层接客: %d楼", req.initial_floor);
    }
}
/* void ElevatorActionServer::add_to_stop_plan(const Request& req)
{
    // 新对象, pick up
    // target_floor 暂时未知，initial_floor 暂时无用
    StopInfo stop(req.initial_floor, true, -1, -1, req.goal_handle);

    // 添加到对应方向的停靠队列
    if (req.direction == Direction::DIRECTION_UP)
    {
        up_stops_.push_back(stop);

        // 按楼层升序排序(向上是从小到大)
        std::sort(up_stops_.begin(), up_stops_.end(), 
                  [](const StopInfo& a, const StopInfo& b) { return a.floor < b.floor; });
        RCLCPP_INFO(this->get_logger(), "添加上行停靠: %d楼", req.initial_floor);

    }
    else if (req.direction == Direction::DIRECTION_DOWN)
    {
        down_stops_.push_back(stop);

        // 按楼层降序排序(向下是从大到小)
        std::sort(down_stops_.begin(), down_stops_.end(), 
                  [](const StopInfo& a, const StopInfo& b) { return a.floor > b.floor; });
        RCLCPP_INFO(this->get_logger(), "添加下行停靠: %d楼", req.initial_floor);
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "Invalid direction!");
    }

} */

// 判断是否有待处理的停靠任务
bool ElevatorActionServer::has_pending_stops()
{
    return !up_stops_.empty() || !down_stops_.empty();
}

// 获取初始运行方向
Direction ElevatorActionServer::get_initial_direction()
{
    if (up_stops_.empty() && down_stops_.empty())
        return Direction::DIRECTION_IDLE;
    
    int nearest_up = -1;   // 上行队列中最近的楼层
    int nearest_down = -1;  // 下行队列中最近的楼层
    
    // 找 up_stops_ 中距离电梯最近的楼层
    if (!up_stops_.empty()) {
        // up_stops_ 按升序排序，需要找到第一个 >= current_floor_ 的元素
        // 或最后一个 < current_floor_ 的元素
        for (const auto& stop : up_stops_) {
            if (stop.floor >= current_floor_) {
                nearest_up = stop.floor;
                break;
            }
        }
        // 如果所有楼层都在电梯下方，取最后一个（最大的）
        if (nearest_up == -1 && !up_stops_.empty()) {
            nearest_up = up_stops_.back().floor;
        }
    }
    
    // 找 down_stops_ 中距离电梯最近的楼层
    if (!down_stops_.empty()) {
        // down_stops_ 按降序排序
        for (const auto& stop : down_stops_) {
            if (stop.floor <= current_floor_) {
                nearest_down = stop.floor;
                break;
            }
        }
        // 如果所有楼层都在电梯上方，取最后一个（最小的）
        if (nearest_down == -1 && !down_stops_.empty()) {
            nearest_down = down_stops_.back().floor;
        }
    }
    
    // 比较哪个更近
    if (nearest_up == -1 && nearest_down == -1) {
        return Direction::DIRECTION_IDLE;
    } else if (nearest_up == -1) {
        return (nearest_down > current_floor_) ? Direction::DIRECTION_UP : Direction::DIRECTION_DOWN;
    } else if (nearest_down == -1) {
        return (nearest_up > current_floor_) ? Direction::DIRECTION_UP : Direction::DIRECTION_DOWN;
    } else {
        return (std::abs(nearest_up - current_floor_) <= std::abs(nearest_down - current_floor_)) 
               ? Direction::DIRECTION_UP : Direction::DIRECTION_DOWN;
    }
}
/* Direction ElevatorActionServer::get_initial_direction()
{
    // 判断电梯需要去哪个方向接第一个乘客
    
    if (up_stops_.empty() && down_stops_.empty())
        return Direction::DIRECTION_IDLE;
    
    // 找到最近的停靠点（无论是UP还是DOWN）
    int nearest_floor = -1;
    Direction nearest_direction = Direction::DIRECTION_IDLE;
    
    if (!up_stops_.empty()) 
    {
        nearest_floor = up_stops_.front().floor;
        nearest_direction = Direction::DIRECTION_UP;
    }
    
    if (!down_stops_.empty()) 
    {
        if (nearest_floor == -1 || 
            std::abs(down_stops_.front().floor - current_floor_) < std::abs(nearest_floor - current_floor_)) 
            {
            nearest_floor = down_stops_.front().floor;
            nearest_direction = Direction::DIRECTION_DOWN;
        }
    }
    
    // 根据最近停靠点相对于当前电梯的位置决定方向
    if (nearest_floor > current_floor_)
        return Direction::DIRECTION_UP;
    else if (nearest_floor < current_floor_)
        return Direction::DIRECTION_DOWN;
    else
        return nearest_direction;  // 就在同一楼层
} */

// 移动一层
void ElevatorActionServer::move_one_floor()
{
    // 边界检查：如果到达边界，停止移动
    if (current_direction_ == Direction::DIRECTION_UP && current_floor_ >= top_floor_) 
    {
        RCLCPP_WARN(this->get_logger(), "已到达顶层 %d 楼，停止向上移动", top_floor_);
        current_direction_ = Direction::DIRECTION_IDLE;
        return;
    }
    if (current_direction_ == Direction::DIRECTION_DOWN && current_floor_ <= ground_floor_) 
    {
        RCLCPP_WARN(this->get_logger(), "已到达底层 %d 楼，停止向下移动", ground_floor_);
        current_direction_ = Direction::DIRECTION_IDLE;
        return;
    }

    // 更新楼层
    if (current_direction_ == Direction::DIRECTION_UP)
    {
        current_floor_++;
        status_ = ElevatorStatus::STATUS_MOVING_UP;
    }
    else if (current_direction_ == Direction::DIRECTION_DOWN)
    {
        current_floor_--;
        status_ = ElevatorStatus::STATUS_MOVING_DOWN;
    }

    // 发布反馈给所有在电梯内的乘客
    for (const auto& stop : up_stops_) 
    {
        if (stop.goal_handle && !stop.is_pickup) 
        {
            auto feedback = std::make_shared<Elevator::Feedback>();
            feedback->current_floor = static_cast<uint32_t>(current_floor_);
            feedback->status = static_cast<uint32_t>(status_);
            feedback->current_load = static_cast<uint32_t>(passenger_count_);
            feedback->direction = static_cast<uint32_t>(current_direction_);
            stop.goal_handle->publish_feedback(feedback);
        }
    }
    for (const auto& stop : down_stops_) 
    {
        if (stop.goal_handle && !stop.is_pickup) 
        {
            auto feedback = std::make_shared<Elevator::Feedback>();
            feedback->current_floor = static_cast<uint32_t>(current_floor_);
            feedback->status = static_cast<uint32_t>(status_);
            feedback->current_load = static_cast<uint32_t>(passenger_count_);
            feedback->direction = static_cast<uint32_t>(current_direction_);
            stop.goal_handle->publish_feedback(feedback);
        }
    }

    RCLCPP_INFO(this->get_logger(), "[Feedback] Floor:%d | Status: %s | Passengers: %d",
                current_floor_,
                current_direction_ == Direction::DIRECTION_UP ? "Moving UP" : "Moving DOWN",
                passenger_count_);

}

// 判断当前楼层是否需要停靠
bool ElevatorActionServer::need_stop(int floor)
{
    auto& current_stops = (current_direction_ == Direction::DIRECTION_UP) ? up_stops_ : down_stops_;

    for (const auto& stop : current_stops)
    {
        if (stop.floor == floor)
        {
            return true;
        }
    }
    return false;
}

// 处理停靠操作
void ElevatorActionServer::handle_stop(int floor)
{
    auto& current_stops = (current_direction_ == Direction::DIRECTION_UP) ? up_stops_ : down_stops_;

    // 存储需要添加的 drop off 任务，避免在遍历中修改容器导致迭代器失效
    std::vector<std::pair<StopInfo, Direction>> new_dropoffs;

    // 遍历当前楼层的所有停靠任务
    for (auto it = current_stops.begin(); it != current_stops.end(); )
    {
        if (it->floor == floor)
        {
            if (it->is_pickup)
            {
                // PICK UP
                passenger_count_++;

                // 随机生成目标楼层               
                int target_floor;
                // 使用乘客想去的方向，而不是电梯方向！
                if (it->direction == Direction::DIRECTION_UP) 
                {
                    // 乘客想往上，目标楼层 > 当前楼层
                    target_floor = floor + (rand() % (top_floor_ - floor)) + 1;
                } 
                else 
                {
                    // 乘客想往下，目标楼层 < 当前楼层
                    target_floor = ground_floor_ + (rand() % (floor - ground_floor_));
                }
                RCLCPP_INFO(this->get_logger(), "乘客目标楼层: %d 楼", target_floor);

                // 计算 drop off 方向
                Direction dropoff_direction = (target_floor > floor) ? Direction::DIRECTION_UP : Direction::DIRECTION_DOWN;
                
                // 先存储 drop off 任务，遍历结束后再添加
                new_dropoffs.push_back({StopInfo(target_floor, false, -1, floor, dropoff_direction, it->goal_handle), dropoff_direction});

                auto feedback = std::make_shared<Elevator::Feedback>();
                feedback->current_floor = current_floor_;
                feedback->status = static_cast<uint32_t>(ElevatorStatus::STATUS_PICKUP);
                feedback->current_load = passenger_count_;
                feedback->direction = static_cast<uint32_t>(current_direction_);
                it->goal_handle->publish_feedback(feedback);

                // 发布 feedback
                feedback->current_floor = static_cast<uint32_t>(current_floor_);
                feedback->status = static_cast<uint32_t>(ElevatorStatus::STATUS_PICKUP);
                feedback->current_load = static_cast<uint32_t>(passenger_count_);
                it->goal_handle->publish_feedback(feedback);
                RCLCPP_INFO(this->get_logger(), "[Feedback] Floor:%d | Status: Arrived | Passengers: %d | Dir: %s",
                            current_floor_,
                            passenger_count_,
                            (current_direction_ == Direction::DIRECTION_UP) ? "UP" : "DOWN");

                // 开门时间
                std::this_thread::sleep_for(std::chrono::seconds(1));
                
            }
            else
            {
                // DROP OFF
                passenger_count_--;

                RCLCPP_INFO(this->get_logger(), "电梯在 %d 楼下乘客", floor);
                status_ = ElevatorStatus::STATUS_DROPOFF;

                // 发送 feedback
                auto feedback = std::make_shared<Elevator::Feedback>();
                feedback->current_floor = current_floor_;
                feedback->status = static_cast<uint32_t>(ElevatorStatus::STATUS_DROPOFF);
                feedback->current_load = passenger_count_;
                feedback->direction = static_cast<uint32_t>(current_direction_);
                it->goal_handle->publish_feedback(feedback);
                RCLCPP_INFO(this->get_logger(), "[Feedback] Floor:%d | Status: Arrived | Passengers: %d | Dir: %s",
                            current_floor_,
                            passenger_count_,
                            (current_direction_ == Direction::DIRECTION_UP) ? "UP" : "DOWN");

                // 开门时间
                std::this_thread::sleep_for(std::chrono::seconds(1));

                // 发送 result（任务完成）
                auto result = std::make_shared<Elevator::Result>();
                result->success = true;
                result->final_floor = current_floor_;
                result->message = "Task completed successfully";
                it->goal_handle->succeed(result);

                RCLCPP_INFO(this->get_logger(), "[Result] Request Done!");
                RCLCPP_INFO(this->get_logger(), "Success: Successfully dropped off at floor %d (pickup from %d)",
                    current_floor_,
                    it->initial_floor);
                RCLCPP_INFO(this->get_logger(), "Target floor: %d", floor);

            }
            it = current_stops.erase(it);
        }
        else
        {
            ++it;
        }
        
    }

    // 遍历结束后，再添加 drop off 任务
    for (const auto& dropoff_pair : new_dropoffs)
    {
        const StopInfo& dropoff = dropoff_pair.first;
        Direction dropoff_direction = dropoff_pair.second;
        auto& dropoff_stops = (dropoff_direction == Direction::DIRECTION_UP) ? up_stops_ : down_stops_;
        dropoff_stops.push_back(dropoff);
    }

    // 统一排序
    std::sort(up_stops_.begin(), up_stops_.end(), 
        [](const StopInfo& a, const StopInfo& b) { return a.floor < b.floor; });
    std::sort(down_stops_.begin(), down_stops_.end(), 
        [](const StopInfo& a, const StopInfo& b) { return a.floor > b.floor; });
}

/* // execute函数实现
void ElevatorActionServer::execute(const std::shared_ptr<GoalHandleElevator> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Executing goal...");

    // 获取目标信息
    auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<Elevator::Feedback>();
    auto result = std::make_shared<Elevator::Result>();
    // 记录开始时间
    auto start_time = std::chrono::steady_clock::now();

    int initial_floor = static_cast<int>(goal->initial_floor);
    int target_floor = static_cast<int>(goal->target_floor);
    uint32_t direction_to_go = goal->direction_to_go;
    feedback->direction = goal->direction_to_go;

    // 步骤1: 移动到乘客所在楼层
    RCLCPP_INFO(this->get_logger(), "正在前往 %d 楼接乘客...", initial_floor);
    status_ = (initial_floor > current_floor_) ? ElevatorStatus::STATUS_MOVING_UP : ElevatorStatus::STATUS_MOVING_DOWN;

    while (current_floor_ != initial_floor && rclcpp::ok())
    {
        // 检查是否收到了取消请求
        if (goal_handle->is_canceling())
        {
            result->success = false;
            result->final_floor = static_cast<uint32_t>(current_floor_);
            result->message = "Goal canceled by client";
            goal_handle->canceled(result);
            RCLCPP_INFO(this->get_logger(), "Goal canceled");
            return;
        }

        // 否则正常执行移动
        if (initial_floor > current_floor_)
        {
            current_floor_++;
        }
        else
        {
            current_floor_--;
        }

        // 发布反馈
        feedback->current_floor = static_cast<uint32_t>(current_floor_);
        feedback->status = static_cast<uint32_t>((current_floor_ < initial_floor) ? ElevatorStatus::STATUS_MOVING_UP : ElevatorStatus::STATUS_MOVING_DOWN);
        feedback->current_load = static_cast<uint32_t>(passenger_count_);
        goal_handle->publish_feedback(feedback);
        RCLCPP_INFO(this->get_logger(), "[Feedback] Floor:%d | Status: Moving %s to %d (pickup) | Passengers: %d | Dir: %s",
                    current_floor_,
                    static_cast<Direction>(feedback->direction) == Direction::DIRECTION_UP ? "UP" : "DOWN",
                    initial_floor,
                    feedback->current_load,
                    static_cast<Direction>(feedback->direction) == Direction::DIRECTION_UP ? "UP" : "DOWN");  
        // 等待
        std::this_thread::sleep_for(std::chrono::milliseconds(move_delay_ms_));
    }

    // 退出while循环说明到达乘客出发楼层, 或者电梯被取消, 或者程序意外中断
    // 步骤2: 开关门接乘客
    status_ = ElevatorStatus::STATUS_PICKUP;
    // 增加乘客数量
    passenger_count_++;
    // 发布反馈
    feedback->current_floor = static_cast<uint32_t>(current_floor_);
    feedback->status = static_cast<uint32_t>(ElevatorStatus::STATUS_PICKUP);
    feedback->current_load = static_cast<uint32_t>(passenger_count_);
    goal_handle->publish_feedback(feedback);
    RCLCPP_INFO(this->get_logger(), "[Feedback] Floor:%d | Status: Arrived | Passengers: %d | Dir: %s",
                current_floor_,
                passenger_count_,
                static_cast<Direction>(feedback->direction) == Direction::DIRECTION_UP ? "UP" : "DOWN");
    std::this_thread::sleep_for(std::chrono::seconds(1));
    feedback->status = static_cast<uint32_t>(ElevatorStatus::STATUS_ARRIVED);
    goal_handle->publish_feedback(feedback);
    RCLCPP_INFO(this->get_logger(), "[Feedback] Floor:%d | Status: Pickup Done | Passengers: %d | Dir: %s",
                current_floor_,
                passenger_count_,
                (static_cast<Direction>(feedback->direction) == Direction::DIRECTION_UP) ? "UP" : "DOWN");  
    
    // 步骤3: 移动到目标楼层
    RCLCPP_INFO(this->get_logger(), "正在前往 %d 楼送乘客...", target_floor);
    status_ = (target_floor > current_floor_) ? ElevatorStatus::STATUS_MOVING_UP : ElevatorStatus::STATUS_MOVING_DOWN;
    // 移动逻辑
    while (current_floor_ != target_floor && rclcpp::ok())
    {
        // 检查是否收到了取消请求
        if (goal_handle->is_canceling())
        {
            result->success = false;
            result->final_floor = static_cast<uint32_t>(current_floor_);
            result->message = "Goal canceled by client";
            goal_handle->canceled(result);
            RCLCPP_INFO(this->get_logger(), "Goal canceled");
            return;
        }
        // 否则正常执行移动
        if (target_floor > current_floor_)
        {
            current_floor_++;
        }
        else
        {
            current_floor_--;
        }

        // 发布反馈
        feedback->current_floor = static_cast<uint32_t>(current_floor_);
        feedback->status = static_cast<uint32_t>(((current_floor_ < target_floor) ? ElevatorStatus::STATUS_MOVING_UP : ElevatorStatus::STATUS_MOVING_DOWN));
        feedback->current_load = static_cast<uint32_t>(passenger_count_);
        goal_handle->publish_feedback(feedback);
        RCLCPP_INFO(this->get_logger(), "[Feedback] Floor:%d | Status: Moving %s to %d (dropoff) | Passengers: %d | Dir: %s",
                    current_floor_,
                    static_cast<Direction>(feedback->direction) == Direction::DIRECTION_UP ? "UP" : "DOWN",
                    target_floor,
                    passenger_count_,
                    static_cast<Direction>(feedback->direction) == Direction::DIRECTION_UP ? "UP" : "DOWN");  
        // 等待
        std::this_thread::sleep_for(std::chrono::milliseconds(move_delay_ms_));
    }

    // 到达目标楼层. 开关门接乘客
    RCLCPP_INFO(this->get_logger(), "开关门...");
    status_ = ElevatorStatus::STATUS_DROPOFF;
    std::this_thread::sleep_for(std::chrono::seconds(1));
    passenger_count_--;
    // 发布反馈
    feedback->current_floor = static_cast<uint32_t>(current_floor_);
    feedback->status = static_cast<uint32_t>(ElevatorStatus::STATUS_DROPOFF);
    feedback->current_load = static_cast<uint32_t>(passenger_count_);
    goal_handle->publish_feedback(feedback);
    RCLCPP_INFO(this->get_logger(), "[Feedback] Floor:%d | Status: Arrived | Passengers: %d | Dir: %s",
                current_floor_,
                passenger_count_,
                static_cast<Direction>(feedback->direction) == Direction::DIRECTION_UP ? "UP" : "DOWN");
    std::this_thread::sleep_for(std::chrono::seconds(1));
    feedback->status = static_cast<uint32_t>(ElevatorStatus::STATUS_ARRIVED);
    goal_handle->publish_feedback(feedback);
    RCLCPP_INFO(this->get_logger(), "[Feedback] Floor:%d | Status: Dropping off at %d | Passengers: %d | Dir: %s",
                current_floor_,
                target_floor,
                passenger_count_,
                static_cast<Direction>(feedback->direction) == Direction::DIRECTION_UP ? "UP" : "DOWN");

    // 成功完成任务
    status_ = ElevatorStatus::STATUS_IDLE;
    result->success = true;
    result->final_floor = static_cast<uint32_t>(current_floor_);
    result->message = "Task completed successfully";
    goal_handle->succeed(result);
    // 计算总时间
    auto end_time = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::seconds>(end_time - start_time);

    RCLCPP_INFO(this->get_logger(), "[Result] Request Done!");
    RCLCPP_INFO(this->get_logger(), "Success: Successfully dropped off at floor %d (pickup from %d)",
                target_floor,
                initial_floor);
    RCLCPP_INFO(this->get_logger(), "Target floor: %d", target_floor);
    RCLCPP_INFO(this->get_logger(), "Total time: %d s", 
                duration.count()); 

    
} */

int main(int argc, char **argv)
{
    try {
        rclcpp::init(argc, argv);
        auto node = std::make_shared<ElevatorActionServer>();
        rclcpp::spin(node);
        rclcpp::shutdown();
        return 0;
    } catch (const std::exception &e) {
        std::cerr << "Error: " << e.what() << std::endl;
        rclcpp::shutdown();
        return 1;
    }
}