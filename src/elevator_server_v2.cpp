#include <rclcpp/rclcpp.hpp>                    // ROS2核心库
#include <rclcpp_action/rclcpp_action.hpp>      // Action通信库
#include <thread>                               // 线程库
#include <chrono>                               // 时间库
#include "elevator_system/action/elevator.hpp"  // 自定义Action接口
#include "elevator_system/elevator_status.hpp"  // 电梯状态枚举

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

};

// 实现handle_goal函数 - 验证并接受请求
// ElevatorActionServer类的handle_goal函数，返回一个GoalResponse类型的结果
rclcpp_action::GoalResponse ElevatorActionServer::handle_goal(
    const rclcpp_action::GoalUUID &uuid,
    std::shared_ptr<const Elevator::Goal> goal)
{
    RCLCPP_INFO(this->get_logger(), "Passenger at Floor %d, pressing %s", 
                goal->initial_floor, goal->direction_to_go == static_cast<uint32_t>(Direction::DIRECTION_UP) ? "🔼 UP" : "🔽 DOWN");

    // 保证方向合理
    bool is_direction_valid = (goal->target_floor > goal->initial_floor) == (goal->direction_to_go == static_cast<uint32_t>(Direction::DIRECTION_UP));

    // 验证楼层是否合法 (1-10)
    if (goal->initial_floor < ground_floor_ || 
        goal->initial_floor > top_floor_ ||
        goal->target_floor < ground_floor_ || 
        goal->target_floor > top_floor_ || 
        !is_direction_valid
    ) {
        RCLCPP_WARN(this->get_logger(), "Invalid request!");
        return rclcpp_action::GoalResponse::REJECT;  // 拒绝
    }

    // 验证起始楼层和目标楼层不同
    if (goal->initial_floor == goal->target_floor) {
        RCLCPP_WARN(this->get_logger(), "Invalid request!");
        return rclcpp_action::GoalResponse::REJECT;  // 拒绝
    }

    RCLCPP_INFO(this->get_logger(), "Goal accepted by server...");

    // 验证通过，接受请求
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

// 使用包装函数(因为本人觉得包装函数的方法比lambda表达式和std::bind()函数更清晰)
void execute_wrapper(
    ElevatorActionServer* instance, 
    std::shared_ptr<rclcpp_action::ServerGoalHandle<elevator_system::action::Elevator>> handle)
    {
        instance->execute(handle);
    }

// 实现handle_accepted函数 - 启动新线程执行任务
void ElevatorActionServer::handle_accepted(
    const std::shared_ptr<GoalHandleElevator> goal_handle
)
{
    std::thread(execute_wrapper, this, goal_handle).detach();
}


// 构造函数实现
ElevatorActionServer::ElevatorActionServer() : Node("elevator_action_server_v2")
{
    RCLCPP_INFO(this->get_logger(), "电梯 Action 服务器 V2 已启动");
    // 从参数服务器获取参数
    ground_floor_ = this->declare_parameter<int>("ground_floor", 1);
    top_floor_ = this->declare_parameter<int>("top_floor", 10);
    move_delay_ms_ = this->declare_parameter<int>("move_delay_ms", 500);

    // 初始化电梯状态
    current_floor_ = ground_floor_;  // 电梯初始在1楼
    passenger_count_ = 0;  // 初始没有乘客
    status_ = ElevatorStatus::STATUS_IDLE;  // 空闲状态
    // 创建Action服务器
    this->action_server_ = rclcpp_action::create_server<Elevator>(
        this,                    // 当前节点
        "elevator",              // Action名称
        std::bind(&ElevatorActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&ElevatorActionServer::handle_cancel, this, std::placeholders::_1),
        std::bind(&ElevatorActionServer::handle_accepted, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "电梯Action服务器已启动");
}

// execute函数实现
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

    
}

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