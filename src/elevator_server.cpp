#include <rclcpp/rclcpp.hpp>                    // ROS2核心库
#include <rclcpp_action/rclcpp_action.hpp>      // Action通信库
#include <thread>                               // 线程库
#include <chrono>                               // 时间库
#include "elevator_system/action/elevator.hpp"  // 自定义Action接口
#include "elevator_system/elevator_status.hpp"  // 电梯状态枚举

#define GROUND_FLOOR 1
#define TOP_FLOOR 10

class ElevatorActionServer : public rclcpp::Node    // 继承自ROS2节点
{
public:
    // 定义类型别名
    using Elevator = elevator_system::action::Elevator;
    using GoalHandleElevator = rclcpp_action::ServerGoalHandle<Elevator>;

    // 构造函数
    ElevatorActionServer();

private:
    // Action服务器
    rclcpp_action::Server<Elevator>::SharedPtr action_server_;

    // 电梯状态变量
    int current_floor_;          // 当前楼层
    ElevatorStatus status_;      // 电梯状态
    int passenger_count_;        // 乘客数量


};

// 实现handle_goal函数 - 验证并接受请求
// ElevatorActionServer类的handle_goal函数，返回一个GoalResponse类型的结果
rclcpp_action::GoalResponse ElevatorActionServer::handle_goal(
    const rclcpp_action::GoalUUID &uuid,
    std::shared_ptr<const Elevator::Goal> goal)
{
    RCLCPP_INFO(this->get_logger(), "Passenger at Floor %d, pressing %s", 
                goal->initial_floor, goal->direction_to_go == 0 ? "🔼 UP" : "🔽 DOWN");

    // 保证方向合理
    bool is_direction_valid = (goal->target_floor > goal->initial_floor) == (goal->direction_to_go == 0);

    // 验证楼层是否合法 (1-10)
    if (goal->initial_floor < GROUND_FLOOR || 
        goal->initial_floor > TOP_FLOOR ||
        goal->target_floor < GROUND_FLOOR || 
        goal->target_floor > TOP_FLOOR || 
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
    RCLCPP_INFO(this->get_logger(), "Cancel request received for goal %d", goal_handle->get_goal_id().id);
    return rclcpp_action::CancelResponse::ACCEPT;
}

// 构造函数实现
ElevatorActionServer() : Node("elevator_action_server")
{
    // 初始化电梯状态
    current_floor_ = GROUND_FLOOR;  // 电梯初始在1楼
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