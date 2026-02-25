#include <rclcpp/rclcpp.hpp>               // ros2 c++核心库
#include <rclcpp_action/rclcpp_action.hpp> // ros2 Action 通信库
#include <chrono>                          // 时间处理
#include <future>                          // 异步操作支持

#include "elevator_system/action/elevator.hpp" // 电梯系统Action接口

class ElevatorActionClient : public rclcpp::Node // 电梯系统Action客户端类
{
public:
    // 类型别名，简化代码
    using Elevator = elevator_system::action::Elevator;
    using GoalHandleElevator = rclcpp_action::ClientGoalHandle<Elevator>;

    // 构造函数
    ElevatorActionClient();

    // 发送目标的公共函数
    void send_goal(int32 initial_floor, int32 target_floor);

    // 三个回调函数
    void goal_response_callback(const rclcpp_action::ClientGoalHandle<Elevator>::SharedPtr &goal_handle);
    void feedback_callback(GoalHandleElevator::SharedPtr, const std::shared_ptr<const Elevator::Feedback> feedback);
    void result_callback(const rclcpp_action::ClientGoalHandle<Elevator>::WrappedResult &result);

private:

    // Action 客户端实例
    rclcpp_action::Client<Elevator>::SharedPtr action_client_; // Action客户端指针

};



// 构造函数实现
ElevatorActionClient::ElevatorActionClient()
    : Node("elevator_action_client")
{
    // 创建Action客户端
    action_client_ = rclcpp_action::create_client<Elevator>(
        this,
        "elevator");

    RCLCPP_INFO(this->get_logger(), "电梯 Action 客户端已创建");
}

// 目标响应回调函数
void ElevatorActionClient::goal_response_callback(const rclcpp_action::ClientGoalHandle<Elevator>::SharedPtr &goal_handle)
{
    if (!goal_handle)
    {
        RCLCPP_ERROR(this->get_logger(), "Goal rejected by server");
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
}

// 反馈回调函数
void ElevatorActionClient::feedback_callback(GoalHandleElevator::SharedPtr,
                                             const std::shared_ptr<const Elevator::Feedback> feedback)
{
    RCLCPP_INFO(this->get_logger(),
                "[Feedback] Floor:%d | Status:%u | Passengers:%u | Dir:%u",
                feedback->current_floor,
                feedback->status,
                feedback->current_load,
                feedback->direction);
}

// 结果回调函数
void ElevatorActionClient::result_callback(
    const rclcpp_action::ClientGoalHandle<Elevator>::WrappedResult &result)
{
    switch (result.code)
    {
    case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(this->get_logger(), "任务完成 - 成功");
        RCLCPP_INFO(this->get_logger(), "最终楼层：%d", result.result->final_floor);
        RCLCPP_INFO(this->get_logger(), "消息：%s", result.result->message.c_str());
        break;
    case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "任务被中止");
        break;
    case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_INFO(this->get_logger(), "任务被取消");
        break;
    default:
        RCLCPP_ERROR(this->get_logger(), "未知结果");
        break;
    }
}

// 发送目标的公共函数
void ElevatorActionClient::send_goal(int32 initial_floor, int32 target_floor)
{
    // 等待服务器可用
    RCLCPP_INFO(this->get_logger(), "等待电梯服务器响应...");
    if (!this->action_client_->wait_for_server(std::chrono::seconds(10)))
    {
        RCLCPP_ERROR(this->get_logger(), "电梯服务器未响应");
        return;
    }
    RCLCPP_INFO(this->get_logger(), "电梯服务器已响应");

    // 创建 Goal 消息
    auto goal_msg = Elevator::Goal();
    goal_msg.initial_floor = initial_floor;
    goal_msg.target_floor = target_floor;
    goal_msg.direction_to_go = static_cast<Direction>((target_floor > initial_floor) ? 0 : 1);
    goal_msg.passenger_count = 0; // 简单情况下初始为0人

    RCLCPP_INFO(this->get_logger(), "发送请求: 初始楼层:%d, 目标楼层:%d, 方向:%s",
                initial_floor,
                target_floor,
                (goal_msg.direction_to_go == Direction::DIRECTION_UP) ? "UP" : "DOWN");

    // 发送 Goal 并获取 future
    auto send_goal_future = action_client_->async_send_goal(
        goal_msg,
        std::bind(&ElevatorActionClient::goal_response_callback,
                  this,
                  std::placeholders::_1));

    // 等待 Goal 被接受, 阻塞等待 future 完成
    rclcpp::spin_until_future_complete(this->get_node_base_interface(),
                                       send_goal_future);
    auto goal_handle = send_goal_future.get();

    if (!goal_handle)
    {
        RCLCPP_ERROR(this->get_logger(), "Goal rejected by server");
        return;
    }

    // 注册反馈回调
    // 订阅反馈，每次服务器发布反馈时触发回调
    goal_handle.subscribe_feedback(
        std::bind(&ElevatorActionClient::feedback_callback, this, std::placeholders::_1, std::placeholders::_2));

    // 注册结果回调
    // 注释代码是初版, 后来发现async_result()方法被废弃了, 改用async_get_result()方法
    /* auto result_future = goal_handle->async_result(
        std::bind(&ElevatorActionClient::result_callback, this, std::placeholders::_1)
    ); */

    auto result_future = goal_handle->async_get_result();
    result_future.then(
        std::bind(&ElevatorActionClient::result_callback,
                  this,
                  std::placeholders::_1));

    // 等待结果
    rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future);
    RCLCPP_INFO(this->get_logger(), "请求完成");
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto client = std::make_shared<ElevatorActionClient>();
    
    // 测试：从 3 楼到 7 楼
    client->send_goal(3, 7);
    
    rclcpp::shutdown();
    return 0;
}