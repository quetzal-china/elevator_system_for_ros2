#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <chrono>
#include <functional>
#include <memory>

#include "elevator_system/action/elevator.hpp"
#include "elevator_system/elevator_status.hpp"

class ElevatorActionClient : public rclcpp::Node
{
public:
    using Elevator = elevator_system::action::Elevator;
    using GoalHandleElevator = rclcpp_action::ClientGoalHandle<Elevator>;

    explicit ElevatorActionClient();
    
    // 发送目标函数 (现在是纯异步非阻塞的)
    void send_goal(int initial_floor, int target_floor);

private:
    rclcpp_action::Client<Elevator>::SharedPtr action_client_;

    // 回调函数签名必须匹配 ROS2 Action Client 的要求
    // 1. 响应回调：参数为 shared_future，通过 future.get() 获取 goal_handle
    void goal_response_callback(std::shared_future<GoalHandleElevator::SharedPtr> future);
    
    // 2. 反馈回调：参数保持不变
    void feedback_callback(GoalHandleElevator::SharedPtr, const std::shared_ptr<const Elevator::Feedback> feedback);
    
    // 3. 结果回调：参数保持不变
    void result_callback(const GoalHandleElevator::WrappedResult &result);
};

// 构造函数
ElevatorActionClient::ElevatorActionClient()
    : Node("elevator_action_client")
{
    action_client_ = rclcpp_action::create_client<Elevator>(this, "elevator");
    RCLCPP_INFO(this->get_logger(), "电梯 Action 客户端已创建");
}

// 发送目标函数实现
void ElevatorActionClient::send_goal(int initial_floor, int target_floor)
{
    // 1. 等待服务器, 短暂阻塞
    if (!this->action_client_->wait_for_action_server(std::chrono::seconds(10)))
    {
        RCLCPP_ERROR(this->get_logger(), "电梯服务器连接超时，发送失败");
        return;
    }

    // 2. 构建 Goal 消息
    auto goal_msg = Elevator::Goal();
    goal_msg.initial_floor = static_cast<uint32_t>(initial_floor);
    goal_msg.target_floor = static_cast<uint32_t>(target_floor);
    goal_msg.direction_to_go = static_cast<uint32_t>((target_floor > initial_floor) ? Direction::DIRECTION_UP : Direction::DIRECTION_DOWN);
    goal_msg.passenger_count = 0;

    RCLCPP_INFO(this->get_logger(), "正在发送请求: %d楼 -> %d楼", initial_floor, target_floor);

    // 3. 配置 SendGoalOptions
    // 我们在这里一次性注册所有回调，不再手动处理 future 或 subscribe_feedback
    auto goal_options = rclcpp_action::Client<Elevator>::SendGoalOptions();
    
    // 响应回调：服务器接受/拒绝目标时触发
    goal_options.goal_response_callback = 
        std::bind(&ElevatorActionClient::goal_response_callback, this, std::placeholders::_1);

    // 反馈回调：服务器周期性发布反馈时触发
    goal_options.feedback_callback = 
        std::bind(&ElevatorActionClient::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);

    // 结果回调：任务完成/中止/取消时触发
    goal_options.result_callback = 
        std::bind(&ElevatorActionClient::result_callback, this, std::placeholders::_1);

    // 4. 发送 Goal (异步非阻塞)
    // 函数调用后立即返回，后续所有逻辑均由上面的回调函数处理
    action_client_->async_send_goal(goal_msg, goal_options);
    
    RCLCPP_INFO(this->get_logger(), "请求已发出，等待服务器处理...");
}

// 响应回调实现
void ElevatorActionClient::goal_response_callback(std::shared_future<GoalHandleElevator::SharedPtr> future)
{
    // 获取 goal_handle, 必须调用 get() 获取实际的 goal_handle
    auto goal_handle = future.get();

    if (!goal_handle)
    {
        RCLCPP_ERROR(this->get_logger(), "请求被服务器拒绝; 目标楼层可能无效");
        return;
    }
    
    RCLCPP_INFO(this->get_logger(), "请求已被服务器接受，正在执行...");
    
    // 反馈回调：服务器周期性发布反馈时触发
    // SendGoalOptions 已经自动处理了反馈订阅
}

// 反馈回调实现
void ElevatorActionClient::feedback_callback(GoalHandleElevator::SharedPtr,
                                             const std::shared_ptr<const Elevator::Feedback> feedback)
{
    // 这个回调现在由 ROS2 底层自动调用，无需手动 subscribe
    RCLCPP_INFO(this->get_logger(), 
                "[Feedback] 当前楼层: %d | 状态: %u | 乘客: %u",
                feedback->current_floor,
                feedback->status,
                feedback->current_load);
}

// 结果回调实现
void ElevatorActionClient::result_callback(const GoalHandleElevator::WrappedResult &result)
{
    // 无论成功、失败还是取消，都会触发此回调
    switch (result.code)
    {
    case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(this->get_logger(), "任务成功完成！最终楼层: %d", result.result->final_floor);
        break;
    case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "任务被中止: %s", result.result->message.c_str());
        break;
    case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_WARN(this->get_logger(), "任务被取消");
        break;
    default:
        RCLCPP_ERROR(this->get_logger(), "未知的任务结果代码");
        break;
    }
    
    // 如果这是一个调度系统，你可以在这里触发下一个任务
    // 例如: schedule_next_task(); 
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto client = std::make_shared<ElevatorActionClient>();

    // 测试发送一个任务
    client->send_goal(3, 7);

    // 必须使用 spin 来驱动回调队列
    // send_goal 是非阻塞的，主线程必须在这里 spin 才能接收到服务器的回应
    rclcpp::spin(client);
    
    rclcpp::shutdown();
    return 0;
}