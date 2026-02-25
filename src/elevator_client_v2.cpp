#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <chrono>
#include <functional>
#include <memory>
#include <random> // 引入随机数库

#include "elevator_system/action/elevator.hpp"
#include "elevator_system/elevator_status.hpp"

class ElevatorActionClientV2 : public rclcpp::Node
{
public:
    using Elevator = elevator_system::action::Elevator;
    using GoalHandleElevator = rclcpp_action::ClientGoalHandle<Elevator>;

    explicit ElevatorActionClientV2() : Node("elevator_action_client_v2"), request_count_(0)
    {
        action_client_ = rclcpp_action::create_client<Elevator>(this, "elevator");
        
        // 初始化随机数引擎
        rng_.seed(std::random_device()());
        dist_ = std::uniform_int_distribution<int>(1, 10); // 楼层范围 1-10

        RCLCPP_INFO(this->get_logger(), "V2 客户端已启动，等待服务器...");
        
        // 等待服务器
        if (!action_client_->wait_for_action_server(std::chrono::seconds(10))) {
            RCLCPP_ERROR(this->get_logger(), "服务器连接失败");
            return;
        }

        // 创建定时器，每 15000ms 触发一次
        request_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(15000),
            std::bind(&ElevatorActionClientV2::timer_callback, this));
    }

private:
    rclcpp_action::Client<Elevator>::SharedPtr action_client_;
    rclcpp::TimerBase::SharedPtr request_timer_;
    
    // 随机数相关成员
    std::mt19937 rng_;
    std::uniform_int_distribution<int> dist_;
    
    int request_count_;
    const int MAX_REQUESTS = 8;

    void timer_callback()
    {
        if (request_count_ >= MAX_REQUESTS) {
            RCLCPP_INFO(this->get_logger(), "已完成 8 次请求，停止定时器。");
            request_timer_->cancel();
            return;
        }

        // 生成随机数据
        int initial_floor = dist_(rng_);
        int target_floor = dist_(rng_);
        
        // 保证数据合理性：起始楼层 != 目标楼层
        while (initial_floor == target_floor) {
            target_floor = dist_(rng_);
        }

        send_goal(initial_floor, target_floor);
        request_count_++;
    }

    void send_goal(int initial_floor, int target_floor)
    {
        auto goal_msg = Elevator::Goal();
        goal_msg.initial_floor = initial_floor;
        goal_msg.target_floor = target_floor;
        goal_msg.direction_to_go = static_cast<uint32_t>((target_floor > initial_floor) ? Direction::DIRECTION_UP : Direction::DIRECTION_DOWN);
        goal_msg.passenger_count = 1;

        RCLCPP_INFO(this->get_logger(), "-----------------------------------");
        RCLCPP_INFO(this->get_logger(), "[请求 #%d] 发送: %d楼 -> %d楼", request_count_ + 1, initial_floor, target_floor);

        auto goal_options = rclcpp_action::Client<Elevator>::SendGoalOptions();
        
        // 注册回调 (使用 Lambda 简化书写，或者保持原样绑定)
        goal_options.goal_response_callback = [this](std::shared_future<GoalHandleElevator::SharedPtr> future) {
            auto handle = future.get();
            if (!handle) RCLCPP_ERROR(this->get_logger(), "请求被拒绝");
            else RCLCPP_INFO(this->get_logger(), "请求已接受");
        };

        goal_options.feedback_callback = [](GoalHandleElevator::SharedPtr, const std::shared_ptr<const Elevator::Feedback> feedback) {
            // 反馈太多可能会刷屏，这里可以选择不打印或只打印关键信息
            // RCLCPP_INFO(rclcpp::get_logger("client"), "Feedback: Floor %d", feedback->current_floor);
        };

        goal_options.result_callback = [this](const GoalHandleElevator::WrappedResult &result) {
            if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                 RCLCPP_INFO(this->get_logger(), "任务结果: 成功到达 %d 楼", result.result->final_floor);
            } else {
                 RCLCPP_WARN(this->get_logger(), "任务结果: 失败或取消");
            }
        };

        action_client_->async_send_goal(goal_msg, goal_options);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ElevatorActionClientV2>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}