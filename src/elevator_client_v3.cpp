#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <chrono>
#include <functional>
#include <memory>
#include <random> // 引入随机数库
#include <cstdlib>

#include "elevator_system/action/elevator.hpp"
#include "elevator_system/elevator_status.hpp"

class ElevatorActionClientV2 : public rclcpp::Node
{
public:
    using Elevator = elevator_system::action::Elevator;
    using GoalHandleElevator = rclcpp_action::ClientGoalHandle<Elevator>;

    explicit ElevatorActionClientV2() : Node("elevator_action_client_v3"), request_count_(0)
    {
        action_client_ = rclcpp_action::create_client<Elevator>(this, "elevator");

        interval_dist_ = std::uniform_int_distribution<int>(2, 20);  // 2-20秒
        
        // 初始化随机数引擎
        rng_.seed(std::random_device()());
        dist_ = std::uniform_int_distribution<int>(1, 10); // 楼层范围 1-10

        RCLCPP_INFO(this->get_logger(), "V3 客户端已启动，等待服务器...");
        
        // 等待服务器
        if (!action_client_->wait_for_action_server(std::chrono::seconds(10))) {
            RCLCPP_ERROR(this->get_logger(), "服务器连接失败");
            return;
        }

        // 重新设置定时器为随机间隔
        int next_interval = interval_dist_(rng_) * 1000;  // 转换为毫秒
        request_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(next_interval),
        std::bind(&ElevatorActionClientV2::timer_callback, this));
    }

private:
    rclcpp_action::Client<Elevator>::SharedPtr action_client_;
    rclcpp::TimerBase::SharedPtr request_timer_;
    
    // 随机数相关成员
    std::mt19937 rng_;
    std::uniform_int_distribution<int> dist_;

    std::uniform_int_distribution<int> interval_dist_;  // 间隔时间分布
    
    int request_count_;
    const int MAX_REQUESTS = 8;

    void timer_callback()
    {
        if (request_count_ >= MAX_REQUESTS) {
            RCLCPP_INFO(this->get_logger(), "已完成 8 次请求，停止定时器。");
            request_timer_->cancel();
            return;
        }
        // 随机生成初始楼层 (1-10)
        int initial_floor = dist_(rng_);
        
        // 随机生成方向 (UP 或 DOWN)
        // 但要考虑边界情况：顶层不能UP，底层不能DOWN
        Direction direction;
        if (initial_floor == 1) {
            direction = Direction::DIRECTION_UP;  // 1楼只能UP
        } else if (initial_floor == 10) {
            direction = Direction::DIRECTION_DOWN;  // 10楼只能DOWN
        } else {
            // 随机选择方向
            direction = (rand() % 2 == 0) ? Direction::DIRECTION_UP : Direction::DIRECTION_DOWN;
        }
        
        send_goal(initial_floor, direction);
        request_count_++;

        // 重新设置定时器为新的随机间隔（如果不是最后一个请求）
        if (request_count_ < MAX_REQUESTS) 
        {
            int next_interval = interval_dist_(rng_) * 1000;  // 新的随机间隔
            request_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(next_interval),
            std::bind(&ElevatorActionClientV2::timer_callback, this));
        }
    }
    void send_goal(int initial_floor, Direction direction)
    {
        auto goal_msg = Elevator::Goal();
        goal_msg.initial_floor = initial_floor;
        goal_msg.direction_to_go = static_cast<uint32_t>(direction);
        // 不设置 target_floor，由服务端随机生成
        goal_msg.passenger_count = 1;
        RCLCPP_INFO(this->get_logger(), "-----------------------------------");
        RCLCPP_INFO(this->get_logger(), "[请求 #%d] 发送: %d楼, 方向: %s", 
                    request_count_ + 1, 
                    initial_floor, 
                    direction == Direction::DIRECTION_UP ? "🔼 UP" : "🔽 DOWN");
        auto goal_options = rclcpp_action::Client<Elevator>::SendGoalOptions();
        
        // 注册回调
        goal_options.goal_response_callback = [this](std::shared_future<GoalHandleElevator::SharedPtr> future) {
            auto handle = future.get();
            if (!handle) RCLCPP_ERROR(this->get_logger(), "请求被拒绝");
            else RCLCPP_INFO(this->get_logger(), "请求已接受");
        };
        goal_options.feedback_callback = [](GoalHandleElevator::SharedPtr, const std::shared_ptr<const Elevator::Feedback> feedback) {
            // 反馈太多可能会刷屏，这里可以选择不打印或只打印关键信息
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