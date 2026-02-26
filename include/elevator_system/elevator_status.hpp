#ifndef ELEVATOR_STATUS_HPP
#define ELEVATOR_STATUS_HPP

enum class ElevatorStatus
{
    STATUS_IDLE = 0,        // 空闲
    STATUS_MOVING_UP = 1,   // 向上运行
    STATUS_MOVING_DOWN = 2, // 向下运行
    STATUS_ARRIVED = 3,     // 已到达
    STATUS_PICKUP = 4,      // 正在接人
    STATUS_DROPOFF = 5,     // 正在下客
    STATUS_ERROR = 6       // 错误状态
};

enum class Direction
{
    DIRECTION_UP = 0,
    DIRECTION_DOWN = 1,
    DIRECTION_IDLE = 2, // 实现v3版本新增的状态,表示电梯空闲
};

// LOOK算法所需的停靠信息
struct StopInfo {
    int floor;           // 停靠楼层
    bool is_pickup;      // true=pick up, false=drop off
    int target_floor;    // 如果是pick up,乘客的目标楼层;如果是drop off,此字段无用
    int initial_floor;      // 如果是drop off，乘客的起始楼层（用于发送result）
    Direction direction; // pick up时：乘客想去的方向
    std::shared_ptr<rclcpp_action::ServerGoalHandle<elevator_system::action::Elevator>> goal_handle;
    
    StopInfo(int f, bool pickup, int target, int initial, Direction dir = Direction::DIRECTION_UP,
         std::shared_ptr<rclcpp_action::ServerGoalHandle<elevator_system::action::Elevator>> gh = nullptr)
    : floor(f), is_pickup(pickup), target_floor(target), initial_floor(initial), direction(dir), goal_handle(gh) {}
};

// 请求信息
struct Request {
    int initial_floor;
    // int target_floor;  target_floor 目标楼层延迟生成
    Direction direction;
    std::shared_ptr<rclcpp_action::ServerGoalHandle<elevator_system::action::Elevator>> goal_handle;
};

#endif // ELEVATOR_STATUS_HPP
