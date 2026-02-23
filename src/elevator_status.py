# elevator_status.py
class ElevatorStatus:
    STATUS_IDLE = 0        # 空闲
    STATUS_MOVING_UP = 1   # 向上运行
    STATUS_MOVING_DOWN = 2 # 向下运行
    STATUS_ARRIVED = 3     # 已到达
    STATUS_PICKUP = 4      # 正在接人
    STATUS_DROPOFF = 5     # 正在下客
    STATUS_ERROR = 6       # 错误状态

class Direction:
    DIRECTION_UP = 0
    DIRECTION_DOWN = 1