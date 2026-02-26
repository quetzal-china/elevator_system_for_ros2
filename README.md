任务点1验证命令:
bash1:
```bash
colcon build --packages-select elevator_system
source ~/ros2_ws/install/setup.bash
ros2 run elevator_system elevator_server --ros-args --params-file config/elevator_params.yaml
```
bash2:
```bash
source ~/ros2_ws/install/setup.bash
ros2 run elevator_system elevator_client
```
在文件`config/elevator_params.yaml`中可以修改电梯参数
- ground_floor
- top_floor
- move_delay_ms
任务点2验证命令:
bash1:
```bash
colcon build --packages-select elevator_system
source install/setup.bash
```
```bash2
source install/setup.bash
ros2 run elevator_system elevator_server_v2 --ros-args --params-file config/elevator_params.yaml
```
```bash3
source install/setup.bash
ros2 run elevator_system elevator_client_v2
```
任务点3验证命令:
bash1:
```bash
colcon build --packages-select elevator_system
source install/setup.bash
```
```bash2
source install/setup.bash
ros2 run elevator_system elevator_server_v3 --ros-args --params-file ~/ros2_ws/src/elevator_system/config/elevator_params.yaml
```
```bash3
source install/setup.bash
ros2 run elevator_system elevator_client_v3
```