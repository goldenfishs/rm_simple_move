# rm_simple_move

一个用于简单航点导航的 ROS2 程序。适用于 RM 和 RC 的固定点导航。

---

## 使用方法

### 1. 克隆仓库
```bash
git clone https://github.com/goldenfishs/rm_simple_move.git
```

### 2. 安装 `rm_msgs`
```bash
git clone https://github.com/goldenfishs/rm_msgs.git
```

---

## 功能说明

### 坐标变换
需要获取的坐标变换：`map -> base_link`

---

### 输入话题 `/goal_pose`
- `float32 x`：目标点的 x 坐标
- `float32 y`：目标点的 y 坐标
- `float32 angle`：目标角度（范围：`-π` 到 `π`）
- `float32 max_speed`：最大输出速度
- `float32 tolerance`：到达目标点的容差
- `bool rotor`：是否开启小陀螺模式

---

### 输出话题 `/data_nav`
- `reached`：是否到达目标点
- 当前 x 坐标
- 当前 y 坐标

---

### 输出话题 `/chassis/data_ai`
- `float32 yaw`：方向角度（`angle`）
- `float32 pit`：0
- `float32 rol`：0
- `float32 vx`：x 方向速度
- `float32 vy`：y 方向速度
- `float32 vz`：0
- `uint8 notice`：是否开启小陀螺模式

---

## 启动方法
1. 编译项目：
   ```bash
   colcon build
   ```
2. 加载环境：
   ```bash
   source install/setup.bash
   ```
3. 启动程序：
   ```bash
   ros2 launch rm_simple_move simple_move.launch.py
   ```

---

## 测试方法
使用 `test/move_test`，启动后输入目标点即可测试。
```