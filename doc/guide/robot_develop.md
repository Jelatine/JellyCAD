# 机器人开发指南

## 概述

JellyCAD 提供了强大的机器人建模和导出功能，支持将设计的机器人模型导出为 URDF（Unified Robot Description Format）和 MJCF（MuJoCo XML Format）格式，方便在 ROS/ROS2 和 MuJoCo 等机器人仿真平台中使用。

### 主要特性

- 基于 DH 参数（标准 DH 和修改 DH）的机器人运动学建模
- 自动生成包含惯性参数的 URDF 文件
- 支持导出 ROS1 和 ROS2 包
- 支持导出 MuJoCo XML 格式
- 自动生成碰撞和视觉几何
- 支持多种关节类型（revolute, continuous, prismatic, fixed, floating, planar）

## 核心概念

### 1. 坐标系（axes）

坐标系用于定义机器人关节的位置和方向。

```lua
-- 创建坐标系
axes.new()                    -- 默认单位矩阵，轴长为1
axes.new(length)              -- 单位矩阵，自定义轴长
axes.new(pose)                -- 指定位姿矩阵，轴长为1
axes.new(pose, length)        -- 指定位姿和轴长
```

**参数说明：**
- `pose` - 位姿数组，包含 6 个元素：`{x, y, z, rx, ry, rz}`
  - `x, y, z` - 位置坐标（米）
  - `rx, ry, rz` - RPY 姿态角（角度）
- `length` - 坐标轴显示长度

**坐标系变换方法：**

```lua
-- RPY 变换
axes:move(pose)               -- pose = {x, y, z, rx, ry, rz}（角度）

-- 标准 DH 参数变换
axes:sdh(a, alpha, d, theta)  -- alpha和theta为角度

-- 修改 DH 参数变换（Craig 标准）
axes:mdh(a, alpha, d, theta)  -- alpha和theta为角度

-- 显示坐标系
axes:show()
```

**示例：**

```lua
-- 创建并显示一个关节坐标系
j1 = axes.new({0, 0, 0.5, 90, 0, 0}, 0.1)
j1:show()

-- 使用 MDH 参数进行坐标变换
j2 = j1:copy():mdh(0, math.rad(90), 0.3, 0)
j2:show()
```

### 2. 连杆（link）

连杆是机器人的刚体部分，包含几何形状和物理属性。

```lua
link.new(name, shape)         -- 使用单个形状创建连杆
link.new(name, shape_list)    -- 使用形状列表创建连杆
```

**参数说明：**
- `name` - 连杆名称（字符串）
- `shape` - 单个形状对象
- `shape_list` - 形状对象列表 `{shape1, shape2, ...}`

**重要方法：**

```lua
-- 为连杆添加子关节
link:add(joint)

-- 导出 URDF
link:export({
    name = 'robot_name',      -- 机器人名称
    path = '/output/path',    -- 输出路径
    ros_version = 2,          -- 1 为 ROS1，2 为 ROS2（可选）
    mujoco = false            -- true 导出 MuJoCo 格式（可选）
})
```

**物理属性设置：**

```lua
shape:mass(value)             -- 设置质量（千克）
shape:color(color)            -- 设置颜色
shape:scale(factor)           -- 缩放（用于单位转换）
```

### 3. 关节（joint）

关节定义了连杆之间的运动关系。

```lua
joint.new(name, axes, type, limits)
```

**参数说明：**
- `name` - 关节名称（字符串）
- `axes` - 关节坐标系（axes 对象）
- `type` - 关节类型（字符串）：
  - `"revolute"` - 旋转关节（有限位）
  - `"continuous"` - 连续旋转关节（无限位）
  - `"prismatic"` - 移动关节
  - `"fixed"` - 固定关节
  - `"floating"` - 浮动关节
  - `"planar"` - 平面关节
- `limits` - 关节限制参数（可选，表格）：
  ```lua
  {
      lower = -3.14,      -- 下限（弧度或米）
      upper = 3.14,       -- 上限（弧度或米）
      effort = 100,       -- 最大力矩/力（牛米或牛）
      velocity = 1.0      -- 最大速度（弧度/秒或米/秒）
  }
  ```

**关节连接方法：**

```lua
-- 方式1：链式调用
urdf = link.new("base_link", base_shape)
joint1 = urdf:add(joint.new("joint1", axes1, "revolute", limits1))
link1 = joint1:next(link.new("link1", link1_shape))

-- 方式2：分步构建
joint1 = joint.new("joint1", axes1, "revolute")
link1 = link.new("link1", shape1)
urdf:add(joint1):next(link1)
```

## 完整开发流程

### 步骤 1：设计连杆几何

首先创建机器人各个连杆的 3D 几何模型。

```lua
-- 示例：创建圆柱形基座
local r_base = 0.05        -- 半径 50mm
local h_base = 0.035       -- 高度 35mm
base_link = cylinder.new(r_base, h_base)
base_link:scale(1e-3)      -- 转换为米
base_link:color('#6495ED') -- 设置颜色
base_link:mass(0.1)        -- 设置质量 0.1kg

-- 示例：创建连杆
link1_shape = cylinder.new(0.032, 0.09)
link1_shape:scale(1e-3):color('#8470FF'):mass(0.4)
```

### 步骤 2：定义关节坐标系

根据机器人运动学，定义各关节坐标系。

**方法 A：使用 DH 参数（推荐）**

```lua
-- MDH 参数表（推荐使用修改 DH）
local mdh_params = {
    {alpha = 0,   a = 0,     d = 0.05,  theta = 0},
    {alpha = 90,  a = 0,     d = 0,     theta = 0},
    {alpha = 0,   a = -0.15, d = 0,     theta = 0},
    {alpha = 0,   a = -0.12, d = 0.05,  theta = 0},
}

-- 生成关节坐标系
local joint_axes = {}
local prev = axes.new(0.1)
for i = 1, #mdh_params do
    joint_axes[i] = prev:copy():mdh(
        mdh_params[i].a,
        mdh_params[i].alpha,
        mdh_params[i].d,
        mdh_params[i].theta
    )
    prev = joint_axes[i]
    prev:show()  -- 显示坐标系用于验证
end
```

**方法 B：直接指定位姿**

```lua
joint_axes1 = axes.new({0, 0, 0.05, 0, 0, 0}, 0.1)
joint_axes2 = axes.new({0, 0, 0.05, 90, 0, 0}, 0.1)
joint_axes3 = axes.new({0, 0.15, 0.05, 90, 0, 0}, 0.1)

-- 显示坐标系
show({joint_axes1, joint_axes2, joint_axes3})
```

### 步骤 3：创建关节和连杆

```lua
-- 定义关节限制
j1_limit = {lower = -6.28, upper = 6.28, velocity = 3.14, effort = 9}
j2_limit = {lower = -6.28, upper = 6.28, velocity = 3.14, effort = 9}
j3_limit = {lower = -3.14, upper = 3.14, velocity = 3.14, effort = 9}

-- 构建机器人树结构
urdf = link.new("base_link", base_link)

joint1 = urdf:add(joint.new("joint1", joint_axes[1], "revolute", j1_limit))
link1 = joint1:next(link.new("link1", link1_shape))

joint2 = link1:add(joint.new("joint2", joint_axes[2], "revolute", j2_limit))
link2 = joint2:next(link.new("link2", link2_shape))

joint3 = link2:add(joint.new("joint3", joint_axes[3], "revolute", j3_limit))
link3 = joint3:next(link.new("link3", link3_shape))
```

### 步骤 4：可视化验证

```lua
-- 显示所有连杆
show({base_link, link1_shape, link2_shape, link3_shape})

-- 显示所有关节坐标系
show(joint_axes)
```

### 步骤 5：导出 URDF

**导出 ROS2 包：**

```lua
urdf:export({
    name = 'my_robot',
    path = 'd:/',
    ros_version = 2
})
```

**导出 ROS1 包：**

```lua
urdf:export({
    name = 'my_robot',
    path = 'd:/',
    ros_version = 1
})
```

**导出 MuJoCo 格式：**

```lua
urdf:export({
    name = 'my_robot',
    path = 'd:/',
    mujoco = true
})
```

## 在 ROS 中使用

### ROS2 使用方法

```bash
# 1. 安装依赖
sudo apt update
sudo apt install ros-$ROS_DISTRO-urdf-launch

# 2. 复制机器人包到工作空间
mkdir -p ~/ws_ros2/src
cp -r /path/to/my_robot ~/ws_ros2/src/

# 3. 编译
cd ~/ws_ros2
colcon build --symlink-install

# 4. 加载环境
source install/setup.bash

# 5. 启动可视化
ros2 launch urdf_launch display.launch.py \
    urdf_package:=my_robot \
    urdf_package_path:=urdf/my_robot.urdf
```

### ROS1 使用方法

```bash
# 1. 安装依赖
sudo apt update
sudo apt-get install ros-$ROS_DISTRO-urdf-tutorial

# 2. 复制机器人包到工作空间
mkdir -p ~/ws_ros1/src
cp -r /path/to/my_robot ~/ws_ros1/src/

# 3. 编译
cd ~/ws_ros1
catkin_make

# 4. 加载环境
source devel/setup.bash

# 5. 启动可视化
roslaunch urdf_tutorial display.launch \
    model:='$(find my_robot)/urdf/my_robot.urdf'
```

## 在 MuJoCo 中使用

```python
import mujoco
import mujoco.viewer

# 加载模型
model = mujoco.MjModel.from_xml_path('path/to/my_robot/my_robot.xml')
data = mujoco.MjData(model)

# 启动可视化
with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        mujoco.mj_step(model, data)
        viewer.sync()
```

## 高级技巧

### 1. 使用辅助函数简化建模

```lua
-- 创建通用电机外壳
function create_motor(radius, height)
    local shell = cylinder.new(radius, height)
    shell:fillet(5, {type = 'circle', min = {radius-0.01, -0.01, height-0.01}})
    return shell
end

-- 创建连接杆
function create_link_pole(r_outer, r_inner, h)
    local pole = cylinder.new(r_inner, h)
    -- 添加连接台阶
    local stair_h = r_outer - r_inner + 2
    local stair = cylinder.new(r_outer-1, stair_h)
        :cut(torus.new(r_outer-1, r_outer-r_inner):pos(0, 0, stair_h))
    pole:fuse(stair):fuse(stair:copy():rx(180):z(h))
    return pole
end
```

### 2. 批量设置属性

```lua
-- 批量设置颜色和质量
local shapes = {base_link, link1, link2, link3}
local colors = {'#6495ED', '#8470FF', '#FFC1C1', '#FFC100'}
local masses = {0.1, 0.4, 1.0, 0.8}

for i, shape in ipairs(shapes) do
    shape:scale(1e-3):color(colors[i]):mass(masses[i])
end
```

### 3. 参数化设计

```lua
-- 定义参数
local params = {
    r_shell = 32,
    h_motor = 90,
    h_upperarm = 150,
    h_forearm = 120,
    r_upperarm = 20,
    r_forearm = 20
}

-- 使用参数构建模型
local upperarm = cylinder.new(params.r_shell, params.h_motor)
upperarm:fuse(create_link_pole(
    params.r_shell,
    params.r_upperarm,
    params.h_upperarm
))
```

### 4. DH 参数验证

```lua
-- 打印 DH 参数表
function print_dh_params(params_table, dh_type)
    print(string.format("\n========== %s Parameters ==========", dh_type))
    print(string.format("%-8s %-12s %-12s %-12s %-12s",
        "Joint", "alpha(rad)", "a(m)", "d(m)", "theta(rad)"))
    print(string.rep("-", 60))
    for i, params in ipairs(params_table) do
        print(string.format("%-8s %-12.3f %-12.3f %-12.3f %-12.3f",
            "Joint"..i, params.alpha, params.a, params.d, params.theta))
    end
    print(string.rep("=", 60))
end

-- 使用
print_dh_params(mdh_params, "MDH")
```

## 常见问题

### Q1: URDF 导出后在 RViz 中显示异常？

**解决方法：**
1. 检查坐标系定义是否正确，使用 `axes:show()` 验证
2. 确认所有形状已正确缩放（`scale(1e-3)` 将毫米转为米）
3. 验证关节轴方向（默认为 Z 轴）

### Q2: 如何选择标准 DH 还是修改 DH？

**建议：**
- **修改 DH（MDH）**：推荐使用，更符合现代机器人学标准（Craig 标准）
- **标准 DH（SDH）**：可能在某些动力学计算中产生问题

```lua
-- 使用修改 DH
local is_mdh = true
if is_mdh then
    joint_axes[i] = prev:copy():mdh(alpha, a, d, theta)
else
    joint_axes[i] = prev:copy():sdh(alpha, a, d, theta)
end
```

### Q3: 如何设置工具坐标系（TCP）？

```lua
-- 添加固定的工具关节
local tool_axes = joint_axes[6]:copy():move({0, 0, 0.1, 0, 0, 0})
local tool_joint = joint.new("tool_joint", tool_axes, "fixed")
local tool_link = link.new("tool_link", shape.new())  -- 空形状

link6:add(tool_joint):next(tool_link)
```

### Q4: 导出的网格文件过大怎么办？

可以调整网格精度参数（修改源码中的 `export_stl_common` 调用）：

```lua
-- 在 jy_urdf_generator.cpp 中调整第三个参数（曲率值）
output_shape.export_stl_common(path_stl.generic_string(), false, 0.5)
-- 数值越大，三角形越少，文件越小（但精度降低）
```

## 完整示例

完整的 6 自由度机械臂示例请参考：
- `scripts/6urdf.lua` - 使用 RPY 方法建模
- `scripts/7robot_arm_dh.lua` - 使用 DH 方法建模

## 参考资料

- [URDF 规范](http://wiki.ros.org/urdf/XML)
- [MuJoCo XML 文档](https://mujoco.readthedocs.io/en/stable/XMLreference.html)
- [机器人学：DH 参数详解](https://en.wikipedia.org/wiki/Denavit%E2%80%93Hartenberg_parameters)
- [JellyCAD 脚本函数参考](/guide/functions)

## 下一步

- 学习[界面交互指南](/guide/interaction)了解可视化调试技巧
- 查看[脚本函数参考](/guide/functions)掌握更多建模方法
- 探索示例脚本 `scripts/` 目录获取灵感