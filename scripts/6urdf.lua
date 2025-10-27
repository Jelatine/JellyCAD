-- 6自由度机械臂URDF建模及URDF导出示例
-- 通用
local r_shell = 32;
local h_motor = 90;
local offset = h_motor / 2 - r_shell
shell = cylinder.new(r_shell, h_motor)
shell:fillet(5, { type = 'circle', min = { r_shell - 1e-2, -1e-2, h_motor - 1e-2 } });
shell:fuse(cylinder.new(r_shell, h_motor / 2):z(h_motor / 2):rx(90));
shell:cut(cylinder.new(r_shell - 8, h_motor - 1):z(2));
-- 电机
motor = cylinder.new(r_shell - 9, h_motor / 3):color('black')
-- shell:cut(text.new('JellyCAD', 10):pos(-r_shell + 10, -4, h_motor + 1):prism(0, 0, -4)) -- 关节盖刻字(操作很耗时)
-- 生成连接柱
function get_pole(r_outer, r2, h)
    local r1 = r_outer - 1
    local h_stair = 2
    local h_cylinder = r1 - r2 + h_stair
    local stair = cylinder.new(r1, h_cylinder):cut(torus.new(r1, r1 - r2):pos(0, 0, h_cylinder))
    stair:fillet(1, { type = 'circle', min = { r1 - 1e-2, -1e-2, h_stair - 1e-2 } });
    local stair_top = stair:copy():rx(180):z(h)
    local pole = cylinder.new(r2, h):fuse(stair):fuse(stair_top)
    return pole
end

-- 基座
local r_base = 50;
local h_base = 35;
base_link = cylinder.new(r_base, h_base);
local R1 = r_base - r_shell
local R0 = R1 + 2
elips = edge.new('elips', { R1 + r_shell, 0, h_base }, { 0, 1, 0 }, R0, R1);
ellipse = face.new(elips);
ellipse:revol({ 0, 0, 0 }, { 0, 0, 1 }, 360)
base_link:cut(ellipse);
base_link:fillet(3, { type = 'bspline_curve', min = { r_base - 1e-2, -1e-2, (h_base - R0) - 1e-2 } });
-- 肩部
sholder = {}
sholder[1] = shell:copy():z(h_base);
-- 上臂
local h_upperarm = 150
local r_upperarm = 20
local z_upperarm = h_base + h_motor / 2
upperarm = {}
upperarm[1] = shell:copy():rot(90, 180, 0)
upperarm[2] = get_pole(r_shell, r_upperarm, h_upperarm):pos(0, -h_motor / 2, h_motor / 2)
upperarm[3] = shell:copy():rot(90, 0, 0):pos(0, 0, h_motor + h_upperarm)
upperarm[1]:move('pos', 0, -h_motor / 2, z_upperarm)
upperarm[2]:move('pos', 0, -h_motor / 2, z_upperarm)
upperarm[3]:move('pos', 0, -h_motor / 2, z_upperarm)
-- 前臂
local h_forearm = 120
local r_forearm = 20
local z_forearm = h_base + h_upperarm + r_shell + h_motor + offset
forearm = {}
forearm[1] = face.new(edge.new('circ', { 0, 0, 0 }, { 0, 0, 1 }, r_shell)):revol({ 0, -r_shell, 0 }, { 1, 0, 0 }, -90)
forearm[2] = get_pole(r_shell, r_forearm, h_forearm)
forearm[3] = shell:copy():rot(90, 0, 180):pos(0, -h_motor / 2, h_motor / 2 + h_forearm)
forearm[1]:move('pos', 0, -offset, z_forearm + r_shell)
forearm[2]:move('pos', 0, -offset, z_forearm + r_shell)
forearm[3]:move('pos', 0, -offset, z_forearm + r_shell)
-- 手腕1
wrist1 = {}
wrist1[1] = shell:copy():rot(180, 0, 0):pos(0, -h_motor - offset, z_forearm + h_forearm + h_motor + r_shell)
-- 手腕2
wrist2 = {}
local z_wrist2 = z_forearm + h_forearm + 2 * h_motor - offset
wrist2[1] = shell:copy():rot(90, 0, 180):pos(0, r_shell - 2 * h_motor, z_wrist2)
-- 手腕3
local h_flank = 10
wrist3 = cylinder.new(r_shell, h_flank):rot(90, 0, 0):pos(0, r_shell - 2 * h_motor, z_wrist2)
-- 毫米单位转为米，生成URDF
base_link:scale(1e-3):color('#6495ED'):mass(0.1)
sholder[1]:scale(1e-3):color('#8470FF'):mass(0.1)                                  -- 肩部模组外壳
sholder[2] = motor:copy():locate(sholder[1]):move('z', 2):scale(1e-3):mass(0.3)    -- J1电机
upperarm[1]:scale(1e-3):color('#FFC1C1'):mass(0.1)                                 -- 关节2模组外壳
upperarm[4] = motor:copy():locate(upperarm[1]):move('y', -2):scale(1e-3):mass(0.3) -- J2电机
upperarm[2]:scale(1e-3):color('#FFC1C1'):mass(0.2)                                 -- 关节2与关节3之间的连接柱
upperarm[3]:scale(1e-3):color('#FFC1C1'):mass(0.1)                                 -- 关节3模组外壳
upperarm[5] = motor:copy():locate(upperarm[3]):move('y', -2):scale(1e-3):mass(0.3) -- J3电机
forearm[1]:scale(1e-3):color('#FFC100'):mass(0.2)                                  -- 关节3与前臂柱转接器
forearm[2]:scale(1e-3):color('#FFC100'):mass(0.1)                                  -- 前臂柱
forearm[3]:scale(1e-3):color('#FFC100'):mass(0.1)                                  -- 关节4模组外壳
forearm[4] = motor:copy():locate(forearm[3]):move('y', 2):scale(1e-3):mass(0.3)    -- J4电机
wrist1[1]:scale(1e-3):color('#FF8247'):mass(0.1)                                   -- 手腕1模组外壳
wrist1[2] = motor:copy():locate(wrist1[1]):move('z', -2):scale(1e-3):mass(0.3)     -- J5电机
wrist2[1]:scale(1e-3):color('#FFE7BA'):mass(0.1)                                   -- 手腕2模组外壳
wrist2[2] = motor:copy():locate(wrist2[1]):move('y', 2):scale(1e-3):mass(0.3)      -- J16电机
wrist3:scale(1e-3):color('#C1CDC1'):mass(0.1)                                      -- 末端法兰
local d1 = z_upperarm * 1e-3
local a2 = (h_upperarm + h_motor) * 1e-3
local a3 = (h_forearm + h_motor / 2 + r_shell) * 1e-3
local d4 = (h_motor + offset) * 1e-3
local d5 = h_motor * 1e-3
local d6 = (h_motor / 2 + h_flank) * 1e-3
joint_axes1 = axes.new({ 0, 0, d1, 0, 0, 0 }, 0.1)
joint_axes2 = joint_axes1:copy():move({ 0, 0, 0, 90, 0, 0 })
joint_axes3 = joint_axes2:copy():move({ 0, a2, 0, 0, 0, 0 })
joint_axes4 = joint_axes3:copy():move({ 0, a3, 0, 0, 0, 0 })
joint_axes5 = joint_axes4:copy():move({ 0, 0, d4, -90, 0, 0 })
joint_axes6 = joint_axes5:copy():move({ 0, 0, d5, 90, 0, 0 })
joint_tool = joint_axes6:copy():move({ 0, 0, d6, 0, 0, 0 })
j1_limit = { lower = -6.28, upper = 6.28, velocity = 3.14, effort = 9 }
j2_limit = { lower = -6.28, upper = 6.28, velocity = 3.14, effort = 9 }
j3_limit = { lower = -3.14, upper = 3.14, velocity = 3.14, effort = 9 }
j4_limit = { lower = -6.28, upper = 6.28, velocity = 3.14, effort = 3 }
j5_limit = { lower = -6.28, upper = 6.28, velocity = 3.14, effort = 3 }
j6_limit = { lower = -6.28, upper = 6.28, velocity = 3.14, effort = 3 }
joint1 = joint.new("joint1", joint_axes1, "revolute", j1_limit)
joint2 = joint.new("joint2", joint_axes2, "revolute", j2_limit)
joint3 = joint.new("joint3", joint_axes3, "revolute", j3_limit)
joint4 = joint.new("joint4", joint_axes4, "revolute", j4_limit)
joint5 = joint.new("joint5", joint_axes5, "revolute", j5_limit)
joint6 = joint.new("joint6", joint_axes6, "revolute", j6_limit)
jointT = joint.new("jointT", joint_tool, "fixed")
urdf = link.new("base_link", base_link)
link1 = link.new("link1", sholder)
link2 = link.new("link2", upperarm)
link3 = link.new("link3", forearm)
link4 = link.new("link4", wrist1)
link5 = link.new("link5", wrist2)
link6 = link.new("link6", wrist3)
link_tool = link.new("link_tool", shape.new())
urdf:add(joint1):next(link1):add(joint2):next(link2):add(joint3):next(link3):add(joint4):next(link4):add(joint5):next(
    link5):add(joint6):next(link6):add(jointT):next(link_tool)
for _, arr in ipairs({ { base_link }, sholder, upperarm, upperarm, forearm, wrist1, wrist2, { wrist3 } }) do
    for _, value in ipairs(arr) do
        value:show()
    end
end
show({ joint_axes1, joint_axes2, joint_axes3, joint_axes4, joint_axes5, joint_axes6, joint_tool })
urdf:export({ name = 'myrobot', path = 'd:/', ros_version = 2 })
-- urdf:export({ name = 'myrobot_mujoco', path = 'd:/', mujoco = true }) -- 导出mujoco
--[[
--- ROS2使用方式: ---
sudo apt update
sudo apt install ros-$ROS_DISTRO-urdf-launch
mkdir -p ~/ws_ros2/src
cp -r /mnt/d/myrobot ~/ws_ros2/src/
cd ~/ws_ros2
colcon build --symlink-install
source install/setup.bash
ros2 launch urdf_launch display.launch.py urdf_package:=myrobot urdf_package_path:=urdf/myrobot.urdf
--- ROS1使用方式: ---
sudo apt update
sudo apt-get install ros-$ROS_DISTRO-urdf-tutorial
mkdir -p ~/ws_ros1/src
cp -r /mnt/d/myrobot ~/ws_ros1/src/
cd ~/ws_ros1
catkin_make
source devel/setup.bash
roslaunch urdf_tutorial display.launch model:='$(find myrobot)/urdf/myrobot.urdf'
--]]
