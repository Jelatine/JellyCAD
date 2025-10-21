-- 6自由度机械臂URDF建模及URDF导出示例
-- 通用
local r_shell = 32;
local h_motor = 90;
local offset = h_motor / 2 - r_shell
shell = cylinder.new(r_shell, h_motor)
shell:fillet(5, { type = 'circle', min = { r_shell - 1e-2, -1e-2, h_motor - 1e-2 } });
shell:fuse(cylinder.new(r_shell, h_motor / 2):z(h_motor / 2):rx(90));
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
sholder = shell:copy():z(h_base);
-- 上臂
local h_upperarm = 150
local r_upperarm = 20
local z_upperarm = h_base + h_motor / 2
upperarm = shell:copy();
upperarm:rot(90, 180, 0);
upperarm:fuse(get_pole(r_shell, r_upperarm, h_upperarm):pos(0, -h_motor / 2, h_motor / 2))
upperarm:fuse(shell:copy():rot(90, 0, 0):pos(0, 0, h_motor + h_upperarm))
upperarm:ry(-90):pos(0, -h_motor / 2, z_upperarm)
-- 前臂
local h_forearm = 120
local r_forearm = 20
local len_j2j3 = h_upperarm + h_motor
forearm = face.new(edge.new('circ', { 0, 0, 0 }, { 0, 0, 1 }, r_shell));
forearm:revol({ 0, -r_shell, 0 }, { 1, 0, 0 }, -90)
forearm:fuse(get_pole(r_shell, r_forearm, h_forearm))
forearm:fuse(shell:copy():rot(90, 0, 180):pos(0, -h_motor / 2, h_motor / 2 + h_forearm))
forearm:ry(-90):pos(-(len_j2j3 + r_shell), -offset, z_upperarm)
-- 手腕1
local len_j3j4 = h_forearm + h_motor / 2 + r_shell
local len_j2j4 = len_j2j3 + len_j3j4
wrist1 = shell:copy()
wrist1:rz(180):pos(-len_j2j4, -h_motor - offset, z_upperarm - h_motor / 2)
-- 手腕2
wrist2 = shell:copy()
wrist2:rot(-90, 0, 0):pos(-len_j2j4, r_shell - 2 * h_motor, z_upperarm - h_motor)
-- 手腕3
local h_flank = 10
wrist3 = cylinder.new(r_shell, h_flank):rot(90, 0, 0):pos(-len_j2j4, r_shell - 2 * h_motor, z_upperarm - h_motor)
-- 毫米单位转为米，生成URDF
base_link:scale(1e-3):color('#6495ED'):mass(0.1)
sholder:scale(1e-3):color('#8470FF'):mass(0.4)
upperarm:scale(1e-3):color('#FFC1C1'):mass(1)
forearm:scale(1e-3):color('#FFC100'):mass(0.8)
wrist1:scale(1e-3):color('#FF8247'):mass(0.4)
wrist2:scale(1e-3):color('#FFE7BA'):mass(0.4)
wrist3:scale(1e-3):color('#C1CDC1'):mass(0.1)
local d1 = z_upperarm * 1e-3
local a2 = -(h_upperarm + h_motor) * 1e-3
local a3 = -(h_forearm + h_motor / 2 + r_shell) * 1e-3
local d4 = (h_motor + offset) * 1e-3
local d5 = h_motor * 1e-3
local d6 = (h_motor / 2 + h_flank) * 1e-3

-- 打印DH参数的辅助函数
function print_dh_params(params_table, dh_type)
    print(string.format("\n========== %s Parameters ==========", dh_type))
    print(string.format("%-8s %-12s %-12s %-12s %-12s", "Joint", "alpha(rad)", "a(m)", "d(m)", "theta(rad)"))
    print(string.rep("-", 60))
    for i, params in ipairs(params_table) do
        print(string.format("%-8s %-12.3f %-12.3f %-12.3f %-12.3f",
            "Joint" .. i, params.alpha, params.a, params.d, params.theta))
    end
    print(string.rep("=", 60) .. "\n")
end

local is_mdh = true
local joint_axes = {}
if is_mdh then
    -- MDH参数表
    local mdh_params = {
        { alpha = 0,   a = 0,  d = d1, theta = 0 },
        { alpha = 90,  a = 0,  d = 0,  theta = 0 },
        { alpha = 0,   a = a2, d = 0,  theta = 0 },
        { alpha = 0,   a = a3, d = d4, theta = 0 },
        { alpha = 90,  a = 0,  d = d5, theta = 0 },
        { alpha = -90, a = 0,  d = d6, theta = 0 }
    }
    print_dh_params(mdh_params, "MDH")
    local prev = axes.new(0.1)
    for i = 1, #mdh_params do
        joint_axes[i] = prev:copy():mdh(mdh_params[i].alpha, mdh_params[i].a, mdh_params[i].d, mdh_params[i].theta)
        prev = joint_axes[i]
        prev:show()
    end
else
    -- SDH参数表，备注：SDH建模输出的URDF，在pinocchio计算rnea，输出关节力矩结果会有问题
    local sdh_params = {
        { alpha = 90,  a = 0,  d = d1, theta = 0 },
        { alpha = 0,   a = a2, d = 0,  theta = 0 },
        { alpha = 0,   a = a3, d = 0,  theta = 0 },
        { alpha = 90,  a = 0,  d = d4, theta = 0 },
        { alpha = -90, a = 0,  d = d5, theta = 0 },
        { alpha = 0,   a = 0,  d = d6, theta = 0 }
    }
    print_dh_params(sdh_params, "SDH")
    local prev = axes.new(0.1)
    for i = 1, #sdh_params do
        joint_axes[i] = prev:copy():sdh(sdh_params[i].alpha, sdh_params[i].a, sdh_params[i].d, sdh_params[i].theta)
        prev = joint_axes[i]
        prev:show()
    end
end
j1_limit = { lower = -6.28, upper = 6.28, velocity = 3.14, effort = 9 }
j2_limit = { lower = -6.28, upper = 6.28, velocity = 3.14, effort = 9 }
j3_limit = { lower = -3.14, upper = 3.14, velocity = 3.14, effort = 9 }
j4_limit = { lower = -6.28, upper = 6.28, velocity = 3.14, effort = 3 }
j5_limit = { lower = -6.28, upper = 6.28, velocity = 3.14, effort = 3 }
j6_limit = { lower = -6.28, upper = 6.28, velocity = 3.14, effort = 3 }
urdf = link.new("base_link", base_link)
joint1 = urdf:add(joint.new("joint1", joint_axes[1], "revolute", j1_limit))
link1 = joint1:next(link.new("link1", sholder))
joint2 = link1:add(joint.new("joint2", joint_axes[2], "revolute", j2_limit))
link2 = joint2:next(link.new("link2", upperarm))
joint3 = link2:add(joint.new("joint3", joint_axes[3], "revolute", j3_limit))
link3 = joint3:next(link.new("link3", forearm))
joint4 = link3:add(joint.new("joint4", joint_axes[4], "revolute", j4_limit))
link4 = joint4:next(link.new("link4", wrist1))
joint5 = link4:add(joint.new("joint5", joint_axes[5], "revolute", j5_limit))
link5 = joint5:next(link.new("link5", wrist2))
joint6 = link5:add(joint.new("joint6", joint_axes[6], "revolute", j6_limit))
link6 = joint6:next(link.new("link6", wrist3))
show({ base_link, sholder, upperarm, forearm, wrist1, wrist2, wrist3 })
urdf:export({ name = 'myrobot', path = 'd:/', ros_version = 2 })
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
