bx = box.new():color('green')
b1 = cylinder.new()
link1 = b1:copy():pos(0, 0, 2):color('#456789')
link2 = b1:copy():pos(0, 0, 3):color('#987654')
j1 = axes.new({ 0, 0, 2.5, 90, 0, 0 }, 3)
j2 = axes.new({ 0, 0, 3.5, 90, 0, 0 }, 3)
j1:show()
j2:show()
bx:mass(1)
b1:mass(2)
urdf = link.new("base_link", { bx, b1 })
urdf:add("joint1", j1, "revolute"):next('link1', link1):add("joint2", j2, "revolute"):next('link2', link2)
urdf:export({ name = 'myrobot', path = 'd:/' })
--[[
local r_base = 0.05;
local r_shell = 0.032;
local h_motor = 0.07;
shell = cylinder.new(r_shell, h_motor)
shell:fillet(0.005, { type = 'circle', min = { r_shell - 1e-5, -1e-5, h_motor - 1e-5 } });
shell:fuse(cylinder.new(r_shell, h_motor / 2):z(h_motor / 2):rx(90));
sholder = shell:copy():z(h_motor / 2);
forearm = shell:copy();
forearm:rot(90, 180, 0):pos(0, -h_motor / 2, h_motor);
base_link = cylinder.new(r_base, h_motor / 2);
base_link:cut(torus.new(r_base, r_base - r_shell):z(h_motor / 2));
show({ base_link, sholder, forearm })
--]]
--[[
ROS2使用方式:
sudo apt update
sudo apt install ros-$ROS_DISTRO-urdf-launch
mkdir -p ~/ws_ros2/src
cp -r /mnt/d/myrobot ~/ws_ros2/src/
cd ~/ws_ros2
colcon build --symlink-install
source install/setup.bash
ros2 launch urdf_launch display.launch.py urdf_package:=myrobot urdf_package_path:=urdf/myrobot.urdf
--]]
