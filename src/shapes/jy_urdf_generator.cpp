/*
 * Copyright (c) 2024. Li Jianbin. All rights reserved.
 * MIT License
 */
#include "jy_urdf_generator.h"
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <vector>

void Link::configure_usertype(sol::state &lua) {
    auto link_user = lua.new_usertype<Link>("link", sol::constructors<Link(const std::string &, const JyShape &),
                                                                      Link(const std::string &, const sol::table &)>());
    link_user["export"] = sol::overload(
            static_cast<void (Link::*)(const std::string &robot_name) const>(&Link::export_urdf),
            static_cast<void (Link::*)(const sol::table &params) const>(&Link::export_urdf));
    link_user["add"] = &Link::add;
    auto joint_user = lua.new_usertype<Joint>("joint", sol::constructors<Joint(const std::string &, const JyAxes &, const std::string &, std::unordered_map<std::string, double>)>());
    joint_user["next"] = &Joint::next;
}

namespace fs = std::filesystem;

static std::string ros2_cmakelists = R"(cmake_minimum_required(VERSION 3.8)
project(robot_description)
find_package(ament_cmake REQUIRED)
install(
  DIRECTORY urdf meshes
  DESTINATION share/${PROJECT_NAME}
)
ament_package()
)";

static std::string ros2_package_xml = R"(<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>robot_description</name>
  <version>0.0.0</version>
  <description>TODO: Package description</description>
  <maintainer email="TODO@email.com">JellyCAD</maintainer>
  <license>TODO: License declaration</license>
  <buildtool_depend>ament_cmake</buildtool_depend>
  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>
  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
)";

static std::string ros1_cmakelists = R"(cmake_minimum_required(VERSION 3.0.2)
project(robot_description)
find_package(catkin REQUIRED)
catkin_package()
install(
  DIRECTORY urdf meshes launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
)";

static std::string ros1_package_xml = R"(<?xml version="1.0"?>
<package format="2">
  <name>robot_description</name>
  <version>0.0.0</version>
  <description>TODO: Package description</description>
  <maintainer email="TODO@email.com">JellyCAD</maintainer>
  <license>TODO: License declaration</license>
  <buildtool_depend>catkin</buildtool_depend>
</package>
)";

static std::string replaceRobotDescription(std::string_view str, std::string_view to) {
    std::string_view from = "robot_description";
    std::string result(str);
    size_t pos = result.find(from);
    if (pos != std::string::npos) {
        result.replace(pos, from.length(), to);
    }
    return result;
}


Link::Link(const std::string &name, const JyShape &shape) : name_(name) {
    shapes_.push_back(shape);
}
Link::Link(const std::string &name, const sol::table &shape_list) : name_(name) {
    for (int i = 1; i <= shape_list.size(); ++i) {
        if (!shape_list[i].is<JyShape>()) { throw std::runtime_error("Wrong type!"); }
        const JyShape &s = shape_list[i];
        shapes_.push_back(s);
    }
    if (shapes_.empty()) { throw std::runtime_error("Link without shape!"); }
}

void Link::export_urdf(const std::string &robot_name) const {
    export_urdf_impl(robot_name, fs::current_path().string());
}


void Link::export_urdf(const sol::table &params) const {
    if (!params["name"].valid() || !params["path"].valid()) {
        throw std::runtime_error("name or path is not provided");
    }
    std::string robot_name = params["name"].get<std::string>();
    std::string root_path = params["path"].get<std::string>();
    ExtendedFile file_type = ExtendedFile::URDF;
    if (params["ros_version"].valid()) {
        file_type = params["ros_version"].get<int>() == 1 ? ExtendedFile::ROS1 : ExtendedFile::ROS2;
    }
    if (params["mujoco"].valid() && params["mujoco"].get<bool>()) {
        file_type = ExtendedFile::MUJOCO;
    }
    export_urdf_impl(robot_name, root_path, file_type);
}


void Link::export_urdf_impl(const std::string &robot_name, const std::string &root_path, const ExtendedFile &file_type) const {
    fs::path path = fs::path(root_path) / robot_name;
    if (fs::exists(path) && fs::is_directory(path)) {
        fs::remove_all(path);
    }
    if (!fs::create_directories(path)) {
        throw std::runtime_error("unable to create robot description directory");
    }

    // 根据文件类型选择不同的子目录结构
    fs::path path_output_dir;
    fs::path file_path;
    if (file_type == ExtendedFile::MUJOCO) {
        path_output_dir = path;
        file_path = path / (robot_name + ".xml");
    } else {
        path_output_dir = path / "urdf";
        if (!fs::create_directories(path_output_dir)) {
            throw std::runtime_error("unable to create robot urdf directory");
        }
        file_path = path_output_dir / (robot_name + ".urdf");
    }

    fs::path path_meshes = path / "meshes";
    if (!fs::create_directories(path_meshes)) {
        throw std::runtime_error("unable to create robot meshes directory");
    }

    JyAxes base_axes({0, 0, 0, 0, 0, 0}, 1);
    CommomData data{robot_name, path_meshes.string()};
    std::ofstream outFile(file_path);
    if (outFile.is_open()) {
        if (file_type == ExtendedFile::MUJOCO) {
            // 生成MuJoCo XML
            outFile << "<mujoco model=\"" + robot_name + "\">\n";
            outFile << "  <compiler angle=\"radian\" meshdir=\"meshes\" autolimits=\"true\"/>\n\n";
            outFile << "  <option timestep=\"0.001\" gravity=\"0 0 -9.81\"/>\n\n";

            outFile << "  <statistic center=\"0 0 0.5\" extent=\"1.5\"/>\n\n";
            // 添加视觉配置
            outFile << "  <visual>\n";
            outFile << "    <rgba haze=\"0.15 0.25 0.35 1\"/>\n";
            outFile << "    <quality shadowsize=\"2048\"/>\n";
            outFile << "    <map stiffness=\"700\" shadowscale=\"0.5\"/>\n";
            outFile << "  </visual>\n\n";
            // 添加默认配置
            outFile << "  <default>\n";
            outFile << "    <joint damping=\"1\" armature=\"0.1\"/>\n";
            outFile << "    <geom contype=\"1\" conaffinity=\"1\" friction=\"1 0.5 0.5\"/>\n";
            outFile << "    <motor ctrllimited=\"true\"/>\n";
            outFile << "    <position kp=\"200\" kv=\"1\"/>\n";
            outFile << "  </default>\n\n";

            // 收集所有非空形状的link名称用于asset声明
            std::vector<std::string> all_link_names;
            std::function<void(const Link &)> collect_links = [&](const Link &link) {
                // 检查link是否有非空形状
                bool has_valid_shape = false;
                for (const auto &shape: link.shapes_) {
                    if (!shape.s_.IsNull()) {
                        has_valid_shape = true;
                        break;
                    }
                }
                if (has_valid_shape) {
                    all_link_names.push_back(link.name_);
                }
                for (const auto &joint: link.joints_) {
                    if (joint->child_) {
                        collect_links(*joint->child_);
                    }
                }
            };
            collect_links(*this);

            outFile << "  <asset>\n";
            outFile << "    <texture type=\"skybox\" builtin=\"gradient\" rgb1=\"0.3 0.5 0.7\" rgb2=\"0 0 0\" width=\"512\" height=\"512\"/>\n";
            outFile << "    <texture name=\"grid\" type=\"2d\" builtin=\"checker\" rgb1=\"0.2 0.3 0.4\" rgb2=\"0.1 0.15 0.2\" width=\"512\" height=\"512\" mark=\"cross\" markrgb=\"1 1 1\"/>\n";
            outFile << "    <material name=\"grid\" texture=\"grid\" texrepeat=\"1 1\" texuniform=\"true\" reflectance=\"0.2\"/>\n";
            for (const auto &link_name: all_link_names) {
                outFile << "    <mesh name=\"" << link_name << "\" file=\"" << link_name << ".stl\"/>\n";
            }
            outFile << "  </asset>\n\n";
            outFile << "  <worldbody>\n";
            outFile << "    <geom name=\"floor\" size=\"0 0 0.05\" type=\"plane\" material=\"grid\" condim=\"3\" friction=\"0.1 0.005 0.0001\"/>\n";
            outFile << "    <light directional=\"true\" diffuse=\"0.8 0.8 0.8\" specular=\"0.2 0.2 0.2\" pos=\"0 0 5\" dir=\"0 0 -1\"/>\n";
            outFile << "    <light directional=\"true\" diffuse=\"0.4 0.4 0.4\" specular=\"0.1 0.1 0.1\" pos=\"0 0 4\" dir=\"0 1 -1\"/>\n";
            struct DriveInfo {
                std::string name;
                double effort;
                double lower;
                double upper;
            };
            // 收集所有需要驱动的关节名称
            std::vector<DriveInfo> drive_infos;
            std::function<void(const Link &)> collect_joints = [&](const Link &link) {
                for (const auto &joint: link.joints_) {
                    // 只为revolute、continuous和prismatic类型的关节添加驱动器
                    if (joint->type_ == "revolute" || joint->type_ == "continuous" || joint->type_ == "prismatic") {
                        drive_infos.push_back({joint->name_, joint->limits_.effort, joint->limits_.lower, joint->limits_.upper});
                    }
                    if (joint->child_) {
                        collect_joints(*joint->child_);
                    }
                }
            };
            collect_joints(*this);

            outFile << handleBody_MuJoCo(*this, Joint(), Joint(), data);
            outFile << "  </worldbody>\n\n";
            outFile << "  <actuator>\n";
            for (const auto &drive: drive_infos) {
                outFile << "    <position name=\"" << drive.name << "_pos\" joint=\"" << drive.name
                        << "\" ctrlrange=\"" << drive.lower << " " << drive.upper << "\" />\n";
            }
            for (const auto &drive: drive_infos) {
                outFile << "    <motor name=\"" << drive.name << "_motor\" joint=\"" << drive.name
                        << "\" gear=\"1\" ctrlrange=\"-" << drive.effort << " " << drive.effort << "\" />\n";
            }
            outFile << "  </actuator>\n";
            outFile << "</mujoco>\n";
        } else {
            // 生成URDF
            outFile << "<?xml version=\"1.0\"?>\n";
            outFile << "<robot name=\"" + robot_name + "\">\n\n";
            outFile << traverseDFS(*this, base_axes, data);
            outFile << "</robot>\n";
        }
        outFile.close();
    } else {
        throw std::runtime_error("unable to open file for writing");
    }
    if (file_type == ExtendedFile::ROS1) {
        std::ofstream outFileCMakeLists(path / "CMakeLists.txt");
        if (outFileCMakeLists.is_open()) {
            outFileCMakeLists << replaceRobotDescription(ros1_cmakelists, robot_name);
            outFileCMakeLists.close();
        } else {
            throw std::runtime_error("unable to open file for writing");
        }
        std::ofstream outFilePackageXml(path / "package.xml");
        if (outFilePackageXml.is_open()) {
            outFilePackageXml << replaceRobotDescription(ros1_package_xml, robot_name);
            outFilePackageXml.close();
        } else {
            throw std::runtime_error("unable to open file for writing");
        }
    } else if (file_type == ExtendedFile::ROS2) {
        std::ofstream outFileCMakeLists(path / "CMakeLists.txt");
        if (outFileCMakeLists.is_open()) {
            outFileCMakeLists << replaceRobotDescription(ros2_cmakelists, robot_name);
            outFileCMakeLists.close();
        } else {
            throw std::runtime_error("unable to open file for writing");
        }
        std::ofstream outFilePackageXml(path / "package.xml");
        if (outFilePackageXml.is_open()) {
            outFilePackageXml << replaceRobotDescription(ros2_package_xml, robot_name);
            outFilePackageXml.close();
        } else {
            throw std::runtime_error("unable to open file for writing");
        }
    }
}


std::string Link::traverseDFS(const Link &link, const JyAxes &parent_axes, const CommomData &data) const {
    std::string file_doc;
    file_doc.append(handleLink(link, parent_axes, data));
    // 遍历所有Joint及其子Link
    for (const auto &joint: link.joints_) {
        if (!joint->child_) {
            continue;
        }
        file_doc.append(handleJoint(*joint, link, parent_axes));
        // 递归遍历子Link
        file_doc.append(traverseDFS(*joint->child_, joint->axes_, data));
    }
    return file_doc;
}


std::string Link::handleJoint(const Joint &joint, const Link &parent_link, const JyAxes &parent_axes) const {
    // 计算当前关节坐标系相对于父连杆的偏移位姿
    const auto &pose = joint.axes_.joint2joint(parent_axes);
    std::stringstream file;
    file << std::fixed << std::setprecision(3);
    file << "  <joint name=\"" << joint.name_ << "\" type=\"" << joint.type_ << "\">\n";
    file << "    <parent link=\"" << parent_link.name_ << "\"/>\n";
    file << "    <child link=\"" << joint.child_->name_ << "\"/>\n";
    file << "    <origin xyz=\"" << pose[0] << " " << pose[1] << " " << pose[2]
         << "\" rpy=\"" << pose[3] << " " << pose[4] << " " << pose[5] << "\"/>\n";
    if (joint.type_ == "revolute" || joint.type_ == "prismatic") {
        file << "    <axis xyz=\"0 0 1\"/>\n";// 旋转轴为坐标系的Z轴
        file << "    <limit lower=\"" << joint.limits_.lower
             << "\" upper=\"" << joint.limits_.upper
             << "\" effort=\"" << joint.limits_.effort
             << "\" velocity=\"" << joint.limits_.velocity << "\"/>\n";
    } else if (joint.type_ == "continuous" || joint.type_ == "floating" || joint.type_ == "planar") {
        file << "    <axis xyz=\"0 0 1\"/>\n";
    } else if (joint.type_ == "fixed") {
    } else {
        throw std::runtime_error("joint type not support");
    }
    file << "  </joint>\n\n";
    return file.str();
}


std::string Link::handleLink(const Link &link, const JyAxes &parent_axes, const CommomData &data) const {
    std::vector<JyShape> shapes;
    for (int i = 0; i < link.shapes_.size(); i++) {
        JyShape copy_shape = link.shapes_[i];
        const auto pose = parent_axes.link2joint(copy_shape);
        copy_shape.rot(pose[3] * 180 / M_PI, pose[4] * 180 / M_PI, pose[5] * 180 / M_PI);
        copy_shape.pos(pose[0], pose[1], pose[2]);
        shapes.push_back(copy_shape);
    }
    JyShape output_shape = JyShape::make_compound(shapes);
    if (output_shape.s_.IsNull()) {
        return "  <link name=\"" + link.name_ + "\"/>\n\n";
    }
    std::stringstream file;
    file << std::fixed << std::setprecision(3);
    file << "  <link name=\"" << link.name_ << "\">\n";
    // Inertial properties
    const auto &inertial = JyShape::inertial(shapes);
    file << "    <inertial>\n";
    file << "      <mass value=\"" << inertial.mass << "\"/>\n";
    file << "      <origin xyz=\"" << inertial.center_of_mass[0] << " " << inertial.center_of_mass[1] << " " << inertial.center_of_mass[2]
         << "\" rpy=\"0 0 0\"/>\n";
    file << "      <inertia ixx=\"" << std::setprecision(6) << inertial.inertia_tensor[0]
         << "\" iyy=\"" << inertial.inertia_tensor[1]
         << "\" izz=\"" << inertial.inertia_tensor[2]
         << "\" ixy=\"" << inertial.inertia_tensor[3]
         << "\" ixz=\"" << inertial.inertia_tensor[4]
         << "\" iyz=\"" << inertial.inertia_tensor[5] << std::setprecision(3) << "\"/>\n";
    file << "    </inertial>\n";

    // Visual properties
    const auto &rgba = output_shape.rgba();// 取第一个shape的rgba
    const auto relate_path = data.robot_name + "/meshes/" + link.name_ + ".stl";
    const auto mesh_path = "package://" + relate_path;
    file << "    <visual>\n";
    file << "      <origin xyz=\"0 0 0\" rpy=\"0 0 0\"/>\n";
    file << "      <geometry>\n";
    file << "        <mesh filename=\"" << mesh_path << "\"/>\n";
    file << "      </geometry>\n";
    file << "      <material name=\"\">\n";
    file << "        <color rgba=\"" << rgba[0] << " " << rgba[1] << " " << rgba[2] << " " << rgba[3] << "\"/>\n";
    file << "      </material>\n";
    file << "    </visual>\n";

    // // Collision properties
    file << "    <collision>\n";
    file << "      <origin xyz=\"0 0 0\" rpy=\"0 0 0\"/>\n";
    file << "      <geometry>\n";
    file << "        <mesh filename=\"" << mesh_path << "\"/>\n";
    file << "      </geometry>\n";
    file << "    </collision>\n";

    file << "  </link>\n\n";

    fs::path path_stl = fs::path(data.path_meshes) / (link.name_ + ".stl");
    std::cout << "export mesh to: " << path_stl.generic_string() << std::endl;
    output_shape.export_stl(path_stl.generic_string());
    return file.str();
}


std::string Link::handleBody_MuJoCo(const Link &link, const Joint &parent_joint, const Joint &grand_joint, const CommomData &data, const int &space) const {
    std::vector<JyShape> shapes;
    for (int i = 0; i < link.shapes_.size(); i++) {
        JyShape copy_shape = link.shapes_[i];
        const auto pose = parent_joint.axes_.link2joint(copy_shape);
        copy_shape.rot(pose[3] * 180 / M_PI, pose[4] * 180 / M_PI, pose[5] * 180 / M_PI);
        copy_shape.pos(pose[0], pose[1], pose[2]);
        shapes.push_back(copy_shape);
    }
    JyShape output_shape = JyShape::make_compound(shapes);
    std::stringstream file;
    file << std::string(space, ' ') << "    <body name=\"" << link.name_ << "\"";
    const auto &pose = parent_joint.axes_.joint2joint(grand_joint.axes_);
    file << " pos=\"" << pose[0] << " " << pose[1] << " " << pose[2] << "\" euler=\"" << pose[3] << " " << pose[4] << " " << pose[5] << "\">\n";
    // 计算惯性属性
    const auto &inertial = JyShape::inertial(shapes);
    file << std::string(space, ' ') << std::fixed << std::setprecision(3) << "      <inertial pos=\""
         << inertial.center_of_mass[0] << " "
         << inertial.center_of_mass[1] << " "
         << inertial.center_of_mass[2]
         << "\" mass=\"" << inertial.mass << "\" diaginertia=\"" << std::fixed
         << std::setprecision(6) << inertial.inertia_tensor[0] << " "
         << inertial.inertia_tensor[1] << " "
         << inertial.inertia_tensor[2] << std::defaultfloat << "\"/>\n";
    const auto &rgba = output_shape.rgba();
    if (!output_shape.s_.IsNull()) {
        file << std::string(space, ' ') << "      <geom type=\"mesh\" mesh=\"" << link.name_ << "\" rgba=\""
             << rgba[0] << " " << rgba[1] << " " << rgba[2] << " " << rgba[3] << "\"/>\n";
    }
    if (!parent_joint.type_.empty()) {
        std::string mujoco_type;
        if (parent_joint.type_ == "revolute" || parent_joint.type_ == "continuous") {
            mujoco_type = "hinge";
        } else if (parent_joint.type_ == "prismatic") {
            mujoco_type = "slide";
        } else if (parent_joint.type_ == "fixed") {
            // fixed类型不需要创建joint，由body嵌套表示
            return "";
        } else if (parent_joint.type_ == "floating") {
            mujoco_type = "free";
        } else if (parent_joint.type_ == "planar") {
            // MuJoCo没有直接的planar类型，可以用两个slide joint模拟
            // 这里简化处理，跳过
            return "";
        } else {
            throw std::runtime_error("joint type not support for MuJoCo: " + parent_joint.type_);
        }
        file << std::string(space, ' ') << "      <joint name=\"" << parent_joint.name_ << "\" type=\"" << mujoco_type << "\"";
        file << " axis=\"0 0 1\"";// Z轴为关节轴
        // 限位（仅对hinge和slide有效）
        if (parent_joint.type_ == "revolute" || parent_joint.type_ == "prismatic") {
            file << " range=\"" << parent_joint.limits_.lower << " " << parent_joint.limits_.upper << "\"";
        }
        file << "/>\n";
    }
    // 遍历子关节
    for (const auto &joint: link.joints_) {
        if (!joint->child_) { continue; }
        file << handleBody_MuJoCo(*joint->child_, *joint, parent_joint, data, space + 2);
    }
    file << std::string(space, ' ') << "    </body>\n";
    if (!output_shape.s_.IsNull()) {
        fs::path path_stl = fs::path(data.path_meshes) / (link.name_ + ".stl");
        std::cout << "export mesh to: " << path_stl.generic_string() << std::endl;
        output_shape.export_stl(path_stl.generic_string());
    }
    return file.str();
}