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

namespace fs = std::filesystem;

static std::string cmakelists = R"(cmake_minimum_required(VERSION 3.8)
project(robot_description)
find_package(ament_cmake REQUIRED)
install(
  DIRECTORY urdf meshes
  DESTINATION share/${PROJECT_NAME}
)
ament_package()
)";

static std::string package_xml = R"(<?xml version="1.0"?>
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


Joint &Link::add(const std::string &name, const JyAxes &axes, const std::string &type) {
    return add(name, axes, type, sol::table());
}

Joint &Link::add(const std::string &name, const JyAxes &axes, const std::string &type, const sol::table &limits) {
    std::shared_ptr<Joint> joint = std::make_shared<Joint>(name, axes, type, limits);
    joints_.push_back(joint);
    return *joint;
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
    export_urdf_impl(robot_name, root_path);
}


void Link::export_urdf_impl(const std::string &robot_name, const std::string &root_path) const {
    fs::path path = fs::path(root_path) / robot_name;
    if (fs::exists(path) && fs::is_directory(path)) {
        fs::remove_all(path);
    }
    if (!fs::create_directories(path)) {
        throw std::runtime_error("unable to create robot description directory");
    }
    fs::path path_urdf = path / "urdf";
    if (!fs::create_directories(path_urdf)) {
        throw std::runtime_error("unable to create robot urdf directory");
    }
    fs::path path_meshes = path / "meshes";
    if (!fs::create_directories(path_meshes)) {
        throw std::runtime_error("unable to create robot meshes directory");
    }
    const auto file_path = path_urdf / (robot_name + ".urdf");
    JyAxes base_axes({0, 0, 0, 0, 0, 0}, 1);
    CommomData data{robot_name, path_meshes.string()};
    std::ofstream outFile(file_path);
    if (outFile.is_open()) {
        outFile << "<?xml version=\"1.0\"?>\n";
        outFile << "<robot name=\"" + robot_name + "\">\n\n";
        outFile << traverseDFS(*this, base_axes, data);
        outFile << "</robot>\n";
        outFile.close();
    } else {
        throw std::runtime_error("unable to open file for writing");
    }
    std::ofstream outFileCMakeLists(path / "CMakeLists.txt");
    if (outFileCMakeLists.is_open()) {
        outFileCMakeLists << replaceRobotDescription(cmakelists, robot_name);
        outFileCMakeLists.close();
    } else {
        throw std::runtime_error("unable to open file for writing");
    }
    std::ofstream outFilePackageXml(path / "package.xml");
    if (outFilePackageXml.is_open()) {
        outFilePackageXml << replaceRobotDescription(package_xml, robot_name);
        outFilePackageXml.close();
    } else {
        throw std::runtime_error("unable to open file for writing");
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
    const auto pose = parent_axes.link2joint(link.shapes_[0]);
    JyShape output_shape;
    std::vector<JyShape> shapes;
    for (int i = 0; i < link.shapes_.size(); i++) {
        JyShape copy_shape = link.shapes_[i];
        copy_shape.rot(pose[3] * 180 / M_PI, pose[4] * 180 / M_PI, pose[5] * 180 / M_PI);
        copy_shape.pos(pose[0], pose[1], pose[2]);
        if (!output_shape.s_.IsNull()) {
            output_shape.fuse(copy_shape);
        } else {
            output_shape = copy_shape;
        }
        shapes.push_back(copy_shape);
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
    file << "      <inertia ixx=\"" << inertial.inertia_tensor[0]
         << "\" iyy=\"" << inertial.inertia_tensor[1]
         << "\" izz=\"" << inertial.inertia_tensor[2]
         << "\" ixy=\"" << inertial.inertia_tensor[3]
         << "\" ixz=\"" << inertial.inertia_tensor[4]
         << "\" iyz=\"" << inertial.inertia_tensor[5] << "\"/>\n";
    file << "    </inertial>\n";

    // Visual properties
    const auto &rgba = link.shapes_[0].rgba();// 取第一个shape的rgba
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
    output_shape.export_stl_common(path_stl.generic_string(), false, 0.1);
    return file.str();
}