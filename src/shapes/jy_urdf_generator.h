/*
 * Copyright (c) 2024. Li Jianbin. All rights reserved.
 * MIT License
 */
#ifndef JY_URDF_GENERATOR_H
#define JY_URDF_GENERATOR_H

#include "shapes/jy_axes.h"
#include "shapes/jy_shape.h"
#include <memory>

class Link;

class Joint {
public:
    JyAxes axes_;
    std::string name_;
    std::string type_;// fixed, revolute, continuous, prismatic, floating, planar
    std::shared_ptr<Link> child_;
    struct Limits {
        double lower = -3.14;
        double upper = 3.14;
        double effort = 100;
        double velocity = 1.0;
    };
    Limits limits_;

    Joint() : axes_(JyAxes()) {}

    Joint(const std::string &name, const JyAxes &axes, const std::string &type) : name_(name), axes_(axes), type_(type) {}

    Joint(const std::string &name, const JyAxes &axes, const std::string &type, const sol::table &limits) : name_(name), axes_(axes), type_(type) {
        if (!limits) return;
        if (limits["lower"].is<double>()) { limits_.lower = limits["lower"].get<double>(); }
        if (limits["upper"].is<double>()) { limits_.upper = limits["upper"].get<double>(); }
        if (limits["effort"].is<double>()) { limits_.effort = limits["effort"].get<double>(); }
        if (limits["velocity"].is<double>()) { limits_.velocity = limits["velocity"].get<double>(); }
    }

    Link &next(const Link &link) {
        child_ = std::make_shared<Link>(link);
        return *child_;
    }
};

class Link {
public:
    static void configure_usertype(sol::state &lua);

public:
    std::string name_;
    std::vector<std::shared_ptr<Joint>> joints_;
    std::vector<JyShape> shapes_;

    Link(const std::string &name, const JyShape &shape);
    Link(const std::string &name, const sol::table &shape_list);

    Joint &add(const Joint &joint) {
        joints_.push_back(std::make_shared<Joint>(joint));
        return *joints_.back();
    }

    void export_urdf(const std::string &robot_name) const;

    void export_urdf(const sol::table &params) const;

private:
    enum class ExtendedFile {
        URDF, // 普通URDF文件
        ROS1, // ROS1的额外文件
        ROS2, // ROS2的额外文件
        MUJOCO// MuJoCo XML文件
    };
    struct CommomData {
        std::string robot_name; // 机器人名称
        std::string path_meshes;// 网格文件路径
    };
    void export_urdf_impl(const std::string &robot_name, const std::string &root_path, const ExtendedFile &file_type = ExtendedFile::URDF) const;

    // 深度优先遍历（DFS）
    std::string traverseDFS(const Link &link, const JyAxes &parent_axes, const CommomData &data) const;

    /*
     * 处理关节（Joint）
     * @param joint 关节对象
     * @param parent_link 父连杆对象
     * @param parent_axes 父连杆的轴坐标(前一个关节的轴坐标)
     * @return 生成的URDF字符串
     */
    std::string handleJoint(const Joint &joint, const Link &parent_link, const JyAxes &parent_axes) const;

    /*
     * 处理连杆（Link）
     * @param link 连杆对象
     * @param parent_axes 父连杆的轴坐标(前一个关节的轴坐标)
     * @return 生成的URDF字符串
     */
    std::string handleLink(const Link &link, const JyAxes &parent_axes, const CommomData &data) const;

    // MuJoCo specific handlers
    std::string handleBody_MuJoCo(const Link &link, const Joint &parent_joint, const Joint &grand_joint, const CommomData &data, const int &space = 0) const;
};

#endif