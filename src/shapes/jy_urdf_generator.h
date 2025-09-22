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
    const JyAxes &axes_;
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

    Joint(const std::string &name, const JyAxes &axes, const std::string &type) : name_(name), axes_(axes), type_(type) {}

    Joint(const std::string &name, const JyAxes &axes, const std::string &type, sol::table &limits) : name_(name), axes_(axes), type_(type) {
        if (!limits) return;
        if (limits["lower"].is<double>()) { limits_.lower = limits["lower"].get<double>(); }
        if (limits["upper"].is<double>()) { limits_.upper = limits["upper"].get<double>(); }
        if (limits["effort"].is<double>()) { limits_.effort = limits["effort"].get<double>(); }
        if (limits["velocity"].is<double>()) { limits_.velocity = limits["velocity"].get<double>(); }
    }

    Link &next(const std::string &name, const JyShape &shape) {
        child_ = std::make_shared<Link>(name, shape);
        return *child_;
    }
    Link &next(const Link &link) {
        child_ = std::make_shared<Link>(link);
        return *child_;
    }
};

class Link {
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

    Joint &add(const std::string &name, const JyAxes &axes, const std::string &type);

    Joint &add(const std::string &name, const JyAxes &axes, const std::string &type, sol::table &limits);

    void export_urdf(const std::string &robot_name) const;

    void export_urdf(const sol::table &params) const;

private:
    struct CommomData {
        std::string robot_name; // 机器人名称
        std::string path_meshes;// 网格文件路径
    };
    void export_urdf_impl(const std::string &robot_name, const std::string &root_path) const;

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
};

#endif