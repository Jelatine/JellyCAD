/*
 * Copyright (c) 2024. Li Jianbin. All rights reserved.
 * MIT License
 */
#ifndef JY_URDF_GENERATOR_H
#define JY_URDF_GENERATOR_H

#include "shapes/jy_axes.h"
#include "shapes/jy_shape.h"
#include <memory>
#include <map>
#include <unordered_map>

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

    Joint(const std::string &name, const JyAxes &axes, const std::string &type, std::unordered_map<std::string, double> limits = {}) : name_(name), axes_(axes), type_(type) {
        if (limits.empty()) return;
        if (limits.count("lower")) { limits_.lower = limits.at("lower"); }
        if (limits.count("upper")) { limits_.upper = limits.at("upper"); }
        if (limits.count("effort")) { limits_.effort = limits.at("effort"); }
        if (limits.count("velocity")) { limits_.velocity = limits.at("velocity"); }
    }

    /**
     * @brief 设置子连杆并返回其引用
     * @note link的值会被复制，已有子树使用共享所有权；
     *       如需继续构建子树，请使用本函数的返回值
     */
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
    Link(const std::string &name, const std::vector<JyShape> &shape_list);

    Joint &add(const Joint &joint) {
        joints_.push_back(std::make_shared<Joint>(joint));
        return *joints_.back();
    }

    void export_urdf(const std::string &robot_name) const;

    enum class Format { URDF, ROS1, ROS2, MUJOCO };
    struct ExportOptions { std::string name; std::string path; Format format = Format::URDF; };
    void export_urdf(const ExportOptions &options) const;

};

#endif
