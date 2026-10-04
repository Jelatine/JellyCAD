/*
 * Copyright (c) 2024. Li Jianbin. All rights reserved.
 * MIT License
 */
#include "jy_urdf_generator.h"
#include "robotics/jy_export_transaction.h"
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <vector>
#include <functional>

namespace fs = std::filesystem;

Link::Link(const std::string &name, const JyShape &shape) : name_(name) {
    shapes_.push_back(shape);
}


void Link::export_urdf(const std::string &robot_name) const {
    jelly::exportRobot(*this, {robot_name, fs::current_path().string(), Format::URDF});
}

Link::Link(const std::string &name, const std::vector<JyShape> &shapes) : name_(name), shapes_(shapes) {
    if (shapes_.empty()) throw std::runtime_error("Link without shape!");
}
void Link::export_urdf(const ExportOptions &options) const {
    jelly::exportRobot(*this, options);
}
