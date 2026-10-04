#include "jy_export_transaction.h"
#include "jy_robot_writer.h"
#include <chrono>
#include <fstream>
#include <set>
#include <cmath>
#include <atomic>

namespace jelly {
namespace fs = std::filesystem;
namespace {
bool safeName(const std::string &name) {
    return !name.empty() && name != "." && name != ".." && name.find_first_not_of("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_.-") == std::string::npos;
}
void checkRelative(const fs::path &path) {
    if (path.empty() || path.is_absolute()) throw std::runtime_error("Invalid export manifest path");
    for (const auto &part : path) if (!safeName(part.string())) throw std::runtime_error("Invalid export manifest path");
}
void checkDestination(const fs::path &root, const fs::path &relative) {
    auto current = root;
    if (fs::is_symlink(fs::symlink_status(root))) throw std::runtime_error("Export destination is a symbolic link");
    for (const auto &part : relative) {
        current /= part;
        if (fs::is_symlink(fs::symlink_status(current))) throw std::runtime_error("Export path contains a symbolic link");
    }
}
struct Staging {
    fs::path path;
    bool retain = false;
    ~Staging() { if (!retain) { std::error_code ignored; fs::remove_all(path, ignored); } }
};
}
void validateRobot(const Link &root, const std::string &name) {
    if (!safeName(name)) throw std::runtime_error("Invalid robot name");
    std::set<const Link *> visited;
    std::set<std::string> links, joints;
    std::function<void(const Link &)> visit = [&](const Link &link) {
        if (!visited.insert(&link).second) throw std::runtime_error("Robot must be a tree");
        if (!safeName(link.name_) || !links.insert(link.name_).second) throw std::runtime_error("Invalid or duplicate link name");
        for (const auto &joint : link.joints_) {
            if (!joint || !joint->child_) throw std::runtime_error("Joint has no child link");
            if (!safeName(joint->name_) || !joints.insert(joint->name_).second) throw std::runtime_error("Invalid or duplicate joint name");
            if (joint->type_ != "fixed" && joint->type_ != "revolute" && joint->type_ != "continuous" && joint->type_ != "prismatic") throw std::runtime_error("Unsupported joint type");
            const auto &limit = joint->limits_;
            if (!std::isfinite(limit.lower) || !std::isfinite(limit.upper) || !std::isfinite(limit.effort) || !std::isfinite(limit.velocity) || limit.lower > limit.upper || limit.effort < 0 || limit.velocity < 0) throw std::runtime_error("Invalid joint limits");
            visit(*joint->child_);
        }
    };
    visit(root);
}
void exportTransaction(const fs::path &destination, const std::function<void(const fs::path &)> &generate) {
    const auto target = fs::absolute(destination);
    if (fs::is_symlink(fs::symlink_status(target)) || (fs::exists(target) && !fs::is_directory(target))) throw std::runtime_error("Invalid export destination");
    fs::create_directories(target.parent_path());
    static std::atomic<unsigned> sequence{0};
    Staging staging;
    do {
        staging.path = target.parent_path() / (".jellycad-stage-" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()) + "-" + std::to_string(sequence++));
    } while (!fs::create_directory(staging.path));
    const auto candidate = staging.path / "candidate";
    fs::create_directory(candidate);
    generate(candidate);
    std::set<fs::path> files;
    for (const auto &entry : fs::recursive_directory_iterator(candidate)) {
        if (entry.is_symlink()) throw std::runtime_error("Generated symbolic link is unsupported");
        if (entry.is_regular_file()) files.insert(entry.path().lexically_relative(candidate));
    }
    const fs::path manifest = ".jellycad-files";
    checkDestination(target, manifest);
    std::set<fs::path> affected = files;
    std::ifstream previous(target / manifest);
    std::string line;
    while (std::getline(previous, line)) {
        checkRelative(fs::path(line));
        affected.insert(fs::path(line));
    }
    std::ofstream nextManifest(candidate / manifest);
    nextManifest.exceptions(std::ios::badbit | std::ios::failbit);
    for (const auto &file : files) nextManifest << file.generic_string() << '\n';
    nextManifest.close();
    files.insert(manifest);
    affected.insert(manifest);
    for (const auto &file : affected) {
        checkRelative(file);
        checkDestination(target, file);
        if (fs::exists(target / file) && !fs::is_regular_file(target / file)) throw std::runtime_error("Export file conflicts with existing directory");
    }
    fs::create_directories(target);
    std::vector<fs::path> backedUp, installed;
    try {
        for (const auto &file : affected) {
            if (!fs::exists(target / file)) continue;
            fs::create_directories((staging.path / "backup" / file).parent_path());
            fs::rename(target / file, staging.path / "backup" / file);
            backedUp.push_back(file);
        }
        for (const auto &file : files) {
            fs::create_directories((target / file).parent_path());
            fs::rename(candidate / file, target / file);
            installed.push_back(file);
        }
    } catch (...) {
        try {
            for (auto i = installed.rbegin(); i != installed.rend(); ++i) fs::remove(target / *i);
            for (auto i = backedUp.rbegin(); i != backedUp.rend(); ++i) fs::rename(staging.path / "backup" / *i, target / *i);
        } catch (...) {
            staging.retain = true;
            throw std::runtime_error("Export rollback failed; backups retained at " + staging.path.string());
        }
        throw;
    }
}
void exportRobot(const Link &root, const Link::ExportOptions &options) {
    validateRobot(root, options.name);
    exportTransaction(fs::path(options.path) / options.name, [&](const fs::path &candidate) {
        RobotWriter(root).write(options.name, candidate.string(), options.format);
    });
}
}
