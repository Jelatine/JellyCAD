#pragma once
#include "shapes/jy_urdf_generator.h"
#include <filesystem>
#include <functional>
namespace jelly {
void validateRobot(const Link &root, const std::string &name);
// Generate a complete candidate before changing any existing output file.
void exportTransaction(const std::filesystem::path &destination,
                       const std::function<void(const std::filesystem::path &)> &generate);
void exportRobot(const Link &root, const Link::ExportOptions &options);
}
