#pragma once
#include "jy_axes.h"
#include <atomic>
#include <cstdint>
#include <functional>
#include <string>
#include <variant>

namespace jelly {
const char *luaVersion();
enum class RunSource { Explicit, Preview, CommandLine };
struct RunRequest {
    std::uint64_t id = 0;
    std::string source;
    bool isFile = true;
    std::string directory;
    RunSource trigger = RunSource::Explicit;
};
enum class RunStatus { Success, Failed, Cancelled };
struct RunResult {
    std::uint64_t id = 0;
    RunStatus status = RunStatus::Failed;
    std::string message;
    long long elapsedMs = 0;
};
using RunEvent = std::variant<std::string, JyShape, JyAxes>;
using EventSink = std::function<void(RunEvent)>;
// Synchronous, GUI-independent engine. All Lua objects live inside this call.
RunResult execute(const RunRequest &request, std::atomic<bool> &cancel, const EventSink &sink = {});
}
