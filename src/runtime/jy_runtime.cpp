#include "jy_runtime.h"
#include "jy_bindings.h"
#include <Standard_Failure.hxx>
#include <chrono>
#include <filesystem>
#include <mutex>

namespace jelly {
const char *luaVersion() { return LUA_VERSION; }
namespace {
std::mutex executionMutex; // cwd is process-global: only one runtime may execute.
struct DirectoryGuard {
    std::filesystem::path original = std::filesystem::current_path();
    ~DirectoryGuard() { std::error_code ignored; std::filesystem::current_path(original, ignored); }
};
void cancelHook(lua_State *state, lua_Debug *) {
    auto *cancel = *static_cast<std::atomic<bool> **>(lua_getextraspace(state));
    if (cancel && cancel->load()) luaL_error(state, "Script cancelled");
}
}
RunResult execute(const RunRequest &request, std::atomic<bool> &cancel, const EventSink &sink) {
    const auto started = std::chrono::steady_clock::now();
    RunResult result;
    result.id = request.id;
    try {
        std::unique_lock<std::mutex> executionLock(executionMutex, std::try_to_lock);
        if (!executionLock.owns_lock()) throw std::runtime_error("Another script is running");
        DirectoryGuard directory;
        const auto source = request.isFile ? std::filesystem::absolute(request.source).string() : request.source;
        const auto workDir = request.directory.empty()
            ? (request.isFile ? std::filesystem::path(source).parent_path() : directory.original)
            : std::filesystem::absolute(request.directory);
        std::filesystem::current_path(workDir);
        sol::state lua;
        lua.open_libraries();
        *static_cast<std::atomic<bool> **>(lua_getextraspace(lua.lua_state())) = &cancel;
        lua_sethook(lua.lua_state(), cancelHook, LUA_MASKCOUNT, 100);
        if (!lua_checkstack(lua.lua_state(), 1000)) throw std::runtime_error("Lua stack overflow");
        auto shape = bindShape(lua);
        auto axes = bindAxes(lua);
        bindEdges(lua);
        bindFaces(lua);
        bindPrimitives(lua);
        bindRobot(lua);
        auto showShape = [&](const JyShape &s) { if (sink && !cancel.load()) sink(s.snapshot()); };
        auto showAxes = [&](const JyAxes &a) { if (sink && !cancel.load()) sink(a); };
        shape["show"] = [&](JyShape &self) -> JyShape & { showShape(self); return self; };
        axes["show"] = [&](const JyAxes &self) { showAxes(self); };
        lua["show"] = sol::overload(showShape, showAxes, [&](const sol::table &list) {
            for (size_t i = 1; i <= list.size() && !cancel.load(); ++i) {
                if (list[i].is<JyShape>()) showShape(list[i].get<JyShape>());
                else if (list[i].is<JyAxes>()) showAxes(list[i].get<JyAxes>());
                else throw std::runtime_error("Wrong shape type");
            }
        });
        lua["print"] = [&](sol::variadic_args args) {
            std::string text;
            sol::function toString = lua["tostring"];
            bool first = true;
            for (const auto &arg : args) {
                if (!first) text += '\t';
                first = false;
                text += toString(arg).get<std::string>();
            }
            if (sink && !cancel.load()) sink(std::move(text));
        };
        auto arg = lua.create_table();
        arg[0] = request.isFile ? source : "=(code)";
        lua["arg"] = arg;
        lua["package"]["path"] = lua["package"]["path"].get<std::string>() + ";" + workDir.generic_string() + "/?.lua";
        if (!cancel.load()) {
            const auto evaluated = request.isFile
                ? lua.safe_script_file(source, sol::script_pass_on_error)
                : lua.safe_script(source, sol::script_pass_on_error);
            if (!evaluated.valid()) throw sol::error(evaluated);
        }
        result.status = RunStatus::Success;
        result.message = "Script completed";
    } catch (const Standard_Failure &error) {
        result.message = error.GetMessageString() ? error.GetMessageString() : "OpenCASCADE error";
    } catch (const std::exception &error) {
        result.message = error.what();
    } catch (...) {
        result.message = "Unknown script error";
    }
    if (cancel.load()) {
        result.status = RunStatus::Cancelled;
        result.message = "Script cancelled";
    }
    result.elapsedMs = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - started).count();
    return result;
}
}
