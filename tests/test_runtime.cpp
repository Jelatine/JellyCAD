#include <gtest/gtest.h>
#include "runtime/jy_runtime.h"
#include "test_support.h"
#include <QTemporaryDir>
#include <filesystem>
#include <future>
#include <thread>
#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>

using namespace jelly;
namespace {
RunResult code(const std::string &source, const EventSink &sink = {}) {
    std::atomic<bool> cancel{false};
    RunRequest request;
    request.isFile = false; request.source = source;
    return execute(request, cancel, sink);
}
}
TEST(Runtime, BindingsAndBooleanOperations) {
    auto result = code("b=box.new(2,2,2); b:cut(box.new()):pos(1,2,3):color('red'); assert(not b:empty()); show({b, axes.new()}); print('中文', true, 42)");
    EXPECT_EQ(result.status, RunStatus::Success) << result.message;
}
TEST(Runtime, TypedOptionsRemainLuaCompatible) {
    auto result = code("box.new():fillet(0.1,{type='line'}); box.new():chamfer(0.1,{min={-9,-9,-9},max={9,9,9}}); assert(box.new():get_edge({type='line'}):type()=='edge')");
    EXPECT_EQ(result.status, RunStatus::Success) << result.message;
}
TEST(Runtime, ErrorsAndStateIsolation) {
    EXPECT_EQ(code("this is not lua !").status, RunStatus::Failed);
    EXPECT_EQ(code("error('failure')").status, RunStatus::Failed);
    EXPECT_EQ(code("box.new(0,0,0)").status, RunStatus::Failed);
    EXPECT_EQ(code("saved_global=123").status, RunStatus::Success);
    EXPECT_EQ(code("assert(saved_global==nil)").status, RunStatus::Success);
}
TEST(Runtime, CancellationAndRestart) {
    std::atomic<bool> cancel{false};
    RunRequest request; request.isFile = false; request.source = "while true do end";
    auto worker = std::async(std::launch::async, [&] { return execute(request, cancel); });
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    cancel = true;
    ASSERT_EQ(worker.wait_for(std::chrono::seconds(3)), std::future_status::ready);
    EXPECT_EQ(worker.get().status, RunStatus::Cancelled);
    EXPECT_EQ(code("assert(box.new())").status, RunStatus::Success);
}
TEST(Runtime, DirectoryRestoredOnSuccessAndFailure) {
    QTemporaryDir temp;
    const auto before = std::filesystem::current_path();
    writeFile(temp.filePath("helper.lua"), "return 42");
    writeFile(temp.filePath("main.lua"), "assert(require('helper')==42); local f=assert(io.open('output.txt','w')); f:write('ok'); f:close()");
    RunRequest request; request.source = temp.filePath("main.lua").toStdString();
    std::atomic<bool> cancel{false};
    EXPECT_EQ(execute(request,cancel).status, RunStatus::Success);
    EXPECT_TRUE(QFile::exists(temp.filePath("output.txt")));
    EXPECT_EQ(std::filesystem::current_path(), before);
    writeFile(temp.filePath("main.lua"), "error('expected')");
    EXPECT_EQ(execute(request,cancel).status, RunStatus::Failed);
    EXPECT_EQ(std::filesystem::current_path(), before);
}
TEST(Runtime, DisplaySnapshotsAreIndependent) {
    std::vector<JyShape> shapes;
    const auto result = code("b=box.new(); show(b); b:scale(2); show(b)", [&](RunEvent event) {
        if (auto *shape = std::get_if<JyShape>(&event)) shapes.push_back(*shape);
    });
    ASSERT_EQ(result.status, RunStatus::Success) << result.message;
    ASSERT_EQ(shapes.size(), 2);
    GProp_GProps first, second;
    BRepGProp::VolumeProperties(shapes[0].data(), first);
    BRepGProp::VolumeProperties(shapes[1].data(), second);
    EXPECT_NEAR(first.Mass(), 1, 1e-6);
    EXPECT_NEAR(second.Mass(), 8, 1e-6);
    EXPECT_FALSE(shapes[0].data().IsPartner(shapes[1].data()));
}
TEST(Runtime, ExportBindings) {
    QTemporaryDir temp;
    RunRequest request; request.isFile = false; request.directory = temp.path().toStdString();
    request.source = "box.new():export_stl('b.stl',{type='ascii',radian=0.05}); box.new():export_step('b.step'); link.new('base',{box.new()}):export({name='robot',path='.',mujoco=true})";
    std::atomic<bool> cancel{false};
    const auto result = execute(request,cancel);
    EXPECT_EQ(result.status, RunStatus::Success) << result.message;
    EXPECT_TRUE(QFile::exists(temp.filePath("robot/robot.xml")));
}
TEST(Runtime, BundledExamplesRemainCompatible) {
    const QStringList examples={"0composite.lua","1solid.lua","2fillet_chamfer.lua","3prism.lua","4boolean_operation.lua","5export.lua","6urdf.lua","7robot_arm_dh.lua"};
    for (const auto &example : examples) {
        SCOPED_TRACE(example.toStdString());
        QTemporaryDir temp;
        QFile source(QString::fromUtf8(JELLYCAD_SOURCE_DIR)+"/scripts/"+example);
        ASSERT_TRUE(source.open(QIODevice::ReadOnly));
        // The published robot examples contain a Windows-only export directory.
        auto script=source.readAll(); script.replace("path = 'd:/'","path = '.'");
        writeFile(temp.filePath(example),script);
        RunRequest request; request.source=temp.filePath(example).toStdString();
        std::atomic<bool> cancel{false};
        const auto result=execute(request,cancel);
        EXPECT_EQ(result.status,RunStatus::Success) << result.message;
    }
}
