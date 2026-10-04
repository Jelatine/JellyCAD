#include <gtest/gtest.h>
#include "test_support.h"
#include "jy_lua_virtual_machine.h"
#include "application/jy_preview_scheduler.h"
#include <QTemporaryDir>

TEST(Application, CompletionBusyAndRestart) {
    ensureApp();
    JyLuaVirtualMachine runner;
    int results = 0;
    QObject::connect(&runner, &JyLuaVirtualMachine::completed, [&](const jelly::RunResult &result) { ++results; EXPECT_EQ(result.status, jelly::RunStatus::Success); });
    ASSERT_TRUE(runner.exec_code("print('ok')"));
    EXPECT_FALSE(runner.exec_code("print('not accepted')"));
    ASSERT_TRUE(waitUntil([&] { return results == 1; }));
    EXPECT_FALSE(runner.isRunning());
    ASSERT_TRUE(runner.exec_code("assert(box.new())"));
    EXPECT_TRUE(waitUntil([&] { return results == 2; }));
}
TEST(Application, FullQueueCanCancelWithoutGuiDelivery) {
    ensureApp();
    JyLuaVirtualMachine runner;
    int results = 0;
    QObject::connect(&runner, &JyLuaVirtualMachine::completed, [&](const jelly::RunResult &result) { ++results; EXPECT_EQ(result.status, jelly::RunStatus::Cancelled); });
    ASSERT_TRUE(runner.exec_code("while true do print('message') end"));
    QThread::msleep(80); // deliberately do not pump the GUI while the producer fills its queue
    runner.stopScript();
    ASSERT_TRUE(waitUntil([&] { return results == 1; }));
    EXPECT_LE(runner.queuePeak(), runner.QueueCapacity);
}
TEST(Application, FailureEmitsOneResult) {
    ensureApp();
    JyLuaVirtualMachine runner;
    int count = 0;
    QObject::connect(&runner, &JyLuaVirtualMachine::completed, [&](const jelly::RunResult &result) { ++count; EXPECT_EQ(result.status, jelly::RunStatus::Failed); });
    ASSERT_TRUE(runner.exec_code("error('expected')"));
    EXPECT_TRUE(waitUntil([&] { return !runner.isRunning(); }));
    EXPECT_EQ(count, 1);
}
TEST(Application, PreviewCoalescesAndRemembersSaves) {
    ensureApp();
    QTemporaryDir temp;
    const auto path = temp.filePath("main.lua");
    writeFile(path,"print(1)");
    JyPreviewScheduler preview;
    preview.setDocument(path);
    EXPECT_FALSE(preview.changed(path));
    writeFile(path,"print(2)");
    preview.remember(); // the explicit save path consumes its own watcher event
    EXPECT_FALSE(preview.changed(path));
    int runs = 0;
    QObject::connect(&preview, &JyPreviewScheduler::requested, [&](const QString &p) { EXPECT_EQ(p,path); ++runs; });
    preview.setBusy(true);
    writeFile(path,"print(3)");
    EXPECT_TRUE(preview.changed(path));
    EXPECT_FALSE(preview.changed(path));
    preview.schedule(); preview.schedule();
    EXPECT_EQ(runs,0);
    preview.setBusy(false);
    ASSERT_TRUE(waitUntil([&] { return runs == 1; }));
    preview.schedule(); preview.setEnabled(false);
    EXPECT_FALSE(waitUntil([&] { return runs > 1; },300));
}
TEST(Application, SwitchingDocumentsDropsOldPreview) {
    ensureApp();
    JyPreviewScheduler preview;
    QTemporaryDir temp;
    writeFile(temp.filePath("a.lua"),"a"); writeFile(temp.filePath("b.lua"),"b");
    preview.setDocument(temp.filePath("a.lua"));
    preview.schedule();
    preview.setDocument(temp.filePath("b.lua"));
    int count=0;
    QObject::connect(&preview, &JyPreviewScheduler::requested, [&] { ++count; });
    EXPECT_FALSE(waitUntil([&] { return count > 0; },300));
}
