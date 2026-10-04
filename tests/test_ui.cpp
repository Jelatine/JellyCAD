#include <gtest/gtest.h>
#include "jy_editor_widget.h"
#include "jy_llm_dialog.h"
#include "jy_main_window.h"
#include <QApplication>
#include <QTemporaryDir>
#include <QSettings>
#include <QSaveFile>
#include <QCloseEvent>
#include <QElapsedTimer>
#include <QTimer>
#include <QThread>

namespace {
bool pump(const std::function<bool()> &predicate, int timeout = 5000) {
    QElapsedTimer timer; timer.start();
    while (!predicate() && timer.elapsed() < timeout) { QCoreApplication::processEvents(); QThread::msleep(1); }
    return predicate();
}
void write(const QString &file, const QByteArray &text) {
    QFile output(file); ASSERT_TRUE(output.open(QIODevice::WriteOnly)); ASSERT_EQ(output.write(text),text.size());
}
class TestWindow : public JyMainWindow { public: using JyMainWindow::closeEvent; };
}
TEST(Editor, AtomicSaveAndFailedSavePreserveDirtyState) {
    QTemporaryDir temp;
    JyEditorWidget editor;
    editor.setFilePath(temp.filePath("code.lua"));
    editor.codeEditor()->insertPlainText("print('中文')");
    ASSERT_TRUE(editor.isModified());
    ASSERT_TRUE(editor.saveFile());
    EXPECT_FALSE(editor.isModified());
    QFile file(temp.filePath("code.lua")); ASSERT_TRUE(file.open(QIODevice::ReadOnly));
    EXPECT_EQ(file.readAll(),QString::fromUtf8("print('中文')").toUtf8());
    editor.setFilePath(temp.filePath("missing/code.lua"));
    editor.codeEditor()->insertPlainText("more");
    EXPECT_FALSE(editor.saveFile());
    EXPECT_TRUE(editor.isModified());
}
TEST(Editor, GenerationFailureDoesNotOverwriteCode) {
    JyEditorWidget editor;
    editor.codeEditor()->setPlainText("original");
    QTimer::singleShot(0,&editor,[&] {
        auto *dialog = editor.findChild<JyLlmDialog *>();
        ASSERT_NE(dialog,nullptr);
        emit dialog->codeStreamUpdate("partial");
        EXPECT_EQ(editor.codeEditor()->toPlainText(),"original");
        dialog->reject();
    });
    ASSERT_TRUE(QMetaObject::invokeMethod(&editor,"onLlmClicked",Qt::DirectConnection));
    EXPECT_EQ(editor.codeEditor()->toPlainText(),"original");
}
TEST(Editor, SuccessfulGenerationIsOneUndoOperation) {
    JyEditorWidget editor;
    editor.codeEditor()->setPlainText("original");
    QTimer::singleShot(0,&editor,[&] {
        auto *dialog = editor.findChild<JyLlmDialog *>();
        ASSERT_NE(dialog,nullptr);
        emit dialog->codeGenerationFinished("replacement");
        dialog->accept();
    });
    ASSERT_TRUE(QMetaObject::invokeMethod(&editor,"onLlmClicked",Qt::DirectConnection));
    EXPECT_EQ(editor.codeEditor()->toPlainText(),"replacement");
    editor.codeEditor()->undo();
    EXPECT_EQ(editor.codeEditor()->toPlainText(),"original");
}
TEST(Window, F5SaveRunsExactlyOnce) {
    QTemporaryDir temp;
    const auto path = temp.filePath("code.lua");
    write(path,"print('before')");
    TestWindow window;
    window.onFileOpenRequested(path);
    auto *editor=window.findChild<JyEditorWidget *>();
    auto *runner=window.findChild<JyLuaVirtualMachine *>();
    ASSERT_NE(editor,nullptr); ASSERT_NE(runner,nullptr);
    int count=0;
    QObject::connect(runner,&JyLuaVirtualMachine::completed,[&](const jelly::RunResult &) { ++count; });
    editor->codeEditor()->moveCursor(QTextCursor::End);
    editor->codeEditor()->insertPlainText("\nprint('after')");
    window.slot_button_run_clicked();
    window.slot_file_changed(path); // atomic-save watcher notification
    ASSERT_TRUE(pump([&] { return count==1; }));
    EXPECT_FALSE(pump([&] { return count>1; },350));
    EXPECT_FALSE(editor->isModified());
}
TEST(Window, ClosingDuringOutputIsAsynchronous) {
    TestWindow window;
    auto *runner=window.findChild<JyLuaVirtualMachine *>();
    ASSERT_NE(runner,nullptr);
    ASSERT_TRUE(runner->exec_code("while true do print('output') end"));
    QThread::msleep(50);
    QCloseEvent close;
    QElapsedTimer timer; timer.start();
    window.closeEvent(&close);
    EXPECT_FALSE(close.isAccepted());
    EXPECT_LT(timer.elapsed(),500);
    ASSERT_TRUE(pump([&] { return !runner->isRunning(); }));
    QCloseEvent finished;
    window.closeEvent(&finished);
    EXPECT_TRUE(finished.isAccepted());
}
int main(int argc,char **argv) {
    QApplication app(argc,argv);
    QTemporaryDir settings;
    QSettings::setDefaultFormat(QSettings::IniFormat);
    QSettings::setPath(QSettings::IniFormat,QSettings::UserScope,settings.path());
    ::testing::InitGoogleTest(&argc,argv);
    return RUN_ALL_TESTS();
}
