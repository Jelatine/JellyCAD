/*
 * Copyright (c) 2024. Li Jianbin. All rights reserved.
 * MIT License
 */
#include "jy_main_window.h"
#include "jy_file_manager.h"
#include "jy_page_help.h"
#include "jy_search_widget.h"
#include <QAbstractItemView>
#include <QCompleter>
#include <QDateTime>
#include <QHeaderView>
#include <QJsonArray>
#include <QJsonObject>
#include <QJsonParseError>
#include <QLineEdit>
#include <QMessageBox>
#include <QShortcut>
#include <QStackedWidget>
#include <QStatusBar>
#include <QTextBrowser>

JyMainWindow::JyMainWindow(QWidget *parent) : QMainWindow(parent),
                                              jy_3d_widget(new Jy3DWidget),
                                              watcher(new QFileSystemWatcher),
                                              lvm(new JyLuaVirtualMachine),
                                              search_widget(new JySearchWidget),
                                              code_editor(new JyCodeEditor),
                                              shape_info_widget(new JyShapeInfoWidget),
                                              file_manager(new JyFileManager) {
    setWindowTitle('[' + file_manager->getWorkingDirectory() + ']' + " - JellyCAD");

    auto m_activity_bar = new JyActivityBar();

    const auto widget_editor_group = new QWidget;
    const auto button_save = new QPushButton("Save");
    const auto button_run = new QPushButton("Run");
    const auto layout_buttons = new QHBoxLayout;
    const auto layout_editor_group = new QVBoxLayout;

    button_save->setToolTip("Ctrl+S");
    button_save->setEnabled(false);
    button_run->setToolTip("F5");
    layout_buttons->addWidget(button_save);
    layout_buttons->addWidget(button_run);
    layout_editor_group->addLayout(layout_buttons);
    layout_editor_group->addWidget(code_editor);
    layout_editor_group->addWidget(search_widget);
    widget_editor_group->setLayout(layout_editor_group);

    connect(search_widget, &JySearchWidget::findNext, this, &JyMainWindow::findNext);
    connect(search_widget, &JySearchWidget::findPrevious, this, &JyMainWindow::findPrevious);
    connect(search_widget, &JySearchWidget::searchTextChanged, this, &JyMainWindow::onSearchTextChanged);
    connect(search_widget, &JySearchWidget::closed, this, &JyMainWindow::hideSearchWidget);

    //!< 终端
    auto widget_terminal = new QWidget;
    widget_terminal->setLayout(new QVBoxLayout);
    auto edit_lua_cmd = new QLineEdit;
    auto command_completer = new QCompleter(code_editor->keyword_list());
    command_completer->setWrapAround(false);
    command_completer->popup()->setStyleSheet("font-size: 20px;");
    command_completer->popup()->setFocusPolicy(Qt::NoFocus);
    edit_lua_cmd->setCompleter(command_completer);
    text_lua_message = new QTextBrowser;
    edit_lua_cmd->setPlaceholderText("Enter Lua command here");
    widget_terminal->layout()->addWidget(edit_lua_cmd);
    widget_terminal->layout()->addWidget(text_lua_message);
    connect(edit_lua_cmd, &QLineEdit::returnPressed, [=]() {
        if (edit_lua_cmd->completer()->popup()->isVisible()) { return; }
        const auto &script_text = edit_lua_cmd->text();
        if (script_text.isEmpty()) { return; }
        qDebug() << "run lua cmd: " << script_text;
        text_lua_message->setTextColor(Qt::cyan);
        text_lua_message->append(script_text);
        lvm->exec_code(script_text);
        edit_lua_cmd->clear();
    });
    //!< 文件管理器
    connect(file_manager, &JyFileManager::fileOpenRequested, [=](const QString &filePath) {
        QFile file_read(filePath);
        if (!file_read.open(QIODevice::ReadOnly)) { return; }
        code_editor->set_text(file_read.readAll());
        if (!watcher->files().empty()) { watcher->removePaths(watcher->files()); }
        watcher->addPath(filePath);
        file_read.close();

        // Clear 3D widget instead of running script
        jy_3d_widget->remove_all();
        statusBar()->clearMessage();

        code_editor->document()->setModified(false);
        code_editor->modificationChanged(false);  // 手动触发信号以确保Save按钮被禁用
        code_editor->setFilePath(filePath);
        QDir dir(filePath);
        QString title = dir.dirName() + '[' + file_manager->getWorkingDirectory() + ']';
        setWindowTitle(title + " - JellyCAD");

        // 启用脚本编辑按钮
        m_activity_bar->setButtonEnabled(1, true);
    });
    connect(file_manager, &JyFileManager::switchToEditor, [=] { m_activity_bar->slot_navigation_buttons_clicked(1); });
    connect(file_manager, &JyFileManager::workingDirectoryChanged, [=](const QString &newPath) {
        // 清除监视的文件
        if (!watcher->files().empty()) { watcher->removePaths(watcher->files()); }

        // 清除编辑器内容
        code_editor->clear();
        code_editor->setFilePath("");
        code_editor->document()->setModified(false);

        // 清除3D界面内容
        jy_3d_widget->remove_all();

        // 清除状态栏消息
        statusBar()->clearMessage();

        // 更新窗口标题
        setWindowTitle('[' + newPath + ']' + " - JellyCAD");

        // 禁用脚本编辑按钮
        m_activity_bar->setButtonEnabled(1, false);
    });

    //!< 形状信息
    connect(jy_3d_widget, &Jy3DWidget::selectedShapeInfo, shape_info_widget, &JyShapeInfoWidget::setShapeInfo);
    connect(shape_info_widget, &JyShapeInfoWidget::insertEdgeInfo, this, &JyMainWindow::onInsertEdgeInfo);
    connect(shape_info_widget, &JyShapeInfoWidget::insertEdgeInfo, [=] { m_activity_bar->slot_navigation_buttons_clicked(1); });

    //!< 工作台: PAGE0: 文件管理器  PAGE1: 脚本编辑器  PAGE2: 终端  PAGE3: 形状信息  PAGE4: 帮助
    auto stack_widget = new QStackedWidget;
    stack_widget->addWidget(file_manager);
    stack_widget->addWidget(widget_editor_group);
    stack_widget->addWidget(widget_terminal);
    stack_widget->addWidget(shape_info_widget);
    stack_widget->addWidget(new JyPageHelp);
    //!< 布局
    addToolBar(Qt::LeftToolBarArea, m_activity_bar);// 活动栏，放置在最左侧
    auto central_widget = new QSplitter;            // 可分割容器，水平分割脚本编辑器与三维显示窗，可拖动分割条调整两控件大小比例
    central_widget->setOrientation(Qt::Horizontal);
    central_widget->addWidget(stack_widget);
    central_widget->addWidget(jy_3d_widget);
    setCentralWidget(central_widget);
    resize(800, 400);// 初始界面大小800x400
    //!< 信号槽连接
    connect(watcher, &QFileSystemWatcher::fileChanged, this, &JyMainWindow::slot_file_changed);
    connect(m_activity_bar, &JyActivityBar::sig_set_side_bar_visible, stack_widget, &QStackedWidget::setVisible);
    connect(m_activity_bar, &JyActivityBar::sig_set_side_bar_index, stack_widget, &QStackedWidget::setCurrentIndex);
    connect(code_editor, &QPlainTextEdit::modificationChanged, [=](bool m) {
        if (m) {
            // 添加星号（如果还没有）
            if (!windowTitle().startsWith("*")) { setWindowTitle("*" + windowTitle()); }
        } else {
            // 移除星号（如果有）
            if (windowTitle().startsWith("*")) { setWindowTitle(windowTitle().mid(1)); }
        }
    });
    connect(code_editor, &QPlainTextEdit::modificationChanged, button_save, &QPushButton::setEnabled);
    connect(button_save, &QPushButton::clicked, this, &JyMainWindow::slot_button_save_clicked);
    connect(button_run, &QPushButton::clicked, this, &JyMainWindow::slot_button_run_clicked);
    connect(lvm, &JyLuaVirtualMachine::displayShape, jy_3d_widget, &Jy3DWidget::onDisplayShape, Qt::BlockingQueuedConnection);
    connect(lvm, &JyLuaVirtualMachine::displayAxes, jy_3d_widget, &Jy3DWidget::onDisplayAxes, Qt::BlockingQueuedConnection);
    connect(lvm, &JyLuaVirtualMachine::scriptStarted, this, &JyMainWindow::onScriptStarted);
    connect(lvm, &JyLuaVirtualMachine::scriptFinished, this, &JyMainWindow::onScriptFinished);
    connect(lvm, &JyLuaVirtualMachine::scriptError, this, &JyMainWindow::onScriptError);
    connect(lvm, &JyLuaVirtualMachine::scriptOutput, this, &JyMainWindow::onScriptOutput, Qt::BlockingQueuedConnection);

    // 创建快捷键 ref:https://doc.qt.io/archives/qt-5.15/qkeysequence.html
    QShortcut *findNextShortcut = new QShortcut(QKeySequence::FindNext, this);
    connect(findNextShortcut, &QShortcut::activated, this, &JyMainWindow::findNext);
    QShortcut *findPrevShortcut = new QShortcut(QKeySequence::FindPrevious, this);
    connect(findPrevShortcut, &QShortcut::activated, this, &JyMainWindow::findPrevious);
    QShortcut *searchShortcut = new QShortcut(QKeySequence::Find, this);
    connect(searchShortcut, &QShortcut::activated, this, &JyMainWindow::showSearchWidget);
    QShortcut *escShortcut = new QShortcut(Qt::Key_Escape, search_widget);
    connect(escShortcut, &QShortcut::activated, this, &JyMainWindow::hideSearchWidget);
    QShortcut *saveShortcut = new QShortcut(QKeySequence::Save, this);// Ctrl+S
    connect(saveShortcut, &QShortcut::activated, button_save, &QPushButton::click);
    QShortcut *runShortcut = new QShortcut(Qt::Key_F5, this);// F5
    connect(runShortcut, &QShortcut::activated, button_run, &QPushButton::click);
}

void JyMainWindow::slot_file_changed(const QString &path) {
    if (watcher->files().isEmpty()) {
        qDebug() << "被检测的文件为空";
        return;
    }
    if (!is_save_from_editor) {
        int how_to_handle = 0;// 0 更新编辑器, 1 不更新
        if (code_editor->document()->isModified()) {
            // 用户编辑过该文件，则弹出保存对话框，是否同步到编辑器
            how_to_handle = QMessageBox::question(this, tr("File Updated"),
                                                  tr("Do you want sync to editor?"),
                                                  tr("Yes"), tr("No"));
        } else {
        }// 用户无编辑过该文件，则更新文件内容到编辑器
        if (how_to_handle == 0) {
            QFile file_read(path);
            if (file_read.open(QIODevice::ReadOnly)) {
                code_editor->set_text(file_read.readAll());
                file_read.close();
                code_editor->document()->setModified(false);
                code_editor->modificationChanged(false);
            }
        }
    }
    is_save_from_editor = false;// 复位标志

    static int i = 0;
    qDebug() << "file changed: " << path << " " << i++;
    jy_3d_widget->remove_all();
    statusBar()->clearMessage();
    const auto &str_prefix = QDateTime::currentDateTime().toString("[yyyy-MM-dd hh:mm:ss] ") + path;
    text_lua_message->setTextColor(Qt::white);
    text_lua_message->append(str_prefix);
    lvm->executeScript(path);// 执行脚本
}

void JyMainWindow::slot_button_save_clicked() {
    qDebug() << "[BUTTON]save";
    if (saveFile()) {
        // Run the script after saving
        QString saved_filename = code_editor->getFilePath();
        if (!saved_filename.isEmpty()) {
            this->slot_file_changed(saved_filename);
        }
    }
}

void JyMainWindow::slot_button_run_clicked() {
    qDebug() << "[BUTTON]run";

    // Run the script
    QString run_filename = code_editor->getFilePath();
    if(code_editor->document()->isModified()) {
        // 如果文件被修改，先保存
        if (!saveFile()) {
            return; // 保存失败或取消，停止运行
        }
    }
    if (!run_filename.isEmpty()) {
        this->slot_file_changed(run_filename);
    }
}

void JyMainWindow::onInsertEdgeInfo(const QString &edgeInfo) {
    QTextCursor cursor = code_editor->textCursor();// 获取光标
    cursor.select(QTextCursor::LineUnderCursor);   // 获取当前行的内容
    QString currentLine = cursor.selectedText();
    cursor.clearSelection();
    cursor.movePosition(QTextCursor::EndOfLine);// 移动光标到行尾
    if (currentLine.trimmed().isEmpty()) {
        cursor.insertText(edgeInfo);// 空行：直接插入文本
    } else {
        cursor.insertText("\n" + edgeInfo);// 非空行：插入换行后再插入文本
    }
    code_editor->setTextCursor(cursor);// 设置光标到插入文本之后
}

void JyMainWindow::closeEvent(QCloseEvent *event) {
    const auto res = ask_whether_to_save();
    if (res == 0) {
        if (saveFile()) {
            event->accept();// [是]保存成功后退出
        } else {
            event->ignore();// 保存失败或取消，不退出
        }
    } else if (res == 1) {
        event->accept();// [否]直接退出
    } else {
        event->ignore();// [取消]忽略关闭事件
    }
}

int JyMainWindow::ask_whether_to_save() {
    if (code_editor->document()->isModified()) {
        // 文件未保存，询问是否保存
        return QMessageBox::question(this, tr("File not saved"),
                                     tr("Do you want to save the file?"),
                                     tr("Yes"), tr("No"), tr("Cancel"));
    }
    return 1;// 文件未修改
}

bool JyMainWindow::saveFile() {
    qDebug() << "[FUNCTION]saveFile";
    QString save_filename = code_editor->getFilePath();

    if (save_filename.isEmpty()) { return false; }

    // Save the file
    if (!watcher->files().empty()) { watcher->removePaths(watcher->files()); }
    QFile file_save(save_filename);
    if (!file_save.open(QIODevice::WriteOnly)) {
        return false;
    }
    is_save_from_editor = true;
    file_save.write(code_editor->get_text().toUtf8());
    file_save.close();
    code_editor->document()->setModified(false);
    code_editor->setFilePath(save_filename);
    watcher->addPath(save_filename);

    return true;
}

void JyMainWindow::showSearchWidget() {
    if (search_widget->isVisible()) {
        // 如果已经显示，则隐藏
        hideSearchWidget();
    } else {
        search_widget->show();
        search_widget->focusSearchBox();

        // 如果有选中的文本，将其作为搜索关键词
        QTextCursor cursor = code_editor->textCursor();
        if (cursor.hasSelection()) {
            search_widget->setSearchText(cursor.selectedText());
        }
    }
}

void JyMainWindow::hideSearchWidget() {
    search_widget->hide();
    code_editor->setFocus();

    // 清除高亮
    QTextCursor cursor = code_editor->textCursor();
    cursor.clearSelection();
    code_editor->setTextCursor(cursor);
}

void JyMainWindow::findNext() {
    performSearch(false);
}

void JyMainWindow::findPrevious() {
    performSearch(true);
}

void JyMainWindow::onSearchTextChanged(const QString &text) {
    lastSearchText = text;
    if (!text.isEmpty()) {
        // 从当前位置开始搜索
        performSearch(false);
    } else {
        // 清除选择
        search_widget->setFoundStatus(true);
        QTextCursor cursor = code_editor->textCursor();
        cursor.clearSelection();
        code_editor->setTextCursor(cursor);
    }
}

void JyMainWindow::performSearch(bool backward) {
    if (lastSearchText.isEmpty()) {
        return;
    }

    QTextDocument::FindFlags flags;
    if (backward) {
        flags |= QTextDocument::FindBackward;
    }

    QTextCursor cursor = code_editor->textCursor();
    QTextCursor newCursor = code_editor->document()->find(lastSearchText, cursor, flags);

    if (newCursor.isNull()) {
        // 如果没找到，从头/尾开始搜索
        if (backward) {
            newCursor = code_editor->document()->find(lastSearchText,
                                                      code_editor->document()->characterCount(),
                                                      flags);
        } else {
            newCursor = code_editor->document()->find(lastSearchText, 0, flags);
        }
    }

    if (!newCursor.isNull()) {
        code_editor->setTextCursor(newCursor);
        search_widget->setFoundStatus(true);
    } else {
        search_widget->setFoundStatus(false);
    }
}

void JyMainWindow::onScriptStarted(const QString &fileName) {
    // 创建进度对话框
    m_progressDialog = new QProgressDialog("正在执行脚本...", "终止脚本", 0, 0, this);
    m_progressDialog->setWindowModality(Qt::WindowModal);
    m_progressDialog->setMinimumDuration(200);
    // 连接终止按钮
    connect(m_progressDialog, &QProgressDialog::canceled, this, &JyMainWindow::onStopScript);
    m_progressDialog->setValue(0);
}

void JyMainWindow::onScriptFinished(const QString &message) {
    if (m_progressDialog) {
        m_progressDialog->close();
        m_progressDialog->deleteLater();
        m_progressDialog = nullptr;
    }
    if (message.isEmpty()) return;
    statusBar()->showMessage(message);
    text_lua_message->setTextColor(Qt::green);
    text_lua_message->append(message);
}

void JyMainWindow::onScriptError(const QString &error) {
    if (m_progressDialog) {
        m_progressDialog->close();
        m_progressDialog->deleteLater();
        m_progressDialog = nullptr;
    }
    statusBar()->showMessage(error);
    text_lua_message->setTextColor(Qt::red);
    text_lua_message->append(error);
}

void JyMainWindow::onScriptOutput(const QString &output) {
    text_lua_message->setTextColor(Qt::gray);
    text_lua_message->append(output);
}

void JyMainWindow::onStopScript() {
    if (lvm && lvm->isRunning()) {
        lvm->stopScript();
    }
}