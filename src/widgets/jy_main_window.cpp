/*
 * Copyright (c) 2024. Li Jianbin. All rights reserved.
 * MIT License
 */
#include "jy_main_window.h"
#include "jy_file_manager.h"
#include "jy_git_manager.h"
#include "jy_page_help.h"
#include <QAbstractItemView>
#include <QCompleter>
#include <QDateTime>
#include <QFileInfo>
#include <QHeaderView>
#include <QJsonArray>
#include <QJsonObject>
#include <QJsonParseError>
#include <QLineEdit>
#include <QMessageBox>
#include <QShortcut>
#include <QStackedWidget>
#include <QStatusBar>
#include <QSettings>
#include <QCheckBox>
#include <QTextBrowser>

JyMainWindow::JyMainWindow(QWidget *parent) : QMainWindow(parent),
                                              jy_3d_widget(new Jy3DWidget),
                                              lvm(new JyLuaVirtualMachine(this)),
                                              m_editorWidget(new JyEditorWidget),
                                              shape_info_widget(new JyShapeInfoWidget),
                                              file_manager(new JyFileManager),
                                              git_manager(new JyGitManager),
                                              m_activity_bar(new JyActivityBar()),
                                              m_progressDialog(nullptr),
                                              m_isStoppingScript(false) {
    m_preview = new JyPreviewScheduler(this);
    auto autoPreview = new QCheckBox(tr("Auto preview"), this);
    QSettings settings("Jelatine", "JellyCAD");
    autoPreview->setChecked(settings.value("autoPreview", true).toBool());
    m_preview->setEnabled(autoPreview->isChecked());
    statusBar()->addPermanentWidget(autoPreview);
    connect(autoPreview, &QCheckBox::toggled, this, [this](bool enabled) {
        m_preview->setEnabled(enabled);
        QSettings("Jelatine", "JellyCAD").setValue("autoPreview", enabled);
    });
    connect(m_preview, &JyPreviewScheduler::requested, this, [this](const QString &path) { runScript(path, jelly::RunSource::Preview); });
    connect(lvm, &JyLuaVirtualMachine::completed, this, [this](const jelly::RunResult &) {
        jy_3d_widget->refresh(true);
        m_preview->setBusy(false);
        if (m_closePending) QTimer::singleShot(0, this, &QWidget::close);
    });
    connect(lvm, &JyLuaVirtualMachine::batchFinished, this, [this] { jy_3d_widget->refresh(false); });
    setWindowTitle('[' + file_manager->getWorkingDirectory() + ']' + " - JellyCAD");

    // Connect editor widget signals
    connect(m_editorWidget, &JyEditorWidget::saveRequested, this, &JyMainWindow::slot_button_save_clicked);
    connect(m_editorWidget, &JyEditorWidget::runRequested, this, &JyMainWindow::slot_button_run_clicked);
    connect(m_editorWidget, &JyEditorWidget::modificationChanged, [=](bool m) {
        if (m) {
            // 添加星号（如果还没有）
            if (!windowTitle().startsWith("*")) { setWindowTitle("*" + windowTitle()); }
        } else {
            // 移除星号（如果有）
            if (windowTitle().startsWith("*")) { setWindowTitle(windowTitle().mid(1)); }
        }
    });

    //!< 终端
    auto widget_terminal = new QWidget;
    widget_terminal->setLayout(new QVBoxLayout);
    auto edit_lua_cmd = new QLineEdit;
    auto command_completer = new QCompleter(m_editorWidget->codeEditor()->keyword_list());
    command_completer->setWrapAround(false);
    command_completer->popup()->setFocusPolicy(Qt::NoFocus);
    edit_lua_cmd->setCompleter(command_completer);
    text_lua_message = new QTextBrowser;
    text_lua_message->document()->setMaximumBlockCount(5000);
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
        if (!m_closePending && !lvm->exec_code(script_text, file_manager->getWorkingDirectory())) statusBar()->showMessage(tr("A script is already running"));
        edit_lua_cmd->clear();
    });
    //!< 文件管理器
    connect(file_manager, &JyFileManager::fileOpenRequested, this, &JyMainWindow::onFileOpenRequested);
    connect(file_manager, &JyFileManager::resetWorkspace, this, &JyMainWindow::onResetWorkspace);
    connect(file_manager, &JyFileManager::openedFileChanged, this, &JyMainWindow::slot_file_changed);

    //!< Git管理器 - 文件放弃修改后重新加载
    connect(git_manager, &JyGitManager::fileDiscarded, this, [this](const QString &filePath) {
        // 检查是否是当前在编辑器中打开的文件
        QString currentFilePath = m_editorWidget->getFilePath();
        if (!currentFilePath.isEmpty() && QFileInfo(currentFilePath).absoluteFilePath() == QFileInfo(filePath).absoluteFilePath()) {
            // 重新加载文件内容到编辑器（loadFile会自动设置为未修改状态）
            m_editorWidget->loadFile(filePath);
            qDebug() << "File discarded and reloaded:" << filePath;
        }
    });

    //!< 形状信息
    connect(jy_3d_widget, &Jy3DWidget::selectedShape, shape_info_widget, &JyShapeInfoWidget::onSelectedShape);
    connect(shape_info_widget, &JyShapeInfoWidget::insertEdgeInfo, this, &JyMainWindow::onInsertEdgeInfo);
    connect(shape_info_widget, &JyShapeInfoWidget::insertEdgeInfo, [=] { m_activity_bar->slot_navigation_buttons_clicked(1); });

    //!< 工作台: PAGE0: 文件管理器  PAGE1: 脚本编辑器  PAGE2: Git版本管理  PAGE3: 终端  PAGE4: 形状信息  PAGE5: 帮助
    //!< 注意：页面顺序需与JyActivityBar中按钮的添加顺序一致
    auto stack_widget = new QStackedWidget;
    stack_widget->addWidget(file_manager);
    stack_widget->addWidget(m_editorWidget);
    stack_widget->addWidget(git_manager);
    stack_widget->addWidget(widget_terminal);
    stack_widget->addWidget(shape_info_widget);
    stack_widget->addWidget(new JyPageHelp);

    // 设置Git管理器的工作目录
    git_manager->setWorkingDirectory(file_manager->getWorkingDirectory());
    //!< 布局
    addToolBar(Qt::LeftToolBarArea, m_activity_bar);// 活动栏，放置在最左侧
    auto central_widget = new QSplitter;            // 可分割容器，水平分割脚本编辑器与三维显示窗，可拖动分割条调整两控件大小比例
    central_widget->setOrientation(Qt::Horizontal);
    central_widget->addWidget(stack_widget);
    central_widget->addWidget(jy_3d_widget);
    setCentralWidget(central_widget);
    resize(800, 400);// 初始界面大小800x400
    //!< 信号槽连接
    connect(m_activity_bar, &JyActivityBar::sig_set_side_bar_visible, stack_widget, &QStackedWidget::setVisible);
    connect(m_activity_bar, &JyActivityBar::sig_set_side_bar_index, stack_widget, &QStackedWidget::setCurrentIndex);
    connect(lvm, &JyLuaVirtualMachine::displayShape, jy_3d_widget, &Jy3DWidget::onDisplayShape);
    connect(lvm, &JyLuaVirtualMachine::displayAxes, jy_3d_widget, &Jy3DWidget::onDisplayAxes);
    connect(lvm, &JyLuaVirtualMachine::scriptStarted, this, &JyMainWindow::onScriptStarted);
    connect(lvm, &JyLuaVirtualMachine::scriptFinished, this, &JyMainWindow::onScriptFinished);
    connect(lvm, &JyLuaVirtualMachine::scriptError, this, &JyMainWindow::onScriptError);
    connect(lvm, &JyLuaVirtualMachine::scriptOutput, this, &JyMainWindow::onScriptOutput);

    // 创建快捷键 ref:https://doc.qt.io/archives/qt-5.15/qkeysequence.html
    QShortcut *findNextShortcut = new QShortcut(QKeySequence::FindNext, this);
    connect(findNextShortcut, &QShortcut::activated, m_editorWidget, &JyEditorWidget::findNext);
    QShortcut *findPrevShortcut = new QShortcut(QKeySequence::FindPrevious, this);
    connect(findPrevShortcut, &QShortcut::activated, m_editorWidget, &JyEditorWidget::findPrevious);
    QShortcut *searchShortcut = new QShortcut(QKeySequence::Find, this);
    connect(searchShortcut, &QShortcut::activated, m_editorWidget, &JyEditorWidget::showSearch);
    QShortcut *escShortcut = new QShortcut(Qt::Key_Escape, m_editorWidget->searchWidget());
    connect(escShortcut, &QShortcut::activated, m_editorWidget, &JyEditorWidget::hideSearch);
    QShortcut *saveShortcut = new QShortcut(QKeySequence::Save, this);// Ctrl+S
    connect(saveShortcut, &QShortcut::activated, this, &JyMainWindow::slot_button_save_clicked);
    QShortcut *runShortcut = new QShortcut(Qt::Key_F5, this);// F5
    connect(runShortcut, &QShortcut::activated, this, &JyMainWindow::slot_button_run_clicked);
}

void JyMainWindow::slot_file_changed(const QString &path) {
    if (m_closePending || !m_preview->changed(path)) return;
    if (m_editorWidget->isModified() && QMessageBox::question(this, tr("File Updated"),
        tr("Reload external changes?"), QMessageBox::Yes | QMessageBox::No) != QMessageBox::Yes) return;
    m_editorWidget->loadFile(path);
    m_preview->schedule();
}


void JyMainWindow::onFileOpenRequested(const QString &filePath) {
    if (m_closePending) return;
    lvm->stopScript();
    // Load file into editor
    m_preview->setDocument(filePath);
    m_editorWidget->loadFile(filePath);

    // Clear 3D widget instead of running script
    jy_3d_widget->remove_all();
    statusBar()->clearMessage();

    // Update window title
    QDir dir(filePath);
    QString title = dir.dirName() + '[' + file_manager->getWorkingDirectory() + ']';
    setWindowTitle(title + " - JellyCAD");

    // Set opened file in file manager
    file_manager->setOpenedFile(filePath);

    // 启用脚本编辑按钮
    m_activity_bar->setButtonEnabled(1, true);

    // 切换到编辑器页面
    m_activity_bar->slot_navigation_buttons_clicked(1);
}

void JyMainWindow::runScript(const QString &path, jelly::RunSource trigger) {
    if (m_closePending || path.isEmpty()) return;
    jelly::RunRequest request;
    request.source = QFileInfo(path).absoluteFilePath().toStdString();
    request.trigger = trigger;
    if (!lvm->submit(std::move(request))) {
        statusBar()->showMessage(tr("A script is already running"));
        return;
    }
    jy_3d_widget->remove_all();
    statusBar()->clearMessage();
    const auto &str_prefix = QDateTime::currentDateTime().toString("[yyyy-MM-dd hh:mm:ss] ") + path;
    text_lua_message->setTextColor(Qt::white);
    text_lua_message->append(str_prefix);

}

void JyMainWindow::slot_button_save_clicked() {
    qDebug() << "[BUTTON]save" << m_editorWidget->getFilePath();
    if (saveFile()) m_preview->schedule();
}

void JyMainWindow::slot_button_run_clicked() {
    if (m_closePending) return;
    if (lvm->isRunning()) { statusBar()->showMessage(tr("A script is already running")); return; }
    if (m_editorWidget->isModified() && !saveFile()) return;
    m_preview->cancelPending();
    runScript(m_editorWidget->getFilePath());
}


void JyMainWindow::onInsertEdgeInfo(const QString &edgeInfo) {
    QTextCursor cursor = m_editorWidget->codeEditor()->textCursor();// 获取光标
    cursor.select(QTextCursor::LineUnderCursor);                    // 获取当前行的内容
    QString currentLine = cursor.selectedText();
    cursor.clearSelection();
    cursor.movePosition(QTextCursor::EndOfLine);// 移动光标到行尾
    if (currentLine.trimmed().isEmpty()) {
        cursor.insertText(edgeInfo);// 空行：直接插入文本
    } else {
        cursor.insertText("\n" + edgeInfo);// 非空行：插入换行后再插入文本
    }
    m_editorWidget->codeEditor()->setTextCursor(cursor);// 设置光标到插入文本之后
}

void JyMainWindow::closeEvent(QCloseEvent *event) {
    if (!m_closePending) {
        const auto answer = ask_whether_to_save();
        if (answer == 2 || (answer == 0 && !saveFile())) { event->ignore(); return; }
        m_closePending = true;
        m_preview->cancelPending();
    }
    if (lvm->isRunning()) {
        event->ignore();
        centralWidget()->setEnabled(false);
        onStopScript();
        return;
    }
    event->accept();
}


int JyMainWindow::ask_whether_to_save() {
    if (m_editorWidget->isModified()) {
        // 文件未保存，询问是否保存
        auto reply = QMessageBox::question(this, tr("File not saved"),
                                           tr("Do you want to save the file?"),
                                           QMessageBox::Yes | QMessageBox::No | QMessageBox::Cancel);
        if (reply == QMessageBox::Yes) return 0;
        if (reply == QMessageBox::No) return 1;
        return 2; // Cancel
    }
    return 1;// 文件未修改
}

bool JyMainWindow::saveFile() {
    qDebug() << "[FUNCTION]saveFile";
    if (!m_editorWidget->saveFile()) {
        QMessageBox::warning(this, tr("Save failed"), tr("The file could not be saved. Your changes are still in the editor."));
        return false;
    }
    m_preview->remember();
    return true;
}

void JyMainWindow::onScriptStarted() {
    m_preview->setBusy(true);
    // 重置停止标志
    m_isStoppingScript = false;

    // 创建进度对话框
    m_progressDialog = new QProgressDialog("正在执行脚本...", "终止脚本", 0, 0, this);
    m_progressDialog->setWindowModality(Qt::WindowModal);
    m_progressDialog->setMinimumDuration(200);

    // 连接终止按钮 - 不让对话框自动关闭
    m_progressDialog->setAutoClose(false);
    m_progressDialog->setAutoReset(false);

    connect(m_progressDialog, &QProgressDialog::canceled, this, &JyMainWindow::onStopScript);
    m_progressDialog->setValue(0);
}

void JyMainWindow::onScriptFinished(const QString &message) {
    // 重置停止标志
    m_isStoppingScript = false;

    // 关闭进度对话框
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
    // 重置停止标志
    m_isStoppingScript = false;

    // 关闭进度对话框
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
    if (m_isStoppingScript) {
        // 已经在停止过程中，不重复处理
        return;
    }

    if (lvm && lvm->isRunning()) {
        m_isStoppingScript = true;

        // 关闭原有进度对话框
        if (m_progressDialog) {
            m_progressDialog->disconnect();// 断开所有信号连接
            m_progressDialog->close();
            m_progressDialog->deleteLater();
        }

        // 创建新的等待对话框
        m_progressDialog = new QProgressDialog("正在停止脚本，请稍候...", QString(), 0, 0, this);
        m_progressDialog->setWindowModality(Qt::WindowModal);
        m_progressDialog->setMinimumDuration(0);// 立即显示
        m_progressDialog->setAutoClose(false);
        m_progressDialog->setAutoReset(false);
        m_progressDialog->setCancelButton(nullptr);// 移除取消按钮
        m_progressDialog->setRange(0, 0);          // 设置为无限进度模式

        // 禁用关闭按钮和 ESC 键
        m_progressDialog->setWindowFlags(Qt::Window | Qt::WindowTitleHint | Qt::CustomizeWindowHint);

        m_progressDialog->show();// 强制显示

        // 请求停止脚本
        lvm->stopScript();
    }
}

void JyMainWindow::onResetWorkspace() {
    m_preview->setDocument({});
    lvm->stopScript();
    // 清除编辑器内容
    m_editorWidget->clearEditor();

    // 清除3D界面内容
    jy_3d_widget->remove_all();

    // 清除状态栏消息
    statusBar()->clearMessage();

    // 更新窗口标题
    setWindowTitle('[' + file_manager->getWorkingDirectory() + ']' + " - JellyCAD");

    // 禁用脚本编辑按钮
    m_activity_bar->setButtonEnabled(1, false);

    git_manager->setWorkingDirectory(file_manager->getWorkingDirectory());
}