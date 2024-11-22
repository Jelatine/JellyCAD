/*
 * Copyright (c) 2024. Li Jianbin. All rights reserved.
 * MIT License
 */
#include "jy_main_window.h"
#include <QMessageBox>
#include <QStatusBar>
#include <QStackedWidget>
#include <QLineEdit>
#include <QTextBrowser>
#include <QDateTime>

JyMainWindow::JyMainWindow(QWidget *parent) : QMainWindow(parent),
                                              jy_3d_widget(new Jy3DWidget),
                                              watcher(new QFileSystemWatcher),
                                              lvm(new JyLuaVirtualMachine),
                                              code_editor(new JyCodeEditor) {
    setWindowTitle("Unnamed - JellyCAD");
    current_file_dir = QApplication::applicationDirPath() + "/scripts"; // 默认当前文件目录为应用程序目录下的scripts文件夹
    QDir dir_current(current_file_dir);
    if (!dir_current.exists()) { QDir(QApplication::applicationDirPath()).mkdir("scripts"); }
    auto m_activity_bar = new JyActivityBar();

    const auto widget_editor_group = new QWidget;
    const auto button_new = new QPushButton("New");
    const auto button_open = new QPushButton("Open");
    const auto button_save = new QPushButton("Save");
    const auto layout_buttons = new QHBoxLayout;
    const auto layout_editor_group = new QVBoxLayout;

    button_new->setToolTip("Ctrl+N");
    button_open->setToolTip("Ctrl+O");
    button_save->setToolTip("Ctrl+S");
    button_save->setEnabled(false);
    layout_buttons->addWidget(button_new);
    layout_buttons->addWidget(button_open);
    layout_buttons->addWidget(button_save);
    layout_editor_group->addLayout(layout_buttons);
    layout_editor_group->addWidget(code_editor);
    widget_editor_group->setLayout(layout_editor_group);

    //!< 终端
    auto widget_terminal = new QWidget;
    widget_terminal->setLayout(new QVBoxLayout);
    auto edit_lua_cmd = new QLineEdit;
    auto text_lua_message = new QTextBrowser;
    edit_lua_cmd->setPlaceholderText("Enter Lua command here");
    widget_terminal->layout()->addWidget(edit_lua_cmd);
    widget_terminal->layout()->addWidget(text_lua_message);
    connect(edit_lua_cmd, &QLineEdit::returnPressed, [=]() {
        qDebug() << "run lua cmd: " << edit_lua_cmd->text();
        text_lua_message->setTextColor(Qt::cyan);
        const auto &script_text = edit_lua_cmd->text();
        text_lua_message->append(script_text);
        lvm->exec_code(script_text.toStdString());
        edit_lua_cmd->clear();
    });
    connect(lvm, &JyLuaVirtualMachine::sig_show_message, [=](const QString &_result, const int &_type) {
        // 日期时间 和等级
        const auto &str_prefix = QDateTime::currentDateTime().toString("[yyyy-MM-dd hh:mm:ss] ") + current_filename;
        if (_type == -2) {
            text_lua_message->setTextColor(Qt::red);
        } else if (_type == -1) {
            text_lua_message->setTextColor(Qt::lightGray);
            text_lua_message->append(str_prefix);
            text_lua_message->setTextColor(Qt::red);
            statusBar()->showMessage(_result);
        } else if (_type == 1) {
            text_lua_message->setTextColor(Qt::lightGray);
            text_lua_message->append(str_prefix);
            text_lua_message->setTextColor(Qt::green);
            statusBar()->showMessage(_result);
        } else if (_type == 0) {
            text_lua_message->setTextColor(Qt::gray);
        } else {}
        text_lua_message->append(_result);
    });



    //!< 工作台: PAGE1: 脚本编辑器  PAGE2: 终端
    auto stack_widget = new QStackedWidget;
    stack_widget->addWidget(widget_editor_group);
    stack_widget->addWidget(widget_terminal);
    //!< 布局
    addToolBar(Qt::LeftToolBarArea, m_activity_bar);    // 活动栏，放置在最左侧
    auto central_widget = new QSplitter; // 可分割容器，水平分割脚本编辑器与三维显示窗，可拖动分割条调整两控件大小比例
    central_widget->setOrientation(Qt::Horizontal);
    central_widget->addWidget(stack_widget);
    central_widget->addWidget(jy_3d_widget);
    setCentralWidget(central_widget);
    resize(800, 400);   // 初始界面大小800x400
    //!< 快捷键动作创建
    auto *acton_save = new QAction;
    acton_save->setShortcut(QKeySequence(Qt::CTRL + Qt::Key_S));
    addAction(acton_save);
    auto *acton_new = new QAction;
    acton_new->setShortcut(QKeySequence(Qt::CTRL + Qt::Key_N));
    addAction(acton_new);
    auto *acton_open = new QAction;
    acton_open->setShortcut(QKeySequence(Qt::CTRL + Qt::Key_O));
    addAction(acton_open);
    //!< 信号槽连接
    connect(watcher, &QFileSystemWatcher::fileChanged, this, &JyMainWindow::slot_file_changed);
    connect(m_activity_bar, &JyActivityBar::sig_set_side_bar_visible, stack_widget, &QStackedWidget::setVisible);
    connect(m_activity_bar, &JyActivityBar::sig_set_side_bar_index, stack_widget, &QStackedWidget::setCurrentIndex);
    connect(code_editor, &QPlainTextEdit::modificationChanged, button_save, &QPushButton::setEnabled);
    connect(code_editor, &QPlainTextEdit::modificationChanged, [=](bool m) {
        if (m) {
            setWindowTitle("*" + windowTitle());
        } else {
            setWindowTitle(windowTitle().remove(0, 1));
        }
    });
    connect(button_new, &QPushButton::clicked, this, &JyMainWindow::slot_button_new_clicked);
    connect(button_open, &QPushButton::clicked, this, &JyMainWindow::slot_button_open_clicked);
    connect(button_save, &QPushButton::clicked, this, &JyMainWindow::slot_button_save_clicked);
    connect(acton_new, &QAction::triggered, button_new, &QPushButton::click);
    connect(acton_open, &QAction::triggered, button_open, &QPushButton::click);
    connect(acton_save, &QAction::triggered, button_save, &QPushButton::click);
    connect(lvm, &JyLuaVirtualMachine::display, jy_3d_widget, &Jy3DWidget::display);
}

void JyMainWindow::slot_file_changed(const QString &path) {
    if (watcher->files().isEmpty()) {
        qDebug() << "被检测的文件为空";
        return;
    }
    if (!is_save_from_editor) {
        int how_to_handle = 0; // 0 更新编辑器, 1 不更新
        if (code_editor->document()->isModified()) {
            // 用户编辑过该文件，则弹出保存对话框，是否同步到编辑器
            how_to_handle = QMessageBox::question(this, tr("File Updated"),
                                                  tr("Do you want sync to editor?"),
                                                  tr("Yes"), tr("No"));
        } else {}// 用户无编辑过该文件，则更新文件内容到编辑器
        if (how_to_handle == 0) {
            QFile file_read(path);
            if (file_read.open(QIODevice::ReadOnly)) {
                code_editor->setPlainText(file_read.readAll());
                file_read.close();
                code_editor->document()->setModified(false);
                code_editor->modificationChanged(false);
            }
        }
    }
    is_save_from_editor = false;    // 复位标志
    static int i = 0;
    qDebug() << "file changed: " << path << " " << i++;
    jy_3d_widget->remove_all();
    statusBar()->clearMessage();
    lvm->run(path.toStdString()); // 执行脚本
}

void JyMainWindow::slot_button_new_clicked() {
    qDebug() << "[BUTTON]new";
    // 如果文件被修改，询问是否保存
    const auto res = ask_whether_to_save();
    if (res == 0) { slot_button_save_clicked(); } // [是]保存后新建
    else if (res == 1) {}// [否]直接新建
    else { return; }// [取消]不新建
    setWindowTitle("Unnamed - JellyCAD");
    code_editor->clear();
    if (!watcher->files().empty()) { watcher->removePaths(watcher->files()); }
    code_editor->document()->setModified(false);
    code_editor->modificationChanged(false);
    jy_3d_widget->remove_all(); // 清空三维显示窗中的所有对象
    statusBar()->clearMessage();
}

void JyMainWindow::slot_button_open_clicked() {
    qDebug() << "[BUTTON]open";
    const auto filename = QFileDialog::getOpenFileName(this, "open file", current_file_dir, "lua (*.lua)");
    if (filename.isEmpty()) { return; }
    QFile file_read(filename);
    if (!file_read.open(QIODevice::ReadOnly)) { return; }
    QDir dir(filename);
    current_filename = dir.dirName();
    code_editor->setPlainText(file_read.readAll());
    if (!watcher->files().empty()) { watcher->removePaths(watcher->files()); }
    watcher->addPath(filename);
    file_read.close();
    this->slot_file_changed(filename);
    code_editor->document()->setModified(false);
    QString title = dir.dirName();
    if (dir.cd("../")) {
        current_file_dir = dir.absolutePath();
        title += '[' + current_file_dir + ']';
    }
    setWindowTitle(title + " - JellyCAD");
}

void JyMainWindow::slot_button_save_clicked() {
    qDebug() << "[BUTTON]save";
    QString save_filename;
    const auto files = watcher->files();
    if (files.empty()) {
        // 文件空时，弹出保存对话框，创建文件
        QString default_filename = current_file_dir + "/unnamed";
        save_filename = QFileDialog::getSaveFileName(this, "save file", default_filename, "lua (*.lua)");
        if (save_filename.isEmpty()) { return; }
    } else if (files.size() == 1) {
        // 文件已打开，保存到打开的文件中
        save_filename = files[0];
    } else { return; }
    QFile file_save(save_filename);
    if (!file_save.open(QIODevice::WriteOnly)) { return; }
    QDir dir(save_filename);
    current_filename = dir.dirName();
    is_save_from_editor = true; // 标记为从编辑器保存
    file_save.write(code_editor->toPlainText().toUtf8());
    file_save.close();
    if (files.empty()) {
        watcher->addPath(save_filename);
        this->slot_file_changed(save_filename);
    }
    code_editor->document()->setModified(false);
    QString title = dir.dirName();
    if (dir.cd("../")) {
        current_file_dir = dir.absolutePath();
        title += '[' + current_file_dir + ']';
    }
    setWindowTitle(title + " - JellyCAD");
}

void JyMainWindow::closeEvent(QCloseEvent *event) {
    const auto res = ask_whether_to_save();
    if (res == 0) {
        slot_button_save_clicked();// [是]保存后退出
        event->accept();
    } else if (res == 1) { event->accept(); }// [否]直接退出
    else { event->ignore(); }// [取消]忽略关闭事件

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