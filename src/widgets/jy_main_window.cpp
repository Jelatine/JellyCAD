/*
 * Copyright (c) 2024. Li Jianbin. All rights reserved.
 * MIT License
 */
#include "jy_main_window.h"
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
#include <QSettings>
#include <QShortcut>
#include <QStackedWidget>
#include <QStatusBar>
#include <QTextBrowser>

JyMainWindow::JyMainWindow(QWidget *parent) : QMainWindow(parent),
                                              jy_3d_widget(new Jy3DWidget),
                                              watcher(new QFileSystemWatcher),
                                              lvm(new JyLuaVirtualMachine),
                                              search_widget(new JySearchWidget),
                                              code_editor(new JyCodeEditor) {
    setWindowTitle("Unnamed - JellyCAD");
    const QString default_dir = QApplication::applicationDirPath() + "/scripts";// 默认当前文件目录为应用程序目录下的scripts文件夹
    QDir dir_current(default_dir);
    if (!dir_current.exists()) { QDir(QApplication::applicationDirPath()).mkdir("scripts"); }
    // 读取软件配置
    settings = new QSettings("Jelatine", "JellyCAD", this);
    current_file_dir = settings->value("lastDirectory", default_dir).toString();
    if (!QDir(current_file_dir).exists()) { current_file_dir = default_dir; }
    lvm->add_package_path(current_file_dir.toStdString() + "/?.lua");

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
    //!< 形状信息
    auto widget_shape_info = new QWidget;
    widget_shape_info->setLayout(new QVBoxLayout);
    treeShapeInfo = new QTreeWidget;
    treeShapeInfo->header()->hide();
    connect(jy_3d_widget, &Jy3DWidget::selectedShapeInfo, this, &JyMainWindow::slot_shape_info);
    button_edge_info = new QPushButton("Insert edge info to editor");
    button_edge_info->setVisible(false);
    connect(button_edge_info, &QPushButton::clicked, this, &JyMainWindow::slot_button_edge_info_clicked);
    widget_shape_info->layout()->addWidget(button_edge_info);
    widget_shape_info->layout()->addWidget(treeShapeInfo);

    //!< 工作台: PAGE1: 脚本编辑器  PAGE2: 终端 PAGE3: 帮助
    auto stack_widget = new QStackedWidget;
    stack_widget->addWidget(widget_editor_group);
    stack_widget->addWidget(widget_terminal);
    stack_widget->addWidget(widget_shape_info);
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
    connect(button_edge_info, &QPushButton::clicked, [=] { m_activity_bar->slot_navigation_buttons_clicked(0); });
    connect(code_editor, &QPlainTextEdit::modificationChanged, button_save, &QPushButton::setEnabled);
    connect(code_editor, &QPlainTextEdit::modificationChanged, [=](bool m) {
        m ? setWindowTitle("*" + windowTitle()) : setWindowTitle(windowTitle().remove(0, 1));// 编辑器文档已被编辑时，标题前加*
    });
    connect(button_new, &QPushButton::clicked, this, &JyMainWindow::slot_button_new_clicked);
    connect(button_open, &QPushButton::clicked, this, &JyMainWindow::slot_button_open_clicked);
    connect(button_save, &QPushButton::clicked, this, &JyMainWindow::slot_button_save_clicked);
    connect(lvm, &JyLuaVirtualMachine::displayShape, jy_3d_widget, &Jy3DWidget::onDisplayShape, Qt::QueuedConnection);
    connect(lvm, &JyLuaVirtualMachine::displayAxes, jy_3d_widget, &Jy3DWidget::onDisplayAxes, Qt::QueuedConnection);
    connect(lvm, &JyLuaVirtualMachine::scriptStarted, this, &JyMainWindow::onScriptStarted);
    connect(lvm, &JyLuaVirtualMachine::scriptFinished, this, &JyMainWindow::onScriptFinished);
    connect(lvm, &JyLuaVirtualMachine::scriptError, this, &JyMainWindow::onScriptError);
    connect(lvm, &JyLuaVirtualMachine::scriptOutput, this, &JyMainWindow::onScriptOutput, Qt::QueuedConnection);

    // 创建快捷键 ref:https://doc.qt.io/archives/qt-5.15/qkeysequence.html
    QShortcut *findNextShortcut = new QShortcut(QKeySequence::FindNext, this);
    connect(findNextShortcut, &QShortcut::activated, this, &JyMainWindow::findNext);
    QShortcut *findPrevShortcut = new QShortcut(QKeySequence::FindPrevious, this);
    connect(findPrevShortcut, &QShortcut::activated, this, &JyMainWindow::findPrevious);
    QShortcut *searchShortcut = new QShortcut(QKeySequence::Find, this);
    connect(searchShortcut, &QShortcut::activated, this, &JyMainWindow::showSearchWidget);
    QShortcut *escShortcut = new QShortcut(Qt::Key_Escape, search_widget);
    connect(escShortcut, &QShortcut::activated, this, &JyMainWindow::hideSearchWidget);
    QShortcut *newShortcut = new QShortcut(QKeySequence::New, this);  // Ctrl+N
    QShortcut *openShortcut = new QShortcut(QKeySequence::Open, this);// Ctrl+O
    QShortcut *saveShortcut = new QShortcut(QKeySequence::Save, this);// Ctrl+S
    connect(newShortcut, &QShortcut::activated, button_new, &QPushButton::click);
    connect(openShortcut, &QShortcut::activated, button_open, &QPushButton::click);
    connect(saveShortcut, &QShortcut::activated, button_save, &QPushButton::click);
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

void JyMainWindow::slot_button_new_clicked() {
    qDebug() << "[BUTTON]new";
    // 如果文件被修改，询问是否保存
    const auto res = ask_whether_to_save();
    if (res == 0) { slot_button_save_clicked(); }// [是]保存后新建
    else if (res == 1) {
    }// [否]直接新建
    else { return; }// [取消]不新建
    setWindowTitle("Unnamed - JellyCAD");
    code_editor->clear();
    if (!watcher->files().empty()) { watcher->removePaths(watcher->files()); }
    code_editor->document()->setModified(false);
    code_editor->modificationChanged(false);
    code_editor->setFilePath();
    jy_3d_widget->remove_all();// 清空三维显示窗中的所有对象
    statusBar()->clearMessage();
}

void JyMainWindow::slot_button_open_clicked() {
    qDebug() << "[BUTTON]open";
    const auto filename = QFileDialog::getOpenFileName(this, "open file", current_file_dir, "lua (*.lua)");
    if (filename.isEmpty()) { return; }
    QFile file_read(filename);
    if (!file_read.open(QIODevice::ReadOnly)) { return; }
    code_editor->set_text(file_read.readAll());
    if (!watcher->files().empty()) { watcher->removePaths(watcher->files()); }
    watcher->addPath(filename);
    file_read.close();
    this->slot_file_changed(filename);
    code_editor->document()->setModified(false);
    code_editor->setFilePath(filename);
    QDir dir(filename);
    QString title = dir.dirName();
    if (dir.cd("../")) {
        current_file_dir = dir.absolutePath();
        title += '[' + current_file_dir + ']';
        lvm->add_package_path(current_file_dir.toStdString() + "/?.lua");
        settings->setValue("lastDirectory", current_file_dir);
        settings->sync();// 确保立即写入
    }
    setWindowTitle(title + " - JellyCAD");
}

void JyMainWindow::slot_button_save_clicked() {
    qDebug() << "[BUTTON]save";
    QString save_filename = code_editor->getFilePath();
    if (save_filename.isEmpty()) {
        // 文件空时，弹出保存对话框，创建文件
        QString default_filename = current_file_dir + "/unnamed";
        save_filename = QFileDialog::getSaveFileName(this, "save file", default_filename, "lua (*.lua)");
        if (save_filename.isEmpty()) { return; }
    }
    if (!watcher->files().empty()) { watcher->removePaths(watcher->files()); }// 清除观察文件
    QFile file_save(save_filename);
    if (!file_save.open(QIODevice::WriteOnly)) { return; }
    is_save_from_editor = true;// 标记为从编辑器保存
    file_save.write(code_editor->get_text().toUtf8());
    file_save.close();
    code_editor->document()->setModified(false);
    code_editor->setFilePath(save_filename);
    QDir dir(save_filename);
    QString title = dir.dirName();
    if (dir.cd("../")) {
        current_file_dir = dir.absolutePath();
        title += '[' + current_file_dir + ']';
        lvm->add_package_path(current_file_dir.toStdString() + "/?.lua");
        settings->setValue("lastDirectory", current_file_dir);
        settings->sync();// 确保立即写入
    }
    setWindowTitle(title + " - JellyCAD");
    watcher->addPath(save_filename);
    this->slot_file_changed(save_filename);
}

void JyMainWindow::addJsonValue(QTreeWidgetItem *parent, const QString &key, const QJsonValue &value) {
    QTreeWidgetItem *item = new QTreeWidgetItem();

    switch (value.type()) {
        case QJsonValue::Object: {
            item->setText(0, key + " {}");
            QJsonObject obj = value.toObject();
            for (auto it = obj.begin(); it != obj.end(); ++it) {
                addJsonValue(item, it.key(), it.value());
            }
            break;
        }
        case QJsonValue::Array: {
            QJsonArray array = value.toArray();
            item->setText(0, key + " [" + QString::number(array.size()) + "]");
            for (int i = 0; i < array.size(); ++i) {
                addJsonValue(item, "[" + QString::number(i) + "]", array[i]);
            }
            break;
        }
        case QJsonValue::String:
            item->setText(0, key + ": \"" + value.toString() + "\"");
            break;
        case QJsonValue::Double:
            item->setText(0, key + ": " + QString::number(value.toDouble()));
            break;
        case QJsonValue::Bool:
            item->setText(0, key + ": " + (value.toBool() ? "true" : "false"));
            break;
        case QJsonValue::Null:
            item->setText(0, key + ": null");
            break;
        default:
            item->setText(0, key + ": undefined");
            break;
    }
    if (parent) {
        parent->addChild(item);
    } else {
        treeShapeInfo->addTopLevelItem(item);
    }
}
void JyMainWindow::slot_shape_info(const QJsonDocument &doc) {
    const auto edge = doc["edge"];
    if (edge.isObject()) {
        button_edge_info->setVisible(true);
        const QJsonObject edge_obj = edge.toObject();
        const auto first = edge_obj["first"].toObject();
        const auto last = edge_obj["last"].toObject();
        const auto type = edge_obj["type"].toString();
        current_edge_info = QString("edge_info = {type = '%1', first = {%2, %3, %4}, last = {%5, %6, %7}, tol = 1e-3}")
                                    .arg(type)
                                    .arg(first["x"].toDouble())
                                    .arg(first["y"].toDouble())
                                    .arg(first["z"].toDouble())
                                    .arg(last["x"].toDouble())
                                    .arg(last["y"].toDouble())
                                    .arg(last["z"].toDouble());
    } else {
        button_edge_info->setVisible(false);
        current_edge_info.clear();
    }
    treeShapeInfo->clear();
    if (doc.isObject()) {
        QJsonObject obj = doc.object();
        for (auto it = obj.begin(); it != obj.end(); ++it) {
            addJsonValue(nullptr, it.key(), it.value());
        }
    } else if (doc.isArray()) {
        QJsonArray array = doc.array();
        for (int i = 0; i < array.size(); ++i) {
            addJsonValue(nullptr, "[" + QString::number(i) + "]", array[i]);
        }
    }
    treeShapeInfo->expandAll();
}


void JyMainWindow::slot_button_edge_info_clicked() {
    QTextCursor cursor = code_editor->textCursor();// 获取光标
    cursor.select(QTextCursor::LineUnderCursor);   // 获取当前行的内容
    QString currentLine = cursor.selectedText();
    cursor.clearSelection();
    cursor.movePosition(QTextCursor::EndOfLine);// 移动光标到行尾
    if (currentLine.trimmed().isEmpty()) {
        cursor.insertText(current_edge_info);// 空行：直接插入文本
    } else {
        cursor.insertText("\n" + current_edge_info);// 非空行：插入换行后再插入文本
    }
    code_editor->setTextCursor(cursor);// 设置光标到插入文本之后
}

void JyMainWindow::closeEvent(QCloseEvent *event) {
    const auto res = ask_whether_to_save();
    if (res == 0) {
        slot_button_save_clicked();// [是]保存后退出
        event->accept();
    } else if (res == 1) {
        event->accept();
    }// [否]直接退出
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