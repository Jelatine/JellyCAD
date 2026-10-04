/*
 * Copyright (c) 2024. Li Jianbin. All rights reserved.
 * MIT License
 */
#include "jy_activity_bar.h"
#include "jy_theme.h"

JyActivityBar::JyActivityBar(QWidget *parent) : QToolBar(parent) {
    setMovable(false);                                     // 工具栏不可移动
    setContextMenuPolicy(Qt::CustomContextMenu);           // 禁止右键菜单
    const auto button_file_manager = new QPushButton(JyTheme::icon("folder"), "");    // 创建文件管理器按钮
    button_file_manager->setToolTip(tr("File Manager"));                              // 设置提示文字
    const auto button_script = new QPushButton(JyTheme::icon("file-code"), "");       // 创建脚本显示按钮
    button_script->setToolTip(tr("Script"));                                          // 设置提示文字
    const auto button_git = new QPushButton(JyTheme::icon("git-branch"), "");         // 创建Git版本管理按钮
    button_git->setToolTip(tr("Version Control"));                                    // 设置提示文字
    const auto button_terminal = new QPushButton(JyTheme::icon("square-terminal"), "");// 创建终端页面选择按钮
    button_terminal->setToolTip(tr("Terminal"));                                      // 设置提示文字
    const auto button_shape_info = new QPushButton(JyTheme::icon("box"), "");         // 创建形状信息按钮
    button_shape_info->setToolTip(tr("Shape Info"));                                  // 设置提示文字
    const auto button_help = new QPushButton(JyTheme::icon("info"), "");              // 创建帮助页面选择按钮
    button_help->setToolTip(tr("Help"));                                              // 设置提示文字
    // 创建按钮组，按钮的ID及布局排序受addButton的顺序影响
    button_group = new QButtonGroup(this);
    button_group->addButton(button_file_manager);
    button_group->addButton(button_script);
    button_group->addButton(button_git);
    button_group->addButton(button_terminal);
    button_group->addButton(button_shape_info);
    button_group->addButton(button_help);
    // 转发按钮信号到主窗口，用于显示/隐藏脚本编辑界面
    // 布局
    const auto widget_tool_buttons = new QWidget(this);           // 按钮容器
    addWidget(widget_tool_buttons);                               //工具栏加入按钮容器
    const auto vbox_layout = new QVBoxLayout(widget_tool_buttons);// 垂直排列按钮
    vbox_layout->setContentsMargins(0, 0, 0, 0);
    vbox_layout->setSpacing(0);
    for (int i = 0; i < button_group->buttons().size(); i++) {
        const auto button = button_group->buttons().at(i);
        button_group->setId(button, i);
        vbox_layout->addWidget(button);
        button->setCheckable(true);
        button->setIconSize(QSize(22, 22));
        button->setFocusPolicy(Qt::NoFocus);
    }
    button_file_manager->setChecked(true);// 默认选中文件管理器
    button_script->setEnabled(false);     // 初始禁用脚本编辑按钮
    connect(button_group, &QButtonGroup::idClicked, this, &JyActivityBar::slot_navigation_buttons_clicked);
    button_group->setExclusive(false);
}

void JyActivityBar::setButtonEnabled(int buttonId, bool enabled) {
    QAbstractButton *button = button_group->button(buttonId);
    if (button) {
        button->setEnabled(enabled);
    }
}

void JyActivityBar::slot_navigation_buttons_clicked(int id) {
    if (button_group->checkedId() == -1) {
        emit sig_set_side_bar_visible(false);
        return;
    }
    for (int i = 0; i < button_group->buttons().size(); i++) {
        button_group->button(i)->setChecked(id == i);
    }
    emit sig_set_side_bar_visible(true);
    emit sig_set_side_bar_index(id);
};