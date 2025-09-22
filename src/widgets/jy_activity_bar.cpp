/*
 * Copyright (c) 2024. Li Jianbin. All rights reserved.
 * MIT License
 */
#include "jy_activity_bar.h"

JyActivityBar::JyActivityBar(QWidget *parent) : QToolBar(parent) {
    setMovable(false); // 工具栏不可移动
    setContextMenuPolicy(Qt::CustomContextMenu);    // 禁止右键菜单
    const auto button_script = new QPushButton("📄");   // 创建脚本显示按钮
    button_script->setToolTip(tr("Script"));   // 设置提示文字
    const auto button_terminal = new QPushButton("🖥️");   // 创建终端页面选择按钮
    button_terminal->setToolTip(tr("Terminal"));   // 设置提示文字
    const auto button_shape_info = new QPushButton("💎");   // 创建形状信息按钮
    button_shape_info->setToolTip(tr("Shape Info"));   // 设置提示文字
    const auto button_help = new QPushButton("ℹ️");   // 创建帮助页面选择按钮
    button_help->setToolTip(tr("Help"));   // 设置提示文字
    // 转发按钮信号到主窗口，用于显示/隐藏脚本编辑界面
    // 布局
    const auto widget_tool_buttons = new QWidget(this); // 按钮容器
    const auto vbox_layout = new QVBoxLayout(widget_tool_buttons); // 垂直排列按钮
    vbox_layout->setContentsMargins(0, 0, 0, 0);
    vbox_layout->setSpacing(0);
    vbox_layout->addWidget(button_script);  // 加入脚本按钮
    vbox_layout->addWidget(button_terminal);  // 加入终端按钮
    vbox_layout->addWidget(button_shape_info);  // 加入形状信息按钮
    vbox_layout->addWidget(button_help);  // 加入帮助按钮
    addWidget(widget_tool_buttons); //工具栏加入按钮容器
    button_group = new QButtonGroup(this);
    button_group->addButton(button_script, 0);
    button_group->addButton(button_terminal, 1);
    button_group->addButton(button_shape_info, 2);
    button_group->addButton(button_help, 3);
    const auto buttons = button_group->buttons();
    for (const auto &button: buttons) {
        button->setCheckable(true);
    }
    button_script->setChecked(true);      // 默认选中
    connect(button_group, &QButtonGroup::idClicked, this, &JyActivityBar::slot_navigation_buttons_clicked);
    button_group->setExclusive(false);
}

void JyActivityBar::slot_navigation_buttons_clicked(int id) {
    if (button_group->checkedId() == -1) {
        emit sig_set_side_bar_visible(false);
        return;
    }
    for (int i = 0; i < button_group->buttons().size(); i++) {
        if (id == i) continue;
        button_group->button(i)->setChecked(false);
    }
    emit sig_set_side_bar_visible(true);
    emit sig_set_side_bar_index(id);
};