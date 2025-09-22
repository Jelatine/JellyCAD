/*
 * Copyright (c) 2024. Li Jianbin. All rights reserved.
 * MIT License
 */
#include "jy_search_widget.h"
#include <QHBoxLayout>
#include <QLabel>
#include <QShortcut>
#include <QStyle>

SearchWidget::SearchWidget(QWidget *parent) : QWidget(parent) {
    hide();
    setStyleSheet("QPushButton{min-width: 28px; min-height: 28px;font-size: 22px;padding: 4px;}");

    // 创建水平布局
    QHBoxLayout *layout = new QHBoxLayout(this);
    layout->setContentsMargins(0, 0, 0, 0);

    // 关闭按钮
    closeButton = new QPushButton(this);
    closeButton->setIcon(style()->standardIcon(QStyle::SP_TitleBarCloseButton));
    closeButton->setFlat(true);
    closeButton->setToolTip("Close the search box (Esc)");
    connect(closeButton, &QPushButton::clicked, this, &SearchWidget::closed);

    // Search input box
    searchLineEdit = new QLineEdit(this);
    searchLineEdit->setPlaceholderText("Enter search text...");
    searchLineEdit->setSizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::Minimum);
    connect(searchLineEdit, &QLineEdit::textChanged, this, &SearchWidget::searchTextChanged);
    connect(searchLineEdit, &QLineEdit::returnPressed, this, &SearchWidget::findNext);

    // Previous button
    prevButton = new QPushButton("⬆️", this);
    prevButton->setFlat(true);
    prevButton->setToolTip("Find previous (Shift+F3)");
    connect(prevButton, &QPushButton::clicked, this, &SearchWidget::findPrevious);

    // Next button
    nextButton = new QPushButton("⬇️", this);
    nextButton->setFlat(true);
    nextButton->setToolTip("Find next (F3)");
    connect(nextButton, &QPushButton::clicked, this, &SearchWidget::findNext);
    // 添加到布局
    layout->addWidget(closeButton);
    layout->addWidget(searchLineEdit);
    layout->addWidget(prevButton);
    layout->addWidget(nextButton);

    // 创建快捷键
    QShortcut *findNextShortcut = new QShortcut(Qt::Key_F3, this);
    connect(findNextShortcut, &QShortcut::activated, this, &SearchWidget::findNext);

    QShortcut *findPrevShortcut = new QShortcut(Qt::SHIFT + Qt::Key_F3, this);
    connect(findPrevShortcut, &QShortcut::activated, this, &SearchWidget::findPrevious);
}

void SearchWidget::focusSearchBox() {
    searchLineEdit->setFocus();
    searchLineEdit->selectAll();
}

void SearchWidget::setSearchText(const QString &text) {
    searchLineEdit->setText(text);
    searchLineEdit->selectAll();
}

void SearchWidget::setFoundStatus(bool found) {
    if (found) {
        searchLineEdit->setStyleSheet("");
    } else {
        searchLineEdit->setStyleSheet("QLineEdit { color: #DA5140; }");
    }
}