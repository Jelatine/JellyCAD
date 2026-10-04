/*
 * Copyright (c) 2024. Li Jianbin. All rights reserved.
 * MIT License
 */
#include "jy_search_widget.h"
#include "jy_theme.h"
#include <QHBoxLayout>
#include <QLabel>
#include <QShortcut>
#include <QStyle>

JySearchWidget::JySearchWidget(QWidget *parent) : QWidget(parent) {
    hide();

    // 创建水平布局
    QHBoxLayout *layout = new QHBoxLayout(this);
    layout->setContentsMargins(0, 0, 0, 0);

    // 关闭按钮
    closeButton = new QPushButton(this);
    closeButton->setIcon(JyTheme::icon("x"));
    closeButton->setFlat(true);
    closeButton->setToolTip("Close the search box (Esc)");
    connect(closeButton, &QPushButton::clicked, this, &JySearchWidget::closed);

    // Search input box
    searchLineEdit = new QLineEdit(this);
    searchLineEdit->setPlaceholderText("Enter search text...");
    searchLineEdit->setSizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::Minimum);
    connect(searchLineEdit, &QLineEdit::textChanged, this, &JySearchWidget::searchTextChanged);
    connect(searchLineEdit, &QLineEdit::returnPressed, this, &JySearchWidget::findNext);

    // Previous button
    prevButton = new QPushButton(JyTheme::icon("chevron-up"), "", this);
    prevButton->setFlat(true);
    prevButton->setToolTip("Find previous (Shift+F3)");
    connect(prevButton, &QPushButton::clicked, this, &JySearchWidget::findPrevious);

    // Next button
    nextButton = new QPushButton(JyTheme::icon("chevron-down"), "", this);
    nextButton->setFlat(true);
    nextButton->setToolTip("Find next (F3)");
    connect(nextButton, &QPushButton::clicked, this, &JySearchWidget::findNext);
    // 添加到布局
    layout->addWidget(closeButton);
    layout->addWidget(searchLineEdit);
    layout->addWidget(prevButton);
    layout->addWidget(nextButton);
}

void JySearchWidget::focusSearchBox() {
    searchLineEdit->setFocus();
    searchLineEdit->selectAll();
}

void JySearchWidget::setSearchText(const QString &text) {
    searchLineEdit->setText(text);
    searchLineEdit->selectAll();
}

void JySearchWidget::setFoundStatus(bool found) {
    if (found) {
        searchLineEdit->setStyleSheet("");
    } else {
        searchLineEdit->setStyleSheet(QString("QLineEdit { color: %1; }").arg(JyTheme::color("danger").name()));
    }
}