/*
 * Copyright (c) 2024. Li Jianbin. All rights reserved.
 * MIT License
 */
#ifndef JY_SEARCH_WIDGET_H
#define JY_SEARCH_WIDGET_H

#include <QCheckBox>
#include <QLineEdit>
#include <QPushButton>
#include <QWidget>

class JySearchWidget : public QWidget {
    Q_OBJECT

public:
    explicit JySearchWidget(QWidget *parent = nullptr);

    void focusSearchBox();

    void setSearchText(const QString &text);

    void setFoundStatus(bool found);

signals:

    void findNext();

    void findPrevious();

    void searchTextChanged(const QString &text);

    void closed();

private:
    QLineEdit *searchLineEdit;
    QPushButton *nextButton;
    QPushButton *prevButton;
    QPushButton *closeButton;
    QCheckBox *caseSensitiveCheckBox;
    QCheckBox *wholeWordCheckBox;
};
#endif