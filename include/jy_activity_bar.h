/*
 * Copyright (c) 2024. Li Jianbin. All rights reserved.
 * MIT License
 */
#ifndef JY_ACTIVITY_BAR_H
#define JY_ACTIVITY_BAR_H

#include <QToolBar>
#include <QPushButton>
#include <QVBoxLayout>
#include <QStyle>
#include <QApplication>
#include <QButtonGroup>

class JyActivityBar : public QToolBar {
Q_OBJECT
public:
    explicit JyActivityBar(QWidget *parent = nullptr);

public slots:

    void slot_navigation_buttons_clicked(int id);

signals:

    void sig_set_side_bar_visible(bool _visible);

    void sig_set_side_bar_index(int _index);

private:
    QButtonGroup *button_group = nullptr;
};

#endif //JY_ACTIVITY_BAR_H
