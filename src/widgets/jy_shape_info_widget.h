/*
 * Copyright (c) 2024. Li Jianbin. All rights reserved.
 * MIT License
 */
#ifndef JY_SHAPE_INFO_WIDGET_H
#define JY_SHAPE_INFO_WIDGET_H

#include "jy_shape.h"
#include <QJsonDocument>
#include <QPushButton>
#include <QTreeWidget>
#include <QVBoxLayout>
#include <QWidget>

class JyShapeInfoWidget : public QWidget {
    Q_OBJECT

public:
    explicit JyShapeInfoWidget(QWidget *parent = nullptr);

public slots:
    void onSelectedShape(const JyShape &shape);

signals:
    void insertEdgeInfo(const QString &edgeInfo);

private slots:
    void onButtonEdgeInfoClicked();

private:
    void addJsonValue(QTreeWidgetItem *parent, const QString &key, const QJsonValue &value);

    QTreeWidget *treeShapeInfo;
    QPushButton *button_edge_info;
    QString current_info;
};

#endif// JY_SHAPE_INFO_WIDGET_H
