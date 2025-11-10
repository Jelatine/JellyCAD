/*
 * Copyright (c) 2024. Li Jianbin. All rights reserved.
 * MIT License
 */
#ifndef JY_SHAPE_INFO_WIDGET_H
#define JY_SHAPE_INFO_WIDGET_H

#include <QWidget>
#include <QTreeWidget>
#include <QPushButton>
#include <QJsonDocument>
#include <QVBoxLayout>

class JyShapeInfoWidget : public QWidget {
    Q_OBJECT

public:
    explicit JyShapeInfoWidget(QWidget *parent = nullptr);

public slots:
    void setShapeInfo(const QJsonDocument &doc);

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

#endif // JY_SHAPE_INFO_WIDGET_H
