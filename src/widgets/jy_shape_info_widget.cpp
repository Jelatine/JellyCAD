/*
 * Copyright (c) 2024. Li Jianbin. All rights reserved.
 * MIT License
 */
#include "jy_shape_info_widget.h"
#include <QHeaderView>
#include <QJsonArray>
#include <QJsonObject>

JyShapeInfoWidget::JyShapeInfoWidget(QWidget *parent) : QWidget(parent) {
    // 创建树形控件
    treeShapeInfo = new QTreeWidget(this);
    treeShapeInfo->header()->hide();

    // 创建边信息按钮
    button_edge_info = new QPushButton("Insert edge info to editor", this);
    button_edge_info->setVisible(false);
    connect(button_edge_info, &QPushButton::clicked, this, &JyShapeInfoWidget::onButtonEdgeInfoClicked);

    // 设置布局
    auto layout = new QVBoxLayout(this);
    layout->addWidget(button_edge_info);
    layout->addWidget(treeShapeInfo);
    setLayout(layout);
}

void JyShapeInfoWidget::setShapeInfo(const QJsonDocument &doc) {
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

void JyShapeInfoWidget::onButtonEdgeInfoClicked() {
    emit insertEdgeInfo(current_edge_info);
}

void JyShapeInfoWidget::addJsonValue(QTreeWidgetItem *parent, const QString &key, const QJsonValue &value) {
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
