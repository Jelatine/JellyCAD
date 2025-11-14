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

void JyShapeInfoWidget::onButtonEdgeInfoClicked() {
    emit insertEdgeInfo(current_info);
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


void JyShapeInfoWidget::onSelectedShape(const JyShape &shape) {
    const TopoDS_Shape topo_shape = shape.data();
    current_info.clear();
    button_edge_info->setVisible(false);
    QJsonDocument json_doc;
    switch (topo_shape.ShapeType()) {
        case TopAbs_VERTEX: {
            const auto props = JyShape::vertex_properties(shape);
            QJsonObject jsonObj{{
                    {"vertex", QJsonObject({
                                       {"x", props[0]},
                                       {"y", props[1]},
                                       {"z", props[2]},
                               })},
            }};
            json_doc.setObject(jsonObj);
            break;
        }
        case TopAbs_EDGE: {
            const auto props = JyShape::edge_properties(shape);
            QJsonObject jsonObj{{
                    {"edge", QJsonObject({
                                     {"type", QString::fromStdString(props.type)},
                                     {"length", props.length},
                                     {"first", QJsonObject({
                                                       {"x", props.first[0]},
                                                       {"y", props.first[1]},
                                                       {"z", props.first[2]},
                                               })},
                                     {"last", QJsonObject({
                                                      {"x", props.last[0]},
                                                      {"y", props.last[1]},
                                                      {"z", props.last[2]},
                                              })},
                             })},
            }};
            json_doc.setObject(jsonObj);
            button_edge_info->setText("Insert edge info to editor");
            button_edge_info->setVisible(true);
            current_info = QString("edge_info = {type = '%1', first = {%2, %3, %4}, last = {%5, %6, %7}, tol = 1e-3}")
                                   .arg(QString::fromStdString(props.type))
                                   .arg(props.first[0])
                                   .arg(props.first[1])
                                   .arg(props.first[2])
                                   .arg(props.last[0])
                                   .arg(props.last[1])
                                   .arg(props.last[2]);
            break;
        }
        case TopAbs_WIRE: {
            json_doc.setObject({{"wire", QJsonObject({})}});
            break;
        }
        case TopAbs_FACE: {
            const auto props = JyShape::face_properties(shape);
            QJsonObject jsonObj{{
                    {"face", QJsonObject({
                                     {"type", QString::fromStdString(props.type)},
                                     {"area", props.area},
                                     {"u_range", QJsonObject({
                                                         {"min", props.uv[0]},
                                                         {"max", props.uv[1]},
                                                 })},
                                     {"v_range", QJsonObject({
                                                         {"min", props.uv[2]},
                                                         {"max", props.uv[3]},
                                                 })},
                                     {"center", QJsonObject({
                                                        {"x", props.center[0]},
                                                        {"y", props.center[1]},
                                                        {"z", props.center[2]},
                                                })},
                             })},
            }};
            json_doc.setObject(jsonObj);
            button_edge_info->setText("Insert face info to editor");
            button_edge_info->setVisible(true);
            current_info = QString("get_face('%1', %2, {%3, %4, %5}, {%6, %7, %8, %9})")
                                   .arg(QString::fromStdString(props.type))
                                   .arg(props.area)
                                   .arg(props.center[0])
                                   .arg(props.center[1])
                                   .arg(props.center[2])
                                   .arg(props.uv[0])
                                   .arg(props.uv[1])
                                   .arg(props.uv[2])
                                   .arg(props.uv[3]);
            break;
        }
        case TopAbs_SHELL: {
            json_doc.setObject({{"shell", QJsonObject({})}});
            break;
        }
        case TopAbs_SOLID:
        case TopAbs_COMPSOLID: {
            json_doc.setObject({{"solid", QJsonObject({})}});
            break;
        }
        case TopAbs_COMPOUND: {
            json_doc.setObject({{"compound", QJsonObject({})}});
            break;
        }
        default:
            return;
    }
    // 添加位姿信息
    QJsonObject root = json_doc.object();
    const auto pose = shape.get_pose();
    QJsonObject poseObj;
    poseObj["position"] = QJsonObject({
            {"x", pose[0]},
            {"y", pose[1]},
            {"z", pose[2]},
    });
    poseObj["orientation"] = QJsonObject({
            {"roll", pose[3]},
            {"pitch", pose[4]},
            {"yaw", pose[5]},
    });
    root["pose"] = poseObj;
    json_doc.setObject(root);
    treeShapeInfo->clear();
    if (json_doc.isObject()) {
        QJsonObject obj = json_doc.object();
        for (auto it = obj.begin(); it != obj.end(); ++it) {
            addJsonValue(nullptr, it.key(), it.value());
        }
    } else if (json_doc.isArray()) {
        QJsonArray array = json_doc.array();
        for (int i = 0; i < array.size(); ++i) {
            addJsonValue(nullptr, "[" + QString::number(i) + "]", array[i]);
        }
    }
    treeShapeInfo->expandAll();
}