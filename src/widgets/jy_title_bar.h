/*
 * Copyright (c) 2024. Li Jianbin. All rights reserved.
 * MIT License
 */
#ifndef JY_TITLE_BAR_H
#define JY_TITLE_BAR_H

#include <QWidget>
#include <QPoint>
#include <QRect>

class QLabel;
class QToolButton;

// Custom window chrome, including move/resize fallback for unsupported platforms.
class JyTitleBar : public QWidget {
    Q_OBJECT
public:
    explicit JyTitleBar(QWidget *window);

protected:
    bool eventFilter(QObject *watched, QEvent *event) override;
    void mousePressEvent(QMouseEvent *event) override;
    void mouseMoveEvent(QMouseEvent *event) override;
    void mouseReleaseEvent(QMouseEvent *event) override;
    void mouseDoubleClickEvent(QMouseEvent *event) override;

private:
    void toggleMaximized();
    void updateWindowState();
    Qt::Edges resizeEdges(const QPoint &position) const;
    QWidget *m_window;
    QLabel *m_title;
    QToolButton *m_maximize;
    bool m_moving = false;
    Qt::Edges m_resizing;
    QPoint m_pressPosition;
    QRect m_pressGeometry;
};

#endif
