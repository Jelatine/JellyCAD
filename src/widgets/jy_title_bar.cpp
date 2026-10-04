/*
 * Copyright (c) 2024. Li Jianbin. All rights reserved.
 * MIT License
 */
#include "jy_title_bar.h"
#include "jy_theme.h"
#include <QApplication>
#include <QHBoxLayout>
#include <QLabel>
#include <QMouseEvent>
#include <QPainter>
#include <QToolButton>
#include <QWindow>

namespace {
QIcon maximizeIcon(bool restore) {
    QPixmap pixmap(32, 32);
    pixmap.fill(Qt::transparent);
    QPainter painter(&pixmap);
    painter.setPen(QPen(JyTheme::color("text"), 2));
    if (restore) {
        painter.drawRect(12, 7, 13, 13);
        painter.fillRect(7, 12, 13, 13, JyTheme::color("bg-panel"));
        painter.drawRect(7, 12, 13, 13);
    } else {
        painter.drawRect(7, 7, 18, 18);
    }
    return QIcon(pixmap);
}
}

JyTitleBar::JyTitleBar(QWidget *window) : QWidget(window), m_window(window) {
    setObjectName("windowTitleBar");
    setFixedHeight(36);
    m_window->setMouseTracking(true);
    setMouseTracking(true);
    auto layout = new QHBoxLayout(this);
    layout->setContentsMargins(12, 0, 0, 0);
    layout->setSpacing(0);
    m_title = new QLabel(window->windowTitle(), this);
    m_title->setObjectName("windowTitleText");
    m_title->setMinimumWidth(0);
    m_title->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
    m_title->setAttribute(Qt::WA_TransparentForMouseEvents);
    m_title->setToolTip(window->windowTitle());
    layout->addWidget(m_title, 1);
    auto button = [this, layout](const QString &name, const QString &label, const QIcon &icon) {
        auto control = new QToolButton(this);
        control->setObjectName(name);
        control->setProperty("windowControl", true);
        control->setFixedSize(44, 36);
        control->setIcon(icon);
        control->setIconSize(QSize(16, 16));
        control->setToolTip(label);
        control->setAccessibleName(label);
        layout->addWidget(control);
        return control;
    };
    auto minimize = button("windowMinimize", tr("Minimize"), JyTheme::icon("minus"));
    m_maximize = button("windowMaximize", tr("Maximize"), maximizeIcon(false));
    auto close = button("windowClose", tr("Close"), JyTheme::icon("x"));
    connect(minimize, &QToolButton::clicked, window, &QWidget::showMinimized);
    connect(m_maximize, &QToolButton::clicked, this, &JyTitleBar::toggleMaximized);
    connect(close, &QToolButton::clicked, window, &QWidget::close);
    connect(window, &QWidget::windowTitleChanged, this, [this](const QString &title) {
        m_title->setText(title);
        m_title->setToolTip(title);
    });
    qApp->installEventFilter(this);
    updateWindowState();
}

void JyTitleBar::toggleMaximized() {
    if (m_window->isFullScreen()) return;
    if (m_window->isMaximized()) m_window->showNormal();
    else m_window->showMaximized();
}

void JyTitleBar::updateWindowState() {
    const bool maximized = m_window->isMaximized();
    const QString label = maximized ? tr("Restore") : tr("Maximize");
    m_maximize->setToolTip(label);
    m_maximize->setAccessibleName(label);
    m_maximize->setIcon(maximizeIcon(maximized));
    m_window->setContentsMargins(maximized || m_window->isFullScreen() ? 0 : 5,
                                maximized || m_window->isFullScreen() ? 0 : 5,
                                maximized || m_window->isFullScreen() ? 0 : 5,
                                maximized || m_window->isFullScreen() ? 0 : 5);
}

Qt::Edges JyTitleBar::resizeEdges(const QPoint &position) const {
    if (m_window->isMaximized() || m_window->isFullScreen()) return {};
    Qt::Edges edges;
    if (position.x() < 5) edges |= Qt::LeftEdge;
    else if (position.x() >= m_window->width() - 5) edges |= Qt::RightEdge;
    if (position.y() < 5) edges |= Qt::TopEdge;
    else if (position.y() >= m_window->height() - 5) edges |= Qt::BottomEdge;
    return edges;
}

bool JyTitleBar::eventFilter(QObject *watched, QEvent *event) {
    if (watched == m_window && event->type() == QEvent::WindowStateChange) updateWindowState();
    auto widget = qobject_cast<QWidget *>(watched);
    if (!widget || widget->window() != m_window) return false;
    if (event->type() == QEvent::MouseButtonPress) {
        auto mouse = static_cast<QMouseEvent *>(event);
        const auto edges = resizeEdges(m_window->mapFromGlobal(mouse->globalPosition().toPoint()));
        if (mouse->button() == Qt::LeftButton && edges) {
            if (!m_window->windowHandle() || !m_window->windowHandle()->startSystemResize(edges)) {
                m_resizing = edges;
                m_pressPosition = mouse->globalPosition().toPoint();
                m_pressGeometry = m_window->geometry();
                m_window->grabMouse();
            }
            return true;
        }
    } else if (event->type() == QEvent::MouseMove && m_resizing) {
        const auto delta = static_cast<QMouseEvent *>(event)->globalPosition().toPoint() - m_pressPosition;
        QRect geometry = m_pressGeometry;
        const auto minimum = m_window->minimumSize().expandedTo(m_window->minimumSizeHint());
        if (m_resizing.testFlag(Qt::LeftEdge)) geometry.setLeft(qBound(geometry.right() - m_window->maximumWidth() + 1, geometry.left() + delta.x(), geometry.right() - minimum.width() + 1));
        if (m_resizing.testFlag(Qt::RightEdge)) geometry.setWidth(qBound(minimum.width(), geometry.width() + delta.x(), m_window->maximumWidth()));
        if (m_resizing.testFlag(Qt::TopEdge)) geometry.setTop(qBound(geometry.bottom() - m_window->maximumHeight() + 1, geometry.top() + delta.y(), geometry.bottom() - minimum.height() + 1));
        if (m_resizing.testFlag(Qt::BottomEdge)) geometry.setHeight(qBound(minimum.height(), geometry.height() + delta.y(), m_window->maximumHeight()));
        m_window->setGeometry(geometry);
        return true;
    } else if (event->type() == QEvent::MouseMove && (widget == m_window || widget == this)) {
        const auto edges = resizeEdges(m_window->mapFromGlobal(static_cast<QMouseEvent *>(event)->globalPosition().toPoint()));
        if (edges == (Qt::LeftEdge | Qt::TopEdge) || edges == (Qt::RightEdge | Qt::BottomEdge)) widget->setCursor(Qt::SizeFDiagCursor);
        else if (edges == (Qt::RightEdge | Qt::TopEdge) || edges == (Qt::LeftEdge | Qt::BottomEdge)) widget->setCursor(Qt::SizeBDiagCursor);
        else if (edges.testFlag(Qt::LeftEdge) || edges.testFlag(Qt::RightEdge)) widget->setCursor(Qt::SizeHorCursor);
        else if (edges) widget->setCursor(Qt::SizeVerCursor);
        else widget->unsetCursor();
    } else if (event->type() == QEvent::Enter && widget != m_window && widget != this) {
        m_window->unsetCursor();
        unsetCursor();
    } else if (event->type() == QEvent::MouseButtonRelease && m_resizing) {
        m_resizing = {};
        m_window->releaseMouse();
        return true;
    }
    return false;
}

void JyTitleBar::mousePressEvent(QMouseEvent *event) {
    if (event->button() != Qt::LeftButton || m_window->isFullScreen()) return;
    if (m_window->windowHandle() && m_window->windowHandle()->startSystemMove()) return;
    if (m_window->isMaximized()) return;
    m_moving = true;
    m_pressPosition = event->globalPosition().toPoint();
    m_pressGeometry = m_window->geometry();
}

void JyTitleBar::mouseMoveEvent(QMouseEvent *event) {
    if (m_moving && event->buttons().testFlag(Qt::LeftButton))
        m_window->move(m_pressGeometry.topLeft() + event->globalPosition().toPoint() - m_pressPosition);
}

void JyTitleBar::mouseReleaseEvent(QMouseEvent *) { m_moving = false; }

void JyTitleBar::mouseDoubleClickEvent(QMouseEvent *event) {
    if (event->button() == Qt::LeftButton) {
        m_moving = false;
        toggleMaximized();
    }
}
