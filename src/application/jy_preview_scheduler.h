#pragma once
#include <QObject>
#include <QTimer>
#include <QByteArray>

// Coalesces file notifications and keeps only the latest preview while busy.
class JyPreviewScheduler : public QObject {
    Q_OBJECT
public:
    explicit JyPreviewScheduler(QObject *parent = nullptr);
    void setDocument(const QString &path);
    void remember();
    bool changed(const QString &path);
    void schedule();
    void cancelPending();
    void setBusy(bool busy);
    void setEnabled(bool enabled);
    bool enabled() const { return m_enabled; }
signals:
    void requested(const QString &path);
private:
    QByteArray fingerprint() const;
    QString m_path;
    QByteArray m_hash;
    QTimer m_timer;
    bool m_enabled = true;
    bool m_busy = false;
    bool m_pending = false;
};
