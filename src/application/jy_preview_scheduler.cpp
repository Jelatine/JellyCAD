#include "jy_preview_scheduler.h"
#include <QCryptographicHash>
#include <QFile>
#include <QFileInfo>
JyPreviewScheduler::JyPreviewScheduler(QObject *parent) : QObject(parent) {
    m_timer.setSingleShot(true);
    m_timer.setInterval(200);
    connect(&m_timer, &QTimer::timeout, this, [this] {
        if (!m_pending || m_busy || !m_enabled) return;
        m_pending = false;
        emit requested(m_path);
    });
}
QByteArray JyPreviewScheduler::fingerprint() const {
    QFile file(m_path);
    if (!file.open(QIODevice::ReadOnly)) return {};
    QCryptographicHash hash(QCryptographicHash::Sha256);
    hash.addData(&file);
    return hash.result();
}
void JyPreviewScheduler::setDocument(const QString &path) {
    cancelPending();
    m_path = path.isEmpty() ? QString() : QFileInfo(path).absoluteFilePath();
    remember();
}
void JyPreviewScheduler::remember() { m_hash = fingerprint(); }
bool JyPreviewScheduler::changed(const QString &path) {
    if (m_path.isEmpty() || QFileInfo(path).absoluteFilePath() != m_path) return false;
    auto hash = fingerprint();
    if (hash.isEmpty() || hash == m_hash) return false;
    m_hash = hash;
    return true;
}
void JyPreviewScheduler::schedule() {
    if (!m_enabled || m_path.isEmpty()) return;
    m_pending = true;
    if (!m_busy) m_timer.start();
}
void JyPreviewScheduler::cancelPending() { m_pending = false; m_timer.stop(); }
void JyPreviewScheduler::setBusy(bool busy) {
    m_busy = busy;
    if (!busy && m_pending && m_enabled) m_timer.start();
}
void JyPreviewScheduler::setEnabled(bool enabled) {
    m_enabled = enabled;
    if (!enabled) cancelPending();
}
