#pragma once
#include <QCoreApplication>
#include <QElapsedTimer>
#include <QThread>
#include <QFile>
#include <functional>
#include <stdexcept>
inline void ensureApp() {
    static int argc = 1;
    static char name[] = "JellyCAD_tests";
    static char *argv[] = {name, nullptr};
    static QCoreApplication app(argc, argv);
}
inline bool waitUntil(const std::function<bool()> &predicate, int timeout = 5000) {
    ensureApp();
    QElapsedTimer timer;
    timer.start();
    while (!predicate() && timer.elapsed() < timeout) {
        QCoreApplication::processEvents();
        QThread::msleep(1);
    }
    return predicate();
}
inline void writeFile(const QString &path, const QByteArray &data) {
    QFile file(path);
    if (!file.open(QIODevice::WriteOnly) || file.write(data) != data.size()) throw std::runtime_error("Cannot write test file");
}
