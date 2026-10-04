#include "runtime/jy_runtime.h"
#include <QCoreApplication>
#include <QCommandLineParser>
#include <iostream>
int main(int argc, char **argv) {
    QCoreApplication app(argc, argv);
    QCoreApplication::setApplicationName("JellyCAD");
    QCoreApplication::setApplicationVersion(JELLY_CAD_VERSION);
    QCommandLineParser parser;
    parser.addHelpOption(); parser.addVersionOption();
    QCommandLineOption file({"f", "file"}, "Script file", "file");
    QCommandLineOption code({"c", "code"}, "Lua code", "code");
    parser.addOption(file); parser.addOption(code); parser.process(app);
    if (!parser.isSet(file) && !parser.isSet(code)) parser.showHelp(1);
    jelly::RunRequest request;
    request.isFile = parser.isSet(file);
    request.source = parser.value(request.isFile ? file : code).toStdString();
    request.trigger = jelly::RunSource::CommandLine;
    std::atomic<bool> cancel{false};
    const auto result = jelly::execute(request, cancel, [](jelly::RunEvent event) {
        if (auto *text = std::get_if<std::string>(&event)) std::cout << *text << '\n';
    });
    if (result.status != jelly::RunStatus::Success) std::cerr << result.message << '\n';
    return result.status == jelly::RunStatus::Success ? 0 : 1;
}
