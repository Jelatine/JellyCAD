/*
 * Copyright (c) 2024. Li Jianbin. All rights reserved.
 * MIT License
 */
#include "jy_code_editor.h"
#include <QDir>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QMenu>
#include <QProcess>
#include <QStandardPaths>
#include <QStyleOption>
#include <QTextStream>

JyCodeEditor::JyCodeEditor(QWidget *parent) : QPlainTextEdit(parent), number_area_(new NumberArea(this)) {
#ifdef Q_OS_WIN
    QStringList possiblePaths = {
            QStandardPaths::writableLocation(QStandardPaths::ApplicationsLocation) + "/Microsoft VS Code/bin/code.cmd",
            "C:/Program Files/Microsoft VS Code/bin/code.cmd",
            "C:/Program Files (x86)/Microsoft VS Code/bin/code.cmd",
            QDir::homePath() + "/AppData/Local/Programs/Microsoft VS Code/bin/code.cmd"};
    for (const QString &path: possiblePaths) {
        if (QFileInfo::exists(path)) {
            m_vscodeCmd = path;
            break;
        }
    }
#endif
    setWordWrapMode(QTextOption::NoWrap);
    QFont t_font = font();
    t_font.setFamily("Courier New");
    t_font.setPointSize(12);
    setFont(t_font);
    QFontMetrics metrics(t_font);
    setTabStopDistance(4 * metrics.averageCharWidth());
    init_highlighter();
    connect(this, &JyCodeEditor::blockCountChanged, this, &JyCodeEditor::slot_update_number_width);
    connect(this, &JyCodeEditor::updateRequest, this, &JyCodeEditor::slot_update_number_area);
    slot_update_number_width(0);
    viewport()->setStyleSheet("border-left: 1px solid #4b5059;");
}
void JyCodeEditor::init_highlighter() {
    highlighter_ = new Highlighter(this->document());
    QFile file(":/lua_syntax.json");
    if (file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        QTextStream stream(&file);
        const auto defaultConfig = stream.readAll();
        QJsonDocument doc = QJsonDocument::fromJson(defaultConfig.toUtf8());
        const auto config = doc.object();
        // 解析规则
        QJsonArray rules = config["rules"].toArray();
        for (const QJsonValue &value: rules) {
            QJsonObject rule = value.toObject();
            Highlighter::HighlightingRule highlightRule;
            highlightRule.pattern = QRegularExpression(rule["pattern"].toString());
            QTextCharFormat format;
            // 设置颜色
            if (rule.contains("color")) {
                format.setForeground(QColor(rule["color"].toString()));
            }
            // 设置粗体
            if (rule.contains("bold") && rule["bold"].toBool()) {
                format.setFontWeight(QFont::Bold);
            }
            // 设置斜体
            if (rule.contains("italic") && rule["italic"].toBool()) {
                format.setFontItalic(true);
            }
            // 设置下划线
            if (rule.contains("underline") && rule["underline"].toBool()) {
                format.setFontUnderline(true);
            }
            // 设置背景色
            if (rule.contains("background")) {
                format.setBackground(QColor(rule["background"].toString()));
            }
            highlightRule.format = format;
            highlighter_->addRule(highlightRule);
        }
        if (config.contains("multilineRules")) {
            QJsonArray multilineRules = config["multilineRules"].toArray();
            for (const QJsonValue &value: multilineRules) {
                QJsonObject rule = value.toObject();
                QString startPattern = rule["startPattern"].toString();
                QString endPattern = rule["endPattern"].toString();
                QTextCharFormat format;
                // 设置颜色
                if (rule.contains("color")) {
                    format.setForeground(QColor(rule["color"].toString()));
                }
                highlighter_->addMultilineRule(startPattern, endPattern, format);
            }
        }
        file.close();
    }
}

int JyCodeEditor::number_area_width() {
    int digits = 1;
    int max = qMax(1, blockCount());
    while (max >= 10) {
        max /= 10;
        ++digits;
    }
    int space = 8 + fontMetrics().horizontalAdvance(QLatin1Char('9')) * digits;
    return space;
}

void JyCodeEditor::paint_line_number(QPaintEvent *event) {
    if (!number_area_) { return; }
    QPainter painter(number_area_);
    QStyleOption opt;
    opt.initFrom(this);                                      //读取qss设置的样式
    QTextBlock t_first_visible_block = firstVisibleBlock();  // 第一个可看到的区间
    int t_block_number = t_first_visible_block.blockNumber();// 第一个区间号
    int t_top = qRound(blockBoundingGeometry(t_first_visible_block).translated(contentOffset()).top());
    int t_bottom = t_top + qRound(blockBoundingRect(t_first_visible_block).height());

    while (t_first_visible_block.isValid() && t_top <= event->rect().bottom()) {
        if (t_first_visible_block.isVisible() && t_bottom >= event->rect().top()) {
            int t_number = t_block_number + 1;
            //            painter.setPen(opt.palette.color(QPalette::WindowText));    // 行号栏字体色
            painter.setPen(QColor("#4b5059"));// 行号栏字体色
            const auto rect = QRect(0, t_top, number_area_->width(), fontMetrics().height());
            painter.drawText(rect, Qt::AlignRight, QString::number(t_number));
        }
        t_first_visible_block = t_first_visible_block.next();
        t_top = t_bottom;
        t_bottom = t_top + qRound(blockBoundingRect(t_first_visible_block).height());
        ++t_block_number;
    }
}


void JyCodeEditor::resizeEvent(QResizeEvent *event) {
    if (!number_area_) { return; }
    QPlainTextEdit::resizeEvent(event);
    QRect cr = contentsRect();
    number_area_->setGeometry(QRect(cr.left(), cr.top(), number_area_width(), cr.height()));
}

void JyCodeEditor::slot_update_number_area(const QRect &rect, int dy) {
    if (!number_area_) { return; }
    if (dy) {
        number_area_->scroll(0, dy);
    } else {
        number_area_->update(0, rect.y(), number_area_->width(), rect.height());
    }
    if (rect.contains(viewport()->rect())) {
        slot_update_number_width(0);
    } else {
    }
}

void JyCodeEditor::contextMenuEvent(QContextMenuEvent *event) {
    // 创建标准右键菜单
    QMenu *menu = createStandardContextMenu();
    // 添加分隔线
    menu->addSeparator();
    // 创建"Open Containing Folder"动作
    QAction *showInExplorerAction = new QAction(tr("Open Containing Folder"), this);
    const bool enabled = !m_filePath.isEmpty() && QFileInfo::exists(m_filePath);
    // 如果没有设置文件路径或文件不存在，则禁用该选项
    showInExplorerAction->setEnabled(enabled);
    // 连接信号槽
    connect(showInExplorerAction, &QAction::triggered, this, &JyCodeEditor::showInExplorer);
    // 添加到菜单
    menu->addAction(showInExplorerAction);
    if (!m_vscodeCmd.isEmpty()) {
        QAction *editInVscode = new QAction(tr("Edit in VSCode"), this);
        editInVscode->setEnabled(enabled);
        connect(editInVscode, &QAction::triggered, this, [this]() {
            QProcess::startDetached(m_vscodeCmd, {m_filePath});
        });
        menu->addAction(editInVscode);
    }
    // 显示菜单
    menu->exec(event->globalPos());
    delete menu;
}

void JyCodeEditor::toggleComment() {
    QTextCursor cursor = textCursor();
    // 保存原始位置信息
    int originalPosition = cursor.position();
    int originalAnchor = cursor.anchor();
    bool hasSelection = cursor.hasSelection();
    if (hasSelection) {
        // 处理选中多行的情况
        int startPos = qMin(cursor.position(), cursor.anchor());
        int endPos = qMax(cursor.position(), cursor.anchor());
        cursor.setPosition(startPos);
        int startBlockNum = cursor.blockNumber();
        cursor.setPosition(endPos);
        int endBlockNum = cursor.blockNumber();
        // 检查选中的所有行是否都已注释
        bool allCommented = true;
        for (int i = startBlockNum; i <= endBlockNum; ++i) {
            QTextBlock block = document()->findBlockByNumber(i);
            QString text = block.text();
            if (!text.trimmed().isEmpty() && !text.startsWith("--")) {
                allCommented = false;
                break;
            }
        }
        // 开始批量操作
        cursor.beginEditBlock();
        // 处理每一行
        for (int i = startBlockNum; i <= endBlockNum; ++i) {
            QTextBlock block = document()->findBlockByNumber(i);
            cursor.setPosition(block.position());
            QString text = block.text();
            if (text.trimmed().isEmpty()) {
                // 空行跳过
                continue;
            }
            if (allCommented) {
                // 解注释
                if (text.startsWith("-- ")) {
                    cursor.movePosition(QTextCursor::StartOfBlock);
                    cursor.movePosition(QTextCursor::Right, QTextCursor::KeepAnchor, 3);
                    cursor.removeSelectedText();
                } else if (text.startsWith("--")) {
                    cursor.movePosition(QTextCursor::StartOfBlock);
                    cursor.movePosition(QTextCursor::Right, QTextCursor::KeepAnchor, 2);
                    cursor.removeSelectedText();
                }
            } else {
                // 添加注释
                cursor.movePosition(QTextCursor::StartOfBlock);
                cursor.insertText("-- ");
            }
        }
        cursor.endEditBlock();
        // 恢复选择区域（调整位置）
        if (hasSelection) {
            // 重新计算选择区域
            QTextBlock startBlock = document()->findBlockByNumber(startBlockNum);
            QTextBlock endBlock = document()->findBlockByNumber(endBlockNum);
            cursor.setPosition(startBlock.position());
            cursor.setPosition(endBlock.position() + endBlock.length() - 1, QTextCursor::KeepAnchor);
            setTextCursor(cursor);
        }
    } else {
        // 处理单行
        cursor.beginEditBlock();
        // 移动到当前行开始
        cursor.movePosition(QTextCursor::StartOfBlock);
        QTextBlock block = cursor.block();
        QString text = block.text();
        if (text.trimmed().isEmpty()) {
            // 空行不处理
            cursor.endEditBlock();
            return;
        }
        if (text.startsWith("-- ")) {
            // 移除注释和空格
            cursor.movePosition(QTextCursor::Right, QTextCursor::KeepAnchor, 3);
            cursor.removeSelectedText();
        } else if (text.startsWith("--")) {
            // 移除注释
            cursor.movePosition(QTextCursor::Right, QTextCursor::KeepAnchor, 2);
            cursor.removeSelectedText();
        } else {
            // 添加注释
            cursor.insertText("-- ");
        }
        cursor.endEditBlock();
        // 恢复光标位置
        if (!hasSelection) {
            // 调整光标位置
            if (text.startsWith("--")) {
                // 解注释后，光标位置需要向前调整
                int adjustment = text.startsWith("-- ") ? 3 : 2;
                cursor.setPosition(qMax(block.position(), originalPosition - adjustment));
            } else {
                // 注释后，光标位置需要向后调整
                cursor.setPosition(originalPosition + 3);
            }
            setTextCursor(cursor);
        }
    }
}

void JyCodeEditor::showInExplorer() {
    if (m_filePath.isEmpty()) {
        return;
    }
    QFileInfo fileInfo(m_filePath);
    if (!fileInfo.exists()) {
        return;
    }
    // 获取文件所在目录
    QString dirPath = fileInfo.absolutePath();
    QString filePath = fileInfo.absoluteFilePath();
#ifdef Q_OS_WIN
    // Windows: 使用explorer.exe并选中文件
    QString param;
    if (!filePath.isEmpty()) {
        param = QString("/select,%1").arg(QDir::toNativeSeparators(filePath));
    }
    QProcess::startDetached("explorer.exe", QStringList() << param);

#elif defined(Q_OS_MAC)
    // macOS: 使用open命令并选中文件
    QStringList args;
    args << "-e";
    args << QString("tell application \"Finder\" to reveal POSIX file \"%1\"").arg(filePath);
    QProcess::execute("/usr/bin/osascript", args);

    // 激活Finder窗口
    args.clear();
    args << "-e";
    args << "tell application \"Finder\" to activate";
    QProcess::execute("/usr/bin/osascript", args);

#elif defined(Q_OS_LINUX)
    // Linux: 尝试不同的文件管理器

    // 方法1: 使用xdg-open（打开目录）
    bool success = false;

    // 尝试使用不同的文件管理器并选中文件
    QStringList fileManagers;
    fileManagers << "nautilus" << "dolphin" << "nemo" << "thunar" << "pcmanfm";

    for (const QString &fm: fileManagers) {
        QProcess process;
        process.start(fm, QStringList() << filePath);
        if (process.waitForStarted(1000)) {
            success = true;
            process.waitForFinished(-1);
            break;
        }
    }

    // 如果没有找到支持的文件管理器，使用xdg-open打开目录
    if (!success) {
        QDesktopServices::openUrl(QUrl::fromLocalFile(dirPath));
    }

#else
    // 其他系统：使用Qt的默认方法打开目录
    QDesktopServices::openUrl(QUrl::fromLocalFile(dirPath));
#endif
}

void JyCodeEditor::Highlighter::addMultilineRule(const QString &startPattern, const QString &endPattern,
                                                 const QTextCharFormat &format) {
    MultilineRule rule;
    rule.startPattern = QRegularExpression(startPattern);
    rule.endPattern = QRegularExpression(endPattern);
    rule.format = format;
    rule.stateId = nextStateId++;
    multilineRules.append(rule);
}
void JyCodeEditor::Highlighter::highlightBlock(const QString &text) {
    // 首先应用单行规则
    foreach (const HighlightingRule &rule, highlightingRules) {
        QRegularExpressionMatchIterator matchIterator = rule.pattern.globalMatch(text);
        while (matchIterator.hasNext()) {
            QRegularExpressionMatch match = matchIterator.next();
            setFormat(match.capturedStart(), match.capturedLength(), rule.format);
        }
    }
    // 处理多行规则
    setCurrentBlockState(0);
    foreach (const MultilineRule &rule, multilineRules) {
        int startIndex = 0;
        // 检查是否在多行块中继续
        if (previousBlockState() == rule.stateId) {
            startIndex = 0;
        } else {
            QRegularExpressionMatch startMatch = rule.startPattern.match(text);
            startIndex = startMatch.capturedStart();
            if (startIndex < 0) {
                continue;// 没有找到开始标记
            }
        }
        // 查找结束标记
        while (startIndex >= 0) {
            QRegularExpressionMatch endMatch = rule.endPattern.match(text, startIndex + (previousBlockState() == rule.stateId ? 0 : rule.startPattern.match(text, startIndex).capturedLength()));
            int endIndex = endMatch.capturedStart();
            int commentLength = 0;

            if (endIndex == -1) {
                // 没有找到结束标记，整行都是多行内容
                setCurrentBlockState(rule.stateId);
                commentLength = text.length() - startIndex;
            } else {
                // 找到结束标记
                commentLength = endIndex - startIndex + endMatch.capturedLength();
            }
            setFormat(startIndex, commentLength, rule.format);
            // 查找下一个开始标记
            if (endIndex != -1) {
                QRegularExpressionMatch nextStart = rule.startPattern.match(text, startIndex + commentLength);
                startIndex = nextStart.capturedStart();
            } else {
                break;
            }
        }
    }
}