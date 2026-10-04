/*
 * Copyright (c) 2024. Li Jianbin. All rights reserved.
 * MIT License
 */
#include "jy_code_editor.h"
#include "jy_theme.h"
#include <QDesktopServices>
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
#include <QAbstractItemView>
#include <QHelpEvent>
#include <QKeyEvent>
#include <QMouseEvent>
#include <QScrollBar>
#include <QStringListModel>
#include <QToolTip>

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
    setMouseTracking(true);
    keyword_list_ = JyLuaAnalysis::keywords();
    keyword_list_.append(JyLuaAnalysis::builtins().keys());
    keyword_list_.sort();
    completer_ = new QCompleter(this);
    completer_->setWidget(this);
    completer_->setModel(new QStringListModel(this));
    completer_->setCaseSensitivity(Qt::CaseSensitive);
    completer_->setCompletionMode(QCompleter::PopupCompletion);
    completer_->popup()->setObjectName("luaCompletion");
    completer_->popup()->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    connect(completer_, qOverload<const QString &>(&QCompleter::activated), this, &JyCodeEditor::insertCompletion);
    analysis_timer_.setSingleShot(true);
    analysis_timer_.setInterval(250);
    connect(&analysis_timer_, &QTimer::timeout, this, &JyCodeEditor::refreshAnalysis);
    connect(this, &QPlainTextEdit::textChanged, this, [this] {
        if (refreshing_) return;
        // Clear folds on edits so deleted/shifted block headers cannot hide unrelated text.
        unfoldAll();
        analysis_dirty_ = true;
        setExtraSelections({});
        QToolTip::hideText();
        analysis_timer_.start();
    });
    connect(this, &QPlainTextEdit::cursorPositionChanged, this, [this] {
        // Moving into a hidden block through search or keyboard navigation reveals it.
        if (!textCursor().block().isVisible()) unfoldAll();
        if (!analysis_dirty_) updateSelections();
        QToolTip::hideText();
    });
    auto action = [this](const QString &label, const QKeySequence &shortcut, auto callback) {
        auto *item = new QAction(label, this);
        item->setShortcut(shortcut);
        item->setShortcutContext(Qt::WidgetShortcut);
        connect(item, &QAction::triggered, this, callback);
        addAction(item);
        code_actions_.append(item);
    };
    action(tr("Complete"), QKeySequence(Qt::CTRL | Qt::Key_Space), &JyCodeEditor::requestCompletion);
    action(tr("Parameter Hint"), QKeySequence(Qt::CTRL | Qt::SHIFT | Qt::Key_Space), &JyCodeEditor::showSignature);
    action(tr("Go to Definition"), QKeySequence(Qt::Key_F12), &JyCodeEditor::goToDefinition);
    action(tr("Find References"), QKeySequence(Qt::SHIFT | Qt::Key_F12), &JyCodeEditor::findReferences);
    action(tr("Format Lua Code"), QKeySequence(Qt::CTRL | Qt::SHIFT | Qt::Key_F), &JyCodeEditor::formatCode);
    action(tr("Check Lua Syntax"), QKeySequence(Qt::CTRL | Qt::SHIFT | Qt::Key_M), &JyCodeEditor::refreshAnalysis);
    action(tr("Toggle Fold"), QKeySequence(Qt::CTRL | Qt::SHIFT | Qt::Key_BracketLeft), [this] { toggleFold(textCursor().blockNumber()); });
    action(tr("Unfold All"), QKeySequence(Qt::CTRL | Qt::SHIFT | Qt::Key_BracketRight), &JyCodeEditor::unfoldAll);
    analysis_timer_.start();
    connect(this, &JyCodeEditor::blockCountChanged, this, &JyCodeEditor::slot_update_number_width);
    connect(this, &JyCodeEditor::updateRequest, this, &JyCodeEditor::slot_update_number_area);
    slot_update_number_width(0);
    viewport()->setStyleSheet(QString("background-color: %1; border-left: 1px solid %2;")
                                 .arg(JyTheme::color("bg-base").name(), JyTheme::color("border").name()));
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
            if (rule["name"].toString() == "comment" || rule["name"].toString() == "string") continue;
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
        file.close();
    }
}


void JyCodeEditor::set_text(const QString &text) {
    is_CRLF = text.contains("\r\n");
    unfoldAll();
    setPlainText(text);
}

QString JyCodeEditor::get_text() const {
    if (is_CRLF) {
        QString result = toPlainText();
        // 先统一为LF，再转为CRLF
        result.replace("\r\n", "\n");// 防止重复转换
        result.replace("\n", "\r\n");
        return result;
    }
    return toPlainText();
}

int JyCodeEditor::number_area_width() {
    int digits = 1;
    int max = qMax(1, blockCount());
    while (max >= 10) {
        max /= 10;
        ++digits;
    }
    int space = 24 + fontMetrics().horizontalAdvance(QLatin1Char('9')) * digits;
    return space;
}

void JyCodeEditor::paint_line_number(QPaintEvent *event) {
    if (!number_area_) { return; }
    QPainter painter(number_area_);
    painter.fillRect(event->rect(), JyTheme::color("bg-base"));
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
            painter.setPen(JyTheme::color("text-disabled"));// 行号栏字体色
            const auto rect = QRect(0, t_top, number_area_->width() - 16, fontMetrics().height());
            painter.drawText(rect, Qt::AlignRight, QString::number(t_number));
            if (analysis_ && !analysis_dirty_) {
                for (const auto &fold : analysis_->folds) {
                    if (fold.first == t_block_number) {
                        const auto icon = JyTheme::icon(folded_lines_.contains(fold.first) ? "chevron-right-muted" : "chevron-down-muted");
                        icon.paint(&painter, QRect(number_area_->width() - 14, t_top, 14, fontMetrics().height()));
                        break;
                    }
                }
            }
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
    menu->setParent(window(), Qt::Popup);
    menu->addSeparator();
    menu->addActions(code_actions_);
    menu->addSeparator();
    // 创建"Open Containing Folder"动作
    QAction *showInExplorerAction = new QAction(tr("Open Containing Folder"), menu);
    const bool enabled = !m_filePath.isEmpty() && QFileInfo::exists(m_filePath);
    // 如果没有设置文件路径或文件不存在，则禁用该选项
    showInExplorerAction->setEnabled(enabled);
    // 连接信号槽
    connect(showInExplorerAction, &QAction::triggered, this, &JyCodeEditor::showInExplorer);
    // 添加到菜单
    menu->addAction(showInExplorerAction);
    if (!m_vscodeCmd.isEmpty()) {
        QAction *editInVscode = new QAction(tr("Edit in VSCode"), menu);
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

void JyCodeEditor::Highlighter::highlightBlock(const QString &text) {
    for (const auto &rule : highlightingRules) {
        auto matches = rule.pattern.globalMatch(text);
        while (matches.hasNext()) {
            const auto match = matches.next();
            setFormat(match.capturedStart(), match.capturedLength(), rule.format);
        }
    }
    QTextCharFormat stringFormat, commentFormat;
    stringFormat.setForeground(QColor("#CE9178"));
    commentFormat.setForeground(QColor("#6A9955"));
    commentFormat.setFontItalic(true);
    // States 1/2: escaped short string. States >= 10: long bracket with
    // (state - 10) / 2 equals signs and the low bit indicating a comment.
    int state = qMax(0, previousBlockState()), i = 0;
    setCurrentBlockState(0);
    while (i < text.size() || state >= 10) {
        const int start = i;
        if (state >= 10) {
            const auto close = "]" + QString((state - 10) / 2, '=') + "]";
            const int end = text.indexOf(close, i);
            setFormat(start, (end < 0 ? text.size() : end + close.size()) - start,
                      (state & 1) ? commentFormat : stringFormat);
            if (end < 0) { setCurrentBlockState(state); return; }
            i = end + close.size(); state = 0; continue;
        }
        if (state == 1 || state == 2 || text[i] == QChar(39) || text[i] == QChar(34)) {
            const QChar quote = state ? QChar(state == 1 ? 39 : 34) : text[i++];
            state = 0;
            while (i < text.size()) {
                if (text[i] == QChar(92)) {
                    ++i;
                    if (i == text.size()) { state = quote == QChar(39) ? 1 : 2; break; }
                    if (text[i] == 'z') {
                        ++i; while (i < text.size() && text[i].isSpace()) ++i;
                        if (i == text.size()) state = quote == QChar(39) ? 1 : 2;
                    } else ++i;
                } else if (text[i++] == quote) { state = 0; break; }
            }
            setFormat(start, i - start, stringFormat);
            setCurrentBlockState(state); continue;
        }
        const bool comment = text.mid(i, 2) == "--";
        const int bracket = i + (comment ? 2 : 0);
        if (bracket < text.size() && text[bracket] == '[') {
            int end = bracket + 1;
            while (end < text.size() && text[end] == '=') ++end;
            if (end < text.size() && text[end] == '[') {
                const int equals = end - bracket - 1;
                const auto close = "]" + QString(equals, '=') + "]";
                const int closeAt = text.indexOf(close, end + 1);
                i = closeAt < 0 ? text.size() : closeAt + close.size();
                setFormat(start, i - start, comment ? commentFormat : stringFormat);
                if (closeAt < 0) { setCurrentBlockState(10 + equals * 2 + (comment ? 1 : 0)); return; }
                continue;
            }
        }
        if (comment) { setFormat(start, text.size() - start, commentFormat); return; }
        ++i;
    }
    setCurrentBlockState(state);
}

const JyLuaAnalysis &JyCodeEditor::analysis() {
    if (analysis_dirty_ || !analysis_) refreshAnalysis();
    return *analysis_;
}

void JyCodeEditor::refreshAnalysis() {
    if (refreshing_) return;
    refreshing_ = true;
    analysis_timer_.stop();
    analysis_ = std::make_unique<JyLuaAnalysis>(toPlainText());
    analysis_dirty_ = false;
    refreshing_ = false;
    updateSelections();
    number_area_->update();
    emit analysisUpdated();
}

void JyCodeEditor::updateSelections() {
    if (!analysis_ || analysis_dirty_) return;
    QList<QTextEdit::ExtraSelection> selections;
    auto add = [&](int start, int length, const QTextCharFormat &format) {
        QTextEdit::ExtraSelection item;
        item.cursor = QTextCursor(document());
        item.cursor.setPosition(qBound(0, start, document()->characterCount() - 1));
        item.cursor.setPosition(qMin(start + length, document()->characterCount() - 1), QTextCursor::KeepAnchor);
        item.format = format;
        selections.append(item);
    };
    for (const auto &diagnostic : analysis_->diagnostics) {
        QTextCharFormat format;
        format.setUnderlineStyle(QTextCharFormat::WaveUnderline);
        format.setUnderlineColor(JyTheme::color(diagnostic.warning ? "warning" : "danger"));
        format.setToolTip(diagnostic.message);
        add(diagnostic.start, diagnostic.length, format);
    }
    const int position = textCursor().position();
    int t = analysis_->tokenAt(position);
    if (t < 0 || analysis_->tokens[t].pair < 0) t = analysis_->tokenAt(position - 1);
    if (t >= 0) {
        const auto &token = analysis_->tokens[t];
        QTextCharFormat match;
        auto matchColor = JyTheme::color("accent");
        matchColor.setAlpha(56);
        match.setBackground(matchColor);
        match.setForeground(JyTheme::color("text"));
        if (token.pair >= 0 && (position == token.start || position == token.end)) {
            add(token.start, token.end - token.start, match);
            const auto &other = analysis_->tokens[token.pair];
            add(other.start, other.end - other.start, match);
        } else if (token.kind == JyLuaAnalysis::Kind::String && token.text.size() >= 2 &&
                   (token.text[0] == '\'' || token.text[0] == '"') && token.text.back() == token.text[0] &&
                   (position == token.start || position == token.start + 1 || position == token.end - 1 || position == token.end)) {
            add(token.start, 1, match); add(token.end - 1, 1, match);
        }
    }
    setExtraSelections(selections);
}

void JyCodeEditor::NumberArea::mousePressEvent(QMouseEvent *event) {
    if (event->button() != Qt::LeftButton || event->position().x() < width() - 16) return;
    QTextBlock block = editor_->firstVisibleBlock();
    while (block.isValid()) {
        const auto geometry = editor_->blockBoundingGeometry(block).translated(editor_->contentOffset());
        if (block.isVisible() && geometry.contains(QPointF(0, event->position().y()))) {
            editor_->toggleFold(block.blockNumber()); return;
        }
        block = block.next();
    }
}

void JyCodeEditor::applyFolds() {
    // Folding is presentation state, never a document edit.
    refreshing_ = true;
    for (QTextBlock b = document()->begin(); b.isValid(); b = b.next()) {
        bool visible = true;
        if (analysis_) for (const auto &range : analysis_->folds) {
            if (folded_lines_.contains(range.first) && b.blockNumber() > range.first && b.blockNumber() <= range.last) {
                visible = false; break;
            }
        }
        b.setVisible(visible);
        b.setLineCount(visible ? 1 : 0);
    }
    document()->markContentsDirty(0, document()->characterCount());
    refreshing_ = false;
    viewport()->update();
    number_area_->update();
}
void JyCodeEditor::toggleFold(int line) {
    const auto &model = analysis();
    for (const auto &range : model.folds) {
        if (range.first != line) continue;
        if (folded_lines_.contains(line)) folded_lines_.remove(line);
        else {
            if (textCursor().blockNumber() > line && textCursor().blockNumber() <= range.last)
                goToPosition(document()->findBlockByNumber(line).position());
            folded_lines_.insert(line);
        }
        applyFolds(); return;
    }
}
void JyCodeEditor::unfoldAll() {
    if (folded_lines_.isEmpty()) return;
    folded_lines_.clear(); applyFolds();
}
void JyCodeEditor::goToPosition(int position) {
    unfoldAll();
    auto cursor = textCursor();
    cursor.setPosition(qBound(0, position, document()->characterCount() - 1));
    setTextCursor(cursor); ensureCursorVisible(); setFocus();
}
int JyCodeEditor::symbolPosition() const {
    const int position = textCursor().position();
    // At the end of a name, use the token to the left (common keyboard workflow).
    if (analysis_ && position > 0) {
        const int t = analysis_->tokenAt(position - 1);
        if (t >= 0 && analysis_->tokens[t].kind == JyLuaAnalysis::Kind::Name && analysis_->tokens[t].end == position) return position - 1;
    }
    return position;
}
void JyCodeEditor::goToDefinition() {
    analysis();
    const int position = analysis_->definitionAt(symbolPosition());
    if (position >= 0) goToPosition(position);
    else emit editorMessage(tr("No definition found in this document."));
}
void JyCodeEditor::findReferences() {
    analysis();
    const auto positions = analysis_->referencesAt(symbolPosition());
    emit referencesFound(positions);
    if (positions.isEmpty()) emit editorMessage(tr("No references found in this document."));
}
void JyCodeEditor::formatCode() {
    const auto &model = analysis();
    for (const auto &diagnostic : model.diagnostics) if (!diagnostic.warning) {
        emit editorMessage(tr("Fix Lua syntax errors before formatting.")); return;
    }
    const auto formatted = model.formatted();
    if (formatted == toPlainText()) return;
    int line = textCursor().blockNumber(), column = textCursor().positionInBlock();
    unfoldAll();
    QTextCursor cursor(document());
    cursor.beginEditBlock(); cursor.select(QTextCursor::Document); cursor.insertText(formatted); cursor.endEditBlock();
    const auto block = document()->findBlockByNumber(line);
    cursor.setPosition(block.position() + qMin(column, block.length() - 1));
    setTextCursor(cursor);
    refreshAnalysis();
}
QString JyCodeEditor::completionPrefix() const {
    const auto text = toPlainText();
    int end = textCursor().position(), start = end;
    while (start > 0 && (text[start - 1].isLetterOrNumber() || text[start - 1] == '_' || text[start - 1] == '.' || text[start - 1] == ':')) --start;
    return text.mid(start, end - start);
}
void JyCodeEditor::requestCompletion() {
    const int position = textCursor().position();
    const auto &model = analysis();
    if (position > 0 && !model.isCode(position - 1)) { completer_->popup()->hide(); return; }
    auto *list = qobject_cast<QStringListModel *>(completer_->model());
    list->setStringList(model.completions(position));
    completer_->setCompletionPrefix(completionPrefix());
    if (completer_->completionCount() == 0) { completer_->popup()->hide(); return; }
    completer_->popup()->setCurrentIndex(completer_->completionModel()->index(0, 0));
    QRect rect = cursorRect();
    rect.setWidth(qMax(260, completer_->popup()->sizeHintForColumn(0) + completer_->popup()->verticalScrollBar()->sizeHint().width()));
    completer_->complete(rect);
}
void JyCodeEditor::insertCompletion(const QString &text) {
    auto cursor = textCursor();
    const int length = completionPrefix().size();
    cursor.setPosition(cursor.position() - length, QTextCursor::KeepAnchor);
    cursor.insertText(text); setTextCursor(cursor);
}
void JyCodeEditor::showSignature() {
    int argument = 0;
    const QString signature = analysis().signatureAt(textCursor().position(), &argument);
    if (signature.isEmpty()) { QToolTip::hideText(); return; }
    QToolTip::showText(mapToGlobal(cursorRect().bottomRight()), signature + tr("\nArgument %1").arg(argument + 1), this);
}
void JyCodeEditor::indentSelection(bool outdent) {
    auto cursor = textCursor();
    const int first = document()->findBlock(cursor.selectionStart()).blockNumber();
    const int last = document()->findBlock(qMax(cursor.selectionStart(), cursor.selectionEnd() - 1)).blockNumber();
    const bool selection = cursor.hasSelection();
    cursor.beginEditBlock();
    for (int line = last; line >= first; --line) {
        const auto block = document()->findBlockByNumber(line);
        cursor.setPosition(block.position());
        if (outdent) {
            int count = 0;
            if (block.text().startsWith('\t')) count = 1;
            else while (count < 4 && count < block.text().size() && block.text()[count] == ' ') ++count;
            cursor.setPosition(block.position() + count, QTextCursor::KeepAnchor); cursor.removeSelectedText();
        } else cursor.insertText("    ");
    }
    cursor.endEditBlock();
    if (selection) {
        cursor.setPosition(document()->findBlockByNumber(first).position());
        const auto end = document()->findBlockByNumber(last);
        cursor.setPosition(end.position() + end.length() - 1, QTextCursor::KeepAnchor);
    }
    setTextCursor(cursor);
}
void JyCodeEditor::autoOutdent() {
    const auto block = textCursor().block();
    const auto trimmed = block.text().trimmed();
    static const QSet<QString> closers = {"end", "else", "elseif", "until", "}", "]", ")"};
    if (!closers.contains(trimmed)) return;
    const auto &model = analysis();
    const int first = block.position() + block.text().indexOf(trimmed);
    if (!model.isCode(first)) return;
    const auto desired = model.formatted().section('\n', block.blockNumber(), block.blockNumber());
    const int oldIndent = block.text().size() - block.text().trimmed().size();
    const int newIndent = desired.size() - desired.trimmed().size();
    if (oldIndent <= newIndent) return;
    auto cursor = textCursor();
    cursor.joinPreviousEditBlock();
    cursor.setPosition(block.position());
    cursor.setPosition(first, QTextCursor::KeepAnchor);
    cursor.insertText(QString(newIndent, ' '));
    cursor.endEditBlock();
    cursor.movePosition(QTextCursor::EndOfBlock);
    setTextCursor(cursor);
}

void JyCodeEditor::keyPressEvent(QKeyEvent *event) {
    if (completer_->popup()->isVisible()) {
        switch (event->key()) {
        case Qt::Key_Enter: case Qt::Key_Return: case Qt::Key_Tab:
            insertCompletion(completer_->currentCompletion()); completer_->popup()->hide(); return;
        case Qt::Key_Escape: completer_->popup()->hide(); return;
        default: break;
        }
    }
    if (event->modifiers() == Qt::ControlModifier && event->key() == Qt::Key_Slash) { toggleComment(); return; }
    if (event->key() == Qt::Key_Backtab || (event->key() == Qt::Key_Tab && event->modifiers() == Qt::ShiftModifier)) { indentSelection(true); return; }
    if (event->key() == Qt::Key_Tab && event->modifiers() == Qt::NoModifier) {
        if (textCursor().hasSelection()) indentSelection(false);
        else insertPlainText(QString(4 - textCursor().positionInBlock() % 4, ' '));
        return;
    }
    auto cursor = textCursor();
    const auto text = toPlainText();
    const int position = cursor.position();
    const QString key = event->text();
    const bool simple = !(event->modifiers() & (Qt::ControlModifier | Qt::AltModifier | Qt::MetaModifier));
    if (simple && (event->key() == Qt::Key_Return || event->key() == Qt::Key_Enter)) {
        const auto &model = analysis();
        int indentation;
        const int previous = model.tokenAt(position - 1);
        if (previous >= 0 && (model.tokens[previous].kind == JyLuaAnalysis::Kind::String ||
            (model.tokens[previous].kind == JyLuaAnalysis::Kind::Comment && model.tokens[previous].text.startsWith("--[")))) {
            const QString line = cursor.block().text();
            indentation = 0; for (QChar ch : line) { if (ch == ' ') ++indentation; else if (ch == '\t') indentation += 4; else break; }
        } else indentation = model.indentationAfter(cursor.selectionStart());
        const bool paired = position > 0 && position < text.size() &&
            ((text[position - 1] == '{' && text[position] == '}') || (text[position - 1] == '(' && text[position] == ')') || (text[position - 1] == '[' && text[position] == ']'));
        cursor.beginEditBlock();
        cursor.insertText("\n" + QString(indentation, ' '));
        const int inside = cursor.position();
        if (paired) cursor.insertText("\n" + QString(qMax(0, indentation - 4), ' '));
        cursor.endEditBlock(); cursor.setPosition(inside); setTextCursor(cursor); return;
    }
    if (simple && key.size() == 1) {
        const QString opening = "([{\"'";
        const QString endChars = ")]}\"'";
        const int index = opening.indexOf(key);
        const auto &model = analysis();
        const bool inCode = model.isCode(position) && (position == 0 || model.isCode(position - 1));
        if (!cursor.hasSelection() && endChars.contains(key) && position < text.size() && text.mid(position, 1) == key) {
            const int t = model.tokenAt(position);
            const bool closingToken = t >= 0 && (model.tokens[t].pair >= 0 ||
                (model.tokens[t].kind == JyLuaAnalysis::Kind::String && model.tokens[t].end - 1 == position));
            if (closingToken) { cursor.movePosition(QTextCursor::Right); setTextCursor(cursor); return; }
        }
        if (index >= 0 && (inCode || cursor.hasSelection())) {
            const QChar end = endChars[index];
            const bool boundary = position >= text.size() || text[position].isSpace() || QString(")]},;:").contains(text[position]);
            if (cursor.hasSelection() || boundary) {
                const int start = cursor.selectionStart();
                const QString selected = cursor.selectedText().replace(QChar::ParagraphSeparator, '\n');
                cursor.beginEditBlock(); cursor.insertText(key + selected + end); cursor.endEditBlock();
                cursor.setPosition(start + 1);
                if (!selected.isEmpty()) cursor.setPosition(start + 1 + selected.size(), QTextCursor::KeepAnchor);
                setTextCursor(cursor);
                if (key == "(") showSignature();
                return;
            }
        }
    }
    if (simple && event->key() == Qt::Key_Backspace && !cursor.hasSelection() && position > 0 && position < text.size()) {
        const QString opens = "([{\"'", closes = ")]}\"'";
        int pair = opens.indexOf(text[position - 1]);
        if (pair >= 0 && closes[pair] == text[position]) {
            const auto &model = analysis(); const int t = model.tokenAt(position - 1);
            if (t >= 0 && (model.tokens[t].pair >= 0 || (model.tokens[t].kind == JyLuaAnalysis::Kind::String && model.tokens[t].start == position - 1 && model.tokens[t].end == position + 1))) {
                cursor.setPosition(position - 1); cursor.setPosition(position + 1, QTextCursor::KeepAnchor);
                cursor.removeSelectedText(); setTextCursor(cursor); return;
            }
        }
    }
    QPlainTextEdit::keyPressEvent(event);
    if (simple && !key.isEmpty()) autoOutdent();
    if (simple && !key.isEmpty() && (key.back().isLetterOrNumber() || key == "_" || key == "." || key == ":")) {
        if (completionPrefix().size() >= 2 || key == "." || key == ":") requestCompletion();
    } else completer_->popup()->hide();
    if (key == "(" || key == "," || key == ")") showSignature();
}

void JyCodeEditor::mouseMoveEvent(QMouseEvent *event) {
    QPlainTextEdit::mouseMoveEvent(event);
    if (!analysis_ || analysis_dirty_) return;
    const int position = cursorForPosition(event->position().toPoint()).position();
    for (const auto &diagnostic : analysis_->diagnostics) {
        if (position >= diagnostic.start && position < diagnostic.start + diagnostic.length) {
            QToolTip::showText(event->globalPosition().toPoint(), diagnostic.message, this); return;
        }
    }
}
