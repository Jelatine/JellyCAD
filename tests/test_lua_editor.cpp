#include <gtest/gtest.h>
#include "jy_code_editor.h"
#include "jy_editor_widget.h"
#include <QApplication>
#include <QKeyEvent>
#include <QTemporaryDir>
#include <QFile>
#include <QDir>

namespace {
void key(JyCodeEditor &editor, int code, const QString &text = {}, Qt::KeyboardModifiers modifiers = Qt::NoModifier) {
    QKeyEvent event(QEvent::KeyPress, code, modifiers, text);
    QApplication::sendEvent(&editor, &event);
}
void at(JyCodeEditor &editor, int position) {
    auto cursor = editor.textCursor(); cursor.setPosition(position); editor.setTextCursor(cursor);
}
}
TEST(LuaAnalysis, ScopesShadowingAndInitializerVisibility) {
    const QString source = "local x = 1\ndo\n local x = x + 1\n print(x)\nend\nprint(x)\n-- x\nprint('x')";
    JyLuaAnalysis model(source);
    const int outer = source.indexOf('x');
    const int inner = source.indexOf("x = x");
    EXPECT_EQ(model.definitionAt(source.indexOf("x +")), outer);
    EXPECT_EQ(model.definitionAt(source.indexOf("x)")), inner);
    EXPECT_EQ(model.definitionAt(source.lastIndexOf("x)")), outer);
    EXPECT_EQ(model.referencesAt(outer).size(), 3);
    EXPECT_EQ(model.referencesAt(inner).size(), 2);
    EXPECT_TRUE(model.referencesAt(source.indexOf("-- x") + 3).isEmpty());
    EXPECT_TRUE(model.referencesAt(source.indexOf("'x'") + 1).isEmpty());
    EXPECT_TRUE(model.diagnostics.isEmpty());
}
TEST(LuaAnalysis, FunctionsLoopsAndRepeatScopes) {
    const QString source = "local function f(a, b)\n if a then return f(b, a) end\n return b\nend\n"
                           "for i=1,3 do print(i) end\nrepeat local done=true until done\nf(1, 2)";
    JyLuaAnalysis model(source);
    EXPECT_TRUE(model.diagnostics.isEmpty());
    EXPECT_EQ(model.definitionAt(source.indexOf("f(b")), source.indexOf("f(a"));
    EXPECT_EQ(model.definitionAt(source.indexOf("a) end")), source.indexOf("a, b"));
    EXPECT_EQ(model.definitionAt(source.indexOf("i)")), source.indexOf("i=1"));
    EXPECT_EQ(model.definitionAt(source.lastIndexOf("done")), source.indexOf("done=true"));
    int arg = -1;
    EXPECT_EQ(model.signatureAt(source.lastIndexOf('2'), &arg), "f(a, b)");
    EXPECT_EQ(arg, 1);
    EXPECT_FALSE(model.completions(source.size()).contains("done"));
    EXPECT_FALSE(model.completions(source.size()).contains("i"));
}
TEST(LuaAnalysis, NestedCallSignatureAndAnonymousFunction) {
    const QString source = "local f = function(a,b,c) return a end\nf({1,2}, print('x,y'), ";
    JyLuaAnalysis model(source);
    int arg = -1;
    EXPECT_EQ(model.signatureAt(source.size(), &arg), "f(a,b,c)");
    EXPECT_EQ(arg, 2);
    JyLuaAnalysis builtin("box.new(1, 2, ");
    EXPECT_EQ(builtin.signatureAt(builtin.source.size(), &arg), "box.new([x, y, z])");
    EXPECT_EQ(arg, 2);
}
TEST(LuaAnalysis, SyntaxCheckNeverRunsCodeAndReportsCorrectLine) {
    QTemporaryDir dir;
    const auto path = dir.filePath("must-not-exist");
    JyLuaAnalysis model(QString("io.open('%1','w'):write('bad')\nwhile true do end").arg(path));
    EXPECT_TRUE(model.diagnostics.isEmpty());
    EXPECT_FALSE(QFile::exists(path));
    JyLuaAnalysis invalid("-- 中文\nlocal x = )\n");
    ASSERT_FALSE(invalid.diagnostics.isEmpty());
    EXPECT_FALSE(invalid.diagnostics[0].warning);
    EXPECT_EQ(invalid.diagnostics[0].start, invalid.source.indexOf("local"));
    JyLuaAnalysis warning("local unused = 1");
    ASSERT_EQ(warning.diagnostics.size(), 1);
    EXPECT_TRUE(warning.diagnostics[0].warning);
    JyLuaAnalysis shebang("#!/usr/bin/env lua\nprint('ok')");
    EXPECT_TRUE(shebang.diagnostics.isEmpty());
}
TEST(LuaAnalysis, LongStringsCommentsAndFormattingPreserveLiteralBytes) {
    const QString source = "if true then\nlocal s = [==[  \n end ' \" -- [[\n  ]==]\n--[=[\nfunction fake()\nend\n]=]\nprint(s)\nelse\nprint('x')\nend\n";
    JyLuaAnalysis model(source);
    ASSERT_TRUE(model.diagnostics.isEmpty());
    const auto formatted = model.formatted();
    EXPECT_TRUE(formatted.contains("    local s = [==[  \n end ' \" -- [[\n  ]==]"));
    EXPECT_TRUE(formatted.contains("\nelse\n    print('x')\nend\n"));
    JyLuaAnalysis after(formatted);
    ASSERT_EQ(model.tokens.size(), after.tokens.size());
    for (int i = 0; i < model.tokens.size(); ++i) EXPECT_EQ(model.tokens[i].text, after.tokens[i].text);
    EXPECT_EQ(after.formatted(), formatted);
    ASSERT_FALSE(model.folds.isEmpty());
    EXPECT_EQ(model.folds[0].first, 0);
    EXPECT_EQ(model.folds[0].last, 11);
    EXPECT_TRUE(model.referencesAt(source.indexOf("fake")).isEmpty());
}
TEST(LuaAnalysis, BranchesNestedTablesAndSingleLineBlocksFormat) {
    const QString source = "local t = {\na = 1,\nb = {2, 3},\n}\nif true then\nif false then print(1) end\nelseif false then\nprint(2)\nelse\nrepeat\nprint(3)\nuntil true\nend";
    JyLuaAnalysis model(source);
    EXPECT_EQ(model.formatted(), "local t = {\n    a = 1,\n    b = {2, 3},\n}\nif true then\n    if false then print(1) end\nelseif false then\n    print(2)\nelse\n    repeat\n        print(3)\n    until true\nend");
    EXPECT_EQ(JyLuaAnalysis("if true then -- hello").indentationAfter(21), 4);
}
TEST(LuaAnalysis, IncompleteDocumentsAlwaysRecover) {
    for (const auto &source : {"function", "function a.", "local", "local x = function(", "if", "for", "while", "repeat", "a({[", "--[==[", "local s='unterminated"}) {
        JyLuaAnalysis model(QString::fromUtf8(source));
        EXPECT_FALSE(model.diagnostics.isEmpty()) << source;
    }
}
TEST(LuaEditor, PairInsertionSkippingDeletionAndWrapping) {
    JyCodeEditor editor;
    key(editor, Qt::Key_ParenLeft, "(");
    EXPECT_EQ(editor.toPlainText(), "()"); EXPECT_EQ(editor.textCursor().position(), 1);
    key(editor, Qt::Key_ParenRight, ")");
    EXPECT_EQ(editor.toPlainText(), "()"); EXPECT_EQ(editor.textCursor().position(), 2);
    at(editor, 1); key(editor, Qt::Key_Backspace);
    EXPECT_TRUE(editor.toPlainText().isEmpty());
    key(editor, Qt::Key_QuoteDbl, "\"");
    EXPECT_EQ(editor.toPlainText(), "\"\"");
    key(editor, Qt::Key_QuoteDbl, "\"");
    EXPECT_EQ(editor.toPlainText(), "\"\""); EXPECT_EQ(editor.textCursor().position(), 2);
    editor.setPlainText("value"); editor.selectAll(); key(editor, Qt::Key_Apostrophe, "'");
    EXPECT_EQ(editor.toPlainText(), "'value'"); EXPECT_EQ(editor.textCursor().selectedText(), "value");
    editor.undo(); EXPECT_EQ(editor.toPlainText(), "value");
    editor.setPlainText("-- comment "); editor.moveCursor(QTextCursor::End);
    key(editor, Qt::Key_QuoteDbl, "\""); EXPECT_EQ(editor.toPlainText(), "-- comment \"");
}
TEST(LuaEditor, EnterIndentationTabAndOutdent) {
    JyCodeEditor editor;
    editor.setPlainText("if true then"); editor.moveCursor(QTextCursor::End);
    key(editor, Qt::Key_Return, "\r"); EXPECT_EQ(editor.toPlainText(), "if true then\n    ");
    editor.setPlainText("{}"); at(editor, 1); key(editor, Qt::Key_Return, "\r");
    EXPECT_EQ(editor.toPlainText(), "{\n    \n}");
    EXPECT_EQ(editor.textCursor().position(), 6);
    editor.setPlainText("one\ntwo\nthree");
    auto cursor = editor.textCursor(); cursor.setPosition(0); cursor.setPosition(8, QTextCursor::KeepAnchor); editor.setTextCursor(cursor);
    key(editor, Qt::Key_Tab, "\t"); EXPECT_EQ(editor.toPlainText(), "    one\n    two\nthree");
    key(editor, Qt::Key_Backtab, {}, Qt::ShiftModifier); EXPECT_EQ(editor.toPlainText(), "one\ntwo\nthree");
}
TEST(LuaEditor, FoldingPreservesTextAndModifiedStateAndRevealsNavigation) {
    JyCodeEditor editor;
    const QString source = "function f()\n if true then\n  print(1)\n end\nend\nf()";
    editor.set_text(source); editor.document()->setModified(false); editor.refreshAnalysis();
    editor.toggleFold(1);
    EXPECT_FALSE(editor.document()->findBlockByNumber(2).isVisible());
    editor.toggleFold(0); editor.toggleFold(0);
    EXPECT_FALSE(editor.document()->findBlockByNumber(2).isVisible()); // inner state preserved
    EXPECT_EQ(editor.get_text(), source); EXPECT_FALSE(editor.document()->isModified());
    editor.goToPosition(source.indexOf("print"));
    EXPECT_TRUE(editor.textCursor().block().isVisible());
    editor.toggleFold(0); editor.moveCursor(QTextCursor::End); editor.insertPlainText("\nprint(2)");
    EXPECT_TRUE(editor.document()->findBlockByNumber(2).isVisible());
}
TEST(LuaEditor, FormatIsOneUndoAndKeepsCRLFAndFinalNewline) {
    JyCodeEditor editor;
    const QString source = "if true then\r\nprint('中文')\r\nend\r\n";
    editor.set_text(source); editor.formatCode();
    EXPECT_EQ(editor.get_text(), "if true then\r\n    print('中文')\r\nend\r\n");
    editor.undo(); EXPECT_EQ(editor.get_text(), source);
    editor.set_text("if true then\nprint(\nend");
    const auto invalid = editor.get_text(); editor.formatCode(); EXPECT_EQ(editor.get_text(), invalid);
}
TEST(LuaEditor, DefinitionReferencesAndProblemNavigation) {
    JyEditorWidget widget;
    auto &editor = *widget.codeEditor();
    editor.set_text("local x=1\nprint(x)\nprint(x)");
    editor.moveCursor(QTextCursor::End); at(editor, editor.toPlainText().lastIndexOf('x'));
    editor.goToDefinition(); EXPECT_EQ(editor.textCursor().position(), 6);
    editor.findReferences();
    auto *references = widget.findChild<QListWidget *>("luaReferences");
    ASSERT_NE(references, nullptr); EXPECT_EQ(references->count(), 3);
    editor.insertPlainText("!"); EXPECT_EQ(references->count(), 0);
    editor.refreshAnalysis();
    auto *problems = widget.findChild<QListWidget *>("luaProblems");
    ASSERT_NE(problems, nullptr); EXPECT_GT(problems->count(), 0);
}

TEST(LuaAnalysis, QualifiedFunctionsRespectReceiverShadowing) {
    const QString source = "local t={}\nfunction t.f(x) return x end\ndo\n local t={}\n t.f=function(y) return y end\n t.f(1)\nend\nt.f(2)";
    JyLuaAnalysis model(source);
    EXPECT_TRUE(model.diagnostics.isEmpty());
    EXPECT_EQ(model.definitionAt(source.lastIndexOf("f(2)")), source.indexOf("f(x)"));
    EXPECT_EQ(model.definitionAt(source.indexOf("f(1)")), source.indexOf("f=function"));
    EXPECT_EQ(model.signatureAt(source.indexOf("1)")), "t.f(y)");
    EXPECT_EQ(model.signatureAt(source.indexOf("2)")), "t.f(x)");
    EXPECT_EQ(model.referencesAt(source.indexOf("f(x)")).size(), 2);
}
TEST(LuaAnalysis, CadCompletionAndSignatureCatalogue) {
    JyLuaAnalysis model("local shape = box.new()\nshape:fi");
    const auto candidates = model.completions(model.source.size());
    EXPECT_TRUE(candidates.contains("shape:fillet"));
    EXPECT_TRUE(candidates.contains("shape"));
    EXPECT_TRUE(candidates.contains("sphere.new"));
    JyLuaAnalysis call("local s = box.new()\ns:prism(1, 2, ");
    int argument = 0;
    EXPECT_EQ(call.signatureAt(call.source.size(), &argument), QString::fromUtf8("JellyCAD · prism(x, y, z)"));
    EXPECT_EQ(argument, 2);
}
TEST(LuaEditor, ClosingKeywordOutdentsAndLineCommentsDoNotSuppressIndent) {
    JyCodeEditor editor;
    editor.setPlainText("if true then -- comment"); editor.moveCursor(QTextCursor::End);
    key(editor, Qt::Key_Return, "\r");
    EXPECT_EQ(editor.toPlainText(), "if true then -- comment\n    ");
    key(editor, Qt::Key_E, "e"); key(editor, Qt::Key_N, "n"); key(editor, Qt::Key_D, "d");
    EXPECT_EQ(editor.toPlainText(), "if true then -- comment\nend");
}
TEST(LuaEditor, CompletionReplacesPrefixAndCanBeDismissed) {
    JyCodeEditor editor;
    editor.resize(600, 400); editor.show(); editor.setFocus();
    editor.setPlainText("sph"); editor.moveCursor(QTextCursor::End); editor.requestCompletion();
    key(editor, Qt::Key_Tab, "\t");
    EXPECT_EQ(editor.toPlainText(), "sphere.new");
    editor.setPlainText("local s=box.new()\ns:fi"); editor.moveCursor(QTextCursor::End); editor.requestCompletion();
    key(editor, Qt::Key_Tab, "\t"); EXPECT_TRUE(editor.toPlainText().endsWith("s:fillet"));
    editor.setPlainText("-- sph"); editor.moveCursor(QTextCursor::End); editor.requestCompletion();
    key(editor, Qt::Key_Tab, "\t"); EXPECT_TRUE(editor.toPlainText().startsWith("-- sph "));
}
TEST(LuaEditor, HighlighterClosesLongCommentsAndIgnoresQuotedDelimiters) {
    JyCodeEditor editor;
    editor.setPlainText("--[=[ comment\nend\n]=]\nprint('--[[')\nlocal value=1");
    auto *highlighter = editor.document()->findChild<QSyntaxHighlighter *>();
    ASSERT_NE(highlighter, nullptr); highlighter->rehighlight();
    const auto commentLine = editor.document()->findBlockByNumber(1);
    const auto codeLine = editor.document()->findBlockByNumber(4);
    EXPECT_GT(commentLine.userState(), 0);
    EXPECT_EQ(codeLine.userState(), 0);
    ASSERT_FALSE(commentLine.layout()->formats().isEmpty());
    EXPECT_EQ(commentLine.layout()->formats().first().format.foreground().color(), QColor("#6A9955"));
    EXPECT_EQ(codeLine.layout()->formats().first().format.foreground().color(), QColor("#C586C0"));
}
TEST(LuaAnalysis, DeepAndMalformedInputDoesNotOverflowParser) {
    JyLuaAnalysis deep("local x=" + QString(1000, '(') + "1" + QString(1000, ')'));
    EXPECT_FALSE(deep.diagnostics.isEmpty());
    // Deterministic malformed input covers recovery of incomplete edits.
    const QStringList pieces = {"local ", "function ", "then ", "end ", "if ", "[", "]", "(", ")", "'x'", "--x\n", "t:", "t.", "{", "}", "=", ",", "repeat ", "for ", "until ", "x ", "1 "};
    quint32 seed = 7;
    for (int n = 0; n < 100; ++n) {
        QString text;
        for (int i = 0; i < 50; ++i) { seed = seed * 1664525u + 1013904223u; text += pieces[int(seed % pieces.size())]; }
        JyLuaAnalysis model(text);
        EXPECT_LE(model.tokens.size(), text.size());
    }
}
