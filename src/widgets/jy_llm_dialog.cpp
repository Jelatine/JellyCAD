/*
 * Copyright (c) 2024. Li Jianbin. All rights reserved.
 * MIT License
 */
#include "jy_llm_dialog.h"
#include <QFormLayout>
#include <QGroupBox>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLabel>
#include <QMessageBox>
#include <QScrollBar>
#include <QSettings>
#include <QShortcut>
#include <QTimer>
#include <QVBoxLayout>

JyLlmDialog::JyLlmDialog(QWidget *parent)
    : QDialog(parent),
      m_networkManager(new QNetworkAccessManager(this)),
      m_currentReply(nullptr),
      m_isStreaming(false),
      m_currentProviderIndex(0) {

    // Initialize providers
    m_providers = {
            {"OpenAI", "https://api.openai.com/v1/chat/completions", {"gpt-4", "gpt-4-turbo", "gpt-3.5-turbo"}},
            {"Anthropic (Claude)", "https://api.anthropic.com/v1/messages", {"claude-3-5-sonnet-20241022", "claude-3-opus-20240229", "claude-3-sonnet-20240229"}},
            {"DeepSeek", "https://api.deepseek.com/v1/chat/completions", {"deepseek-chat", "deepseek-coder"}},
            {"ModelScope", "https://api-inference.modelscope.cn/v1/chat/completions", {"Qwen/Qwen3-Coder-480B-A35B-Instruct"}},
            {"Aliyun (DashScope)", "https://dashscope.aliyuncs.com/compatible-mode/v1/chat/completions", {"qwen-turbo", "qwen-plus", "qwen-max", "qwen-long"}},
            {"Custom", "", {}}};

    setWindowTitle(tr("LLM Code Assistant"));
    resize(500, 400);

    setupUi();
    setupConnections();
    loadSettings();
}

JyLlmDialog::~JyLlmDialog() {
    if (m_currentReply) {
        m_currentReply->abort();
        m_currentReply->deleteLater();
    }
}

void JyLlmDialog::setupUi() {
    auto mainLayout = new QVBoxLayout(this);
    mainLayout->setSpacing(10);

    // Settings toggle button
    m_settingsToggleButton = new QPushButton(tr("⚙ Settings"));
    m_settingsToggleButton->setCheckable(true);
    m_settingsToggleButton->setMaximumWidth(100);
    mainLayout->addWidget(m_settingsToggleButton);

    // Settings Panel
    m_settingsPanel = new QWidget(this);
    auto settingsLayout = new QFormLayout(m_settingsPanel);

    m_providerCombo = new QComboBox;
    for (const auto &provider: m_providers) {
        m_providerCombo->addItem(provider.name);
    }
    settingsLayout->addRow(tr("Provider:"), m_providerCombo);

    m_apiKeyEdit = new QLineEdit;
    m_apiKeyEdit->setEchoMode(QLineEdit::Password);
    m_apiKeyEdit->setPlaceholderText(tr("Enter your API key"));
    settingsLayout->addRow(tr("API Key:"), m_apiKeyEdit);

    m_customUrlEdit = new QLineEdit;
    m_customUrlEdit->setPlaceholderText(tr("Enter custom API URL"));
    m_customUrlEdit->setVisible(false);
    settingsLayout->addRow(tr("Custom URL:"), m_customUrlEdit);

    m_modelCombo = new QComboBox;
    settingsLayout->addRow(tr("Model:"), m_modelCombo);

    m_settingsPanel->setVisible(false);
    mainLayout->addWidget(m_settingsPanel);

    // Prompt input
    auto promptLabel = new QLabel(tr("Enter your request (Press Ctrl+Enter to send):"));
    mainLayout->addWidget(promptLabel);

    m_promptInput = new QTextEdit;
    m_promptInput->setPlaceholderText(tr("Example: Create a function to draw a cube with side length 10"));
    m_promptInput->setMaximumHeight(120);
    mainLayout->addWidget(m_promptInput);

    // Send button
    m_sendButton = new QPushButton(tr("Send"));
    m_sendButton->setDefault(true);
    mainLayout->addWidget(m_sendButton);

    // Progress area
    m_statusLabel = new QLabel(tr("Ready"));
    m_statusLabel->setStyleSheet("color: #666;");
    mainLayout->addWidget(m_statusLabel);

    m_progressBar = new QProgressBar;
    m_progressBar->setRange(0, 0);// Indeterminate progress
    m_progressBar->setVisible(false);
    mainLayout->addWidget(m_progressBar);

    mainLayout->addStretch();

    // Update model list
    onProviderChanged(0);
}

void JyLlmDialog::setupConnections() {
    connect(m_settingsToggleButton, &QPushButton::toggled,
            this, &JyLlmDialog::onToggleSettings);
    connect(m_sendButton, &QPushButton::clicked,
            this, &JyLlmDialog::onSendClicked);
    connect(m_providerCombo, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &JyLlmDialog::onProviderChanged);

    // Ctrl+Enter to send
    auto sendShortcut = new QShortcut(QKeySequence(Qt::CTRL | Qt::Key_Return), this);
    connect(sendShortcut, &QShortcut::activated, this, &JyLlmDialog::onSendClicked);
}

void JyLlmDialog::setCurrentCode(const QString &code) {
    m_currentCode = code;
}

void JyLlmDialog::onToggleSettings() {
    m_settingsPanel->setVisible(m_settingsToggleButton->isChecked());
}

void JyLlmDialog::onProviderChanged(int index) {
    m_currentProviderIndex = index;

    // Show/hide custom URL field
    bool isCustom = (index == m_providers.size() - 1);
    m_customUrlEdit->setVisible(isCustom);

    // Update model list
    m_modelCombo->clear();
    if (index >= 0 && index < m_providers.size()) {
        const auto &provider = m_providers[index];
        if (provider.name == "Custom") {
            m_modelCombo->setEditable(true);
            m_modelCombo->setPlaceholderText(tr("Enter model name"));
        } else {
            m_modelCombo->setEditable(false);
            m_modelCombo->addItems(provider.models);
        }
    }
}

void JyLlmDialog::onSendClicked() {
    QString prompt = m_promptInput->toPlainText().trimmed();
    if (prompt.isEmpty()) {
        QMessageBox::warning(this, tr("Input Required"),
                             tr("Please enter your request."));
        return;
    }

    // Check configuration
    if (m_apiKeyEdit->text().trimmed().isEmpty()) {
        QMessageBox::warning(this, tr("Configuration Error"),
                             tr("Please enter your API key in settings."));
        m_settingsToggleButton->setChecked(true);
        return;
    }

    if (m_modelCombo->currentText().trimmed().isEmpty()) {
        QMessageBox::warning(this, tr("Configuration Error"),
                             tr("Please select or enter a model in settings."));
        m_settingsToggleButton->setChecked(true);
        return;
    }

    // Save settings
    saveSettings();

    // Send request
    sendRequest(prompt);
}

void JyLlmDialog::sendRequest(const QString &userMessage) {
    if (m_currentProviderIndex < 0 || m_currentProviderIndex >= m_providers.size()) {
        return;
    }

    const auto &provider = m_providers[m_currentProviderIndex];
    QString apiUrl = provider.apiUrl;

    // Use custom URL if custom provider
    if (provider.name == "Custom") {
        apiUrl = m_customUrlEdit->text().trimmed();
        if (apiUrl.isEmpty()) {
            QMessageBox::warning(this, tr("Configuration Error"),
                                 tr("Please enter a custom API URL."));
            return;
        }
    }

    // Build system prompt with JellyCAD API documentation
    QString apiDocumentation = R"(
JellyCAD Lua API Reference:

Basic Shapes:
- box.new(width,depth,height) - Create a box
- box.new(P1,P2) - Make a box with corners P1{dx1,dy1,dz1},P2{dx2,dy2,dz2}
- cylinder.new(radius, height) - Create a cylinder
- cone.new(radius1, radius2, height) - Create a cone
- sphere.new(radius) - Create a sphere
- face.new(polygon) - Create a face from polygon
- polygon.new(points) - Create a polygon Wire from points table
- line.new(vertex1, vertex2) - Make a line Edge from vertex1{x1,y1,z1} to vertex2{x2,y2,z2}
- circle.new(center, normal, radius) - Make a circle Edge with center{cx,cy,cz} and normal{nx,ny,nz} and radius r
- ellipse.new(center, normal, majorRadius, minorRadius) - Make an ellipse Edge with center{cx,cy,cz} and normal{nx,ny,nz} and major radius r1 and minor radius r2
- bezier.new(controlPoints) - Make a Bezier Curve Edge from control points table

Object Methods:
- :pos(x, y, z) - Set position
- :move('pos', dx, dy, dz) - Move by offset
- :rot(rx, ry, rz) - Set rotation (degrees)
- :rx(angle) / :ry(angle) / :rz(angle) - Rotate around axis
- :color(color_name) - Set color (e.g., "red", "lightblue", "#808080")

Boolean Operations:
- :fuse(other) - Union with another shape
- :cut(other) - Subtract another shape
- :common(other) - Intersection with another shape

Edge Operations:
- :fillet(radius, filter) - Round edges
- :chamfer(distance, filter) - Chamfer edges

Filter examples:
  {type='line', first={x,y,z}, last={x,y,z}, tol=1e-4}
  {type='circle'}
  {min={x,y,z}, max={x,y,z}}

Face Operations:
- :prism(dx, dy, dz) - Extrude face
- :revol(pos,dir,angle) - Revolve face around pos{px,py,pz} with dir{dx,dy,dz} and angle (degrees)

Display:
- show(shape) or show({shape1, shape2, ...}) - Display shapes

Example:
b = box.new(1, 1, 1):color('red'):pos(0, 0, 0)
c = cylinder.new(0.5, 2):color('blue'):pos(2, 0, 0)
b:fillet(0.1, {min={0,0,0.9}})
show({b, c})
-- Create and show a vase 
local profile=polygon.new({{0,0,0},{3,0,0},{4,0,2},{3.5,0,5},{4,0,8},{0,0,8}})
face.new(profile):revol({0,0,0}, {0,0,1}, 360):show()
)";

    QString systemPrompt;
    if (m_currentCode.isEmpty()) {
        systemPrompt = QString("You are a helpful Lua programming assistant for 3D CAD modeling using JellyCAD. "
                               "The user wants to create a new script. "
                               "Provide clean, well-commented Lua code. "
                               "\n\n"
                               "IMPORTANT OUTPUT RULES:\n"
                               "- Output ONLY the Lua code, nothing else\n"
                               "- DO NOT use markdown code blocks (no ```lua or ``` markers)\n"
                               "- DO NOT add explanations, descriptions, or additional text\n"
                               "- Start directly with the code\n"
                               "\n\n"
                               "%1")
                               .arg(apiDocumentation);
    } else {
        systemPrompt = QString("You are a helpful Lua programming assistant for 3D CAD modeling using JellyCAD. "
                               "The user has the following code in their editor:\n\n"
                               "```lua\n%1\n```\n\n"
                               "Help them modify or improve this code based on their request. "
                               "\n\n"
                               "IMPORTANT OUTPUT RULES:\n"
                               "- Output ONLY the complete modified Lua code, nothing else\n"
                               "- DO NOT use markdown code blocks (no ```lua or ``` markers)\n"
                               "- DO NOT add explanations, descriptions, or additional text\n"
                               "- Start directly with the code\n"
                               "\n\n"
                               "%2")
                               .arg(m_currentCode, apiDocumentation);
    }

    // Build request based on provider
    QJsonObject requestBody;

    if (provider.name == "Anthropic (Claude)") {
        // Anthropic format
        QJsonArray messages;
        QJsonObject userMsg;
        userMsg["role"] = "user";
        userMsg["content"] = userMessage;
        messages.append(userMsg);

        requestBody["model"] = m_modelCombo->currentText();
        requestBody["messages"] = messages;
        requestBody["system"] = systemPrompt;
        requestBody["max_tokens"] = 4096;
        requestBody["stream"] = true;
    } else {
        // OpenAI/DeepSeek/ModelScope/Aliyun format
        QJsonArray messages;

        QJsonObject systemMsg;
        systemMsg["role"] = "system";
        systemMsg["content"] = systemPrompt;
        messages.append(systemMsg);

        QJsonObject userMsg;
        userMsg["role"] = "user";
        userMsg["content"] = userMessage;
        messages.append(userMsg);

        requestBody["model"] = m_modelCombo->currentText();
        requestBody["messages"] = messages;
        requestBody["stream"] = true;
    }

    // Create request
    QNetworkRequest request(apiUrl);
    request.setHeader(QNetworkRequest::ContentTypeHeader, "application/json");

    // Set authentication headers
    QString apiKey = m_apiKeyEdit->text().trimmed();
    if (provider.name == "Anthropic (Claude)") {
        request.setRawHeader("x-api-key", apiKey.toUtf8());
        request.setRawHeader("anthropic-version", "2023-06-01");
    } else {
        request.setRawHeader("Authorization", QString("Bearer %1").arg(apiKey).toUtf8());
    }

    // Send request
    QJsonDocument doc(requestBody);
    if (m_currentReply) {
        m_currentReply->abort();
        m_currentReply->deleteLater();
    }

    m_currentReply = m_networkManager->post(request, doc.toJson());
    m_isStreaming = true;
    m_streamBuffer.clear();
    m_generatedCode.clear();

    // Show progress
    updateProgress(tr("Generating code..."));
    m_sendButton->setEnabled(false);
    m_promptInput->setEnabled(false);

    connect(m_currentReply, &QNetworkReply::readyRead,
            this, &JyLlmDialog::onNetworkReplyReceived);
    connect(m_currentReply, &QNetworkReply::finished,
            this, [this]() {
                m_isStreaming = false;
                m_progressBar->setVisible(false);
                m_sendButton->setEnabled(true);
                m_promptInput->setEnabled(true);

                if (m_currentReply->error() == QNetworkReply::NoError) {
                    updateProgress(tr("Code generation completed!"));
                    emit codeGenerationFinished(m_generatedCode);

                    // Auto close dialog after success
                    QTimer::singleShot(1000, this, &QDialog::accept);
                } else {
                    updateProgress(tr("Error occurred"));
                }
            });
    connect(m_currentReply, QOverload<QNetworkReply::NetworkError>::of(&QNetworkReply::errorOccurred),
            this, &JyLlmDialog::onNetworkError);
}

void JyLlmDialog::onNetworkReplyReceived() {
    if (!m_currentReply) return;

    QByteArray data = m_currentReply->readAll();
    processStreamData(data);
}

void JyLlmDialog::processStreamData(const QByteArray &data) {
    m_streamBuffer += QString::fromUtf8(data);

    // Process SSE (Server-Sent Events) format
    QStringList lines = m_streamBuffer.split("\n");
    m_streamBuffer = lines.takeLast();// Keep incomplete line in buffer

    for (const QString &line: lines) {
        if (line.startsWith("data: ")) {
            QString jsonData = line.mid(6).trimmed();

            if (jsonData == "[DONE]") {
                continue;
            }

            QJsonDocument doc = QJsonDocument::fromJson(jsonData.toUtf8());
            if (doc.isNull() || !doc.isObject()) {
                continue;
            }

            QJsonObject obj = doc.object();
            QString deltaContent;

            // Parse based on provider format
            if (m_currentProviderIndex >= 0 && m_currentProviderIndex < m_providers.size()) {
                const auto &provider = m_providers[m_currentProviderIndex];

                if (provider.name == "Anthropic (Claude)") {
                    // Anthropic streaming format
                    QString type = obj["type"].toString();
                    if (type == "content_block_delta") {
                        QJsonObject delta = obj["delta"].toObject();
                        deltaContent = delta["text"].toString();
                    }
                } else {
                    // OpenAI/DeepSeek/ModelScope/Aliyun format
                    QJsonArray choices = obj["choices"].toArray();
                    if (!choices.isEmpty()) {
                        QJsonObject choice = choices[0].toObject();
                        QJsonObject delta = choice["delta"].toObject();
                        deltaContent = delta["content"].toString();
                    }
                }
            }

            if (!deltaContent.isEmpty()) {
                m_generatedCode += deltaContent;
                emit codeStreamUpdate(deltaContent);
            }
        }
    }
}

void JyLlmDialog::onNetworkError(QNetworkReply::NetworkError error) {
    Q_UNUSED(error);

    if (m_currentReply) {
        QString errorString = m_currentReply->errorString();
        QByteArray response = m_currentReply->readAll();

        QString errorMsg = QString("Network Error: %1").arg(errorString);
        if (!response.isEmpty()) {
            errorMsg += QString("\nResponse: %1").arg(QString::fromUtf8(response));
        }

        QMessageBox::critical(this, tr("Error"), errorMsg);
        updateProgress(tr("Error: %1").arg(errorString));
    }
}

void JyLlmDialog::updateProgress(const QString &status) {
    m_statusLabel->setText(status);
    m_progressBar->setVisible(m_isStreaming);
}

void JyLlmDialog::loadSettings() {
    QSettings settings("JellyCAD", "LLM_Dialog");

    m_currentProviderIndex = settings.value("provider_index", 0).toInt();
    m_providerCombo->setCurrentIndex(m_currentProviderIndex);

    m_apiKeyEdit->setText(settings.value("api_key", "").toString());
    m_customUrlEdit->setText(settings.value("custom_url", "").toString());

    QString model = settings.value("model", "").toString();
    if (!model.isEmpty()) {
        int index = m_modelCombo->findText(model);
        if (index >= 0) {
            m_modelCombo->setCurrentIndex(index);
        } else if (m_modelCombo->isEditable()) {
            m_modelCombo->setCurrentText(model);
        }
    }
}

void JyLlmDialog::saveSettings() {
    QSettings settings("JellyCAD", "LLM_Dialog");

    settings.setValue("provider_index", m_providerCombo->currentIndex());
    settings.setValue("api_key", m_apiKeyEdit->text());
    settings.setValue("custom_url", m_customUrlEdit->text());
    settings.setValue("model", m_modelCombo->currentText());
}
