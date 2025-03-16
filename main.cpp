#include <fstream>
#include <vector>
#include <queue>
#include <limits>
#include <string>
#include <algorithm>
#include <QApplication>
#include <QMainWindow>
#include <QWidget>
#include <QPushButton>
#include <QVBoxLayout>
#include <QFormLayout>
#include <QLabel>
#include <QLineEdit>
#include <QTextEdit>
#include <QGraphicsView>
#include <QGraphicsScene>
#include <QTableWidget>
#include <QHeaderView>
#include <QFileDialog>
#include <QMessageBox>
#include <QGroupBox>
#include <QGraphicsEllipseItem>
#include <QComboBox>
#include <QPainter>
#include <QPainterPath>
#include <cmath>

class Graph {
private:
    int numVertices;
    std::vector<std::vector<int>> adjacencyMatrix;
    std::vector<std::vector<int>> adjacencyList;

public:
    explicit Graph(int n = 0) : numVertices(n) {
        adjacencyMatrix.resize(n, std::vector<int>(n, 0));
        adjacencyList.resize(n);
    }

    void resize(int n) {
        numVertices = n;
        adjacencyMatrix.resize(n, std::vector<int>(n, 0));
        adjacencyList.resize(n);
        for (auto& list : adjacencyList) {
            list.clear();
        }
    }

    void addEdge(int u, int v) {
        adjacencyMatrix[u][v] = 1;
        adjacencyMatrix[v][u] = 1;
        adjacencyList[u].push_back(v);
        adjacencyList[v].push_back(u);
    }

    // Dijkstra's algorithm for shortest path
    [[nodiscard]] auto dijkstra(int startVertex, std::vector<std::vector<std::string>>& steps) const -> std::vector<int> {
        std::vector<int> distance(numVertices, std::numeric_limits<int>::max());
        std::vector<bool> visited(numVertices, false);
        std::vector<int> prev(numVertices, -1);
        distance[startVertex] = 0;

        steps.push_back({"Инициализация",
                         std::format("Инициализация: расстояние[{}] = 0, остальные = ∞", startVertex)});

        for (int i = 0; i < numVertices; i++) {
            int minVertex = -1;
            for (int v = 0; v < numVertices; v++) {
                if (!visited[v] && (minVertex == -1 || distance[v] < distance[minVertex])) {
                    minVertex = v;
                }
            }

            if (distance[minVertex] == std::numeric_limits<int>::max()) {
                break;
            }

            visited[minVertex] = true;

            steps.push_back({"Выбор вершины",
                             std::format("Итерация {}: Выбрана вершина {} с расстоянием = {}",
                                         i + 1, minVertex, distance[minVertex])});

            for (int v = 0; v < numVertices; v++) {
                if (adjacencyMatrix[minVertex][v] > 0 && !visited[v]) {
                    int newDist = distance[minVertex] + 1;
                    if (newDist < distance[v]) {
                        distance[v] = newDist;
                        prev[v] = minVertex;

                        steps.push_back({"Релаксация",
                                         std::format("Обновление: расстояние[{}] = {} через вершину {}",
                                                     v, newDist, minVertex)});
                    }
                }
            }
        }

        return distance;
    }

    // Breadth-First Search for finding shortest paths
    [[nodiscard]] auto bfs(int startVertex, std::vector<std::vector<std::string>>& steps) const -> std::vector<int> {
        std::vector<int> distance(numVertices, std::numeric_limits<int>::max());
        std::vector<int> prev(numVertices, -1);
        std::queue<int> q;

        distance[startVertex] = 0;
        q.push(startVertex);

        steps.push_back({"Инициализация",
                         std::format("Инициализация: расстояние[{}] = 0, остальные = ∞", startVertex)});

        int iteration = 1;
        while (!q.empty()) {
            int curr = q.front();
            q.pop();

            steps.push_back({"Обработка вершины",
                             std::format("Итерация {}: Обрабатываем вершину {}", iteration++, curr)});

            for (int neighbor : adjacencyList[curr]) {
                if (distance[neighbor] == std::numeric_limits<int>::max()) {
                    distance[neighbor] = distance[curr] + 1;
                    prev[neighbor] = curr;
                    q.push(neighbor);

                    steps.push_back({"Обновление вершины",
                                     std::format("Обновление: расстояние[{}] = {} через вершину {}",
                                                 neighbor, distance[neighbor], curr)});
                }
            }
        }

        return distance;
    }

    // Bellman-Ford algorithm for finding shortest paths
    [[nodiscard]] auto bellmanFord(int startVertex, std::vector<std::vector<std::string>>& steps) const -> std::vector<int> {
        std::vector<int> distance(numVertices, std::numeric_limits<int>::max());
        std::vector<int> prev(numVertices, -1);
        distance[startVertex] = 0;

        steps.push_back({"Инициализация",
                         std::format("Инициализация: расстояние[{}] = 0, остальные = ∞", startVertex)});

        std::vector<std::pair<int, int>> edges;
        for (int u = 0; u < numVertices; ++u) {
            for (int v : adjacencyList[u]) {
                if (u < v) {
                    edges.emplace_back(u, v);
                }
            }
        }

        for (int i = 0; i < numVertices - 1; ++i) {
            steps.push_back({"Итерация",
                             std::format("Итерация {} из {}", i + 1, numVertices - 1)});

            bool anyChange = false;
            for (const auto& [u, v] : edges) {
                if (distance[u] != std::numeric_limits<int>::max() &&
                    distance[u] + 1 < distance[v]) {
                    distance[v] = distance[u] + 1;
                    prev[v] = u;
                    anyChange = true;

                    steps.push_back({"Релаксация",
                                     std::format("Релаксация: расстояние[{}] = {} через вершину {}",
                                                 v, distance[v], u)});
                }

                if (distance[v] != std::numeric_limits<int>::max() &&
                    distance[v] + 1 < distance[u]) {
                    distance[u] = distance[v] + 1;
                    prev[u] = v;
                    anyChange = true;

                    steps.push_back({"Релаксация",
                                     std::format("Релаксация: расстояние[{}] = {} через вершину {}",
                                                 u, distance[u], v)});
                }
            }

            if (!anyChange) {
                steps.push_back({"Досрочное завершение",
                                 "Больше изменений нет, алгоритм завершен досрочно"});
                break;
            }
        }

        return distance;
    }

    // Floyd-Warshall algorithm for finding all shortest paths
    [[nodiscard]] auto floydWarshall(std::vector<std::vector<std::string>>& steps) const -> std::vector<std::vector<int>> {
        std::vector<std::vector<int>> dist(numVertices,
                                           std::vector<int>(numVertices,
                                                            std::numeric_limits<int>::max()));

        for (int i = 0; i < numVertices; ++i) {
            dist[i][i] = 0;
            for (int j = 0; j < numVertices; ++j) {
                if (adjacencyMatrix[i][j] == 1) {
                    dist[i][j] = 1;
                }
            }
        }

        steps.push_back({"Инициализация", "Инициализация матрицы расстояний"});

        for (int k = 0; k < numVertices; ++k) {
            steps.push_back({"Новая промежуточная вершина",
                             std::format("Рассматриваем промежуточную вершину k = {}", k)});

            for (int i = 0; i < numVertices; ++i) {
                for (int j = 0; j < numVertices; ++j) {
                    if (dist[i][k] != std::numeric_limits<int>::max() &&
                        dist[k][j] != std::numeric_limits<int>::max() &&
                        dist[i][k] + dist[k][j] < dist[i][j]) {
                        dist[i][j] = dist[i][k] + dist[k][j];

                        steps.push_back({"Обновление расстояния",
                                         std::format("Обновление: расстояние [{}][{}] = {} через вершину {}",
                                                     i, j, dist[i][j], k)});
                    }
                }
            }
        }

        return dist;
    }

    [[nodiscard]] auto getNumVertices() const noexcept -> int {
        return numVertices;
    }

    [[nodiscard]] auto getAdjacencyMatrix() const -> const std::vector<std::vector<int>>& {
        return adjacencyMatrix;
    }
};

class GraphView : public QGraphicsView {
Q_OBJECT

public:
    explicit GraphView(QWidget *parent = nullptr) : QGraphicsView(parent) {
        scene = new QGraphicsScene(this);
        setScene(scene);
        setRenderHint(QPainter::Antialiasing);
        setDragMode(QGraphicsView::ScrollHandDrag);
        setViewportUpdateMode(QGraphicsView::FullViewportUpdate);
    }

    ~GraphView() override {
        delete scene;
    }

    void drawGraph(const std::vector<std::vector<int>> &matrix, int startVertex = -1, int endVertex = -1,
                   const std::vector<int> *distances = nullptr) {
        scene->clear();

        int numVertices = matrix.size();
        if (numVertices == 0) return;

        scene->setBackgroundBrush(QBrush(QColor("#F8FBFD")));

        const int MAX_VERTICES_TO_SHOW = 50;
        bool isLargeGraph = numVertices > MAX_VERTICES_TO_SHOW;

        if (isLargeGraph) {
            QGraphicsTextItem *warningText = scene->addText(
                    QString("Граф содержит %1 вершин. Отображается упрощенная визуализация.").arg(numVertices));
            warningText->setDefaultTextColor(QColor("#C0392B"));
            warningText->setPos(-150, -200);
            QFont warningFont = warningText->font();
            warningFont.setBold(true);
            warningText->setFont(warningFont);
        }

        int verticesToShow = isLargeGraph ? MAX_VERTICES_TO_SHOW : numVertices;

        std::vector<QPointF> positions;
        double radius = isLargeGraph ? 250.0 : 150.0;
        double centerX = 0;
        double centerY = 0;

        for (int i = 0; i < verticesToShow; ++i) {
            double angle = 2.0 * M_PI * i / verticesToShow;
            double x = centerX + radius * cos(angle);
            double y = centerY + radius * sin(angle);
            positions.push_back(QPointF(x, y));
        }

        for (int i = 0; i < verticesToShow; ++i) {
            for (int j = i + 1; j < verticesToShow; ++j) {
                if (matrix[i][j] == 1) {
                    QGraphicsLineItem *line = scene->addLine(positions[i].x(), positions[i].y(),
                                                             positions[j].x(), positions[j].y(),
                                                             QPen(QColor("#85C1E9"), 1.5));

                    if (distances && startVertex != -1 && endVertex != -1 &&
                        ((i == startVertex && j == endVertex) ||
                         (j == startVertex && i == endVertex))) {
                        line->setPen(QPen(QColor("#1A5276"), 2.5));
                    }
                }
            }
        }

        double vertexRadius = isLargeGraph ? 15.0 : 20.0;
        for (int i = 0; i < verticesToShow; ++i) {
            QBrush brush(QColor("#D6EAF8"));
            QPen pen(QColor("#3498DB"), 1.5);

            if (i == startVertex) {
                brush = QBrush(QColor("#2980B9"));
                pen = QPen(QColor("#1A5276"), 2);
            } else if (i == endVertex) {
                brush = QBrush(QColor("#3498DB"));
                pen = QPen(QColor("#1A5276"), 2);
            }

            QGraphicsEllipseItem *vertex = scene->addEllipse(
                    positions[i].x() - vertexRadius,
                    positions[i].y() - vertexRadius,
                    2 * vertexRadius, 2 * vertexRadius,
                    pen, brush);

            QGraphicsTextItem *label = scene->addText(QString::number(i));

            QFont font = label->font();
            font.setBold(true);
            if (isLargeGraph) font.setPointSize(font.pointSize() - 1);
            label->setFont(font);

            if (i == startVertex || i == endVertex) {
                label->setDefaultTextColor(QColor("#FFFFFF"));
            } else {
                label->setDefaultTextColor(QColor("#1A5276"));
            }

            QRectF textRect = label->boundingRect();
            label->setPos(positions[i].x() - textRect.width() / 2, positions[i].y() - textRect.height() / 2);

            if (distances && i != startVertex && (*distances)[i] != std::numeric_limits<int>::max() &&
                i < verticesToShow) {
                QGraphicsTextItem *distLabel = scene->addText("d=" + QString::number((*distances)[i]));

                QFont distFont = distLabel->font();
                distFont.setBold(true);
                if (isLargeGraph) distFont.setPointSize(distFont.pointSize() - 1);
                distLabel->setFont(distFont);
                distLabel->setDefaultTextColor(QColor("#1A5276"));

                distLabel->setPos(positions[i].x() + vertexRadius, positions[i].y() + vertexRadius);

                QRectF bgRect = distLabel->boundingRect().adjusted(-2, -2, 2, 2);
                QGraphicsRectItem *bg = scene->addRect(
                        bgRect.translated(positions[i].x() + vertexRadius, positions[i].y() + vertexRadius),
                        QPen(Qt::NoPen),
                        QBrush(QColor("#D6EAF8"))
                );
                bg->setZValue(distLabel->zValue() - 1);
            }
        }

        if (isLargeGraph) {
            QString hiddenInfo = QString("Отображено %1 из %2 вершин. Вычисления выполняются для всего графа.").arg(
                    verticesToShow).arg(numVertices);
            QGraphicsTextItem *infoText = scene->addText(hiddenInfo);
            infoText->setDefaultTextColor(QColor("#2874A6"));
            infoText->setPos(-150, 220);
        }

        scene->setSceneRect(scene->itemsBoundingRect().adjusted(-50, -50, 50, 50));
        fitInView(scene->sceneRect(), Qt::KeepAspectRatio);
    }

    void resizeEvent(QResizeEvent *event) {
        QGraphicsView::resizeEvent(event);
        fitInView(scene->sceneRect(), Qt::KeepAspectRatio);
    }

private:
    QGraphicsScene *scene;
};

class MainWindow : public QMainWindow {
Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr) : QMainWindow(parent), startVertex(0), endVertex(0) {
        setupUI();
    }

    ~MainWindow() {}

private slots:

    void loadGraphFromFile() {
        QString filename = QFileDialog::getOpenFileName(this, "Открыть файл графа", "", "Текстовые файлы (*.txt)");
        if (filename.isEmpty()) {
            return;
        }

        std::ifstream file(filename.toStdString());
        if (!file) {
            QMessageBox::critical(this, "Ошибка", "Не удалось открыть файл: " + filename);
            return;
        }

        try {
            int numVertices, numEdges;
            file >> numVertices;

            if (numVertices <= 0 || numVertices > 1000) {
                QMessageBox::critical(this, "Ошибка",
                                      "Некорректное количество вершин: " + QString::number(numVertices));
                file.close();
                return;
            }

            file >> numEdges;

            if (numEdges < 0 || numEdges > numVertices * numVertices) {
                QMessageBox::critical(this, "Ошибка", "Некорректное количество рёбер: " + QString::number(numEdges));
                file.close();
                return;
            }

            graph.resize(numVertices);

            for (int i = 0; i < numEdges; ++i) {
                int u, v;
                if (!(file >> u >> v)) {
                    QMessageBox::critical(this, "Ошибка", "Ошибка чтения ребра #" + QString::number(i + 1));
                    file.close();
                    return;
                }

                if (u < 0 || u >= numVertices || v < 0 || v >= numVertices) {
                    QMessageBox::critical(this, "Ошибка",
                                          QString("Некорректные индексы вершин ребра #%1: %2 - %3").arg(i + 1).arg(
                                                  u).arg(v));
                    file.close();
                    return;
                }

                graph.addEdge(u, v);
            }

            int startVert = 0;
            if (file >> startVert) {
                if (startVert < 0 || startVert >= numVertices) {
                    QMessageBox::warning(this, "Предупреждение",
                                         QString("Указанная начальная вершина %1 некорректна, используется вершина 0").arg(
                                                 startVert));
                    startVert = 0;
                }
            }

            file.close();

            bool isLargeGraph = numVertices > 100 || numEdges > 500;
            if (isLargeGraph) {
                QMessageBox::information(this, "Информация",
                                         QString("Загружен большой граф: %1 вершин, %2 рёбер.\nВизуализация будет упрощена.").arg(
                                                 numVertices).arg(numEdges));
            }

            updateAdjacencyMatrix();
            graphView->drawGraph(graph.getAdjacencyMatrix());

            startVertexEdit->setText(QString::number(startVert));
            if (numVertices > 1) {
                endVertexEdit->setText(startVert == 0 ? "1" : "0");
            } else {
                endVertexEdit->setText("0");
            }
            updateVertexInputValidators();

            QMessageBox::information(this, "Успех", "Граф успешно загружен. Вершин: " +
                                                    QString::number(numVertices) + ", Рёбер: " +
                                                    QString::number(numEdges));
        }
        catch (const std::exception &e) {
            QMessageBox::critical(this, "Ошибка", "Произошла ошибка при чтении файла: " + QString(e.what()));
            file.close();
        }
    }

    void updateVertexInputValidators() {
        int maxVertex = graph.getNumVertices() - 1;
        if (maxVertex >= 0) {
            auto *validator = new QIntValidator(0, maxVertex, this);
            startVertexEdit->setValidator(validator);
            endVertexEdit->setValidator(validator);

            QString placeholderText = QString("Введите число от 0 до %1").arg(maxVertex);
            startVertexEdit->setPlaceholderText(placeholderText);
            endVertexEdit->setPlaceholderText(placeholderText);

            startVertexEdit->setStyleSheet(
                    "background-color: #F8FBFD; border: 1px solid #AED6F1; border-radius: 4px; padding: 4px;");
            endVertexEdit->setStyleSheet(
                    "background-color: #F8FBFD; border: 1px solid #AED6F1; border-radius: 4px; padding: 4px;");
        }
    }

    void updateStartVertex(const QString &text) {
        bool ok;
        int vertex = text.toInt(&ok);

        if (ok && vertex >= 0 && vertex < graph.getNumVertices()) {
            startVertex = vertex;
            updateGraphView();
            startVertexEdit->setStyleSheet(
                    "background-color: #F8FBFD; border: 1px solid #AED6F1; border-radius: 4px; padding: 4px;");
        } else {
            startVertexEdit->setStyleSheet(
                    "background-color: #FDEDEC; border: 1px solid #E74C3C; border-radius: 4px; padding: 4px;");
        }
    }

    void updateEndVertex(const QString &text) {
        bool ok;
        int vertex = text.toInt(&ok);

        if (ok && vertex >= 0 && vertex < graph.getNumVertices()) {
            endVertex = vertex;
            updateGraphView();
            endVertexEdit->setStyleSheet(
                    "background-color: #F8FBFD; border: 1px solid #AED6F1; border-radius: 4px; padding: 4px;");
        } else {
            endVertexEdit->setStyleSheet(
                    "background-color: #FDEDEC; border: 1px solid #E74C3C; border-radius: 4px; padding: 4px;");
        }
    }

    void updateGraphView() {
        graphView->drawGraph(graph.getAdjacencyMatrix(), startVertex, endVertex);
    }

    void calculateShortestPath() {
        if (graph.getNumVertices() == 0) {
            QMessageBox::warning(this, "Предупреждение", "Пожалуйста, сначала загрузите граф.");
            return;
        }

        bool startOk, endOk;
        int start = startVertexEdit->text().toInt(&startOk);
        int end = endVertexEdit->text().toInt(&endOk);

        if (!startOk || start < 0 || start >= graph.getNumVertices()) {
            QMessageBox::warning(this, "Ошибка", "Некорректно указана начальная вершина. Должно быть число от 0 до " +
                                                 QString::number(graph.getNumVertices() - 1));
            startVertexEdit->setFocus();
            return;
        }

        if (!endOk || end < 0 || end >= graph.getNumVertices()) {
            QMessageBox::warning(this, "Ошибка", "Некорректно указана конечная вершина. Должно быть число от 0 до " +
                                                 QString::number(graph.getNumVertices() - 1));
            endVertexEdit->setFocus();
            return;
        }

        QString algorithm = algorithmComboBox->currentText();
        std::vector<std::vector<std::string>> steps;

        if (algorithm == "Алгоритм Дейкстры") {
            std::vector<int> distances = graph.dijkstra(startVertex, steps);
            showResults(distances, steps, algorithm);
        } else if (algorithm == "Поиск в ширину (BFS)") {
            std::vector<int> distances = graph.bfs(startVertex, steps);
            showResults(distances, steps, algorithm);
        } else if (algorithm == "Алгоритм Беллмана-Форда") {
            std::vector<int> distances = graph.bellmanFord(startVertex, steps);
            showResults(distances, steps, algorithm);
        } else if (algorithm == "Алгоритм Флойда-Уоршелла") {
            std::vector<std::vector<int>> allDistances = graph.floydWarshall(steps);
            showFloydWarshallResults(allDistances, steps);
        }
    }

    void showAlgorithmInfo() {
        QString algorithm = algorithmComboBox->currentText();
        infoTextEdit->setHtml(getAlgorithmInfo(algorithm));
    }

private:
    void setupUI() {
        auto *centralWidget = new QWidget(this);
        auto *mainLayout = new QVBoxLayout(centralWidget);

        centralWidget->setStyleSheet("background-color: #EBF5FB;");

        auto *topLayout = new QHBoxLayout();

        auto *fileBox = new QGroupBox("Загрузка файла", centralWidget);
        fileBox->setStyleSheet(
                "QGroupBox { background-color: #D6EAF8; border-radius: 8px; border: 1px solid #AED6F1; padding: 10px; }");
        auto *fileLayout = new QVBoxLayout(fileBox);
        loadFileButton = new QPushButton("Загрузить файл графа", fileBox);
        loadFileButton->setStyleSheet(
                "QPushButton { background-color: #3498DB; color: white; border-radius: 4px; padding: 8px; font-weight: bold; }"
                "QPushButton:hover { background-color: #2980B9; }");
        fileLayout->addWidget(loadFileButton);
        topLayout->addWidget(fileBox);

        auto *matrixBox = new QGroupBox("Таблица смежности", centralWidget);
        matrixBox->setStyleSheet(
                "QGroupBox { background-color: #D6EAF8; border-radius: 8px; border: 1px solid #AED6F1; padding: 10px; }");
        auto *matrixLayout = new QVBoxLayout(matrixBox);
        adjacencyMatrixTable = new QTableWidget(0, 0, matrixBox);
        adjacencyMatrixTable->setStyleSheet("QTableWidget { background-color: #F8FBFD; border: 1px solid #AED6F1; }"
                                            "QHeaderView::section { background-color: #3498DB; color: white; padding: 4px; }");
        adjacencyMatrixTable->setEditTriggers(QAbstractItemView::NoEditTriggers);
        matrixLayout->addWidget(adjacencyMatrixTable);
        topLayout->addWidget(matrixBox);

        mainLayout->addLayout(topLayout);

        auto *middleLayout = new QHBoxLayout();

        auto *vertexBox = new QGroupBox("Выбор вершин", centralWidget);
        vertexBox->setStyleSheet(
                "QGroupBox { background-color: #D6EAF8; border-radius: 8px; border: 1px solid #AED6F1; padding: 10px; }"
                "QLineEdit { background-color: #F8FBFD; border: 1px solid #AED6F1; border-radius: 4px; padding: 4px; }");
        auto *vertexLayout = new QFormLayout(vertexBox);
        startVertexEdit = new QLineEdit(vertexBox);
        endVertexEdit = new QLineEdit(vertexBox);
        vertexLayout->addRow("Начальная вершина:", startVertexEdit);
        vertexLayout->addRow("Конечная вершина:", endVertexEdit);
        middleLayout->addWidget(vertexBox);

        auto *algoBox = new QGroupBox("Выбор алгоритма", centralWidget);
        algoBox->setStyleSheet(
                "QGroupBox { background-color: #D6EAF8; border-radius: 8px; border: 1px solid #AED6F1; padding: 10px; }"
                "QComboBox { background-color: #F8FBFD; border: 1px solid #AED6F1; border-radius: 4px; padding: 4px; }");
        auto *algoLayout = new QVBoxLayout(algoBox);
        algorithmComboBox = new QComboBox(algoBox);
        algorithmComboBox->addItem("Алгоритм Дейкстры");
        algorithmComboBox->addItem("Поиск в ширину (BFS)");
        algorithmComboBox->addItem("Алгоритм Беллмана-Форда");
        algorithmComboBox->addItem("Алгоритм Флойда-Уоршелла");
        calculateButton = new QPushButton("Найти кратчайший путь", algoBox);
        calculateButton->setStyleSheet(
                "QPushButton { background-color: #3498DB; color: white; border-radius: 4px; padding: 8px; font-weight: bold; }"
                "QPushButton:hover { background-color: #2980B9; }");
        algoLayout->addWidget(algorithmComboBox);
        algoLayout->addWidget(calculateButton);
        middleLayout->addWidget(algoBox);

        mainLayout->addLayout(middleLayout);

        auto *bottomLayout = new QHBoxLayout();

        auto *infoBox = new QGroupBox("Информация об алгоритме", centralWidget);
        infoBox->setStyleSheet(
                "QGroupBox { background-color: #D6EAF8; border-radius: 8px; border: 1px solid #AED6F1; padding: 10px; }");
        auto *infoLayout = new QVBoxLayout(infoBox);
        infoTextEdit = new QTextEdit(infoBox);
        infoTextEdit->setReadOnly(true);
        infoTextEdit->setStyleSheet("QTextEdit { background-color: #F8FBFD; border: 1px solid #AED6F1; }");
        infoLayout->addWidget(infoTextEdit);
        bottomLayout->addWidget(infoBox);

        auto *graphBox = new QGroupBox("Визуализация графа", centralWidget);
        graphBox->setStyleSheet(
                "QGroupBox { background-color: #D6EAF8; border-radius: 8px; border: 1px solid #AED6F1; padding: 10px; }");
        auto *graphLayout = new QVBoxLayout(graphBox);
        graphView = new GraphView(graphBox);
        graphView->setStyleSheet("background-color: #F8FBFD; border: 1px solid #AED6F1;");
        graphLayout->addWidget(graphView);
        bottomLayout->addWidget(graphBox);

        mainLayout->addLayout(bottomLayout);

        setCentralWidget(centralWidget);

        setWindowTitle("Поиск кратчайшего пути в графе");
        resize(1000, 700);

        connect(loadFileButton, &QPushButton::clicked, this, &MainWindow::loadGraphFromFile);
        connect(calculateButton, &QPushButton::clicked, this, &MainWindow::calculateShortestPath);
        connect(algorithmComboBox, &QComboBox::currentTextChanged, this, &MainWindow::showAlgorithmInfo);
        connect(startVertexEdit, &QLineEdit::textChanged, this, &MainWindow::updateStartVertex);
        connect(endVertexEdit, &QLineEdit::textChanged, this, &MainWindow::updateEndVertex);

        showAlgorithmInfo();
    }

    void updateAdjacencyMatrix() {
        const std::vector<std::vector<int>> &matrix = graph.getAdjacencyMatrix();
        int numVertices = graph.getNumVertices();

        const int MAX_MATRIX_SIZE = 30;
        bool isLargeMatrix = numVertices > MAX_MATRIX_SIZE;
        int displaySize = isLargeMatrix ? MAX_MATRIX_SIZE : numVertices;

        adjacencyMatrixTable->setRowCount(displaySize);
        adjacencyMatrixTable->setColumnCount(displaySize);

        QStringList headers;
        for (int i = 0; i < displaySize; ++i) {
            headers << QString::number(i);
        }
        adjacencyMatrixTable->setHorizontalHeaderLabels(headers);
        adjacencyMatrixTable->setVerticalHeaderLabels(headers);

        for (int i = 0; i < displaySize; ++i) {
            for (int j = 0; j < displaySize; ++j) {
                auto *item = new QTableWidgetItem(QString::number(matrix[i][j]));
                item->setTextAlignment(Qt::AlignCenter);

                if (matrix[i][j] == 1) {
                    item->setBackground(QBrush(QColor("#AED6F1")));
                } else {
                    item->setBackground(QBrush(QColor("#E8F1F8")));
                }

                adjacencyMatrixTable->setItem(i, j, item);
            }
        }

        if (isLargeMatrix) {
            auto *infoLabel = new QLabel(
                    QString("Отображены первые %1 из %2 вершин.").arg(displaySize).arg(numVertices));
            infoLabel->setStyleSheet("color: #2874A6; font-weight: bold; margin-top: 5px;");

            QLayout *matrixLayout = adjacencyMatrixTable->parentWidget()->layout();
            if (matrixLayout) {
                dynamic_cast<QVBoxLayout *>(matrixLayout)->addWidget(infoLabel);
            }
        }

        adjacencyMatrixTable->resizeColumnsToContents();

        adjacencyMatrixTable->setEditTriggers(QAbstractItemView::NoEditTriggers);
    }

    void showResults(const std::vector<int> &distances, const std::vector<std::vector<std::string>> &steps,
                     const QString &algorithm) {
        QString resultHtml = "<h2>" + algorithm + "</h2>";
        resultHtml += "<h3>Теоретическая информация:</h3>";
        resultHtml += getAlgorithmInfo(algorithm);

        resultHtml += "<h3>Шаги выполнения алгоритма:</h3>";
        resultHtml += "<table border='1' cellspacing='0' cellpadding='4'>";
        resultHtml += "<tr><th>Шаг</th><th>Описание</th></tr>";

        for (const auto &step: steps) {
            resultHtml += "<tr><td>" + QString::fromStdString(step[0]) + "</td><td>" +
                          QString::fromStdString(step[1]) + "</td></tr>";
        }

        resultHtml += "</table>";

        resultHtml += "<h3>Результаты:</h3>";
        resultHtml += "<p>Кратчайшие расстояния от вершины " + QString::number(startVertex) + ":</p>";
        resultHtml += "<table border='1' cellspacing='0' cellpadding='4'>";
        resultHtml += "<tr><th>Вершина</th><th>Расстояние</th></tr>";

        for (int i = 0; i < distances.size(); ++i) {
            resultHtml += "<tr><td>" + QString::number(i) + "</td><td>";
            if (distances[i] == std::numeric_limits<int>::max()) {
                resultHtml += "Недостижима";
            } else {
                resultHtml += QString::number(distances[i]);
            }
            resultHtml += "</td></tr>";
        }

        resultHtml += "</table>";

        infoTextEdit->setHtml(resultHtml);

        graphView->drawGraph(graph.getAdjacencyMatrix(), startVertex, endVertex, &distances);
    }

    void showFloydWarshallResults(const std::vector<std::vector<int>> &distances,
                                  const std::vector<std::vector<std::string>> &steps) {
        QString resultHtml = "<h2>Алгоритм Флойда-Уоршелла</h2>";
        resultHtml += "<h3>Теоретическая информация:</h3>";
        resultHtml += getAlgorithmInfo("Алгоритм Флойда-Уоршелла");

        resultHtml += "<h3>Шаги выполнения алгоритма:</h3>";
        resultHtml += "<table border='1' cellspacing='0' cellpadding='4'>";
        resultHtml += "<tr><th>Шаг</th><th>Описание</th></tr>";

        for (const auto &step: steps) {
            resultHtml += "<tr><td>" + QString::fromStdString(step[0]) + "</td><td>" +
                          QString::fromStdString(step[1]) + "</td></tr>";
        }

        resultHtml += "</table>";

        resultHtml += "<h3>Результаты:</h3>";
        resultHtml += "<p>Матрица кратчайших расстояний между всеми парами вершин:</p>";
        resultHtml += "<table border='1' cellspacing='0' cellpadding='4'>";

        resultHtml += "<tr><th></th>";
        for (int i = 0; i < distances.size(); ++i) {
            resultHtml += "<th>" + QString::number(i) + "</th>";
        }
        resultHtml += "</tr>";

        for (int i = 0; i < distances.size(); ++i) {
            resultHtml += "<tr><th>" + QString::number(i) + "</th>";
            for (int j = 0; j < distances[i].size(); ++j) {
                resultHtml += "<td>";
                if (distances[i][j] == std::numeric_limits<int>::max()) {
                    resultHtml += "∞";
                } else {
                    resultHtml += QString::number(distances[i][j]);
                }
                resultHtml += "</td>";
            }
            resultHtml += "</tr>";
        }

        resultHtml += "</table>";

        resultHtml += "<p>Кратчайшее расстояние от вершины " + QString::number(startVertex) +
                      " до вершины " + QString::number(endVertex) + " равно ";
        if (distances[startVertex][endVertex] == std::numeric_limits<int>::max()) {
            resultHtml += "∞ (недостижима)";
        } else {
            resultHtml += QString::number(distances[startVertex][endVertex]);
        }
        resultHtml += "</p>";

        infoTextEdit->setHtml(resultHtml);

        const std::vector<int>& singleSourceDistances = distances[startVertex];
        graphView->drawGraph(graph.getAdjacencyMatrix(), startVertex, endVertex, &singleSourceDistances);
    }

    static QString getAlgorithmInfo(const QString &algorithm) {
        if (algorithm == "Алгоритм Дейкстры") {
            return "<p><b>Алгоритм Дейкстры</b> - это алгоритм поиска кратчайшего пути от одной "
                   "вершины графа до всех остальных. Работает только для графов с неотрицательными весами рёбер.</p>"
                   "<p><b>Основные свойства:</b></p>"
                   "<ul>"
                   "<li>Временная сложность: O((V + E) log V) с использованием бинарной кучи</li>"
                   "<li>Пространственная сложность: O(V)</li>"
                   "<li>Не работает с отрицательными весами рёбер</li>"
                   "<li>Гарантирует нахождение оптимального решения</li>"
                   "</ul>"
                   "<p><b>Лемма:</b> Если для всех вершин v ∈ V, кратчайшие пути от источника s до v "
                   "содержат не более k рёбер, то после k итераций алгоритма Дейкстры значение d[v] "
                   "будет равно длине кратчайшего пути от s до v.</p>"
                   "<p><b>Доказательство:</b> На каждой итерации алгоритм выбирает вершину с минимальным "
                   "значением d[v] и фиксирует её. После этого обновляются расстояния до соседних "
                   "вершин. Таким образом, на k-й итерации будут найдены все кратчайшие пути, содержащие "
                   "ровно k рёбер.</p>";
        } else if (algorithm == "Поиск в ширину (BFS)") {
            return "<p><b>Поиск в ширину (BFS)</b> - это алгоритм обхода графа. В случае, когда все "
                   "рёбра имеют одинаковый вес (или вес равен 1), BFS может быть использован для "
                   "нахождения кратчайших путей от исходной вершины до всех остальных.</p>"
                   "<p><b>Основные свойства:</b></p>"
                   "<ul>"
                   "<li>Временная сложность: O(V + E)</li>"
                   "<li>Пространственная сложность: O(V)</li>"
                   "<li>Работает на невзвешенных графах или графах с одинаковыми весами рёбер</li>"
                   "<li>Гарантирует нахождение кратчайшего пути в невзвешенных графах</li>"
                   "</ul>"
                   "<p><b>Теорема:</b> BFS находит кратчайший путь в невзвешенном графе.</p>"
                   "<p><b>Доказательство:</b> BFS обрабатывает вершины по уровням, начиная от исходной "
                   "вершины. Вершины на уровне i находятся на расстоянии i от исходной вершины. BFS "
                   "гарантирует, что вершина будет обработана на минимально возможном уровне.</p>";
        } else if (algorithm == "Алгоритм Беллмана-Форда") {
            return "<p><b>Алгоритм Беллмана-Форда</b> - это алгоритм поиска кратчайших путей от одной "
                   "вершины до всех остальных во взвешенном графе. В отличие от алгоритма Дейкстры, "
                   "он может работать с графами, содержащими рёбра с отрицательными весами (но не с "
                   "отрицательными циклами).</p>"
                   "<p><b>Основные свойства:</b></p>"
                   "<ul>"
                   "<li>Временная сложность: O(V * E)</li>"
                   "<li>Пространственная сложность: O(V)</li>"
                   "<li>Может работать с отрицательными весами рёбер (но не с отрицательными циклами)</li>"
                   "<li>Может обнаруживать отрицательные циклы</li>"
                   "<li>Обычно медленнее алгоритма Дейкстры для графов с неотрицательными весами</li>"
                   "</ul>"
                   "<p><b>Теорема:</b> Если в графе нет отрицательных циклов, достижимых из исходной "
                   "вершины, то после |V|-1 итераций алгоритма Беллмана-Форда значения d[v] будут "
                   "равны длинам кратчайших путей от исходной вершины до всех других вершин.</p>"
                   "<p><b>Доказательство:</b> Кратчайший простой путь в графе содержит не более |V|-1 "
                   "рёбер. На каждой итерации алгоритм выполняет релаксацию всех рёбер, что "
                   "гарантирует нахождение всех кратчайших путей длиной до i рёбер после i итераций.</p>";
        } else if (algorithm == "Алгоритм Флойда-Уоршелла") {
            return "<p><b>Алгоритм Флойда-Уоршелла</b> - это алгоритм нахождения кратчайших путей между "
                   "всеми парами вершин во взвешенном графе. Он может работать с отрицательными "
                   "весами рёбер, но не с отрицательными циклами.</p>"
                   "<p><b>Основные свойства:</b></p>"
                   "<ul>"
                   "<li>Временная сложность: O(V^3)</li>"
                   "<li>Пространственная сложность: O(V^2)</li>"
                   "<li>Может работать с отрицательными весами рёбер (но не с отрицательными циклами)</li>"
                   "<li>Находит кратчайшие пути между всеми парами вершин за одно выполнение</li>"
                   "<li>Использует метод динамического программирования</li>"
                   "</ul>"
                   "<p><b>Принцип работы:</b> Алгоритм основан на том, что если существует более короткий "
                   "путь из i в j через вершину k, то мы обновляем текущую оценку. Для каждой пары "
                   "вершин (i, j) мы проверяем, улучшится ли путь, если мы пройдём через вершину k.</p>"
                   "<p><b>Доказательство корректности:</b> Пусть dist[i][j][k] - длина кратчайшего пути из i "
                   "в j, который проходит только через вершины {0, 1, ..., k-1} в качестве промежуточных. "
                   "Тогда для новой вершины k мы имеем:</p>"
                   "<p>dist[i][j][k+1] = min(dist[i][j][k], dist[i][k][k] + dist[k][j][k])</p>"
                   "<p>Поскольку мы перебираем все возможные промежуточные вершины, алгоритм гарантирует "
                   "нахождение кратчайших путей между всеми парами вершин.</p>";
        }

        return "Выберите алгоритм, чтобы увидеть информацию.";
    }

    Graph graph;
    int startVertex;
    int endVertex;

    QPushButton *loadFileButton;
    QTableWidget *adjacencyMatrixTable;
    QLineEdit *startVertexEdit;
    QLineEdit *endVertexEdit;
    QComboBox *algorithmComboBox;
    QPushButton *calculateButton;
    QTextEdit *infoTextEdit;
    GraphView *graphView;
};

#include "main.moc"

int main(int argc, char *argv[]) {
    QApplication app(argc, argv);
    MainWindow window;
    window.show();
    return QApplication::exec();
}