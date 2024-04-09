#include <iostream>
#include <vector>
#include <algorithm>
#include <unordered_set>
#include <chrono>
#include <queue>

using namespace std;

class Graph {
public:
    int vertices;
    vector < vector < int>> adjacencyMatrix;

    Graph(int v) : vertices(v), adjacencyMatrix(v + 1, vector<int>(v + 1, 0)) {}

    void addEdge(int v, int w) {
        adjacencyMatrix[v][w] = 1;
        adjacencyMatrix[w][v] = 1;
    }

    void printPath(const vector<int>& path) {
        cout << "Гамильтонов путь: ";
        for (int vertex : path) {
            cout << vertex << " ";
        }
        cout << endl;
    }

    bool isSafe(int v, const vector<int>& path, int pos, const unordered_set<int>& visited) {
        if (adjacencyMatrix[path[pos - 1]][v] == 0 || visited.count(v) > 0)
            return false;

        return true;
    }

    bool hamiltonianPathUtil(vector<int>& path, int pos, const unordered_set<int>& visited, int endVertex) {
        if (pos == vertices) {
            if (endVertex == -1 || path[pos - 1] == endVertex) {
                printPath(path);
                return true;
            }
            else {
                return false;
            }
        }

        for (int v = 1; v <= vertices; ++v) {
            if (isSafe(v, path, pos, visited)) {
                path[pos] = v;
                unordered_set<int> updatedVisited = visited;
                updatedVisited.insert(v);
                if (hamiltonianPathUtil(path, pos + 1, updatedVisited, endVertex))
                    return true;
                path[pos] = -1; // backtrack
            }
        }

        return false;
    }

    void findHamiltonianPath(int startVertex, int endVertex) {
        if (startVertex < 1 || startVertex > vertices || (endVertex != -1 && (endVertex < 1 || endVertex > vertices))) {
            cout << "Введены некорректные вершины." << endl;
            return;
        }

        vector<int> path(vertices, -1);
        unordered_set<int> visited;
        path[0] = startVertex;
        visited.insert(startVertex);

        if (!hamiltonianPathUtil(path, 1, visited, endVertex)) {
            cout << "Гамильтонов путь не найден." << endl;
        }
    }

    void findShortestPath(vector<vector<int>>& graph, int start, int end) {
        int n = graph.size();
        vector<int> dist(n, numeric_limits<int>::max()); // Вектор для хранения расстояний до вершин
        vector<int> path(n, -1); // Вектор для хранения предыдущих вершин

        priority_queue<pair<int, int>> pq; // Двоичная куча для выбора вершины с минимальным расстоянием
        pq.push({ 0, start }); // Начальное расстояние до начальной вершины равно 0

        dist[start] = 0; // Расстояние от начальной вершины до самой себя равно 0

        while (!pq.empty()) {
            int u = pq.top().second;
            int d = -pq.top().first;
            pq.pop();

            // Если текущее расстояние больше, чем уже найденное
            if (d > dist[u]) {
                continue;
            }

            // Поиск кратчайшего пути до конечной вершины
            if (u == end) {
                break;
            }

            // Обновление расстояний до смежных вершин
            for (int v = 0; v < n; ++v) {
                if (graph[u][v] != 0 && dist[u] + graph[u][v] < dist[v]) {
                    dist[v] = dist[u] + graph[u][v];
                    pq.push({ -dist[v], v });
                    path[v] = u;
                }
            }
        }

        // Вывод кратчайшего пути
        vector<int> shortestPath;
        int vertex = end;
        while (vertex != -1) {
            shortestPath.push_back(vertex);
            vertex = path[vertex];
        }

        if (shortestPath.size() == 0) {
            cout << "Пути между заданными вершинами не существует" << endl;
        }
        else {
            cout << "Кратчайший путь: ";
            for (int i = shortestPath.size() - 1; i >= 0; i--) {
                if (i == 0 || i == shortestPath.size() - 1)
                    cout << shortestPath[i] + 1;
                else
                    cout << shortestPath[i] + 1;
                if (i != 0)
                    cout << "-";
            }
            cout << endl;
        }
    }

};

int main() {
    setlocale(LC_ALL, "Russian");
    int vertices;
    cout << "Введите количество вершин: ";
    cin >> vertices;

    Graph graph(vertices);

    // Пример матрицы смежности
    vector < vector < int>> exampleMatrix = {
    {0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0}, //1
    {1, 0, 1, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0}, //2
    {0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0}, //3
    {0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0}, //4
    {0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 1}, //5
    {0, 0, 0, 0, 1, 0, 1, 1, 0, 0, 0, 0, 1}, //6
    {0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0}, //7
    {0, 0, 0, 0, 0, 1, 1, 0, 1, 0, 0, 0, 0}, //8
    {0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 1}, //9
    {1, 1, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0}, //10
    {1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0}, //11
    {0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1}, //12
    {0, 0, 0, 0, 1, 1, 0, 0, 1, 0, 0, 1, 0}, //13
    };


    // Копирование примера в матрицу смежности графа
    for (int i = 1; i <= vertices; ++i) {
        for (int j = 1; j <= vertices; ++j) {
            graph.adjacencyMatrix[i][j] = exampleMatrix[i - 1][j - 1];
        }
    }

    cout << "Введите начальную вершину для поиска Гамильтонова пути: ";
    int startVertex;
    cin >> startVertex;

    cout << "Введите конечную вершину для поиска Гамильтонова пути (-1, если не важно): ";
    int endVertex;
    cin >> endVertex;

    int startt, endd;
    cout << "Введите начальную вершину: ";
    cin >> startt;
    cout << "Введите конечную вершину: ";
    cin >> endd;



    auto start = chrono::high_resolution_clock::now();

    graph.findShortestPath(exampleMatrix, startt - 1, endd - 1);
    graph.findHamiltonianPath(startVertex, endVertex);

    auto end = chrono::high_resolution_clock::now();
    auto duration = chrono::duration_cast<chrono::microseconds>(end - start);

    cout << "Время: " << duration.count() << " микросекунд " << endl;
    
    
    return 0;
}
