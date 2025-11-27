#include <iostream>
#include <fstream>
#include <vector>
#include <unordered_map>
#include <algorithm>
#include <climits>
#include <cfloat>
#include <cmath>

#define INF INT_MAX

using namespace std;

struct Colonia
{
    string nombre;
    int x, y, esCentral;
    Colonia()
    {
        nombre = "";
        x = y = 0;
        esCentral = -1;
    }
    Colonia(string nombre, int x, int y, int esCentral)
    {
        this->nombre = nombre;
        this->x = x;
        this->y = y;
        this->esCentral = esCentral;
    }
};

struct Graph
{
    int V, E, K, Q;
    vector<pair<int, pair<int, int>>> edges;
    vector<pair<int, int>> builtEdges;
    Graph(int V, int E, int K, int Q)
    {
        this->V = V;
        this->E = E;
        this->K = K;
        this->Q = Q;
    }
};

unordered_map<string, int> coloniaIndexMap;
unordered_map<int, Colonia> coloniaNameMap;

// Algoritmo de Kruskal
// 1 – Cableado óptimo de nueva conexión.
/*
    Deberá el programa debe desplegar cuál es la forma óptima de cablear con una nueva fibra óptica conectando colonias
    de tal forma que se pueda compartir información entre cuales quiera dos colonias en el menor costo pósible,
    aprovechando que ya existen conexiones del nuevo cableado (las cuales no deben incluir en el costo)
*/

struct DisjointSets
{
    int *parent, *rank;
    int n;
    DisjointSets(int n)
    {
        this->n = n;
        parent = new int[n + 1];
        rank = new int[n + 1];
        for (int i = 0; i <= n; i++)
        {
            rank[i] = 0;
            parent[i] = i;
        }
    }
    // Para encontrar el parent de 'u'
    int find(int u)
    {
        if (u != parent[u])
        {
            parent[u] = find(parent[u]);
        }
        return parent[u];
    }
    void merge(int x, int y)
    {
        x = find(x);
        y = find(y);
        if (rank[x] > rank[y])
        {
            parent[y] = x;
        }
        else
        {
            parent[x] = y;
        }
        if (rank[x] == rank[y])
        {
            rank[y]++;
        }
    }
};

void cableadoOptimo(Graph &graph, ofstream &outFile)
{
    sort(graph.edges.begin(), graph.edges.end());
    DisjointSets ds(graph.V);

    // Agregar las conexiones ya existentes
    for (auto it : graph.builtEdges)
    {
        ds.merge(it.first, it.second);
    }

    // Kruskal
    vector<pair<int, pair<int, int>>> selectedEdges;
    int costMSTKruskal = 0;
    for (auto it : graph.edges)
    {
        int col1 = ds.find(it.second.first);
        int col2 = ds.find(it.second.second);
        if (col1 != col2)
        {
            costMSTKruskal += it.first;
            selectedEdges.push_back(it);
            ds.merge(it.second.first, it.second.second);
        }
    }

    // Output, escribir las conexiones seleccionadas y el costo total
    for (auto it : selectedEdges)
    {
        string colonia1 = coloniaNameMap[it.second.first].nombre;
        string colonia2 = coloniaNameMap[it.second.second].nombre;
        int costo = it.first;
        outFile << colonia1 << " - " << colonia2 << " " << costo << endl;
    }
    outFile << endl
            << "Costo Total: " << costMSTKruskal << endl;
}

// Algoritmo del Travelling Salesman
// 2 – La ruta óptima.
/*
    Debido a que las ciudades apenas están entrando al mundo tecnológico, se requiere que alguien visite cada colonia
    que "no son centrales" para ir a dejar estados de cuenta físicos, publicidad, avisos y notificaciones impresos.
    por eso se quiere saber ¿cuál es la ruta más corta posible que visita cada colonia exactamente una vez y al finalizar
    regresa a la colonia origen? Tomar en cuenta que muchas veces el costo mínimo puede pasar por una colonia central o
    más intermedias. El programa debe desplegar la ruta a considerar así como el costo.
*/

inline int safe_sum(int a, int b)
{
    if (a == INT_MAX || b == INT_MAX)
        return INT_MAX; // any INF makes INF
    if (a > INT_MAX - b)
        return INT_MAX; // would overflow → clamp to INF
    return a + b;
}

int totalCost(int mask, int pos, int n, vector<vector<int>> &cost, vector<vector<int>> &dp, int nonCentralMask, int startingColonia, vector<vector<int>> &nextBest)
{
    // se cancelan los bits de las colonias centrales, solo revisamos las no centrales
    if ((mask & nonCentralMask) == nonCentralMask)
        return cost[pos][startingColonia];

    int &memo = dp[mask][pos];
    if (memo != -1)
        return memo;

    int ans = INT_MAX;
    int bestNext = -1;

    // Try visiting every city that has not been visited yet
    for (int i = 0; i < n; i++)
    {
        if ((mask & (1 << i)) == 0)
        {
            if (cost[pos][i] == INT_MAX)
                continue; // Poda si no hay camino entre pos e i

            // If city i is not visited, visit it and update the mask
            int candidate = safe_sum(cost[pos][i], totalCost((mask | (1 << i)), i, n, cost, dp, nonCentralMask, startingColonia, nextBest));
            if (candidate < ans)
            {
                ans = candidate;
                bestNext = i;
            }
        }
    }

    nextBest[mask][pos] = bestNext;
    return memo = ans;
}

void rutaOptima(Graph &graph, ofstream &outFile)
{
    // build cost matrix
    vector<vector<int>> cost(graph.V, vector<int>(graph.V, INT_MAX));
    for (int i = 0; i < graph.V; i++)
    {
        cost[i][i] = 0;
    }
    for (auto it : graph.edges)
    {
        cost[it.second.first][it.second.second] = it.first;
        cost[it.second.second][it.second.first] = it.first;
    }

    int nonCentralMask = 0;
    int startingColonia = -1;
    for (int i = 0; i < graph.V; i++)
    {
        if (coloniaNameMap[i].esCentral == 0)
        {
            nonCentralMask |= (1 << i);
            // seleccionar la primera colonia no central como punto de inicio
            if (startingColonia == -1)
            {
                startingColonia = i;
            }
        }
    }

    vector<vector<int>> dp(1 << graph.V, vector<int>(graph.V, -1));
    vector<vector<int>> nextBest(1 << graph.V, vector<int>(graph.V, -1));

    int startMask = (1 << startingColonia);

    int costo = totalCost(startMask, startingColonia, graph.V, cost, dp, nonCentralMask, startingColonia, nextBest);

    // Reconstruir la ruta
    int pos = startingColonia;
    int first = true;
    vector<int> path;
    path.push_back(startingColonia); // agregar el punto de inicio
    while ((startMask & nonCentralMask) != nonCentralMask)
    {
        int next = nextBest[startMask][pos];
        startMask |= (1 << next);
        pos = next;
        path.push_back(pos);
    }
    path.push_back(startingColonia); // agregar el punto final

    // imprimir la ruta en orden
    for (int i = path.size() - 1; i >= 0; i--)
    {
        outFile << coloniaNameMap[path[i]].nombre;
        if (i != 0)
        {
            outFile << " - ";
        }
    }

    outFile << endl
            << endl
            << "La Ruta Óptima tiene un costo total de: " << costo << endl;
}

// Algoritmo de Floyd Warshall
// 3 – Caminos más cortos entre centrales.
/*
    El programa deberá generar la ruta óptima para ir de todas las centrales entre si,
    se puede pasar por una colonia no central.
*/

// Encontrar los caminos mas cortos de una central a otra para todas las centrales

// Junta el recorrido de i->k y de k-j
vector<int> mergePaths(vector<int> &path1, vector<int> &path2, int k)
{
    vector<int> merge = path1;
    merge.push_back(k);
    for (int &node : path2)
    {
        merge.push_back(node);
    }
    return merge;
}

void floydWarshall(vector<vector<int>> &dist, vector<vector<vector<int>>> &distAux)
{
    int n = dist.size();
    for (int k = 0; k < n; k++)
    { // nodo base
        for (int i = 0; i < n; i++)
        { // nodo origen
            for (int j = 0; j < n; j++)
            {                                             // nodo destino
                if (dist[i][k] < INF && dist[k][j] < INF) // Si hay conexión entre el nodo base k y los dos nodos i,j
                    if (dist[i][j] > dist[i][k] + dist[k][j])
                    {
                        dist[i][j] = dist[i][k] + dist[k][j]; // Nueva distancia
                        // Guarda el nuevo recorrido
                        distAux[i][j] = (mergePaths(distAux[i][k], distAux[k][j], k));
                    }
            }
        }
    }
}

void caminosCentrales(Graph &graph, ofstream &outFile)
{

    // Vector de distancias inicializado en infinito
    vector<vector<int>> dist(graph.V, vector<int>(graph.V, INF));
    // El peso de un  nodo a sí mismo es 0
    for (int i = 0; i < graph.V; i++)
    {
        dist[i][i] = 0;
    }
    // Matriz auxiliar para guardar el recorrido de menor costo
    vector<vector<vector<int>>> distAux(graph.V, vector<vector<int>>(graph.V));
    int cost;
    vector<int> coloniasIntermedias;
    pair<int, int> currEdge;
    int currCost;
    // Se llena la matriz de distancias para floyd
    for (pair<int, pair<int, int>> edge : graph.edges)
    {
        currCost = edge.first;
        currEdge = edge.second;
        dist[currEdge.first][currEdge.second] = currCost;
        dist[currEdge.second][currEdge.first] = currCost;
    }
    // Floyd warshall te dice la distancia más corta para todos los pares de origen destino
    floydWarshall(dist, distAux);

    // Para todos los nodos con todos los nodos
    for (int coloniaOrigenIdx = 0; coloniaOrigenIdx < graph.V; coloniaOrigenIdx++)
    {
        for (int coloniaDestinoIdx = coloniaOrigenIdx + 1; coloniaDestinoIdx < graph.V; coloniaDestinoIdx++)
        {
            // Si son colonias
            if (coloniaNameMap[coloniaOrigenIdx].esCentral && coloniaNameMap[coloniaDestinoIdx].esCentral)
            {
                // Recorrido intermedio
                coloniasIntermedias = distAux[coloniaOrigenIdx][coloniaDestinoIdx];
                // Costo del recorrido
                cost = dist[coloniaOrigenIdx][coloniaDestinoIdx];
                // Origen, intermedio, destino, costo
                outFile << coloniaNameMap[coloniaOrigenIdx].nombre << " - ";
                for (int &coloniaIntermedia : coloniasIntermedias)
                {
                    outFile << coloniaNameMap[coloniaIntermedia].nombre << " - ";
                }
                outFile << coloniaNameMap[coloniaDestinoIdx].nombre;
                outFile << " (" << cost << ")" << endl;
            }
        }
    }
}

// Ordenamiento y búsqueda binaria
// 4 – Conexión de nuevas colonias.
/*
    Se leera a continuación una serie de puntos cartecianos en el mapa de la ciudad en donde se planea conectar nuevas colonias,
    y se deberá decir cual es la colina y punto carteciano más cercano con el cual se debe conectar.
*/

double dist(Colonia &col1, Colonia &col2)
{
    double dx = col1.x - col2.x;
    double dy = col1.y - col2.y;
    return sqrt(dx * dx + dy * dy);
}

// Complejidad: O(v)
int conexionMasCercana(Graph &graph, Colonia coloniaNueva)
{
    int minColoniaIdx = 0;
    int minDist = dist(coloniaNameMap[0], coloniaNueva);
    Colonia currColonia;
    int currDist;
    for (int colIdx = 1; colIdx < graph.V; colIdx++)
    {
        currColonia = coloniaNameMap[colIdx];
        currDist = dist(coloniaNameMap[colIdx], coloniaNueva);
        if (currDist < minDist)
        {
            minColoniaIdx = colIdx;
            minDist = currDist;
        }
    }
    return minColoniaIdx;
}

// Complejidad: O(q * v)
// Fuerza bruta es mejor que divide and conquer porque se conoce el punto de referencia
void conexionNuevasColonias(Graph &graph, ofstream &outFile)
{
    for (int i = graph.V; i < graph.V + graph.Q; i++)
    {
        Colonia coloniaNueva = coloniaNameMap[i];
        int coloniaCercanaIdx = conexionMasCercana(graph, coloniaNueva);
        Colonia coloniaCercana = coloniaNameMap[coloniaCercanaIdx];
        outFile << coloniaNueva.nombre << " debe conectarse con " << coloniaCercana.nombre << endl;
    }
}

Graph readInputsAndProcess()
{
    // n, m, k, q
    int colonias, conexiones, nuevasConexiones, nuevasColonias;
    cin >> colonias >> conexiones >> nuevasConexiones >> nuevasColonias;

    Graph graph(colonias, conexiones, nuevasConexiones, nuevasColonias);

    // Luego vienen n lineas con el nombre de la colonia (no contienen espacios), su posición en el plano carteciano y si es o no una central.
    for (int i = 0; i < colonias; i++)
    {
        string nombreColonia;
        int x, y, esCentral;
        cin >> nombreColonia >> x >> y >> esCentral;
        Colonia colonia(nombreColonia, x, y, esCentral);
        coloniaIndexMap[nombreColonia] = i;
        coloniaNameMap[i] = colonia;
    }

    // Posteriomente vienen m lineas con las conexiones entre colonias y su costo.
    for (int i = 0; i < conexiones; i++)
    {
        string colonia1, colonia2;
        int costo;
        cin >> colonia1 >> colonia2 >> costo;
        int index1 = coloniaIndexMap[colonia1];
        int index2 = coloniaIndexMap[colonia2];
        graph.edges.push_back({costo, {index1, index2}});
    }

    // Posteriormente viene k lineas con las conexiones entre colonias que cuentan con el nuevo cableado
    for (int i = 0; i < nuevasConexiones; i++)
    {
        string colonia1, colonia2;
        cin >> colonia1 >> colonia2;
        int index1 = coloniaIndexMap[colonia1];
        int index2 = coloniaIndexMap[colonia2];
        graph.builtEdges.push_back({index1, index2});
    }

    // Por último vienen q lineas con el nombre de la nueva colonia (no contiene espacios) y los puntos cartecianos de las nuevas colonias.
    for (int i = 0; i < nuevasColonias; i++)
    {
        string nombreColonia;
        int x, y;
        cin >> nombreColonia >> x >> y;
        Colonia nuevaColonia(nombreColonia, x, y, -1);
        coloniaIndexMap[nombreColonia] = colonias + i;
        coloniaNameMap[colonias + i] = nuevaColonia;
    }

    return graph;
}

int main()
{
    ofstream outFile("checking2.txt");

    Graph graph = readInputsAndProcess();

    outFile << "-------------------" << endl;
    outFile << "1 – Cableado óptimo de nueva conexión." << endl
            << endl;
    cableadoOptimo(graph, outFile);
    outFile << endl
            << "-------------------" << endl;
    outFile << "2 – La ruta óptima." << endl
            << endl;
    rutaOptima(graph, outFile);
    outFile << endl
            << "-------------------" << endl;
    outFile << "3 – Caminos más cortos entre centrales." << endl
            << endl;
    caminosCentrales(graph, outFile);
    outFile << endl
            << "-------------------" << endl;
    outFile << "4 – Conexión de nuevas colonias." << endl
            << endl;
    conexionNuevasColonias(graph, outFile);
    outFile << endl
            << "-------------------" << endl;

    outFile.close();
    return 0;
}

// c++ -std=c++11 -o main main.cpp