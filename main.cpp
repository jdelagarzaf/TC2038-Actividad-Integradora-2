#include <iostream>
#include <fstream>
#include <vector>
#include <unordered_map>
#include <algorithm>

using namespace std;

struct Colonia {
    string nombre;
    int x, y, esCentral;
    Colonia() {
        nombre = "";
        x = y = 0;
        esCentral = -1;
    }
    Colonia(string nombre, int x, int y, int esCentral) {
        this->nombre = nombre;
        this->x = x;
        this->y = y;
        this->esCentral = esCentral;
    }
};

struct Graph{
	int V, E, K, Q;
	vector<pair<int, pair<int, int>>> edges;
    vector<pair<int, int>> builtEdges;
	Graph(int V, int E, int K, int Q){
		this->V = V;
		this->E = E;
		this->K = K;
		this->Q = Q;
	}
};

unordered_map<string, int> coloniaIndexMap;
unordered_map<int, Colonia> coloniaNameMap;

struct DisjointSets{
	int *parent, *rank;
	int n;
	DisjointSets(int n){
		this->n = n;
		parent = new int[n+1];
		rank = new int[n+1];
		for (int i=0; i<=n; i++){
			rank[i] = 0;
			parent[i] = i;
		}
	}
	// Para encontrar el parent de 'u'
	int find(int u){
		if (u != parent[u]){
			parent[u] = find(parent[u]);
		}
		return parent[u];
	}
	void merge(int x, int y){
		x = find(x);
		y = find(y);
		if (rank[x] > rank[y]){
			parent[y] = x;
		}
		else{
			parent[x] = y;
		}
		if (rank[x] == rank[y]){
			rank[y]++;
		}
	}
};

// Algoritmo de Prim
// 1 – Cableado óptimo de nueva conexión.
/*
    Deberá el programa debe desplegar cuál es la forma óptima de cablear con una nueva fibra óptica conectando colonias 
    de tal forma que se pueda compartir información entre cuales quiera dos colonias en el menor costo pósible, 
    aprovechando que ya existen conexiones del nuevo cableado (las cuales no deben incluir en el costo)
*/
void cableadoOptimo(Graph& graph, ofstream& outFile) {
    sort(graph.edges.begin(), graph.edges.end());
	DisjointSets ds(graph.V);

    // Agregar las conexiones ya existentes
    for(auto it:graph.builtEdges) {
        ds.merge(it.first, it.second);
    }

    // Kruskal
    vector<pair<int, pair<int, int>>> selectedEdges;
	int costMSTKruskal = 0;
	for(auto it:graph.edges) {
		int p1 = ds.find(it.second.first);
		int p2 = ds.find(it.second.second);
		if (p1 != p2) {
			costMSTKruskal += it.first;
            selectedEdges.push_back(it);
			ds.merge(it.second.first, it.second.second);
		}
	}

    // Output, escribir las conexiones seleccionadas y el costo total
    for (auto it:selectedEdges){
        string colonia1 = coloniaNameMap[it.second.first].nombre;
        string colonia2 = coloniaNameMap[it.second.second].nombre;
        int costo = it.first;
        outFile << colonia1 << " - " << colonia2 << " " << costo << endl;
    }
    outFile << endl << "Costo Total: " << costMSTKruskal << endl;
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
void rutaOptima(Graph& graph, ofstream& outFile) {
    
}

// Algoritmo de Dijkstra? o Travelling Salesman modificado tal vez
// 3 – Caminos más cortos entre centrales.
/*
    El programa deberá generar la ruta óptima para ir de todas las centrales entre si, 
    se puede pasar por una colonia no central.
*/
void caminosCentrales(Graph& graph, ofstream& outFile) {

}

// Algoritmo del punto más cercano
// 4 – Conexión de nuevas colonias.
/*
    Se leera a continuación una serie de puntos cartecianos en el mapa de la ciudad en donde se planea conectar nuevas colonias, 
    y se deberá decir cual es la colina y punto carteciano más cercano con el cual se debe conectar.
*/
void conexionNuevasColonias(Graph& graph, ofstream& outFile) {

}

Graph readInputsAndProcess() {
    // n, m, k, q
    int colonias, conexiones, nuevasConexiones, nuevasColonias;
    cin >> colonias >> conexiones >> nuevasConexiones >> nuevasColonias;

    Graph graph(colonias, conexiones, nuevasConexiones, nuevasColonias);

    // Luego vienen n lineas con el nombre de la colonia (no contienen espacios), su posición en el plano carteciano y si es o no una central.
    for (int i=0; i < colonias; i++){
        string nombreColonia;
        int x, y, esCentral;
        cin >> nombreColonia >> x >> y >> esCentral;
        Colonia colonia(nombreColonia, x, y, esCentral);
        coloniaIndexMap[nombreColonia] = i;
        coloniaNameMap[i] = colonia;
    }

    // Posteriomente vienen m lineas con las conexiones entre colonias y su costo.
    for (int i=0; i < conexiones; i++){
        string colonia1, colonia2;
        int costo;
        cin >> colonia1 >> colonia2 >> costo;
        int index1 = coloniaIndexMap[colonia1];
        int index2 = coloniaIndexMap[colonia2];
        graph.edges.push_back({costo, {index1, index2}});
    }

    // Posteriormente viene k lineas con las conexiones entre colonias que cuentan con el nuevo cableado
    for (int i=0; i < nuevasConexiones; i++){
        string colonia1, colonia2;
        cin >> colonia1 >> colonia2;
        int index1 = coloniaIndexMap[colonia1];
        int index2 = coloniaIndexMap[colonia2];
        graph.builtEdges.push_back({index1, index2});
    }

    // Por último vienen q lineas con el nombre de la nueva colonia (no contiene espacios) y los puntos cartecianos de las nuevas colonias.
    for (int i=0; i < nuevasColonias; i++){
        string nombreColonia;
        int x, y;
        cin >> nombreColonia >> x >> y;
        Colonia nuevaColonia(nombreColonia, x, y, -1);
        coloniaIndexMap[nombreColonia] = colonias + i;
        coloniaNameMap[colonias + i] = nuevaColonia;
    }

    return graph;
}

int main() {
    ofstream outFile("checking2.txt");
    
    Graph graph = readInputsAndProcess();

    outFile << "-------------------" << endl;
    outFile << "1 – Cableado óptimo de nueva conexión." << endl << endl;
    cableadoOptimo(graph, outFile);
    outFile << endl << "-------------------" << endl;
    outFile << "2 – La ruta óptima." << endl << endl;
    rutaOptima(graph, outFile);
    outFile << endl << "-------------------" << endl;
    outFile << "3 – Caminos más cortos entre centrales." << endl << endl;
    caminosCentrales(graph, outFile);
    outFile << endl << "-------------------" << endl;
    outFile << "4 – Conexión de nuevas colonias." << endl << endl;
    conexionNuevasColonias(graph, outFile);
    outFile << endl << "-------------------" << endl;

    outFile.close();
    return 0;
}