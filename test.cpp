#include <iostream>
#include <fstream>
#include <vector>
#include <unordered_map>
#include <algorithm>
#include <climits>

#define INF INT_MAX

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


// Algoritmo de Floyd Warshall
// 3 – Caminos más cortos entre centrales.
/*
    El programa deberá generar la ruta óptima para ir de todas las centrales entre si, 
    se puede pasar por una colonia no central.
*/

//Encontrar los caminos mas cortos de una central a otra para todas las centrales

vector<int> mergePaths(vector<int>& path1, vector<int>& path2,int k){
    vector<int> merge = path1;
    merge.push_back(k);
    for(int& node : path2){
        merge.push_back(node);
    }
    return merge;
}

void floydWarshall(vector<vector<int>> &dist, vector<vector<vector<int>>>& distAux) {
    int n = dist.size();
    for(int k = 0; k < n; k++) { //nodo base
        for(int i = 0; i < n; i++) { //nodo origen
            for(int j = 0; j < n; j++) { //nodo destino
                if(dist[i][k] < INF && dist[k][j] < INF) //Si hay conexión entre el nodo base k y los dos nodos i,j
                    if(dist[i][j] > dist[i][k] + dist[k][j]){
                        dist[i][j] =  dist[i][k] + dist[k][j]; //Nueva distancia
                        distAux[i][j] = (mergePaths(distAux[i][k],distAux[k][j],k));
                    }
                    
            }
        }
    }
}

void caminosCentrales(vector<vector<int>> dist) {
    
    vector<vector<vector<int>>> distAux (dist.size(), vector<vector<int>>(dist.size(), vector<int>()));
    int cost;
    vector<int> coloniasIntermedias;

    for(int i = 0; i < dist.size(); i++){
        for(int j = 0; j < dist.size(); j++){
            if(dist[i][j] == INF){
                cout << "INF" << " ";
            } else{
                cout << dist[i][j] << " ";
            }
        }
        cout << endl;
    }
    cout << endl;
    cout << endl;

    floydWarshall(dist, distAux);

    for(int i = 0; i < dist.size(); i++){
        for(int j = 0; j < dist.size(); j++){
            if(dist[i][j] == INF){
                cout << "INF" << " ";
            } else{
                cout << dist[i][j] << " ";
            }
        }
        cout << endl;
    }

    for(int coloniaOrigenIdx = 0; coloniaOrigenIdx < dist.size(); coloniaOrigenIdx++){
        for(int coloniaDestinoIdx = coloniaOrigenIdx + 1; coloniaDestinoIdx < dist.size(); coloniaDestinoIdx++){
            cout << coloniaOrigenIdx << "-" << coloniaDestinoIdx << endl;
            coloniasIntermedias = distAux[coloniaOrigenIdx][coloniaDestinoIdx];
            cost = dist[coloniaOrigenIdx][coloniaDestinoIdx];
            cout << coloniaOrigenIdx << " - ";
            for(int& coloniaIntermedia : coloniasIntermedias){
                cout << coloniaIntermedia << " - ";
            }
            cout << coloniaDestinoIdx;
            cout << " (" << cost << ")" << endl;
        }
    }
    


}


int main(){

    vector<vector<int>> dist = {
        {    0,    5,  INF,    2,  INF },
        {    5,    0,    3,  INF,  INF },
        {  INF,    3,    0,  INF,  INF },
        {    2,  INF,  INF,    0,    4 },
        {  INF,  INF,  INF,    4,    0 }
    };

    caminosCentrales(dist);


    return 0;
}

//c++ -std=c++11 -o test test.cpp