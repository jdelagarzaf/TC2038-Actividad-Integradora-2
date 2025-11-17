// Jorge Adrian de la Garza Flores - A00838816
// Actividad 11 - Sparko Birthday II
// 13 de Octubre del 2025
#include <iostream>
#include <vector>
#include <climits>

using namespace std;

// Función auxiliar para sumar dos enteros de manera segura, sin overflow
// Input: dos enteros a y b
// Output: la suma de a y b, o INT_MAX si hay overflow
// Complejidad: O(1)
inline int safe_sum(int a, int b) {
    if (a == INT_MAX || b == INT_MAX) return INT_MAX;   // any INF makes INF
    if (a > INT_MAX - b) return INT_MAX;                // would overflow → clamp to INF
    return a + b;
}

// Función recursiva para calcular el costo mínimo del TSP usando bitmasking
// Input: mask (bitmask de ciudades visitadas), pos (ciudad actual), n (número total de ciudades), cost (matriz de costos)
// Output: el costo mínimo para visitar todas las ciudades y regresar a la ciudad inicial
// Complejidad: O(n^2 * 2^n) en el peor caso
int totalCost(int mask, int pos, int n, vector<vector<int>> &cost, vector<vector<int>> &dp, int nonCentrlMask) {
    // Base case: if all cities are visited, return the cost to return to the starting city (0)
    if ((mask & nonCentrlMask) == nonCentrlMask) return cost[pos][0];

    int &memo = dp[mask][pos];
    if (memo != -1) return memo;

    int ans = INT_MAX;

    // Try visiting every city that has not been visited yet
    for (int i = 0; i < n; i++) {
        if ((mask & (1 << i)) == 0) {
            if (cost[pos][i] == INT_MAX) continue; // Poda si no hay camino entre pos e i
            // If city i is not visited, visit it and update the mask
            ans = min(ans, safe_sum(cost[pos][i], totalCost((mask | (1 << i)), i, n, cost, dp, nonCentrlMask)));
        }
    }
    return memo = ans;
}

// Función principal para resolver el TSP
// Input: cost (matriz de costos entre ciudades)
// Output: el costo mínimo para visitar todas las ciudades y regresar a la ciudad inicial
// Complejidad: O(n^2 * 2^n) en el peor caso
string travelingSalesmanProblem(vector<vector<int>> &cost) {
    int n = cost.size();
    if (n == 1) return "0";
    vector<vector<int>> dp(1 << n, vector<int>(n, -1));
    // Start from city 0, and only city 0 is visited initially (mask = 1)
    int result = totalCost(1, 0, n, cost, dp);
    return result == INT_MAX ? "INF" : to_string(result);
}

// Función para construir la matriz de costos
// Input: cost (matriz de costos), roads (número de caminos)
// Output: la matriz de costos actualizada con los pesos de los caminos
// Complejidad: O(doghouses + roads)
void buildCostMatrix(vector<vector<int>> &cost, int roads){
    // Ciclo O(doghouses)
    // Inicializar la diagonal principal a 0.
    for (int i = 0; i < cost.size(); i++) {
        cost[i][i] = 0;
    }

    char origin, destination;
    int weight;
    // Ciclo O(roads)
    for (int i = 0; i < roads; i++) {
        cin >> origin >> destination >> weight;
        cost[origin - 'A'][destination - 'A'] = weight;
        cost[destination - 'A'][origin - 'A'] = weight;
    }
}

// Función principal, se encarga de leer la entrada y ejecutar el algoritmo TSP
// Input: Ninguno, pero lee el número de casas de perro y caminos, así como los detalles de los caminos
// Output: Imprime el costo mínimo para visitar todas las casas de perro y regresar a la casa inicial
// Complejidad: O(doghouses + roads + n^2 * 2^n) en el peor caso
// Se simplifica a O(n^2 * 2^n) en el peor caso
int main() {
    int doghouses, roads;
    cin >> doghouses >> roads;
    vector<vector<int>> cost(doghouses, vector<int>(doghouses, INT_MAX));
    buildCostMatrix(cost, roads); // O(doghouses + roads)
    cout << travelingSalesmanProblem(cost) << endl; // O(n^2 * 2^n) en el peor caso
    return 0;
}