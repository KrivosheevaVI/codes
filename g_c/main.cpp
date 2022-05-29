#include <iostream>
#include <vector>
#include <algorithm>
#include <set>
#include <fstream>
#include <random>

using namespace std;

class Graph {
    int n;
    vector<vector<int>> g;
    vector<int> colors;
public:
    Graph(int number) {
        n = number;
        g.resize(n);
        colors.resize(n, -1);
    }

    void add_edge(int a, int b) {
        g[a].push_back(b);
        g[b].push_back(a);
    }

    int get_color(int a) {
        return colors[a];
    }

    void set_color(int a, int c) {
        colors[a] = c;
    }

    vector<int> get_neighbors(int a) {
        return g[a];
    }
};

void run_test(string filename) {
    ifstream in(filename, ios_base::in);
    int n;
    in >> n;
    Graph g(n);
    int m;
    in >> m; //количество ребеp

    for (int i = 0; i < m; i++) {
        int a, b;
        in >> a >> b;
        g.add_edge(a, b);
    }
    vector<int> permutation;
    for (int i = 0; i < n; i++) {
        permutation.push_back(i);
    }

    mt19937 rnd(time(0));

    int result = 1e9;
    for(int it = 0; it < 1000; it++) {
        shuffle(permutation.begin(), permutation.end(), rnd);
        int max_color = 0;

        for (int i = 0; i < n; i++) {
            vector<int> neighbor = g.get_neighbors(permutation[i]);
            set<int> colors;

            for (int j = 0; j < neighbor.size(); j++) {
                colors.insert(g.get_color(neighbor[j]));
            }

            int c = 0;
            while (colors.count(c) > 0) {
                c++;
            }
            g.set_color(permutation[i], c);
            max_color = max(max_color, c);
        }
        result = min(result, max_color + 1);
    }
    cout << filename << ' ' << result << endl;

}

int main() {
    vector<string> tests = {
            "gc_1000_1",

            "gc_1000_3",

            "gc_1000_5",

            "gc_1000_7",

            "gc_1000_9",


            "gc_100_1",

            "gc_100_3",

            "gc_100_5",

            "gc_100_7",

            "gc_100_9",


            "gc_20_1",

            "gc_20_3",

            "gc_20_5",

            "gc_20_7",

            "gc_20_9",


            "gc_250_1",

            "gc_250_3",

            "gc_250_5",

            "gc_250_7",

            "gc_250_9",


            "gc_4_1",


            "gc_500_1",

            "gc_500_3",

            "gc_500_5",

            "gc_500_7",

            "gc_500_9",


            "gc_50_1",

            "gc_50_3",

            "gc_50_5",

            "gc_50_7",

            "gc_50_9",

            "gc_70_1",

            "gc_70_3",

            "gc_70_5",

            "gc_70_7",

            "gc_70_9"

    };

    for (int i = 0; i < tests.size(); i++) {
        run_test(tests[i]);
    }

    return 0;
}
