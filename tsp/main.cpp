#include <iostream>
#include <vector>
#include <fstream>
#include <cmath>
#include <set>
#include <iomanip>
#include <algorithm>
#include <random>

using namespace std;

struct Point {
    long double x = 0;
    long double y = 0;
};


class Tree {
    int n;
    vector<vector<int>> g;

    void dfs(int v, vector<int> &vertexes, vector<bool> &used) {
        if (used[v]) {
            return;
        }

        vertexes.push_back(v);
        used[v] = true;

        for (int i = 0; i < g[v].size(); i++) {
            dfs(g[v][i], vertexes, used);
        }

        vertexes.push_back(v);
    }

public:
    Tree(int number) {
        n = number;
        g.resize(n);
    }

    void add_edge(int a, int b) {
        g[a].push_back(b);
        g[b].push_back(a);
    }

    vector<int> dfs_path() {
        vector<bool> used(n, false);
        vector<int> dfs_path;

        dfs(0, dfs_path, used);
        set<int> was_in_path;

        vector<int> path;

        for (int i = 0; i < dfs_path.size(); i++) {
            if (was_in_path.count(dfs_path[i]) == 0) {
                path.push_back(dfs_path[i]);
                was_in_path.insert(dfs_path[i]);
            }
        }

        return path;
    }
};


long double distance(const Point &a, const Point &b) {
    return sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
}

vector<int> reverse_vector(vector<int> v, int from, int to) {
    reverse(v.begin() + from, v.begin() + to);
    return v;
}

long double path_length(const vector<int> &path, const vector<Point> &points) {
    long double dist = 0;

    for (int i = 1; i < path.size(); i++) {
        dist += distance(points[path[i - 1]], points[path[i]]);
    }

    dist += distance(points[path.back()], points[path[0]]);

    return dist;
}

void run_test(const string &filename) {
    ifstream in(filename, ios_base::in);
    int n;
    in >> n;

    vector<Point> points;

    for (int i = 0; i < n; i++) {
        Point a;
        in >> a.x >> a.y;
        points.push_back(a);
    }
    long double result = 1e18;
    mt19937 rnd(time(0));
    for(int iter = 0; iter < 100; iter++) {
        shuffle(points.begin(), points.end(), rnd);
        vector<long double> dists(n, 1e18);
        vector<int> closest(n, -1);
        vector<bool> used(n, false);

        dists[0] = 0;

        Tree tree(n);

        for (int i = 0; i < n; i++) {
            int ind = -1;

            for (int j = 0; j < n; j++) {
                if (!used[j] && (ind == -1 || dists[j] < dists[ind])) {
                    ind = j;
                }
            }

            used[ind] = true;

            if (closest[ind] != -1) {
                tree.add_edge(closest[ind], ind);
            }

            for (int j = 0; j < n; j++) {
                if (!used[j] && distance(points[ind], points[j]) < dists[j]) {
                    dists[j] = distance(points[ind], points[j]);
                    closest[j] = ind;
                }
            }
        }

        vector<int> path = tree.dfs_path();

        bool was_better = true;

        while (was_better) {
            was_better = false;
            for (int it = 0; it < 20000; it++) {
                int i = rnd() % n;
                int j = rnd() % n;
                if (i > j) {
                    swap(i, j);
                }
                vector<int> new_path = reverse_vector(path, i, j);

                if (1.0001 * path_length(new_path, points) < path_length(path, points)) {
                    path = new_path;
                    was_better = true;
                }
            }
        }

        result = min(result, path_length(path, points));
    }
    cout << filename << ' ' << fixed << setprecision(5) << result << '\n';
}

int main() {
    vector<string> tests = {
            "tsp_100_1",
            "tsp_100_2",
            "tsp_100_3",
            "tsp_100_4",
            "tsp_100_5",
            "tsp_100_6",
            "tsp_1000_1",
            "tsp_101_1",
            "tsp_105_1",
            "tsp_1060_1",
            "tsp_107_1",
            "tsp_1084_1",
            "tsp_1173_1",
            "tsp_11849_1",
            "tsp_124_1",
            "tsp_127_1",
            "tsp_1291_1",
            "tsp_1304_1",
            "tsp_1323_1",
            "tsp_136_1",
            "tsp_1379_1",
            "tsp_1400_1",
            "tsp_14051_1",
            "tsp_1432_1",
            "tsp_144_1",
            "tsp_150_1",
            "tsp_150_2",
            "tsp_152_1",
            "tsp_1577_1",
            "tsp_159_1",
            "tsp_1655_1",
            "tsp_1748_1",
            "tsp_1817_1",
            "tsp_18512_1",
            "tsp_1889_1",
            "tsp_195_1",
            "tsp_198_1",
            "tsp_200_1",
            "tsp_200_2",
            "tsp_2103_1",
            "tsp_2152_1",
            "tsp_225_1",
            "tsp_226_1",
            "tsp_2319_1",
            "tsp_2392_1",
            "tsp_262_1",
            "tsp_264_1",
            "tsp_299_1",
            "tsp_3038_1",
            "tsp_318_1",
            "tsp_318_2",
            "tsp_33810_1",
            "tsp_3795_1",
            "tsp_400_1",
            "tsp_417_1",
            "tsp_439_1",
            "tsp_442_1",
            "tsp_4461_1",
            "tsp_493_1",
            "tsp_5_1",
            "tsp_51_1",
            "tsp_574_1",
            "tsp_575_1",
            "tsp_5915_1",
            "tsp_5934_1",
            "tsp_654_1",
            "tsp_657_1",
            "tsp_70_1",
            "tsp_724_1",
            "tsp_7397_1",
            "tsp_76_1",
            "tsp_76_2",
            "tsp_783_1",
            "tsp_99_1"
    };

    for (int i = 0; i < tests.size(); i++) {
        run_test(tests[i]);
    }

    return 0;
}
