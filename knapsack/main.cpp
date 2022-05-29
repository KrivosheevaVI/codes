#include <iostream>
#include <vector>
#include <fstream>

using namespace std;

void run_test(string filename) {
    ifstream in(filename, ios_base::in);

    int n, Size;
    in >> n >> Size;

    int coefficient = 1;

    if(Size > 10000) {
        coefficient = 10;
    }

    Size /= coefficient;

    vector<int> value(n);
    vector<int> weight(n);

    for (int i = 0; i < n; i++) {
        in >> value[i] >> weight[i];
        weight[i] = (weight[i] - 1) / coefficient + 1;
    }
    vector<vector<int>> dp(n + 1, vector<int>(Size + 1));
    vector<vector<int>> parent(n + 1, vector<int>(Size + 1));
    dp[0][0] = 0;

    for (int i = 1; i <= Size; i++) {
        dp[0][i] = -1;
    }

    for (int i = 1; i <= n; i++) {
        for (int j = 0; j <= Size; j++) {
            int p1, p2;
            p1 = p2 = -1;
            p2 = dp[i - 1][j];
            if (j - weight[i - 1] >= 0) {
                p1 = dp[i - 1][j - weight[i - 1]] + value[i - 1];

            }
            if (p1 >= p2) {
                dp[i][j] = p1;
                parent[i][j] = 1; //взяли предмет

            } else {
                dp[i][j] = p2;
                parent[i][j] = 0; //не взяли предмет
            }
        }
    }
    int id = 0;
    for (int i = 1; i <= Size; i++) {
        if (dp[n][i] > dp[n][id]) {
            id = i;
        }
    }
    cout << filename << ' ' << dp[n][id] << endl;

    vector<int> answer(n, 0);
    int j = id;
    int i = n;

    while (i > 0) {
        if (parent[i][j] == 1) {
            answer[i - 1] = 1;
            i--;
            j = j - weight[i];
        } else {
            i--;
        }
    }
}

int main() {
//    freopen("ks_1000_0", "r", stdin);
    vector<string> tests = {
            "ks_4_0",
            "ks_19_0",
            "ks_10000_0",
            "ks_1000_0",
            "ks_100_0",
            "ks_100_1",
            "ks_100_2",
            "ks_200_0",
            "ks_200_1",
            "ks_300_0",
            "ks_30_0",
            "ks_400_0",
            "ks_40_0",
            "ks_45_0",
            "ks_500_0",
            "ks_50_0",
            "ks_50_1",
            "ks_60_0"
    };

    for(int i = 0; i < tests.size(); i++) {
        string test = tests[i];
        run_test(test);
    }
}