#include <iostream>
#include <chrono>
#include <math.h>
#include <eigen3/Eigen/Dense>
#include <vector>

using namespace std;
using namespace Eigen;

int main(int argc, char** argv) {
    cout << " this is scratch.cpp\n";

    // test of array allocation and assignment
    int n = 4;
    // auto a = vector<ArrayXd>(n, ArrayXd::Zero(3));
    const ArrayXd lefts = ArrayXd::Ones(n);

    /*
    vector<vector<ArrayXd>> dv(n, vector<ArrayXd>(n, ArrayXd::Zero(3)));
    for (int i = 0; i < n; i++) {
        // cout << a[i] << ' ';
        for (int j = 0; j < n; j++) {
            for (int k = 0; k < 3; k++) {
                cout << dv[i][j][k] << ' ';
            }
        }
    }
    */
    cout << '\n';
}
