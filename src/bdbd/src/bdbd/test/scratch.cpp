#include <iostream>
#include <chrono>
#include <math.h>
#include <eigen3/Eigen/Dense>

using namespace std;
using namespace Eigen;

int main(int argc, char** argv) {
    cout << " this is scratch.cpp\n";

    // demo of dot product using blocks
    auto a = VectorXd(8);
    a << 1,2,3,4,5,6,7,8;
    cout << a << "\n";
    auto b = VectorXd(8);
    b << 2,4,6,8,10,12,14,16;
    cout << b << "\n";
    int i = 3;
    auto c = a.reverse().tail(i-1).dot(b.segment(1, i-1));
    cout << c << "\n";

}
