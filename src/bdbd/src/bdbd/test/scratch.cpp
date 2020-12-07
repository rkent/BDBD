// scratchpad for testing c++ constructs. Compile using:
// c++ -o scratch scratch.cpp
#include <iostream>
#include <chrono>
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <vector>

using namespace std;
using namespace Eigen;

double f = 0.0;
double theta1 = M_PI;
double theta2 = M_PI + .1;

double amean(double a1, double a2, double f) {
    // https://en.wikipedia.org/wiki/Mean_of_circular_quantities
    double sina = f * sin(a1) + (1. - f) * sin(a2);
    double cosa = f * cos(a1) + (1. - f) * cos(a2);
    return atan2(sina, cosa);
}

int main(int argc, char** argv) {
    cout << " this is xx scratch.cpp\n";
    cout << amean(theta1, theta2, f) << '\n';
}
