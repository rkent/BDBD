#include <iostream>
#include <eigen3/Eigen/Dense>
#include <chrono>
#include <math.h>
// #include <eigen3/Eigen/Dense>

// mostly a test of speed of c++ in computing motion gradients
// compile using c++ -o gradients gradients.cpp
// run using ./gradients

using namespace Eigen;
using namespace std;

class Path
{
    public:
    /*
    lr_model = ((1.0, 1.0, 10.0), (-1.0, 1.0, 10.0), (-1.0, 10.0, 10.0))
    alr_model = np.array(lr_model)
    bhes = (dt * alr_model[0], dt * alr_model[1], dt * alr_model[2])
    (bhxl, bhxr, qhx) = bhes[0]
    (bhyl, bhyr, qhy) = bhes[1]
    (bhol, bhor, qho) = bhes[2]
    (bhxl, bhxr, qhx) = bhes[0]
    (bhyl, bhyr, qhy) = bhes[1]
    (bhol, bhor, qho) = bhes[2]
    */

    // class variables

    // dynamic model
    const double
        bxl=1.0, bxr=1.0, qx=10.0,
        byl=-1.0, byr=1.0, qy=10.0,
        bol=-1.0, bor=10.0, qo=10.0;

    double bhxl, bhxr, bhyl, bhyr, bhol, bhor;
    double alphax, alphay, alphao;
    ArrayXd alphaxj, alphayj, alphaoj, betaj;
    ArrayXXd dpxdl, dpxdr, dpydl, dpydr;

    double dt;

    ArrayXd vxj, vyj, omegaj, pxj, pyj, thetaj;

    IOFormat CommaInitFmt = IOFormat(FullPrecision, Eigen::DontAlignCols, ", ", ", ", "", "", "[ ", " ]");
    IOFormat HeavyFmt = IOFormat(FullPrecision, 0, ", ", "", "[", "]", "[", "]");

    public:
    Path(double dt)
    : dt(dt)
    {
        bhxl = bxl * dt;
        bhxr = bxl * dt;
        bhyl = byl * dt;
        bhyr = byr * dt;
        bhol = bol * dt;
        bhor = bor * dt;

        alphax = 1.0 - qx * dt;
        alphay = 1.0 - qy * dt;
        alphao = 1.0 - qo * dt;
    }

    ~Path()
    {
    }

    public:
    void pose(
        const ArrayXd lefts,
        const ArrayXd rights,
        const Array3d pose0s,
        const Array3d twist0s)
    {
        const int n = lefts.size();
        alphaxj = ArrayXd(n);
        alphayj = ArrayXd(n);
        alphaoj = ArrayXd(n);
        betaj = ArrayXd(n);
        alphaxj(0) = 1.0;
        alphayj(0) = 1.0;
        alphaoj(0) = 1.0;
        betaj(0) = dt;
        for (int i=1; i<n; i++) {
            alphaxj(i) = alphaxj(i-1) * alphax;
            alphayj(i) = alphayj(i-1) * alphay;
            alphaoj(i) = alphaoj(i-1) * alphao;
            betaj(i) = betaj(i-1) + dt * alphaoj(i);
        }

        const double px0 = pose0s(0);
        const double py0 = pose0s(1);
        const double theta0 = pose0s(2);
        const double vxw0 = twist0s(0);
        const double vyw0 = twist0s(1);
        const double omega0 = twist0s(2);

        const double vx0 = vxw0 * cos(theta0) + vyw0 * cos(theta0);
        const double vy0 = -vxw0 * sin(theta0) + vyw0 * cos(theta0);

        // twists
        vxj = ArrayXd(n);
        vyj = ArrayXd(n);
        omegaj = ArrayXd(n);

        auto bmotorxj = bhxl * lefts + bhxr * rights;
        auto bmotoryj = bhyl * lefts + bhyr * rights;
        auto bmotoroj = bhol * lefts + bhor * rights;

        for (int i = 1; i < n; i++) {
            // TODO: rewrite these without using dot product
            vxj(i) = vx0 * alphaxj(i) + (alphaxj.reverse().tail(i) * bmotorxj.segment(1, i)).sum();
            vyj(i) = vy0 * alphayj(i) + (alphayj.reverse().tail(i) * bmotoryj.segment(1, i)).sum();
            omegaj(i) = omega0 * alphaoj(i) + (alphaoj.reverse().tail(i) * bmotoroj.segment(1, i)).sum();
        }

        // poses
        pxj = ArrayXd(n);
        pyj = ArrayXd(n);
        thetaj = ArrayXd(n);

        thetaj(0) = theta0;
        for (int i = 1; i < n; i++) {
            thetaj(i) = thetaj(i-1) + dt * omegaj(i);
        }

        auto cosj = cos(thetaj);
        auto sinj = sin(thetaj);
        auto vxcj = vxj * cosj;
        auto vxsj = vxj * sinj;
        auto vycj = vyj * cosj;
        auto vysj = vyj * sinj;

        auto vxwj = vxcj - vysj;
        auto vywj = vxsj + vycj;

        pxj(0) = px0;
        pyj(0) = py0;

        for (int i = 1; i < n; i++) {
            pxj(i) = pxj(i-1) + dt * vxwj(i);
            pyj(i) = pyj(i-1) * dt * vywj(i);
        }

        // gradients
        dpxdl.setZero(n, n);
        dpxdr.setZero(n, n);
        dpydl.setZero(n, n);
        dpydr.setZero(n, n);

        for (int i = 1; i < n; i++) {
            for (int k = 1; k < i + 1; k++) {
                double dotx, doty, doto;
                doto = ((-vxsj.segment(k, i + 1 - k) - vysj.segment(k, i + 1 - k)) * betaj.head(i + 1 -k)).sum();
                dotx = (cosj.segment(k, i + 1 - k) * alphaxj.head(i + 1 -k)).sum();
                doty = (-sinj.segment(k, i + 1 - k) * alphayj.head(i + 1 -k)).sum();
                dpxdl(i, k) = dt * (bhol * doto + bhxl * dotx + bhyl * doty);
                dpxdr(i, k) = dt * (bhor * doto + bhxr * dotx + bhyr * doty);

                doto = ((vxsj.segment(k, i + 1 - k) - vysj.segment(k, i + 1 - k)) * betaj.head(i + 1 -k)).sum();
                dotx = (sinj.segment(k, i + 1 - k) * alphaxj.head(i + 1 -k)).sum();
                doty = (cosj.segment(k, i + 1 - k) * alphayj.head(i + 1 -k)).sum();
                dpydl(i, k) = dt * (bhol * doto + bhxl * dotx + bhyl * doty);
                dpydr(i, k) = dt * (bhor * doto + bhxr * dotx + bhyr * doty);


            }
        }
    }
};

int main(int argc, char** argv) {
    cout << "gradients" << std::endl;
    const double dt = 0.05;
    const int n = 50;
    const Array3d pose0s = Array3d::Zero();
    const Array3d twist0s = Array3d::Zero();
    Path path(dt);

    const ArrayXd lefts = ArrayXd::Ones(n);
    const ArrayXd rights = ArrayXd::Ones(n);

    auto start = std::chrono::steady_clock::now();
    path.pose(lefts, rights, pose0s, twist0s);
    auto end = std::chrono::steady_clock::now();

    cout << "  vxj" << path.vxj.format(path.CommaInitFmt) << "\n";
    cout << "  vyj" << path.vyj.format(path.CommaInitFmt) << "\n";
    cout << "  omj" << path.omegaj.format(path.CommaInitFmt) << "\n";

    cout << " thetaj" << path.thetaj.format(path.CommaInitFmt) << "\n";
    cout << " pxj" << path.pxj.format(path.CommaInitFmt) << "\n";
    cout << " pyj" << path.pyj.format(path.CommaInitFmt) << "\n";
    std::chrono::duration<double> elapsed_seconds = end-start;
    std::cout << "elapsed time: " << elapsed_seconds.count() << "s\n";

    return 0; 
}
