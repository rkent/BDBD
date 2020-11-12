#include <iostream>
#include <eigen3/Eigen/Dense>
#include <chrono>
#include <math.h>
#include <vector>
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
    // 1st derivatives
    ArrayXXd dpxdl, dpxdr, dpydl, dpydr;
    // 2nd derivatives
    vector<vector<ArrayXd>> d2pxdldl;
    vector<vector<ArrayXd>> d2pxdldr;
    vector<vector<ArrayXd>> d2pxdrdr;
    vector<vector<ArrayXd>> d2pydldl;
    vector<vector<ArrayXd>> d2pydldr;
    vector<vector<ArrayXd>> d2pydrdr;
    
    double dt;

    ArrayXd vxj, vyj, omegaj, pxj, pyj, thetaj;

    IOFormat CommaInitFmt = IOFormat(6, Eigen::DontAlignCols, ", ", ", ", "", "", "[ ", " ]");
    IOFormat HeavyFmt = IOFormat(10, 0, ", ", " ", "\n[", "]", "[", "]");

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
        for (int i = 1; i < n; i++) {
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
            pyj(i) = pyj(i-1) + dt * vywj(i);
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

                doto = ((vxcj.segment(k, i + 1 - k) - vysj.segment(k, i + 1 - k)) * betaj.head(i + 1 -k)).sum();
                dotx = (sinj.segment(k, i + 1 - k) * alphaxj.head(i + 1 -k)).sum();
                doty = (cosj.segment(k, i + 1 - k) * alphayj.head(i + 1 -k)).sum();
                dpydl(i, k) = dt * (bhol * doto + bhxl * dotx + bhyl * doty);
                dpydr(i, k) = dt * (bhor * doto + bhxr * dotx + bhyr * doty);


            }
        }

        // Hessians
        for (int j = 0; j < n; j++) {
            d2pxdldl.push_back(vector<ArrayXd>());
            d2pxdldr.push_back(vector<ArrayXd>());
            d2pxdrdr.push_back(vector<ArrayXd>());
            d2pydldl.push_back(vector<ArrayXd>());
            d2pydldr.push_back(vector<ArrayXd>());
            d2pydrdr.push_back(vector<ArrayXd>());
            for (int k = 0; k < n; k++) {
                d2pxdldl.back().push_back(ArrayXd::Zero(n));
                d2pxdldr.back().push_back(ArrayXd::Zero(n));
                d2pxdrdr.back().push_back(ArrayXd::Zero(n));
                d2pydldl.back().push_back(ArrayXd::Zero(n));
                d2pydldr.back().push_back(ArrayXd::Zero(n));
                d2pydrdr.back().push_back(ArrayXd::Zero(n));
            }
        }
        for (int j = 1; j < n; j++) {
            // cout << "j " << j << '\n';
            double vxwdt = vxwj[j] * dt;
            double vywdt = vywj[j] * dt;
            double sdt = sinj[j] * dt;
            double cdt = cosj[j] * dt;

            for (int k = 1; k <= j; k++) {
                // cout << "k " << k << '\n';
                double betaljk = betaj[j-k] * bhol;
                double betarjk = betaj[j-k] * bhor;
                double alphaxljk = alphaxj[j-k] * bhxl;
                double alphaxrjk = alphaxj[j-k] * bhxr;
                double alphayljk = alphayj[j-k] * bhyl;
                double alphayrjk = alphayj[j-k] * bhyr;
                for (int m = 1; m <= j; m++) {
                    // cout << "m " << m << " j-m " << j-m << "\n";
                    double betaljm = betaj[j-m] * bhol;
                    double betarjm = betaj[j-m] * bhor;
                    double alphaxljm = alphaxj[j-m] * bhxl;
                    double alphaxrjm = alphaxj[j-m] * bhxr;
                    double alphayljm = alphaxj[j-m] * bhyl;
                    double alphayrjm = alphaxj[j-m] * bhyr;

                    double sumxll = (
                        -vxwdt * betaljk * betaljm
                        +sdt * (-betaljk * alphaxljm -alphaxljk * betaljm)
                        +cdt * (-betaljk * alphayljm -alphayljk * betaljm)
                    );
                    double sumxlr = (
                        -vxwdt * betaljk * betarjm
                        +sdt * (-betaljk * alphaxrjm -alphaxljk * betarjm)
                        +cdt * (-betaljk * alphayrjm -alphayljk * betarjm)
                    );
                    double sumxrr = (
                        -vxwdt * betarjk * betarjm
                        +sdt * (-betarjk * alphaxrjm -alphaxrjk * betarjm)
                        +cdt * (-betarjk * alphayrjm -alphayrjk * betarjm)
                    );
                    double sumyll = (
                        -vywdt * betaljk * betaljm
                        +sdt * (-betaljk * alphayljm -alphayljk * betaljm)
                        +cdt * (betaljk * alphayljm +alphayljk * betaljm)
                    );
                    double sumylr = (
                        -vywdt * betaljk * betarjm
                        +sdt * (-betaljk * alphayrjm -alphayljk * betarjm)
                        +cdt * (betaljk * alphayrjm +alphayljk * betarjm)
                    );
                    double sumyrr = (
                        -vywdt * betarjk * betarjm
                        +sdt * (-betarjk * alphayrjm -alphayrjk * betarjm)
                        +cdt * (betarjk * alphayrjm +alphayrjk * betarjm)
                    );

                    for (int i = j; i < n; i++) {
                        // cout << "i " << i << '\n';
                        d2pxdldl[k][m][i] += sumxll;
                        d2pxdldr[k][m][i] += sumxlr;
                        d2pxdrdr[k][m][i] += sumxrr;
                        d2pydldl[k][m][i] += sumyll;
                        d2pydldr[k][m][i] += sumylr;
                        d2pydrdr[k][m][i] += sumyrr;
                    }
                }
            }
        }

    }
};

int main(int argc, char** argv) {
    std::cout.setf(std::ios::unitbuf);
    cout << "gradients" << std::endl;
    const double dt = 0.05;
    const int n = 4;
    const Array3d pose0s = Array3d::Zero();
    const Array3d twist0s = Array3d::Zero();
    Path path(dt);

    const ArrayXd lefts = ArrayXd::Ones(n);
    const ArrayXd rights = ArrayXd::Ones(n);
    auto start = std::chrono::steady_clock::now();

    for (int i = 0; i < 10; i++)
        path.pose(lefts, rights, pose0s, twist0s);
    auto end = std::chrono::steady_clock::now();

    /* */
    cout << "  vxj" << path.vxj.format(path.CommaInitFmt) << "\n";
    cout << "  vyj" << path.vyj.format(path.CommaInitFmt) << "\n";
    cout << "  omj" << path.omegaj.format(path.CommaInitFmt) << "\n";

    cout << " thetaj" << path.thetaj.format(path.CommaInitFmt) << "\n";
    cout << " pxj" << path.pxj.format(path.CommaInitFmt) << "\n";
    cout << " pyj" << path.pyj.format(path.CommaInitFmt) << "\n";
    cout << " dpxdl" << path.dpxdl.format(path.HeavyFmt) << "\n";
    cout << " dpxdr" << path.dpxdr.format(path.HeavyFmt) << "\n";
    cout << " dpydl" << path.dpydl.format(path.HeavyFmt) << "\n";
    cout << " dpydr" << path.dpydr.format(path.HeavyFmt) << "\n";

    for (int j = 0; j < n; j++) {
        for (int k = 0; k < n; k++) {
            cout << " d2pxdldr[" << j << "][" << k << "]" << path.d2pxdldr[j][k].format(path.CommaInitFmt) << '\n'; 

        }
    }
    /* */
    std::chrono::duration<double> elapsed_seconds = end-start;
    std::cout << "elapsed time: " << elapsed_seconds.count() / 10.0 << "s\n";

    return 0; 
}
