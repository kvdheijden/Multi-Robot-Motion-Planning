#include "../test/benchmark.h"

#include <vector>
#include <cmath>

static constexpr int NUM_POINTS = 12;
static constexpr double FULL_CIRCLE = 2 * M_PI;
static constexpr double PERPENDICULAR_ANGLE = M_PI_2;

int main(int argc, char *argv[]) {
    for (int m = 3; m <= 100; m++) {
        std::vector<Point> Sv(
                {Point(0.0, 12.0),
                 Point(-3.0, 9.0),
                 Point(3.0, 9.0),
                 Point(-6.0, 6.0),
                 Point(0.0, 6.0),
                 Point(-9.0, 3.0),
                 Point(-3.0, 3.0),
                 Point(-12.0, 0.0),
                 Point(-6.0, 0.0),
                 Point(-9.0, -3.0),
                 Point(-3.0, -3.0),
                 Point(-6.0, -6.0)}
        );
        std::vector<Point> Tv(
                {Point(0.0, -12.0),
                 Point(3.0, -9.0),
                 Point(-3.0, -9.0),
                 Point(6.0, -6.0),
                 Point(0.0, -6.0),
                 Point(9.0, -3.0),
                 Point(3.0, -3.0),
                 Point(12.0, 0.0),
                 Point(6.0, 0.0),
                 Point(9.0, 3.0),
                 Point(3.0, 3.0),
                 Point(6.0, 6.0)}
        );
        double innerR = 13;

        double alpha = FULL_CIRCLE / m;
        double beta = PERPENDICULAR_ANGLE - (alpha / 2);
        double outerR = innerR / sin(beta);

        std::vector<Point> Wv;
        for (int i = 0; i < m; i++) {
            double angle = i * FULL_CIRCLE / m;
            double amp = (i % 2 == 0) ? outerR : 2. * outerR;
            Wv.emplace_back(amp * cos(angle), amp * sin(angle));
        }

        std::cout << "M = " << m << std::endl;
        benchmark(Workspace(Wv.begin(), Wv.end()), ConfigurationSet(Sv, Tv), std::string("benchmark_generated_workspace_") + std::to_string(m));
        std::cout << std::endl;
    }
}