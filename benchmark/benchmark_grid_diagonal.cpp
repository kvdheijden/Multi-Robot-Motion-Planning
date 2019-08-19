#include "../test/benchmark.h"

static constexpr int NUM_POINTS = 12;
static constexpr int NUM_WORKSPACE_POINTS = 4;

int main(int argc, char *argv[]) {
    std::vector<Point> S(
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
    std::vector<Point> T(
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

    Point W[NUM_WORKSPACE_POINTS] = {
            Point(0, -17.0),
            Point(17.0, 0.0),
            Point(0.0, 17.0),
            Point(-17.0, 0.0)
    };

    benchmark(Workspace(W, W + NUM_WORKSPACE_POINTS), ConfigurationSet(Sv, Tv), std::string("benchmark_grid_diagonal"));
}