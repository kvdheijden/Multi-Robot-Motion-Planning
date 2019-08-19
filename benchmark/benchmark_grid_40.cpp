#include "../test/benchmark.h"

static constexpr int NUM_POINTS = 40;
static constexpr int NUM_WORKSPACE_POINTS = 4;

int main(int argc, char *argv[]) {
    std::vector<Point> S;
    std::vector<Point> T;

    double dim = 16.0, step = 4.0;
    double x = -dim, y = -dim;
    for (int i = 0; i < NUM_POINTS; i++) {
        S.emplace_back(x, y);
        T.emplace_back(-x, -y);

        x += step;
        if (x > dim) {
            x = -dim;
            y += step;
        }
    }

    Point W[NUM_WORKSPACE_POINTS] = {
            Point(-18.0, -18.0),
            Point(18.0, -18.0),
            Point(18.0, 18.0),
            Point(-18.0, 18.0)
    };

    benchmark(Workspace(W, W + NUM_WORKSPACE_POINTS), ConfigurationSet(Sv, Tv), std::string("benchmark_grid_40"));
}