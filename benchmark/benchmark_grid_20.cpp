#include "../test/benchmark.h"

static constexpr int NUM_POINTS = 20;
static constexpr int NUM_WORKSPACE_POINTS = 4;

int main(int argc, char *argv[]) {
    std::vector<Point> Sv;
    std::vector<Point> Tv;

    double dimX = 14.0, dimY = 8.0, step = 4.0;
    double x = -dimX, y = -dimY;
    for (int i = 0; i < NUM_POINTS; i++) {
        Sv.emplace_back(x, y);
        Tv.emplace_back(-x, -y);

        x += step;
        if (x > dimX) {
            x = -dimX;
            y += step;
        }
    }

    Point W[NUM_WORKSPACE_POINTS] = {
            Point(-15.0, -9.0),
            Point(15.0, -9.0),
            Point(15.0, 9.0),
            Point(-15.0, 9.0)
    };

    benchmark(Workspace(W, W + NUM_WORKSPACE_POINTS), ConfigurationSet(Sv, Tv), std::string("benchmark_grid_20"));
}