#include "../test/benchmark.h"

static constexpr int NUM_POINTS = 26;
static constexpr int NUM_WORKSPACE_POINTS = 28;

int main(int argc, char *argv[]) {
    std::vector<Point> S;
    std::vector<Point> T;

    for (double x = 2.0; x <= 50.0; x += 4.0) {
        for (double y = 2.0; y <= 6.0; y += 4.0) {
            S.emplace_back(Point(x, y));
            T.emplace_back(Point(52.0 - x, 38.0 - y));
        }
    }

    Point W[NUM_WORKSPACE_POINTS] = {
            Point(0.0, 0.0),
            Point(52.0, 0.0),
            Point(52.0, 8.0),
            Point(2.0, 8.0),
            Point(2.0, 10.0),
            Point(52.0, 10.0),
            Point(52.0, 16.0),
            Point(2.0, 16.0),
            Point(2.0, 18.0),
            Point(52.0, 18.0),
            Point(52.0, 24.0),
            Point(2.0, 24.0),
            Point(2.0, 26.0),
            Point(52.0, 26.0),
            Point(52.0, 38.0),
            Point(0.0, 38.0),
            Point(0.0, 30.0),
            Point(50.0, 30.0),
            Point(50.0, 28.0),
            Point(0.0, 28.0),
            Point(0.0, 22.0),
            Point(50.0, 22.0),
            Point(50.0, 20.0),
            Point(0.0, 20.0),
            Point(0.0, 14.0),
            Point(50.0, 14.0),
            Point(50.0, 12.0),
            Point(0.0, 12.0),
    };

    benchmark(Workspace(W, W + NUM_WORKSPACE_POINTS), ConfigurationSet(Sv, Tv), std::string("benchmark_small_corridor"));
}