#include "../test/benchmark.h"

#include <vector>
#include <cmath>

//static constexpr int NUM_POINTS = 400;
static constexpr int NUM_WORKSPACE_POINTS = 4;

int main(int argc, char *argv[]) {
    for (int n = 2; n <= 40; n += 2) {
        int NUM_POINTS = (n * n) / 2;

        std::vector<Point> Sv;
        std::vector<Point> Tv;

        int w = ceil(sqrt(2.0 * NUM_POINTS));
        int dim = (w % 2 == 0) ? 4 * (w / 2) - 2 : 4 * (w - 1) / 2;

        int x = -dim, y = -dim;
        for (int i = 0; i < NUM_POINTS; i++) {
            Sv.emplace_back(x, y);
            Tv.emplace_back(-x, -y);

            x += 4;
            if (x > dim) {
                x = -dim;
                y += 4;
            }
        }

        int dim2 = dim + 2;
        Point W[NUM_WORKSPACE_POINTS] = {
                Point(-dim2, -dim2),
                Point(dim2, -dim2),
                Point(dim2, dim2),
                Point(-dim2, dim2)
        };

        std::cout << "N = " << NUM_POINTS << std::endl;
        benchmark(Workspace(W, W + NUM_WORKSPACE_POINTS), ConfigurationSet(Sv, Tv), std::string("benchmark_generated_grid_") + std::to_string(NUM_POINTS));
        std::cout << std::endl;
    }
}