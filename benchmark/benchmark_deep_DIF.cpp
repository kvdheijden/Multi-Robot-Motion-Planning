#include "../test/benchmark.h"
#include "../test/OffWriter.h"

static constexpr int NUM_POINTS = 7;
static constexpr int NUM_WORKSPACE_POINTS = 68;

std::string cwd() {
    constexpr int MAX_PATH_LEN = 256;
    char tmp[MAX_PATH_LEN];
    return (getcwd(tmp, MAX_PATH_LEN)) ? std::string(tmp) : std::string("");
}

int main(int argc, char *argv[]) {
    Point S[NUM_POINTS] = {
            Point(-3.6, 8.0),
            Point(3.6, 4.0),
            Point(7.6, 4.0),
            Point(2.0, 0.0),
            Point(-7.6, -4.0),
            Point(-3.6, -4.0),
            Point(3.6, -8.0)
    };

    Point T[NUM_POINTS] = {
            Point(3.6, 8.0),
            Point(-7.6, 4.0),
            Point(-3.6, 4.0),
            Point(-2.0, 0.0),
            Point(3.6, -4.0),
            Point(7.6, -4.0),
            Point(-3.6, -8.0)
    };

    Point W[NUM_WORKSPACE_POINTS] = {
            // TR
            Point(4.6, 4.4),
            Point(4.4, 4.6),
            Point(4.6, 4.6),
            Point(4.8, 4.4),
            Point(4.8, 3.0),
            Point(8.6, 3.0),
            Point(8.6, 5.0),
            Point(6.8, 5.0),
            Point(6.8, 6.6),
            Point(4.6, 6.6),
            Point(4.6, 9.0),
            Point(2.6, 9.0),
            Point(2.6, 4.6),
            Point(2.8, 4.6),
            Point(2.6, 4.4),
            Point(2.6, 0.8),
            Point(2.4, 1.0),

            // TL
            Point(-2.4, 1.0),
            Point(-2.6, 0.8),
            Point(-2.6, 4.4),
            Point(-2.8, 4.6),
            Point(-2.6, 4.6),
            Point(-2.6, 9.0),
            Point(-4.6, 9.0),
            Point(-4.6, 6.6),
            Point(-6.8, 6.6),
            Point(-6.8, 5.0),
            Point(-8.6, 5.0),
            Point(-8.6, 3.0),
            Point(-4.8, 3.0),
            Point(-4.8, 4.4),
            Point(-4.6, 4.6),
            Point(-4.4, 4.6),
            Point(-4.6, 4.4),

            // BL
            Point(-4.6, -4.4),
            Point(-4.4, -4.6),
            Point(-4.6, -4.6),
            Point(-4.8, -4.4),
            Point(-4.8, -3.0),
            Point(-8.6, -3.0),
            Point(-8.6, -5.0),
            Point(-6.8, -5.0),
            Point(-6.8, -6.6),
            Point(-4.6, -6.6),
            Point(-4.6, -9.0),
            Point(-2.6, -9.0),
            Point(-2.6, -4.6),
            Point(-2.8, -4.6),
            Point(-2.6, -4.4),
            Point(-2.6, -0.8),
            Point(-2.4, -1.0),

            // BR
            Point(2.4, -1.0),
            Point(2.6, -0.8),
            Point(2.6, -4.4),
            Point(2.8, -4.6),
            Point(2.6, -4.6),
            Point(2.6, -9.0),
            Point(4.6, -9.0),
            Point(4.6, -6.6),
            Point(6.8, -6.6),
            Point(6.8, -5.0),
            Point(8.6, -5.0),
            Point(8.6, -3.0),
            Point(4.8, -3.0),
            Point(4.8, -4.4),
            Point(4.6, -4.6),
            Point(4.4, -4.6),
            Point(4.6, -4.4)
    };

    std::vector<Point> Sv(S, S + NUM_POINTS);
    std::vector<Point> Tv(T, T + NUM_POINTS);

    benchmark(Workspace(W, W + NUM_WORKSPACE_POINTS), ConfigurationSet(Sv, Tv), std::string("benchmark_deep_DIF"));
}