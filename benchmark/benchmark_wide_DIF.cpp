#include "../test/benchmark.h"
#include "../test/OffWriter.h"

static constexpr int NUM_POINTS = 11;
static constexpr int NUM_WORKSPACE_POINTS = 92;

std::string cwd() {
    constexpr int MAX_PATH_LEN = 256;
    char tmp[MAX_PATH_LEN];
    return (getcwd(tmp, MAX_PATH_LEN)) ? std::string(tmp) : std::string("");
}

int main(int argc, char *argv[]) {
    Point S[NUM_POINTS] = {
            Point(-12.0, 2.0),
            Point(-8.0, 2.0),
            Point(-8.0, -6.0),
            Point(-4.0, 2.0),
            Point(-2.2, 6.0),
            Point(0.0, 2.0),
            Point(2.2, -6.0),
            Point(4.0, 2.0),
            Point(8.0, 2.0),
            Point(8.0, 6.0),
            Point(12.0, -2.0)
    };

    Point T[NUM_POINTS] = {
            Point(-12.0, -2.0),
            Point(-8.0, -2.0),
            Point(-8.0, 6.0),
            Point(-4.0, -2.0),
            Point(-2.2, -6.0),
            Point(0.0, -2.0),
            Point(2.2, 6.0),
            Point(4.0, -2.0),
            Point(8.0, -2.0),
            Point(8.0, -6.0),
            Point(12.0, 2.0)
    };

    Point W[NUM_WORKSPACE_POINTS] = {
            // TR
            Point(8.8, 1.4),
            Point(9.0, 1.0),
            Point(13.0, 1.0),
            Point(13.0, 3.0),
            Point(10.0, 3.0),
            Point(9.0, 4.0),
            Point(9.0, 7.0),
            Point(7.0, 7.0),
            Point(7.0, 3.0),
            Point(7.4, 2.8),
            Point(4.6, 2.8),
            Point(4.6, 3.0),
            Point(3.4, 3.0),
            Point(3.2, 2.6),
            Point(2.8, 2.4),
            Point(1.2, 2.4),
            Point(0.8, 2.6),
            Point(2.6, 2.6),
            Point(3.0, 2.8),
            Point(3.2, 3.2),
            Point(3.2, 7.0),
            Point(1.2, 7.0),
            Point(1.2, 4.6),

            // TL
            Point(-1.2, 4.6),
            Point(-1.2, 7.0),
            Point(-3.2, 7.0),
            Point(-3.2, 3.2),
            Point(-3.0, 2.8),
            Point(-2.6, 2.6),
            Point(-0.8, 2.6),
            Point(-1.2, 2.4),
            Point(-2.8, 2.4),
            Point(-3.2, 2.6),
            Point(-3.4, 3.0),
            Point(-4.6, 3.0),
            Point(-4.6, 2.8),
            Point(-7.4, 2.8),
            Point(-7.0, 3.0),
            Point(-7.0, 7.0),
            Point(-9.0, 7.0),
            Point(-9.0, 4.0),
            Point(-10.0, 3.0),
            Point(-13.0, 3.0),
            Point(-13.0, 1.0),
            Point(-9.0, 1.0),
            Point(-8.8, 1.4),

            // BL
            Point(-8.8, -1.4),
            Point(-9.0, -1.0),
            Point(-13.0, -1.0),
            Point(-13.0, -3.0),
            Point(-10.0, -3.0),
            Point(-9.0, -4.0),
            Point(-9.0, -7.0),
            Point(-7.0, -7.0),
            Point(-7.0, -3.0),
            Point(-7.4, -2.8),
            Point(-4.6, -2.8),
            Point(-4.6, -3.0),
            Point(-3.4, -3.0),
            Point(-3.2, -2.6),
            Point(-2.8, -2.4),
            Point(-1.2, -2.4),
            Point(-0.8, -2.6),
            Point(-2.6, -2.6),
            Point(-3.0, -2.8),
            Point(-3.2, -3.2),
            Point(-3.2, -7.0),
            Point(-1.2, -7.0),
            Point(-1.2, -4.6),

            // BR
            Point(1.2, -4.6),
            Point(1.2, -7.0),
            Point(3.2, -7.0),
            Point(3.2, -3.2),
            Point(3.0, -2.8),
            Point(2.6, -2.6),
            Point(0.8, -2.6),
            Point(1.2, -2.4),
            Point(2.8, -2.4),
            Point(3.2, -2.6),
            Point(3.4, -3.0),
            Point(4.6, -3.0),
            Point(4.6, -2.8),
            Point(7.4, -2.8),
            Point(7.0, -3.0),
            Point(7.0, -7.0),
            Point(9.0, -7.0),
            Point(9.0, -4.0),
            Point(10.0, -3.0),
            Point(13.0, -3.0),
            Point(13.0, -1.0),
            Point(9.0, -1.0),
            Point(8.8, -1.4)
    };

    std::vector<Point> Sv(S, S + NUM_POINTS);
    std::vector<Point> Tv(T, T + NUM_POINTS);

    benchmark(Workspace(W, W + NUM_WORKSPACE_POINTS), ConfigurationSet(Sv, Tv), std::string("benchmark_wide_DIF"));
}