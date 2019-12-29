#include <iostream>
#include <boost/filesystem.hpp>

#include "free_space.h"

int main(int argc, char *argv[]) {
    (void) argc;
    (void) argv;

    constexpr int M[] = {1, 2, 3, 5, 8, 10, 20, 30, 50, 80, 100, 200, 300, 500};
    constexpr int n = 4;

    constexpr double separation = 4.;

    const boost::filesystem::path base("/home/koen/Documents/datasets/grid");
    for (int m : M) {
        int side = std::ceil(std::sqrt(2 * m));
        Point points[n] = {
                {0,                 0},
                {separation * side, 0},
                {separation * side, separation * side},
                {0,                 separation * side}
        };

        const boost::filesystem::path path = base / ("n" + std::to_string(n) + "_m" + std::to_string(m));
        boost::filesystem::create_directories(path);

        Workspace W(points, points + n);
        FreeSpace F;
        generate_free_space(W, F);

        boost::filesystem::ofstream(path / "w.txt") << W;
        boost::filesystem::ofstream(path / "F_0.txt") << F[0];

        double x = separation / 2., y = separation / 2.;
        for (int i = 0; i < m; i++) {
            boost::filesystem::ofstream(path / ("s_" + std::to_string(i) + ".txt")) << Point(x, y);
            boost::filesystem::ofstream(path / ("t_" + std::to_string(i) + ".txt"))
                    << Point(separation * side - x, separation * side - y);
            x += separation;
            if (x >= separation * side) {
                x = separation / 2.;
                y += separation;
            }
        }
    }

    return 0;
}
