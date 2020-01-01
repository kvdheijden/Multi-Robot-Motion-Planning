#include <iostream>
#include <boost/filesystem.hpp>

#include "free_space.h"

int main(int argc, char *argv[]) {
    (void) argc;
    (void) argv;

    constexpr int M[] = {100}; // # of robots
    constexpr int Z[] = {4, 9, 19, 29, 39, 49, 59, 69, 79, 89, 99}; // # of zig-zags, Workspace complexity = 6 + z * 6
    const boost::filesystem::path base("/home/koen/Documents/datasets/zigzag");
    constexpr double separation = 4.;

    for (int m : M) {
        for (int z : Z) {
            int w = static_cast<int>(std::ceil(std::sqrt(m)));
            double wz = separation * w - 2.5;

            double x = separation / 2.;
            double y = separation / 2.;

            int n = 6 + z * 6;
            const boost::filesystem::path path = base / ("n" + std::to_string(n) + "_m" + std::to_string(m));
            boost::filesystem::create_directories(path);

            std::cout << "Creating " << path.filename() << std::endl;

            for (int i = 0; i < m; i++) {
                Point s(x, y);
                Point t(x, y - (10 * z) - (separation * w));

                boost::filesystem::ofstream(path / ("s_" + std::to_string(i) + ".txt")) << s;
                boost::filesystem::ofstream(path / ("t_" + std::to_string(i) + ".txt")) << t;

                x += separation;
                if (x > separation * w) {
                    x = separation / 2.;
                    y += separation;
                }
            }

            std::vector<Point> points;
            x = 0, y = 0;
            for (int i = 0; i < z; i++) {
                points.emplace_back(x, y);
                x += wz;
                y -= 2.5;
                points.emplace_back(x, y);
                x -= wz;
                y -= 2.5;
                points.emplace_back(x, y);
                y -= 5.0;
            }

            points.emplace_back(x, y);
            y -= separation * w;
            points.emplace_back(x, y);
            x += separation * w;
            points.emplace_back(x, y);
            y += separation * w;

            for (int i = 0; i < z; i++) {
                points.emplace_back(x, y);
                x -= wz;
                y += 2.5;
                points.emplace_back(x, y);
                x += wz;
                y += 2.5;
                points.emplace_back(x, y);
                y += 5.0;
            }

            points.emplace_back(x, y);
            y += separation * w;
            points.emplace_back(x, y);
            x -= separation * w;
            points.emplace_back(x, y);
            y -= separation * w;

            if (x != 0 || y != 0) {
                std::cout << "Eindpoint mismatch!" << std::endl;
                return 1;
            }

            Workspace W(points.begin(), points.end());
            boost::filesystem::ofstream(path / "w.txt") << W;

            FreeSpace F;
            generate_free_space(W, F);
            if (F.size() != 1) {
                std::cout << "Free space mismatch!" << std::endl;
                return 1;
            }
            boost::filesystem::ofstream(path / "F_0.txt") << F[0];
        }
    }


    return 0;
}
