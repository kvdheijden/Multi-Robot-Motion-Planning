#include "free_space.h"

#include <CGAL/approximated_offset_2.h>

void generate_free_space(const Workspace &W, FreeSpace &F) {
    double eps = 0.0001; //std::numeric_limits<double>::epsilon();
    Kernel::FT r(1 - eps);

    F.clear();
    CGAL::approximated_inset_2(W, r, eps, std::back_inserter(F));

    for (Polygon &f : F) {
        if (f.orientation() != CGAL::Orientation::COUNTERCLOCKWISE) {
            f.reverse_orientation();
        }
        CGAL_assertion(f.orientation() == CGAL::Orientation::COUNTERCLOCKWISE);
    }
}