#ifndef MULTI_ROBOT_MOTION_PLANNING_EDGE_WEIGHT_H
#define MULTI_ROBOT_MOTION_PLANNING_EDGE_WEIGHT_H

#include "InterferenceForest.h"

#define EDGE_WEIGHT_GEODESIC

constexpr enum edge_weight_fcn {
    CONSTANT,
    EUCLIDEAN,
    EUCLIDEAN_SQUARED,
    GEODESIC
} edgeWeightFcn =
#if defined(EDGE_WEIGHT_EUCLIDEAN)
        EUCLIDEAN
#elif defined(EDGE_WEIGHT_EUCLIDEAN_SQUARED)
        EUCLIDEAN_SQUARED
#elif defined(EDGE_WEIGHT_GEODESIC)
        GEODESIC
#else
        CONSTANT
#endif
;

void edge_weight(MotionGraph &G_i, const Polygon &F_i);
void edge_weight(InterferenceForest &G);

#endif //MULTI_ROBOT_MOTION_PLANNING_EDGE_WEIGHT_H
