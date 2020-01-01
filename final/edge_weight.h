#ifndef MULTI_ROBOT_MOTION_PLANNING_EDGE_WEIGHT_H
#define MULTI_ROBOT_MOTION_PLANNING_EDGE_WEIGHT_H

#include "InterferenceForest.h"

enum edge_weight_fcn {
    CONSTANT,
    EUCLIDEAN,
    EUCLIDEAN_SQUARED,
    GEODESIC
};

void edge_weight(InterferenceForest &G, edge_weight_fcn e);

#endif //MULTI_ROBOT_MOTION_PLANNING_EDGE_WEIGHT_H
