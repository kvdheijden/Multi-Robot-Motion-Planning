#ifndef MULTI_ROBOT_MOTION_PLANNING_INTERFERENCEFOREST_H
#define MULTI_ROBOT_MOTION_PLANNING_INTERFERENCEFOREST_H

#include <boost/graph/directed_graph.hpp>
#include "MotionGraph.h"

struct InterferenceForestVertex {
    MotionGraph motionGraph;
    const Polygon *freeSpaceComponent = nullptr;
};

typedef boost::directed_graph<InterferenceForestVertex>                          InterferenceForest;
typedef boost::graph_traits<InterferenceForest>::vertex_descriptor  InterferenceForestVertexDescriptor;

#endif //MULTI_ROBOT_MOTION_PLANNING_INTERFERENCEFOREST_H
