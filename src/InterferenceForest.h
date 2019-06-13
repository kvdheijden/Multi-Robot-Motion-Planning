#ifndef MULTI_ROBOT_MOTION_PLANNING_INTERFERENCEFOREST_H
#define MULTI_ROBOT_MOTION_PLANNING_INTERFERENCEFOREST_H

#include <boost/graph/directed_graph.hpp>
#include "MotionGraph.h"

struct InterferenceForestVertex {
    MotionGraph motionGraph;
    const Polygon *freeSpaceComponent;
    bool visited;

    int index;

    InterferenceForestVertex &operator=(const InterferenceForestVertex &that) {
        if (this != &that) {
            motionGraph = that.motionGraph;
            freeSpaceComponent = that.freeSpaceComponent;
            visited = that.visited;
            index = that.index;
        }
        return *this;
    }
};

static_assert(std::is_default_constructible<InterferenceForestVertex>(), "");
static_assert(std::is_assignable<InterferenceForestVertex, InterferenceForestVertex>(), "");
static_assert(std::is_copy_constructible<InterferenceForestVertex>(), "");

typedef boost::directed_graph<InterferenceForestVertex> InterferenceForest;
typedef boost::graph_traits<InterferenceForest>::vertex_descriptor InterferenceForestVertexDescriptor;
typedef boost::graph_traits<InterferenceForest>::edge_descriptor InterferenceForestEdgeDescriptor;

#endif //MULTI_ROBOT_MOTION_PLANNING_INTERFERENCEFOREST_H
