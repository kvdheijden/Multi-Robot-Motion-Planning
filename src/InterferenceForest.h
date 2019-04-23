#ifndef MULTI_ROBOT_MOTION_PLANNING_INTERFERENCEFOREST_H
#define MULTI_ROBOT_MOTION_PLANNING_INTERFERENCEFOREST_H

#include <boost/graph/directed_graph.hpp>
#include "MotionGraph.h"

struct InterferenceForestVertex {
    MotionGraph *motionGraph;
    const Polygon *freeSpaceComponent;
    bool visited;

    InterferenceForestVertex() :
            motionGraph(new MotionGraph()),
            freeSpaceComponent(nullptr),
            visited(false) {

    }

    InterferenceForestVertex(const InterferenceForestVertex &other) :
            motionGraph(new MotionGraph(*other.motionGraph)),
            freeSpaceComponent(other.freeSpaceComponent),
            visited(other.visited) {

    }

    InterferenceForestVertex &operator=(const InterferenceForestVertex &other) {
        if (this != &other) {
            delete motionGraph;
            motionGraph = new MotionGraph(*other.motionGraph);
            freeSpaceComponent = other.freeSpaceComponent;
            visited = other.visited;
        }
        return *this;
    }

    ~InterferenceForestVertex() {
        delete motionGraph;
    }
};

static_assert(std::is_default_constructible<InterferenceForestVertex>(), "");
static_assert(std::is_assignable<InterferenceForestVertex, InterferenceForestVertex>(), "");
static_assert(std::is_copy_constructible<InterferenceForestVertex>(), "");

typedef boost::directed_graph<InterferenceForestVertex> InterferenceForest;
typedef boost::graph_traits<InterferenceForest>::vertex_descriptor InterferenceForestVertexDescriptor;
typedef boost::graph_traits<InterferenceForest>::edge_descriptor InterferenceForestEdgeDescriptor;

#endif //MULTI_ROBOT_MOTION_PLANNING_INTERFERENCEFOREST_H
