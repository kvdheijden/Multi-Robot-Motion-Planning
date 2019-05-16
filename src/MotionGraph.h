#ifndef MULTI_ROBOT_MOTION_PLANNING_MOTIONGRAPH_H
#define MULTI_ROBOT_MOTION_PLANNING_MOTIONGRAPH_H

#include <boost/graph/undirected_graph.hpp>
#include "Configuration.h"

struct MotionGraphVertex {
    const Configuration *configuration;
    bool hasPebble;

    bool visited;
    typename boost::graph_traits<boost::undirected_graph<>>::vertex_descriptor predecessor;

    bool operator==(const MotionGraphVertex &that) const {
        return *(this->configuration) == *(that.configuration);
    }
};

static_assert(std::is_default_constructible<MotionGraphVertex>(), "");
static_assert(std::is_assignable<MotionGraphVertex, MotionGraphVertex>(), "");
static_assert(std::is_copy_constructible<MotionGraphVertex>(), "");

typedef boost::property<boost::edge_weight_t, int> MotionGraphEdge;
typedef boost::undirected_graph<MotionGraphVertex, MotionGraphEdge> MotionGraph;

typedef std::pair<const Configuration *, const Configuration *> Move;

typedef boost::graph_traits<MotionGraph>::vertex_descriptor MotionGraphVertexDescriptor;
typedef boost::graph_traits<MotionGraph>::edge_descriptor MotionGraphEdgeDescriptor;

#endif //MULTI_ROBOT_MOTION_PLANNING_MOTIONGRAPH_H
