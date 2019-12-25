#ifndef MULTI_ROBOT_MOTION_PLANNING_MOTIONGRAPH_H
#define MULTI_ROBOT_MOTION_PLANNING_MOTIONGRAPH_H

#include <boost/graph/undirected_graph.hpp>
#include "Configuration.h"

enum Color {
    RED,
    BLUE,
    PURPLE
};

struct MotionGraphVertex {
    const Configuration *configuration;
    bool hasPebble;
    int value;
    Color color;

    bool visited;
    typename boost::graph_traits<boost::undirected_graph<>>::vertex_descriptor predecessor;

    MotionGraphVertex &operator=(const MotionGraphVertex &that) {
        if (this != &that) {
            configuration = that.configuration;
            hasPebble = that.hasPebble;
            visited = that.visited;
            predecessor = that.predecessor;
            value = that.value;
        }
        return *this;
    }

    bool operator==(const MotionGraphVertex &that) const {
        return *(this->configuration) == *(that.configuration);
    }

    bool operator!=(const MotionGraphVertex &that) const {
        return *(this->configuration) != *(that.configuration);
    }
};

static_assert(std::is_default_constructible<MotionGraphVertex>(), "");
static_assert(std::is_assignable<MotionGraphVertex, MotionGraphVertex>(), "");
static_assert(std::is_copy_constructible<MotionGraphVertex>(), "");

typedef boost::property<boost::edge_weight_t, Kernel::FT> MotionGraphEdge;
typedef boost::undirected_graph<MotionGraphVertex, MotionGraphEdge> MotionGraph;

typedef std::pair<std::reference_wrapper<const Configuration>, std::reference_wrapper<const Configuration>> Move;

typedef boost::graph_traits<MotionGraph>::vertex_descriptor MotionGraphVertexDescriptor;
typedef boost::graph_traits<MotionGraph>::edge_descriptor MotionGraphEdgeDescriptor;

typedef boost::graph_traits<MotionGraph>::vertex_iterator MotionGraphVertexIterator;
typedef boost::graph_traits<MotionGraph>::edge_iterator MotionGraphEdgeIterator;

#endif //MULTI_ROBOT_MOTION_PLANNING_MOTIONGRAPH_H
