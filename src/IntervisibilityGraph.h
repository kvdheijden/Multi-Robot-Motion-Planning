#ifndef MULTI_ROBOT_MOTION_PLANNING_INTERVISIBILITYGRAPH_H
#define MULTI_ROBOT_MOTION_PLANNING_INTERVISIBILITYGRAPH_H

#include <boost/graph/undirected_graph.hpp>
#include "cgal_types.h"

struct IntervisibilityGraphVertex {
    const Arrangement::Point_2 *p;

    IntervisibilityGraphVertex &operator=(const IntervisibilityGraphVertex &that) {
        if (*this != that) {
            p = that.p;
        }
        return *this;
    }

    bool operator==(const IntervisibilityGraphVertex &that) const {
        return p->equals(*that.p);
    }

    bool operator!=(const IntervisibilityGraphVertex &that) const {
        return !p->equals(*that.p);
    }
};

typedef boost::undirected_graph<IntervisibilityGraphVertex, boost::property<boost::edge_weight_t, Kernel::RT>> IntervisibilityGraph;

typedef boost::graph_traits<IntervisibilityGraph>::vertex_descriptor IntervisibilityGraphVertexDescriptor;
typedef boost::graph_traits<IntervisibilityGraph>::edge_descriptor IntervisibilityGraphEdgeDescriptor;

typedef boost::graph_traits<IntervisibilityGraph>::vertex_iterator IntervisibilityGraphVertexIterator;
typedef boost::graph_traits<IntervisibilityGraph>::edge_iterator IntervisibilityGraphEdgeIterator;

#endif //MULTI_ROBOT_MOTION_PLANNING_INTERVISIBILITYGRAPH_H
