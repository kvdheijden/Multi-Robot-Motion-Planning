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

struct IntervisibilityGraphEdge {
    Kernel::RT edge_weight;
};

typedef boost::undirected_graph<IntervisibilityGraphVertex, IntervisibilityGraphEdge> IntervisibilityGraph;

typedef boost::graph_traits<IntervisibilityGraph>::vertex_descriptor IntervisibilityGraphVertexDescriptor;
typedef boost::graph_traits<IntervisibilityGraph>::edge_descriptor IntervisibilityGraphEdgeDescriptor;

typedef boost::graph_traits<IntervisibilityGraph>::vertex_iterator IntervisibilityGraphVertexIterator;

#endif //MULTI_ROBOT_MOTION_PLANNING_INTERVISIBILITYGRAPH_H
