#ifndef MULTI_ROBOT_MOTION_PLANNING_MOTIONGRAPH_H
#define MULTI_ROBOT_MOTION_PLANNING_MOTIONGRAPH_H

#include <boost/graph/undirected_graph.hpp>
#include "Configuration.h"

typedef boost::undirected_graph<Configuration>              MotionGraph;
typedef boost::graph_traits<MotionGraph>::vertex_descriptor MotionGraphVertexDescriptor;

#endif //MULTI_ROBOT_MOTION_PLANNING_MOTIONGRAPH_H
