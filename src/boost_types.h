#ifndef MULTI_ROBOT_MOTION_PLANNING_BOOST_TYPES_H
#define MULTI_ROBOT_MOTION_PLANNING_BOOST_TYPES_H

#include <boost/graph/undirected_graph.hpp>
#include <boost/graph/directed_graph.hpp>

class Vertex {
public:
    bool has_pebble;
    const bool is_start_config;
    const int id;

    Vertex(bool is_start_config, int id) : is_start_config(is_start_config), id(id), has_pebble(is_start_config)
    {

    }
};

typedef boost::undirected_graph<Vertex>     MotionGraph;
typedef boost::directed_graph<MotionGraph>  InterferenceForest;

typedef boost::graph_traits<MotionGraph>::vertex_descriptor         MotionGraphVertex_t;
typedef boost::graph_traits<InterferenceForest>::vertex_descriptor  InterferenceForestVertex_t;

#endif //MULTI_ROBOT_MOTION_PLANNING_BOOST_TYPES_H
