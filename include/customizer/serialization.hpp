#ifndef OSRM_CUSTOMIZER_SERIALIZATION_HPP
#define OSRM_CUSTOMIZER_SERIALIZATION_HPP

#include "customizer/edge_based_graph.hpp"

#include "partitioner/cell_storage.hpp"

#include "storage/serialization.hpp"
#include "storage/shared_memory_ownership.hpp"
#include "storage/tar.hpp"

#include "../../../src/protobuf/mld.pb.h"


namespace osrm
{
namespace customizer
{
namespace serialization
{

template <storage::Ownership Ownership>
inline void read(storage::tar::FileReader &reader,
                 const std::string &name,
                 detail::CellMetricImpl<Ownership> &metric)
{
    storage::serialization::read(reader, name + "/weights", metric.weights);
    storage::serialization::read(reader, name + "/durations", metric.durations);
}

template <storage::Ownership Ownership>
inline void write(storage::tar::FileWriter &writer,
                  const std::string &name,
                  const detail::CellMetricImpl<Ownership> &metric)
{
    storage::serialization::write(writer, name + "/weights", metric.weights);
    storage::serialization::write(writer, name + "/durations", metric.durations);
}

template <typename EdgeDataT, storage::Ownership Ownership>
inline void read(storage::tar::FileReader &reader,
                 const std::string &name,
                 MultiLevelGraph<EdgeDataT, Ownership> &graph)
{
    storage::serialization::read(reader, name + "/node_array", graph.node_array);
    storage::serialization::read(reader, name + "/node_weights", graph.node_weights);
    storage::serialization::read(reader, name + "/node_durations", graph.node_durations);
    storage::serialization::read(reader, name + "/edge_array", graph.edge_array);
    storage::serialization::read(reader, name + "/is_forward_edge", graph.is_forward_edge);
    storage::serialization::read(reader, name + "/is_backward_edge", graph.is_backward_edge);
    storage::serialization::read(reader, name + "/node_to_edge_offset", graph.node_to_edge_offset);
}

template <typename EdgeDataT, storage::Ownership Ownership>
inline void write(storage::tar::FileWriter &writer,
                  const std::string &name,
                  const MultiLevelGraph<EdgeDataT, Ownership> &graph)
{
    storage::serialization::write(writer, name + "/node_array", graph.node_array);
    storage::serialization::write(writer, name + "/node_weights", graph.node_weights);
    storage::serialization::write(writer, name + "/node_durations", graph.node_durations);
    storage::serialization::write(writer, name + "/edge_array", graph.edge_array);
    storage::serialization::write(writer, name + "/is_forward_edge", graph.is_forward_edge);
    storage::serialization::write(writer, name + "/is_backward_edge", graph.is_backward_edge);
    storage::serialization::write(writer, name + "/node_to_edge_offset", graph.node_to_edge_offset);

     std::cout << "#### query graph: node array: " << graph.node_array.size()
            << " node_weights: " << graph.node_weights.size()
            << " edge_array: " << graph.edge_array.size()
            << " is_forward_edge: " << graph.is_forward_edge.size()
            << " is_backward_edge: " << graph.is_backward_edge.size()
            << " node_to_edge_offset: " << graph.node_to_edge_offset.size()
            << " num levels: " << (int)graph.node_to_edge_offset.back()
            << std::endl;
     pbmld::QueryGraph pb_graph;

     for (auto i : graph.node_array){
        pb_graph.add_nodes(i.first_edge);
     }

     int index  = 0;
     for (auto i : graph.edge_array){
        auto pb_edge = pb_graph.add_edges();
        pb_edge->set_target(i.target);
        pb_edge->set_turn_id(i.data.turn_id);
        pb_edge->set_is_forward(graph.is_forward_edge[index]);
        pb_edge->set_is_backward(graph.is_backward_edge[index]);
        index++;
     }

     for (auto i : graph.node_to_edge_offset){
        pb_graph.add_node_level_offset((unsigned int)i);
     }

     for (auto i : graph.node_weights){
        pb_graph.add_node_weights(i);
     }

     std::fstream pb_out("1.mld.graph.pb", std::ios::out | std::ios::binary);
     pb_graph.SerializeToOstream(&pb_out);
}
}
}
}

#endif
