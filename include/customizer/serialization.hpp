#ifndef OSRM_CUSTOMIZER_SERIALIZATION_HPP
#define OSRM_CUSTOMIZER_SERIALIZATION_HPP

#include "customizer/edge_based_graph.hpp"

#include "partitioner/cell_storage.hpp"
#include "protobuf/graph.pb.h"
#include "protobuf/metric.pb.h"
#include "storage/serialization.hpp"
#include "storage/shared_memory_ownership.hpp"
#include "storage/tar.hpp"
#include "util/log.hpp"

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

template <storage::Ownership Ownership>
inline void writePB(const std::string &name, const detail::CellMetricImpl<Ownership> &metric)
{
    pbmldm::Metric pb_metric;
    for (auto index : util::irange<std::size_t>(0, metric.weights.size()))
    {
        pb_metric.add_weights(metric.weights[index]);
    }

    std::fstream pb_output(name + ".pb", std::ios::out | std::ios::binary);
    pb_metric.SerializeToOstream(&pb_output);
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
}

template <typename EdgeDataT, storage::Ownership Ownership>
inline void writePB(const std::string &name, const MultiLevelGraph<EdgeDataT, Ownership> &graph)
{
    pbmldg::MLDQueryGraph pb_graph;
    for (auto index : util::irange<std::size_t>(0, graph.node_array.size()))
    {
        pb_graph.add_nodes(graph.node_array[index].first_edge);
    }
    for (auto index : util::irange<std::size_t>(0, graph.edge_array.size()))
    {
        pbmldg::EdgeData *pb_edge_data = pb_graph.add_edges();
        pb_edge_data->set_target(graph.edge_array[index].target);
        pb_edge_data->set_turnid(graph.edge_array[index].data.turn_id);
    }

    for (auto index : util::irange<std::size_t>(0, graph.is_forward_edge.size()))
    {
        pb_graph.add_isforward(graph.is_forward_edge[index]);
    }
    for (auto index : util::irange<std::size_t>(0, graph.is_backward_edge.size()))
    {
        pb_graph.add_isbackward(graph.is_backward_edge[index]);
    }
    for (auto index : util::irange<std::size_t>(0, graph.node_to_edge_offset.size()))
    {
        pb_graph.add_nodeleveloffset(graph.node_to_edge_offset[index]);
    }

    for (auto index : util::irange<std::size_t>(0, graph.node_to_edge_offset.size()))
    {
        pb_graph.add_nodeweights(graph.node_weights[index]);
    }

    std::fstream pb_output(name + ".pb", std::ios::out | std::ios::binary);
    pb_graph.SerializeToOstream(&pb_output);
}
} // namespace serialization
} // namespace customizer
} // namespace osrm

#endif
