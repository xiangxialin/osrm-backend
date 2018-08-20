#ifndef OSRM_EXTRACTOR_IO_HPP
#define OSRM_EXTRACTOR_IO_HPP

#include "extractor/conditional_turn_penalty.hpp"
#include "extractor/datasources.hpp"
#include "extractor/edge_based_edge.hpp"
#include "extractor/intersection_bearings_container.hpp"
#include "extractor/maneuver_override.hpp"
#include "extractor/name_table.hpp"
#include "extractor/nbg_to_ebg.hpp"
#include "extractor/node_data_container.hpp"
#include "extractor/profile_properties.hpp"
#include "extractor/restriction.hpp"
#include "extractor/segment_data_container.hpp"

#include "storage/io.hpp"
#include "storage/serialization.hpp"

#include "util/deallocating_vector.hpp"

#include "../../../src/protobuf/node-based-graph.pb.h"
#include "../../../src/protobuf/edge-based-graph.pb.h"
#include "../../../src/protobuf/scc.pb.h"


#include <boost/assert.hpp>

namespace osrm
{
namespace extractor
{
namespace serialization
{

// read/write for bearing data
template <storage::Ownership Ownership>
inline void read(storage::tar::FileReader &reader,
                 const std::string &name,
                 detail::IntersectionBearingsContainer<Ownership> &intersection_bearings)
{
    storage::serialization::read(reader, name + "/bearing_values", intersection_bearings.values);
    storage::serialization::read(
        reader, name + "/node_to_class_id", intersection_bearings.node_to_class_id);
    util::serialization::read(
        reader, name + "/class_id_to_ranges", intersection_bearings.class_id_to_ranges_table);
}

template <storage::Ownership Ownership>
inline void write(storage::tar::FileWriter &writer,
                  const std::string &name,
                  const detail::IntersectionBearingsContainer<Ownership> &intersection_bearings)
{
    storage::serialization::write(writer, name + "/bearing_values", intersection_bearings.values);
    storage::serialization::write(
        writer, name + "/node_to_class_id", intersection_bearings.node_to_class_id);
    util::serialization::write(
        writer, name + "/class_id_to_ranges", intersection_bearings.class_id_to_ranges_table);
}

// read/write for properties file
inline void
read(storage::tar::FileReader &reader, const std::string &name, ProfileProperties &properties)
{
    reader.ReadInto(name, properties);
}

inline void write(storage::tar::FileWriter &writer,
                  const std::string &name,
                  const ProfileProperties &properties)
{
    writer.WriteElementCount64(name, 1);
    writer.WriteFrom(name, properties);
}

// read/write for datasources file
inline void read(storage::tar::FileReader &reader, const std::string &name, Datasources &sources)
{
    reader.ReadInto(name, sources);
}

inline void write(storage::tar::FileWriter &writer, const std::string &name, Datasources &sources)
{
    writer.WriteElementCount64(name, 1);
    writer.WriteFrom(name, sources);
}

// read/write for segment data file
template <storage::Ownership Ownership>
inline void read(storage::tar::FileReader &reader,
                 const std::string &name,
                 detail::SegmentDataContainerImpl<Ownership> &segment_data)
{
    storage::serialization::read(reader, name + "/index", segment_data.index);
    storage::serialization::read(reader, name + "/nodes", segment_data.nodes);
    util::serialization::read(reader, name + "/forward_weights", segment_data.fwd_weights);
    util::serialization::read(reader, name + "/reverse_weights", segment_data.rev_weights);
    util::serialization::read(reader, name + "/forward_durations", segment_data.fwd_durations);
    util::serialization::read(reader, name + "/reverse_durations", segment_data.rev_durations);
    storage::serialization::read(
        reader, name + "/forward_data_sources", segment_data.fwd_datasources);
    storage::serialization::read(
        reader, name + "/reverse_data_sources", segment_data.rev_datasources);
}

template <storage::Ownership Ownership>
inline void write(storage::tar::FileWriter &writer,
                  const std::string &name,
                  const detail::SegmentDataContainerImpl<Ownership> &segment_data)
{
    storage::serialization::write(writer, name + "/index", segment_data.index);
    storage::serialization::write(writer, name + "/nodes", segment_data.nodes);
    util::serialization::write(writer, name + "/forward_weights", segment_data.fwd_weights);
    util::serialization::write(writer, name + "/reverse_weights", segment_data.rev_weights);
    util::serialization::write(writer, name + "/forward_durations", segment_data.fwd_durations);
    util::serialization::write(writer, name + "/reverse_durations", segment_data.rev_durations);
    storage::serialization::write(
        writer, name + "/forward_data_sources", segment_data.fwd_datasources);
    storage::serialization::write(
        writer, name + "/reverse_data_sources", segment_data.rev_datasources);


    std::cout << "#### cnbg: " << segment_data.index.size() << ", "<< segment_data.nodes.size()
    << ", "<< segment_data.fwd_weights.size()<< ", "<< segment_data.rev_weights.size() << std::endl;

    pbnbg::CompressedNbg pb_cnbg;
    for (auto i : segment_data.index){
        pb_cnbg.add_index(i);
    }
    for (auto i : segment_data.nodes){
        pb_cnbg.add_nodes(i);
    }
    for (auto i : segment_data.fwd_weights){
        pb_cnbg.add_forward_weights(i);
    }
    for (auto i : segment_data.rev_weights){
        pb_cnbg.add_reverse_weights(i);
    }

    std::fstream pb_out("1.nbg.compressed.pb", std::ios::out | std::ios::binary);
    pb_cnbg.SerializeToOstream(&pb_out);

}

template <storage::Ownership Ownership>
inline void read(storage::tar::FileReader &reader,
                 const std::string &name,
                 detail::EdgeBasedNodeDataContainerImpl<Ownership> &node_data_container)
{
    // read actual data
    storage::serialization::read(reader, name + "/nodes", node_data_container.nodes);
    storage::serialization::read(
        reader, name + "/annotations", node_data_container.annotation_data);
}

template <storage::Ownership Ownership>
inline void write(storage::tar::FileWriter &writer,
                  const std::string &name,
                  const detail::EdgeBasedNodeDataContainerImpl<Ownership> &node_data_container)
{
    storage::serialization::write(writer, name + "/nodes", node_data_container.nodes);
    storage::serialization::write(
        writer, name + "/annotations", node_data_container.annotation_data);

    std::cout << "#### ebg nodes: " << node_data_container.nodes.size() << ", "
        << node_data_container.annotation_data.size() << std::endl;
    pbebg::EdgeBasedNodeContainer pb_nodes;
    for (auto i : node_data_container.nodes){
        auto c = pb_nodes.add_nodes();
        c->set_geometry_id(i.geometry_id.id);
        c->set_component_id(i.component_id.id);
        c->set_annotation_id(i.annotation_id);
        c->set_is_tiny(i.component_id.is_tiny);
        c->set_segregated(i.annotation_id);
    }

    for (auto i : node_data_container.annotation_data){
        auto c = pb_nodes.add_annotation_data();
        c->set_name_id(i.name_id);
    }

    std::fstream pb_out("1.ebg.nodes.pb", std::ios::out | std::ios::binary);
    pb_nodes.SerializeToOstream(&pb_out);
}


template <storage::Ownership Ownership>
inline void writeScc(const detail::EdgeBasedNodeDataContainerImpl<Ownership> &node_data_container,
                     util::DeallocatingVector<extractor::EdgeBasedEdge> &edge_based_edge_list)
{

    //std::cout<< "### scc: node_data_container.node.size: " << node_data_container.nodes.size()
    //      << " edge_based_edge_list: "<< edge_based_edge_list.size() << std::endl;

    std::map<std::uint32_t, std::uint32_t> node_component_map;
    std::uint32_t max_component_id = 0;
    for(unsigned long i = 0; i < node_data_container.nodes.size(); ++i) {
        node_component_map[i] = node_data_container.nodes[i].component_id.id;
        if(node_data_container.nodes[i].component_id.id >  max_component_id ){
            max_component_id = node_data_container.nodes[i].component_id.id;
        }
    }

    std::vector< std::vector<std::uint32_t> > scc_info;
    for(std::uint32_t i = 0; i <= max_component_id; ++i){
        std::vector<std::uint32_t> x;
        scc_info.push_back(x);
    }

    for(auto i = edge_based_edge_list.begin(); i != edge_based_edge_list.end(); ++i){
        if(i->source >= node_component_map.size() || i->target >=  node_component_map.size()){
            std::cout << "## scc err: " << node_component_map.size() << " i->source: " << i->source
                << " i->target: "<< i->target << std::endl;
            continue;
        }

        bool found = false;
        for ( auto j : scc_info[node_component_map[i->source]]) {
            if(j == node_component_map[i->target]){
                found = true;
                break;
            }
        }
        if (!found){
            scc_info[node_component_map[i->source]].push_back(node_component_map[i->target]);
        }
    }

    int isolated_component_num = 0;
    pbscc::SCCGraph pb_scc;
    pb_scc.set_v(max_component_id+1);
    for (auto i : scc_info){
        auto c = pb_scc.add_adj();
        for(auto j: i){
            c->add_targets(j);
        }
        if (i.size() == 0) {
            isolated_component_num++;
        }
    }

    std::cout<< "### scc: node_component_map: " << node_component_map.size() << " scc_info: "<< scc_info.size()
        << "isolated component: " << isolated_component_num << std::endl;

    std::fstream pb_out("1.ebg.scc.pb", std::ios::out | std::ios::binary);
    pb_scc.SerializeToOstream(&pb_out);
}

inline void read(storage::io::BufferReader &reader, ConditionalTurnPenalty &turn_penalty)
{
    reader.ReadInto(turn_penalty.turn_offset);
    reader.ReadInto(turn_penalty.location.lat);
    reader.ReadInto(turn_penalty.location.lon);
    auto const num_conditions = reader.ReadElementCount64();
    turn_penalty.conditions.resize(num_conditions);
    for (auto &condition : turn_penalty.conditions)
    {
        reader.ReadInto(condition.modifier);
        storage::serialization::read(reader, condition.times);
        storage::serialization::read(reader, condition.weekdays);
        storage::serialization::read(reader, condition.monthdays);
    }
}

inline void write(storage::io::BufferWriter &writer, const ConditionalTurnPenalty &turn_penalty)
{
    writer.WriteFrom(turn_penalty.turn_offset);
    writer.WriteFrom(static_cast<util::FixedLatitude::value_type>(turn_penalty.location.lat));
    writer.WriteFrom(static_cast<util::FixedLongitude::value_type>(turn_penalty.location.lon));
    writer.WriteElementCount64(turn_penalty.conditions.size());
    for (const auto &c : turn_penalty.conditions)
    {
        writer.WriteFrom(c.modifier);
        storage::serialization::write(writer, c.times);
        storage::serialization::write(writer, c.weekdays);
        storage::serialization::write(writer, c.monthdays);
    }
}

inline void write(storage::io::BufferWriter &writer,
                  const std::vector<ConditionalTurnPenalty> &conditional_penalties)
{
    writer.WriteElementCount64(conditional_penalties.size());
    for (const auto &penalty : conditional_penalties)
    {
        write(writer, penalty);
    }
}

inline void read(storage::io::BufferReader &reader,
                 std::vector<ConditionalTurnPenalty> &conditional_penalties)
{
    auto num_elements = reader.ReadElementCount64();
    conditional_penalties.resize(num_elements);
    for (auto &penalty : conditional_penalties)
    {
        read(reader, penalty);
    }
}

inline void write(storage::tar::FileWriter &writer,
                  const std::string &name,
                  const std::vector<ConditionalTurnPenalty> &conditional_penalties)
{
    storage::io::BufferWriter buffer_writer;
    write(buffer_writer, conditional_penalties);

    storage::serialization::write(writer, name, buffer_writer.GetBuffer());
}

inline void read(storage::tar::FileReader &reader,
                 const std::string &name,
                 std::vector<ConditionalTurnPenalty> &conditional_penalties)
{
    std::string buffer;
    storage::serialization::read(reader, name, buffer);

    storage::io::BufferReader buffer_reader{buffer};
    read(buffer_reader, conditional_penalties);
}

template <storage::Ownership Ownership>
inline void write(storage::tar::FileWriter &writer,
                  const std::string &name,
                  const detail::NameTableImpl<Ownership> &name_table)
{
    storage::io::BufferWriter buffer_writer;
    util::serialization::write(writer, name, name_table.indexed_data);
}

template <storage::Ownership Ownership>
inline void read(storage::tar::FileReader &reader,
                 const std::string &name,
                 detail::NameTableImpl<Ownership> &name_table)
{
    std::string buffer;
    util::serialization::read(reader, name, name_table.indexed_data);
}
}
}
}

#endif
