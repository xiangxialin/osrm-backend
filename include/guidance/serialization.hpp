#ifndef OSRM_GUIDANCE_IO_HPP
#define OSRM_GUIDANCE_IO_HPP

#include "../../src/protobuf/edges.pb.h"
#include "guidance/turn_data_container.hpp"

#include "storage/serialization.hpp"
#include "storage/tar.hpp"

#include <boost/assert.hpp>

namespace osrm
{
namespace guidance
{
namespace serialization
{

// read/write for turn data file
template <storage::Ownership Ownership>
inline void read(storage::tar::FileReader &reader,
                 const std::string &name,
                 guidance::detail::TurnDataContainerImpl<Ownership> &turn_data_container)
{
    storage::serialization::read(
        reader, name + "/turn_instructions", turn_data_container.turn_instructions);
    storage::serialization::read(
        reader, name + "/lane_data_ids", turn_data_container.lane_data_ids);
    storage::serialization::read(
        reader, name + "/entry_class_ids", turn_data_container.entry_class_ids);
    storage::serialization::read(
        reader, name + "/pre_turn_bearings", turn_data_container.pre_turn_bearings);
    storage::serialization::read(
        reader, name + "/post_turn_bearings", turn_data_container.post_turn_bearings);
}

template <storage::Ownership Ownership>
inline void write(storage::tar::FileWriter &writer,
                  const std::string &name,
                  const guidance::detail::TurnDataContainerImpl<Ownership> &turn_data_container)
{
    storage::serialization::write(
        writer, name + "/turn_instructions", turn_data_container.turn_instructions);
    storage::serialization::write(
        writer, name + "/lane_data_ids", turn_data_container.lane_data_ids);
    storage::serialization::write(
        writer, name + "/entry_class_ids", turn_data_container.entry_class_ids);
    storage::serialization::write(
        writer, name + "/pre_turn_bearings", turn_data_container.pre_turn_bearings);
    storage::serialization::write(
        writer, name + "/post_turn_bearings", turn_data_container.post_turn_bearings);
}

template <storage::Ownership Ownership>
inline void writePB(const std::string &name,
                    const guidance::detail::TurnDataContainerImpl<Ownership> &turn_data_container)
{
    pbmlde::Edges pb_edges;
    for (auto index : util::irange<std::size_t>(0, turn_data_container.turn_instructions.size()))
    {
        pb_edges.add_turntype(turn_data_container.turn_instructions[index].type);
        pb_edges.add_directionmodifier(
            turn_data_container.turn_instructions[index].direction_modifier);
    }

    std::fstream pb_output(name + ".pb", std::ios::out | std::ios::binary);
    pb_edges.SerializeToOstream(&pb_output);
}
} // namespace serialization
} // namespace guidance
} // namespace osrm

#endif
