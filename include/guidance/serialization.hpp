#ifndef OSRM_GUIDANCE_IO_HPP
#define OSRM_GUIDANCE_IO_HPP

#include "guidance/turn_data_container.hpp"

#include "storage/serialization.hpp"
#include "storage/tar.hpp"

#include <boost/assert.hpp>

#include "../../../src/protobuf/edge-based-graph.pb.h"


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

    std::cout << "#### turn instruction: " << turn_data_container.turn_instructions.size() << std::endl;
    pbebg::TurnInstructions pb_instructions;
    for (auto i : turn_data_container.turn_instructions){
        pb_instructions.add_turn_instruction(i.pack_to_uint32());
    }
    std::fstream pb_out("1.ebg.turn.instruction.pb", std::ios::out | std::ios::binary);
    pb_instructions.SerializeToOstream(&pb_out);
}
}
}
}

#endif
