#ifndef OSRM_PARTITIONER_SERIALIZATION_HPP
#define OSRM_PARTITIONER_SERIALIZATION_HPP

#include "partitioner/cell_storage.hpp"
#include "partitioner/edge_based_graph.hpp"
#include "partitioner/multi_level_graph.hpp"
#include "partitioner/multi_level_partition.hpp"

#include "storage/block.hpp"
#include "storage/io.hpp"
#include "storage/serialization.hpp"
#include "storage/shared_memory_ownership.hpp"
#include "storage/tar.hpp"

#include "../../../src/protobuf/mld.pb.h"


namespace osrm
{
namespace partitioner
{
namespace serialization
{

template <storage::Ownership Ownership>
inline void read(storage::tar::FileReader &reader,
                 const std::string &name,
                 detail::MultiLevelPartitionImpl<Ownership> &mlp)
{
    reader.ReadInto(name + "/level_data", *mlp.level_data);
    storage::serialization::read(reader, name + "/partition", mlp.partition);
    storage::serialization::read(reader, name + "/cell_to_children", mlp.cell_to_children);
}

template <storage::Ownership Ownership>
inline void write(storage::tar::FileWriter &writer,
                  const std::string &name,
                  const detail::MultiLevelPartitionImpl<Ownership> &mlp)
{
    writer.WriteElementCount64(name + "/level_data", 1);
    writer.WriteFrom(name + "/level_data", *mlp.level_data);
    storage::serialization::write(writer, name + "/partition", mlp.partition);
    storage::serialization::write(writer, name + "/cell_to_children", mlp.cell_to_children);

    std::cout << "#### partitions: partition.size: " << mlp.partition.size()
        << " cell_to_children.size: " << mlp.cell_to_children.size()
        << " levelData.num_level: " << mlp.level_data->num_level
        << " levelData.lidx_to_offset.size: " << mlp.level_data->lidx_to_offset.size()
        << " levelData.lidx_to_mask.size: " << mlp.level_data->lidx_to_mask.size()
        << " levelData.bit_to_level.size: " << mlp.level_data->bit_to_level.size()
        << " levelData.lidx_to_children_offsets.size: " << mlp.level_data->lidx_to_children_offsets.size()
        << std::endl;

    pbmld::Partitions pb_partitions;
    for (auto i : mlp.partition ){
        pb_partitions.add_partition(i);
    }

    auto pb_level_data = new pbmld::LevelData;
    pb_level_data->set_number_level(mlp.level_data->num_level);
    for(auto i : mlp.level_data->lidx_to_offset) {
        pb_level_data->add_lidx_to_offset(i);
    }
    for(auto i : mlp.level_data->lidx_to_mask) {
        pb_level_data->add_lidx_to_mask(i);
    }
    for(auto i : mlp.level_data->bit_to_level) {
        pb_level_data->add_bit_to_level(i);
    }
    pb_partitions.set_allocated_level_data(pb_level_data);
    std::fstream pb_out("1.mld.partitions.pb", std::ios::out | std::ios::binary);
    pb_partitions.SerializeToOstream(&pb_out);
}

template <storage::Ownership Ownership>
inline void read(storage::tar::FileReader &reader,
                 const std::string &name,
                 detail::CellStorageImpl<Ownership> &storage)
{
    storage::serialization::read(reader, name + "/source_boundary", storage.source_boundary);
    storage::serialization::read(
        reader, name + "/destination_boundary", storage.destination_boundary);
    storage::serialization::read(reader, name + "/cells", storage.cells);
    storage::serialization::read(
        reader, name + "/level_to_cell_offset", storage.level_to_cell_offset);
}

template <storage::Ownership Ownership>
inline void write(storage::tar::FileWriter &writer,
                  const std::string &name,
                  const detail::CellStorageImpl<Ownership> &storage)
{
    storage::serialization::write(writer, name + "/source_boundary", storage.source_boundary);
    storage::serialization::write(
        writer, name + "/destination_boundary", storage.destination_boundary);
    storage::serialization::write(writer, name + "/cells", storage.cells);
    storage::serialization::write(
        writer, name + "/level_to_cell_offset", storage.level_to_cell_offset);

    std::cout << "#### cells: source_boundary: " << storage.source_boundary.size()
        << " destination_boundary: " << storage.destination_boundary.size()
        << " cells: " << storage.cells.size()
        << " level_to_cell_offset: " << storage.level_to_cell_offset.size() << std::endl;

    pbmld::Cells pb_cells;
    for (auto i : storage.source_boundary ){
        pb_cells.add_source_boundary(i);
    }
    for (auto i : storage.destination_boundary ){
        pb_cells.add_destination_boundary(i);
    }
    for (auto i : storage.level_to_cell_offset ){
        pb_cells.add_level_offset(i);
    }
    for (auto i : storage.cells ){
        auto cell = pb_cells.add_cells();
        cell->set_value_offset(i.value_offset);
        cell->set_source_boundary_offset(i.source_boundary_offset);
        cell->set_destination_boundary_offset(i.destination_boundary_offset);
        cell->set_source_node_number(i.num_source_nodes);
        cell->set_destination_node_number(i.num_destination_nodes);
    }
    std::fstream pb_out("1.mld.cells.pb", std::ios::out | std::ios::binary);
    pb_cells.SerializeToOstream(&pb_out);
}
}
}
}

#endif
