#ifndef OSRM_PARTITIONER_SERIALIZATION_HPP
#define OSRM_PARTITIONER_SERIALIZATION_HPP

#include "partitioner/cell_storage.hpp"
#include "partitioner/edge_based_graph.hpp"
#include "partitioner/multi_level_graph.hpp"
#include "partitioner/multi_level_partition.hpp"

#include "protobuf/cells.pb.h"
#include "protobuf/partition.pb.h"
#include "storage/block.hpp"
#include "storage/io.hpp"
#include "storage/serialization.hpp"
#include "storage/shared_memory_ownership.hpp"
#include "storage/tar.hpp"

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
}

template <storage::Ownership Ownership>
inline void writePB(const std::string &path, const detail::MultiLevelPartitionImpl<Ownership> &mlp)
{
    pbmldp::Partitions pb_partition;
    for (auto index : util::irange<std::size_t>(0, mlp.partition.size()))
    {
        pb_partition.add_partition(mlp.partition[index]);
    }

    pbmldp::LevelData *pb_level_data = pb_partition.mutable_leveldata();
    pb_level_data->set_numberlevel(mlp.level_data->num_level);

    for (auto elem : mlp.level_data->lidx_to_offset)
    {
        pb_level_data->add_lidxtooffset(elem);
    }

    for (auto elem : mlp.level_data->lidx_to_mask)
    {
        pb_level_data->add_lidxtomask(elem);
    }

    for (auto elem : mlp.level_data->bit_to_level)
    {
        pb_level_data->add_bittolevel(elem);
    }

    std::fstream pb_output(path + ".pb", std::ios::out | std::ios::binary);
    pb_partition.SerializeToOstream(&pb_output);
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
}

template <storage::Ownership Ownership>
inline void writePB(const std::string &path, const detail::CellStorageImpl<Ownership> &storage)
{
    pbmldc::Cells pb_cells;
    for (auto index : util::irange<std::size_t>(0, storage.source_boundary.size()))
    {
        pb_cells.add_sourceboundary(storage.source_boundary[index]);
    }
    for (auto index : util::irange<std::size_t>(0, storage.destination_boundary.size()))
    {
        pb_cells.add_destinationboundary(storage.destination_boundary[index]);
    }

    for (auto index : util::irange<std::size_t>(0, storage.cells.size()))
    {
        pbmldc::CellData *pb_cell_data = pb_cells.add_cells();
        pb_cell_data->set_valueoffset(storage.cells[index].value_offset);
        pb_cell_data->set_sourceboundaryoffset(storage.cells[index].source_boundary_offset);
        pb_cell_data->set_destinationboundaryoffset(
            storage.cells[index].destination_boundary_offset);
        pb_cell_data->set_sourcenodesnumber(storage.cells[index].num_source_nodes);
        pb_cell_data->set_destinationnodesnumber(storage.cells[index].num_destination_nodes);
    }

    for (auto index : util::irange<std::size_t>(0, storage.level_to_cell_offset.size()))
    {
        pb_cells.add_leveloffset(storage.level_to_cell_offset[index]);
    }
    std::fstream pb_output(path + ".pb", std::ios::out | std::ios::binary);
    pb_cells.SerializeToOstream(&pb_output);
}

} // namespace serialization
} // namespace partitioner
} // namespace osrm

#endif
