/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_mesh.h"

#include "DNA_pointcloud_types.h"

#include "node_geometry_util.hh"

#include "BKE_pointcloud.hh"

#include "BLI_math_base.hh"
#include "BLI_math_vector.h"
#include "BLI_task.hh"

namespace blender::nodes::node_geo_bdk_fluid_surface_cc {

  enum NodeGeometryBDKFluidSurfaceType
  {
    GEO_NODE_BDK_FLUID_SURFACE_TYPE_SQUARE = 0,
    GEO_NODE_BDK_FLUID_SURFACE_TYPE_HEXAGONAL = 1
  };

  static void node_declare(NodeDeclarationBuilder& b)
  {
    b.add_input<decl::Int>(N_("FluidGridType")).min(0).max(1);
    b.add_input<decl::Float>(N_("FluidGridSpacing")).min(0).default_value(24);
    b.add_input<decl::Int>(N_("FluidXSize")).min(2).default_value(48);
    b.add_input<decl::Int>(N_("FluidYSize")).min(2).default_value(48);
    b.add_input<decl::Float>(N_("UTiles")).min(1).default_value(1);
    b.add_input<decl::Float>(N_("UOffset"));
    b.add_input<decl::Float>(N_("VTiles")).min(1).default_value(1);
    b.add_input<decl::Float>(N_("VOffset"));

    b.add_output<decl::Geometry>(N_("Geometry")).propagate_all();
  }

  struct HexGridConfig {
    int x_size;
    int y_size;
    float spacing;
    float u_offset;
    float v_offset;
    float u_tiles;
    float v_tiles;
  };

  struct SquareGridConfig {
    int x_size;
    int y_size;
    float spacing;
    float u_offset;
    float v_offset;
    float u_tiles;
    float v_tiles;
  };

  static void calculate_hex_grid_positions(const HexGridConfig& config, MutableSpan<float3> positions) {

    constexpr float root_3_over_2 = 0.866025f;

    const float y_max = root_3_over_2 * (config.y_size - 1) * config.spacing;
    const float x_max_even = (config.x_size - 1) * config.spacing;
    const float x_min_odd = 0.5f * config.spacing;
    const float x_max_odd = (config.x_size - 0.5f) * config.spacing;

    //const float dy = root_3_over_2 * config.spacing;
    //const float normal_z = 4 * dy * config.spacing;

     // Y == 0 ROW ////////////////////////////////////////////////////////////////
    float3* position = positions.data();
    for (int x = 0; x < config.x_size; ++x) {
      position->x = config.spacing * x;
      position->y = 0;
      position->z = 0;
      ++position;
    }

    for (int y = 1; y < config.y_size - 1; ++y) {
      //float row_v = config.v_offset + (config.v_tiles * y / (config.y_size - 1));
      float row_y = root_3_over_2 * y * config.spacing;

      if (y & 1) {
        // ODD ROW
        position->x = x_min_odd;
        position->y = row_y;
        position->z = 0;
        ++position;

        for (int x = 1; x < config.x_size - 1; ++x) {
          position->x = (0.5f + x) * config.spacing;
          position->y = row_y;
          position->z = 0;
          ++position;
        }

        // X == FluidXSize - 1 ///////////////////////////////////////////////////
        position->x = x_max_odd;
        position->y = row_y;
        position->z = 0;
        ++position;
      }
      else {
        // EVEN ROW
        position->x = 0;
        position->y = row_y;
        position->z = 0;
        ++position;

        for (int x = 1; x < config.x_size - 1; x++) {
          position->x = config.spacing * x;
          position->y = row_y;
          position->z = 0;
          ++position;
        }

        position->x = x_max_even;
        position->y = row_y;
        position->z = 0;
        ++position;
      }
    }

    // Y == FLUIDYSIZE-1 ROW ////////////////////////////////////////////////
    if ((config.y_size - 1) & 1) {
      // last row is odd
      for (int x = 0; x < config.x_size; ++x) {
        position->x = (0.5f + x) * config.spacing;
        position->y = y_max;
        position->z = 0;
        ++position;
      }
    }
    else {
      for (int x = 0; x < config.x_size; ++x) {
        position->x = config.spacing * x;
        position->y = y_max;
        position->z = 0;
        ++position;
      }
    }

    const float offset_x = 0.5f * (config.x_size - 1) * config.spacing;
    const float offset_y = root_3_over_2 * (0.5f * (config.y_size - 1) * config.spacing);

    for (float3& position : positions) {
      position.x -= offset_x;
      position.y -= offset_y;
    }
  }

  static void calculate_hex_grid_edges(const HexGridConfig& config, MutableSpan<int2> edges)
  {
    // Horizontal edges.
    int edge_index = 0;
    for (int y = 0; y < config.y_size; ++y) {
      for (int x = 0; x < config.x_size - 1; ++x, ++edge_index) {
        int2& edge = edges[edge_index];
        edge.x = (y * config.x_size) + x;
        edge.y = (y * config.x_size) + x + 1;
      }
    }

    // Vertical edges.
    for (int y = 0; y < config.y_size - 1; ++y) {
      for (int x = 0; x < config.x_size; ++x, ++edge_index) {
        int2& edge = edges[edge_index];
        edge.x = (y * config.x_size) + x;
        edge.y = ((y + 1) * config.x_size) + x;
      }
    }

    // Diagonal edges.
    for (int y = 0; y < config.y_size - 1; ++y) {
      if (y % 2 == 0) {
        for (int x = 0; x < config.x_size - 1; ++x, ++edge_index) {
          int2& edge = edges[edge_index];
          edge.x = (y * config.x_size) + x + 1;
          edge.y = ((y + 1) * config.x_size) + x;
        }
      }
      else {
        for (int x = 0; x < config.x_size - 1; ++x, ++edge_index) {
          int2& edge = edges[edge_index];
          edge.x = (y * config.x_size) + x;
          edge.y = ((y + 1) * config.x_size) + x + 1;
        }
      }
    }
  }

  static void calculate_hex_grid_polys(const HexGridConfig& config, MutableSpan<int> corner_verts, MutableSpan<int> corner_edges) {
    int loop_index = 0;

    const int horizontal_edge_count = (config.x_size - 1) * config.y_size;
    const int vertical_edge_count = (config.y_size - 1) * config.x_size;
    //const int diagonal_edge_count = (config.x_size - 1) * (config.y_size - 1);
    const int horizontal_edge_offset = 0;
    const int horizontal_edge_stride = config.x_size - 1;
    const int vertical_edge_offset = horizontal_edge_count;
    const int vertical_edge_row_stride = config.x_size;
    const int diagonal_edge_offset = vertical_edge_offset + vertical_edge_count;
    const int diagonal_edge_row_stride = config.x_size - 1;

    // For each row
    for (int y = 0; y < config.y_size - 1; y++)
    {
      if (y % 2 == 0)
      {
        for (int x = 0; x < config.x_size - 1; x++)
        {
          {
            int v1 = (y + 0) * config.x_size + (x + 0);
            int v2 = (y + 1) * config.x_size + (x + 0);
            int v3 = (y + 0) * config.x_size + (x + 1);
            int e1 = vertical_edge_offset + x + (vertical_edge_row_stride * y);
            int e2 = diagonal_edge_offset + x + (diagonal_edge_row_stride * y);
            int e3 = horizontal_edge_offset + x + (horizontal_edge_stride * y);
            corner_verts[loop_index + 0] = v1;
            corner_edges[loop_index + 0] = e1;
            corner_verts[loop_index + 1] = v2;
            corner_edges[loop_index + 1] = e2;
            corner_verts[loop_index + 2] = v3;
            corner_edges[loop_index + 2] = e3;
            loop_index += 3;
          }
          {
            int v1 = (y + 1) * config.x_size + (x + 0);
            int v2 = (y + 1) * config.x_size + (x + 1);
            int v3 = (y + 0) * config.x_size + (x + 1);
            int e1 = horizontal_edge_offset + x + (horizontal_edge_stride * (y + 1));
            int e2 = vertical_edge_offset + x + 1 + (vertical_edge_row_stride * y);
            int e3 = diagonal_edge_offset + x + (diagonal_edge_row_stride * y);
            corner_verts[loop_index + 0] = v1;
            corner_edges[loop_index + 0] = e1;
            corner_verts[loop_index + 1] = v2;
            corner_edges[loop_index + 1] = e2;
            corner_verts[loop_index + 2] = v3;
            corner_edges[loop_index + 2] = e3;
            loop_index += 3;
          }
        }
      }
      else
      {
        // For each element
        for (int x = 0; x < config.x_size - 1; x++)
        {
          {
            int v1 = (y + 0) * config.x_size + (x + 0);
            int v2 = (y + 1) * config.x_size + (x + 1);
            int v3 = (y + 0) * config.x_size + (x + 1);
            int e1 = diagonal_edge_offset + x + (diagonal_edge_row_stride * y);
            int e2 = vertical_edge_offset + x + 1 + (vertical_edge_row_stride * y);
            int e3 = horizontal_edge_offset + x + (horizontal_edge_stride * y);
            corner_verts[loop_index + 0] = v1;
            corner_edges[loop_index + 0] = e1;
            corner_verts[loop_index + 1] = v2;
            corner_edges[loop_index + 1] = e2;
            corner_verts[loop_index + 2] = v3;
            corner_edges[loop_index + 2] = e3;
            loop_index += 3;
          }
          {
            int v1 = (y + 0) * config.x_size + (x + 0);
            int v2 = (y + 1) * config.x_size + (x + 0);
            int v3 = (y + 1) * config.x_size + (x + 1);
            int e1 = vertical_edge_offset + x + (vertical_edge_row_stride * y);
            int e2 = horizontal_edge_offset + x + (horizontal_edge_stride * (y + 1));
            int e3 = diagonal_edge_offset + x + (diagonal_edge_row_stride * y);
            corner_verts[loop_index + 0] = v1;
            corner_edges[loop_index + 0] = e1;
            corner_verts[loop_index + 1] = v2;
            corner_edges[loop_index + 1] = e2;
            corner_verts[loop_index + 2] = v3;
            corner_edges[loop_index + 2] = e3;
            loop_index += 3;
          }
        }
      }
    }
  }

  static Mesh* create_hex_grid(GeoNodeExecParams& params)
  {
    HexGridConfig config;
    config.spacing = params.get_input<float>("FluidGridSpacing");
    config.x_size = params.get_input<int>("FluidXSize");
    config.y_size = params.get_input<int>("FluidYSize");
    config.u_offset = params.get_input<float>("UOffset");
    config.v_offset = params.get_input<float>("VOffset");
    config.u_tiles = params.get_input<int>("UTiles");
    config.v_tiles = params.get_input<int>("VTiles");

    const int vertex_count = config.x_size * config.y_size;
    const int poly_count = (config.x_size - 1) * (config.y_size - 1) * 2;
    const int loop_count = poly_count * 3;
    const int edge_count = (3 * config.x_size * config.y_size) - (2 * config.y_size) - (2 * config.x_size) + 1;

    if (vertex_count == 0) {
      return nullptr;
    }

    Mesh* mesh = BKE_mesh_new_nomain(vertex_count, edge_count, poly_count, loop_count);

    MutableSpan<float3> positions = mesh->vert_positions_for_write();
    MutableSpan<int2> edges = mesh->edges_for_write();
    MutableSpan<int> corner_verts = mesh->corner_verts_for_write();
    MutableSpan<int> corner_edges = mesh->corner_edges_for_write();

    //BKE_mesh_smooth_flag_set(mesh, false);

    offset_indices::fill_constant_group_size(3, 0, mesh->face_offsets_for_write());

    calculate_hex_grid_positions(config, positions);
    calculate_hex_grid_edges(config, edges);
    calculate_hex_grid_polys(config, corner_verts, corner_edges);

    //mesh->tag_loose_verts_none();
    //mesh->tag_loose_edges_none();

    return mesh;
  }

  static void calculate_square_grid_positions(const SquareGridConfig& config, MutableSpan<float3>& positions)
  {
    //const float reciprocal_fluid_x_size_min_1 = (1.f / (config.x_size - 1));
    //const float normal_z = -4 * (config.spacing * config.spacing);

    float3* position = positions.begin();

    const float offset_x = 0.5f * (config.x_size - 1) * config.spacing;
    const float offset_y = 0.5f * (config.y_size - 1) * config.spacing;

    for (int y = 0; y < config.y_size; ++y)
    {
      const float row_y = y * config.spacing;

      for (int x = 0; x < config.x_size; ++x)
      {
        position->x = x * config.spacing;
        position->y = row_y;
        position->z = 0.0f;
        ++position;
      }
    }

    for (float3& position : positions) {
      position.x -= offset_x;
      position.y -= offset_y;
    }
  }

  static void calculate_square_grid_edges(const SquareGridConfig& config, MutableSpan<int2> edges)
  {
    int edge_index = 0;

    for (int y = 0; y < config.y_size; ++y)
    {
      const int row_vertex_index = y * config.x_size;

      // Horizontal edges
      for (int x = 0; x < config.x_size - 1; ++x, ++edge_index)
      {
        edges[edge_index].x = row_vertex_index + x;
        edges[edge_index].y = row_vertex_index + x + 1;
      }

      // Vertical edges
      if (y < config.y_size - 1)
      {
        for (int x = 0; x < config.x_size; ++x, ++edge_index)
        {
          edges[edge_index].x = row_vertex_index + x;
          edges[edge_index].y = row_vertex_index + x + config.x_size;
        }
      }
    }
  }

  static void calculate_square_grid_polys(const SquareGridConfig& config, MutableSpan<int>& corner_verts, MutableSpan<int>& corner_edges)
  {
    size_t loop_index = 0;

    const int edges_per_row = ((config.x_size * 2) - 1);

    // For each row
    for (int y = 0; y < config.y_size - 1; y++)
    {
      const int row_edge_start = y * edges_per_row;

      // For each element
      for (int x = 0; x < config.x_size - 1; x++)
      {
        const int x_offset = row_edge_start + x;

        corner_verts[loop_index + 0] = (y + 0) * config.x_size + (x + 0);
        corner_edges[loop_index + 0] = x_offset;

        corner_verts[loop_index + 1] = (y + 0) * config.x_size + (x + 1);
        corner_edges[loop_index + 1] = x_offset + config.x_size;

        corner_verts[loop_index + 2] = (y + 1) * config.x_size + (x + 1);
        corner_edges[loop_index + 2] = x_offset + edges_per_row;

        corner_verts[loop_index + 3] = (y + 1) * config.x_size + (x + 0);
        corner_edges[loop_index + 3] = x_offset + config.x_size - 1;

        loop_index += 4;
      }
    }
  }

  static Mesh* create_square_grid(GeoNodeExecParams& params) {
    SquareGridConfig config;
    config.spacing = params.get_input<float>("FluidGridSpacing");
    config.u_offset = params.get_input<float>("UOffset");
    config.v_offset = params.get_input<float>("VOffset");
    config.u_tiles = params.get_input<float>("UTiles");
    config.v_tiles = params.get_input<float>("VTiles");
    config.x_size = params.get_input<int>("FluidXSize");
    config.y_size = params.get_input<int>("FluidYSize");

    const int vertex_count = config.x_size * config.y_size;
    const int poly_count = (config.x_size - 1) * (config.y_size - 1);
    const int loop_count = poly_count * 4;
    const int edge_count = ((config.x_size - 1) * config.y_size) + (config.x_size * (config.y_size - 1));

    if (vertex_count == 0) {
      return nullptr;
    }

    Mesh* mesh = BKE_mesh_new_nomain(vertex_count, edge_count, poly_count, loop_count);

    MutableSpan<float3> positions = mesh->vert_positions_for_write();
    MutableSpan<int2> edges = mesh->edges_for_write();
    MutableSpan<int> corner_verts = mesh->corner_verts_for_write();
    MutableSpan<int> corner_edges = mesh->corner_edges_for_write();

    offset_indices::fill_constant_group_size(4, 0, mesh->face_offsets_for_write());

    calculate_square_grid_positions(config, positions);
    calculate_square_grid_edges(config, edges);
    calculate_square_grid_polys(config, corner_verts, corner_edges);

    mesh->tag_loose_verts_none();
    mesh->tag_loose_edges_none();

    return mesh;
  }

  static void node_geo_exec(GeoNodeExecParams params)
  {
    Mesh* mesh = nullptr;

    const NodeGeometryBDKFluidSurfaceType type = static_cast<NodeGeometryBDKFluidSurfaceType>(params.extract_input<int>("FluidGridType"));

    switch (type) {
    case GEO_NODE_BDK_FLUID_SURFACE_TYPE_SQUARE:
      mesh = create_square_grid(params);
      break;
    case GEO_NODE_BDK_FLUID_SURFACE_TYPE_HEXAGONAL:
      mesh = create_hex_grid(params);
      break;
    default:
      params.error_message_add(NodeWarningType::Error, "Invalid fluid grid type");
      // TODO: set error somewhere
      params.set_default_remaining_outputs();
      return;
    }

    params.set_output("Geometry", GeometrySet::from_mesh(mesh));
  }

  static void node_register()
  {
    namespace file_ns = blender::nodes::node_geo_bdk_fluid_surface_cc;

    static blender::bke::bNodeType ntype;

    geo_node_type_base(&ntype, GEO_NODE_BDK_FLUID_SURFACE, "BDK FluidSurface", NODE_CLASS_GEOMETRY);
    ntype.declare = file_ns::node_declare;
    ntype.geometry_node_execute = file_ns::node_geo_exec;
    blender::bke::node_register_type(&ntype);
  }

  NOD_REGISTER_NODE(node_register)

} // namespace blender::nodes::node_geo_bdk_fluid_surface_cc
