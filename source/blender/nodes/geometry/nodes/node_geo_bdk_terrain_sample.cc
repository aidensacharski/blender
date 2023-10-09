/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_mesh.h"

#include "DNA_pointcloud_types.h"

#include "node_geometry_util.hh"

#include "BKE_pointcloud.hh"

#include "BLI_math_base.hh"
#include "BLI_math_geom.h"
#include "BLI_math_vector.hh"
#include "BLI_task.hh"

using namespace blender;

namespace blender::nodes::node_geo_bdk_terrain_sample_cc {

  class SampleFunction : public mf::MultiFunction {
  private:
    GeometrySet target_;
    int heightmap_resolution_;

  public:
    SampleFunction(GeometrySet target, const int heightmap_resolution)
      : target_(std::move(target)), heightmap_resolution_(heightmap_resolution)
    {
      target_.ensure_owns_direct_data();
      static const mf::Signature signature = []() {
        mf::Signature signature;
        mf::SignatureBuilder builder{"BDK Terrain Sample", signature};
        builder.single_input<float3>("Source Position");
        builder.single_output<bool>("Is Inside"/*, mf::ParamFlag::SupportsUnusedOutput*/);
        builder.single_output<float3>("Position"/*, mf::ParamFlag::SupportsUnusedOutput */);
        builder.single_output<float3>("Normal"/*, mf::ParamFlag::SupportsUnusedOutput*/);
        builder.single_output<int>("Face Index"/*, mf::ParamFlag::SupportsUnusedOutput */);
        builder.single_output<int>("Vertex Index"/*, mf::ParamFlag::SupportsUnusedOutput */);
        return signature;
      }();
      this->set_signature(&signature);
    }

    void call(const IndexMask& mask, mf::Params params, mf::Context /*context*/) const override
    {
      const VArray<float3>& src_positions = params.readonly_single_input<float3>(0, "Source Position");
      MutableSpan<bool> is_insides = params.uninitialized_single_output<bool>(1, "Is Inside");
      MutableSpan<float3> positions = params.uninitialized_single_output<float3>(2, "Position");
      MutableSpan<float3> normals = params.uninitialized_single_output<float3>(3, "Normal");
      MutableSpan<int> face_indices = params.uninitialized_single_output<int>(4, "Face Index");
      MutableSpan<int> vertex_indices = params.uninitialized_single_output<int>(5, "Vertex Index");

      index_mask::masked_fill(is_insides, false, mask);
      index_mask::masked_fill(normals, float3(0.0f, 0.0f, 1.0f), mask);
      index_mask::masked_fill(face_indices, -1, mask);
      index_mask::masked_fill(vertex_indices, -1, mask);

      if (!target_.has_mesh()) {
        return;
      }
      const Mesh* mesh = target_.get_mesh();
      const Span<float3>& vert_positions = mesh->vert_positions();
      const OffsetIndices<int> faces = mesh->faces();
      const Span<int> corner_verts = mesh->corner_verts();

      if (vert_positions.is_empty()) {
        return;
      }

      const float2 min = vert_positions.first().xy();
      const float2 max = vert_positions.last().xy();
      const float2 size = max - min;

      mask.foreach_index_optimized<int>(GrainSize(512), [&](const int j) {
        const float3& source_position = src_positions[j];

        // Check if the source position is outside the terrain bounds.
        if (source_position.x < min.x ||
            source_position.x > max.x ||
            source_position.y < min.y ||
            source_position.y > max.y) {
          positions[j] = src_positions[j];
          return;
        }

        const int x = ((source_position.x - min.x) / size.x) * (heightmap_resolution_ - 1);
        const int y = ((source_position.y - min.y) / size.y) * (heightmap_resolution_ - 1);
        const int vertex_index = (y * heightmap_resolution_) + x;
        const int face_index = (y * (heightmap_resolution_ - 1)) + x;

        if (vertex_index < 0 || vertex_index >= vert_positions.size()) {
          positions[j] = src_positions[j];
          return;
        }

        // v2 -- v3
        // |     |
        // |     |
        // v0 -- v1
        const std::array<int, 4> face_vertex_indices = { vertex_index, vertex_index + 1, vertex_index + heightmap_resolution_, vertex_index + heightmap_resolution_ + 1 };
        const float3& v0 = vert_positions[face_vertex_indices[0]];
        const float3& v1 = vert_positions[face_vertex_indices[1]];
        const float3& v2 = vert_positions[face_vertex_indices[2]];
        const float3& v3 = vert_positions[face_vertex_indices[3]];
        const int face_corner_vert = corner_verts[faces[face_index].start()];
        const bool is_edge_turned = (face_corner_vert == face_vertex_indices[1]) || (face_corner_vert == face_vertex_indices[2]);

        // get normalized coordinates for the sample position
        const float2 quad_uv = {
          (source_position.x - v0.x) / (v1.x - v0.x),
          (source_position.y - v0.y) / (v2.y - v0.y)
        };

        float verts[3][3];

        // TODO: there's probably a more elegant way to do this.
        if (is_edge_turned) {
          if (quad_uv.x > quad_uv.y) {
            copy_v3_v3(verts[0], v1);
            copy_v3_v3(verts[1], v3);
            copy_v3_v3(verts[2], v2);
          }
          else {
            copy_v3_v3(verts[0], v1);
            copy_v3_v3(verts[1], v2);
            copy_v3_v3(verts[2], v0);
          }
        }
        else {
          if (quad_uv.x > quad_uv.y) {
            copy_v3_v3(verts[0], v0);
            copy_v3_v3(verts[1], v1);
            copy_v3_v3(verts[2], v3);
          }
          else {
            copy_v3_v3(verts[0], v0);
            copy_v3_v3(verts[1], v3);
            copy_v3_v3(verts[2], v2);
          }
        }

        float2 uv;
        resolve_tri_uv_v2(uv, source_position.xy(), verts[0], verts[1], verts[2]);
        interp_barycentric_tri_v3(verts, uv.x, uv.y, positions[j]);

        is_insides[j] = true;
        normals[j] = math::normal_tri(float3(verts[0]), float3(verts[1]), float3(verts[2]));
        face_indices[j] = face_index;
        vertex_indices[j] = vertex_index;
        });
    }
  };

  static void node_declare(NodeDeclarationBuilder& b)
  {
    b.add_input<decl::Geometry>(N_("Terrain"));
    b.add_input<decl::Vector>(N_("Source Position")).implicit_field(implicit_field_inputs::position);
    b.add_output<decl::Bool>(N_("Is Inside")).dependent_field();
    b.add_output<decl::Vector>(N_("Normal")).dependent_field();
    b.add_output<decl::Vector>(N_("Position")).dependent_field();
    b.add_output<decl::Int>(N_("Face Index")).dependent_field();
    b.add_output<decl::Int>(N_("Vertex Index")).dependent_field();
  }

  static void node_geo_exec(GeoNodeExecParams params)
  {
    GeometrySet geometry_set = params.get_input<GeometrySet>("Terrain");

    if (!geometry_set.has_mesh()) {
      params.set_default_remaining_outputs();
      return;
    }


    const Mesh* mesh = geometry_set.get_mesh();
    const int heightmap_size = math::sqrt(mesh->totvert);
    const int vertex_count = heightmap_size * heightmap_size;

    if (mesh->totvert != vertex_count) {
      params.error_message_add(NodeWarningType::Error, "Incorrect vertex count");
      params.set_default_remaining_outputs();
      return;
    }

    Field<float3> source_position_field = params.extract_input<Field<float3>>("Source Position");

    auto sample_fn = std::make_unique<SampleFunction>(std::move(geometry_set), heightmap_size);
    auto sample_op = FieldOperation::Create(std::move(sample_fn), { std::move(source_position_field) });

    params.set_output("Is Inside", Field<bool>(sample_op, 0));
    params.set_output("Position", Field<float3>(sample_op, 1));
    params.set_output("Normal", Field<float3>(sample_op, 2));
    params.set_output("Face Index", Field<int>(sample_op, 3));
    params.set_output("Vertex Index", Field<int>(sample_op, 4));
  }

  static void node_register()
  {
    namespace file_ns = blender::nodes::node_geo_bdk_terrain_sample_cc;

    static bNodeType ntype;

    geo_node_type_base(&ntype, GEO_NODE_BDK_TERRAIN_SAMPLE, "BDK Terrain Sample", NODE_CLASS_GEOMETRY);
    ntype.declare = file_ns::node_declare;
    ntype.geometry_node_execute = file_ns::node_geo_exec;
    nodeRegisterType(&ntype);
  }

  NOD_REGISTER_NODE(node_register)

} // namespace blender::nodes::node_geo_bdk_terrain_sample_cc
