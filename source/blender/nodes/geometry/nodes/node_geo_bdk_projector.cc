/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_geometry_set_instances.hh"
#include "BKE_mesh.hh"

#include "DNA_pointcloud_types.h"

#include "node_geometry_util.hh"

#include "BKE_pointcloud.hh"
#include "BKE_material.h"

#include "BLI_math_base.hh"
#include "BLI_math_matrix.hh"
#include "BLI_math_geom.h"
#include "BLI_math_vector.h"
#include "BLI_task.hh"

#include "GEO_transform.hh"

#include "RNA_access.hh"

namespace blender::nodes::node_geo_bdk_projector_cc {

  static void node_declare(NodeDeclarationBuilder& b)
  {
    b.add_input<decl::Object>(N_("Target"));

    b.add_input<decl::Float>(N_("FOV")).min(0).default_value(M_PI_2).subtype(PropertySubType::PROP_ANGLE);
    b.add_input<decl::Float>(N_("MaxTraceDistance")).min(0).default_value(1).subtype(PropertySubType::PROP_DISTANCE);
    b.add_input<decl::Float>(N_("USize")).min(1).default_value(256).subtype(PropertySubType::PROP_PIXEL);
    b.add_input<decl::Float>(N_("VSize")).min(1).default_value(256).subtype(PropertySubType::PROP_PIXEL);
    b.add_input<decl::Float>(N_("DrawScale")).min(0).default_value(1).subtype(PropertySubType::PROP_FACTOR);

    b.add_output<decl::Geometry>(N_("Geometry")).propagate_all();
    b.add_output<decl::Geometry>(N_("Frustum")).propagate_all();
    b.add_output<decl::Vector>(N_("UV Map")).field_on_all();
    b.add_output<decl::Float>(N_("Attenuation")).field_on_all();
  }

  struct FrustumConfig {
    float fov;
    float max_trace_distance;
    float usize;
    float vsize;
    float draw_scale;
  };

  struct Plane {
    float3 origin;
    float3 normal;

    Plane() = default;
    Plane(const float3 origin, const float3 normal)
      : origin(origin)
      , normal(normal) {
    }

    Plane(const float3 a, const float3 b, const float3 c) :
      origin(a) {
      normal = math::normalize(math::cross(c - a, b - a));
    }
  };

  struct Frustum {
    std::array<float3, 8> corners;
    std::array<Plane, 6> planes;
  };

  // http://davidlively.com/programming/graphics/frustum-calculation-and-culling-hopefully-demystified/
  static Frustum calculate_frustum(FrustumConfig config) {

    using namespace blender::math;

    Frustum frustum = {};

    const float3 forward(1, 0, 0);
    const float3 left(0, 1, 0);
    const float3 up(0, 0, 1);

    const float usize = config.draw_scale * config.usize;
    const float vsize = config.draw_scale * config.vsize;
    const float radius = math::sqrt((0.25f * usize * usize) + (0.25f * vsize * vsize));

    frustum.corners[0] = radius * math::normalize(up + left);
    frustum.corners[1] = radius * math::normalize(up - left);
    frustum.corners[2] = radius * math::normalize(-up - left);
    frustum.corners[3] = radius * math::normalize(-up + left);

    frustum.planes[0] = Plane(float3(0), forward);

    if (config.fov == 0.0f)
    {
      frustum.planes[1] = Plane(frustum.corners[0], -math::normalize(up + left));
      frustum.planes[2] = Plane(frustum.corners[1], -math::normalize(up - left));
      frustum.planes[3] = Plane(frustum.corners[2], -math::normalize(-up - left));
      frustum.planes[4] = Plane(frustum.corners[3], -math::normalize(-up + left));

      frustum.corners[4] = frustum.corners[0] + (config.max_trace_distance * forward);
      frustum.corners[5] = frustum.corners[1] + (config.max_trace_distance * forward);
      frustum.corners[6] = frustum.corners[2] + (config.max_trace_distance * forward);
      frustum.corners[7] = frustum.corners[3] + (config.max_trace_distance * forward);
    }
    else
    {
      const float tan_half_fov = math::tan(0.5f * config.fov);
      const float cos_half_fov = math::cos(0.5f * config.fov);

      const float3 frustum_origin = -forward * (0.5f * config.usize / tan_half_fov);

      frustum.planes[1] = Plane(frustum_origin, frustum.corners[1], frustum.corners[0]);
      frustum.planes[2] = Plane(frustum_origin, frustum.corners[2], frustum.corners[1]);
      frustum.planes[3] = Plane(frustum_origin, frustum.corners[3], frustum.corners[2]);
      frustum.planes[4] = Plane(frustum_origin, frustum.corners[0], frustum.corners[3]);

      frustum.corners[4] = frustum.corners[0] + config.max_trace_distance * math::normalize(frustum.corners[0] - frustum_origin) / cos_half_fov;
      frustum.corners[5] = frustum.corners[1] + config.max_trace_distance * math::normalize(frustum.corners[1] - frustum_origin) / cos_half_fov;
      frustum.corners[6] = frustum.corners[2] + config.max_trace_distance * math::normalize(frustum.corners[2] - frustum_origin) / cos_half_fov;
      frustum.corners[7] = frustum.corners[3] + config.max_trace_distance * math::normalize(frustum.corners[3] - frustum_origin) / cos_half_fov;
    }

    frustum.planes[5] = Plane(frustum.corners[7], -forward);

    return frustum;
  }

  struct ClipTriangle {
    std::array<float3, 3> co;
  };

  // TODO: hugely inefficient to re-make the list for every plane; maybe use a *gasp* linked list or some other construct
  static std::vector<ClipTriangle> clip_triangles(const Plane& plane, const std::vector<ClipTriangle>& triangles) {

    std::vector<ClipTriangle> output;

    float4 p;
    plane_from_point_normal_v3(p, plane.origin, plane.normal);

    for (const ClipTriangle& triangle : triangles) {
      const float d0 = dist_signed_squared_to_plane_v3(triangle.co[0], p);
      const float d1 = dist_signed_squared_to_plane_v3(triangle.co[1], p);
      const float d2 = dist_signed_squared_to_plane_v3(triangle.co[2], p);
      const int s0 = d0 >= 0.0f ? 1 : -1;
      const int s1 = d1 >= 0.0f ? 1 : -1;
      const int s2 = d2 >= 0.0f ? 1 : -1;
      const int sign_sum = s0 + s1 + s2;

      if (sign_sum == 3) {  // 3 positive
        output.push_back(triangle);
      }
      else if (sign_sum == -1) {  // 1 positive
        const int positive_index = s0 == 1 ? 0 : (s1 == 1 ? 1 : 2);
        const float3& A = triangle.co[positive_index];
        const float3& b = triangle.co[(positive_index + 1) % 3];
        const float3& c = triangle.co[(positive_index + 2) % 3];
        float3 B, C;
        isect_line_plane_v3(B, A, b, plane.origin, plane.normal);
        isect_line_plane_v3(C, A, c, plane.origin, plane.normal);
        output.push_back(ClipTriangle({ { A, B, C } }));
      }
      else if (sign_sum == 1) { // 2 positive
        const int negative_index = s0 == -1 ? 0 : (s1 == -1 ? 1 : 2);
        const float3& C = triangle.co[negative_index];
        const float3& a = triangle.co[(negative_index + 1) % 3];
        const float3& b = triangle.co[(negative_index + 2) % 3];
        float3 A, B;
        isect_line_plane_v3(A, a, C, plane.origin, plane.normal);
        isect_line_plane_v3(B, b, C, plane.origin, plane.normal);
        output.push_back(ClipTriangle({ { a, b, A } }));
        output.push_back(ClipTriangle({ { A, b, B } }));
      }
    }

    return output;
  }

  static bool is_object_bdk_terrain_info(Object* ob) {
    PointerRNA target_ptr = RNA_id_pointer_create(&ob->id);
    PropertyRNA* bdk_prop = RNA_struct_find_property(&target_ptr, "bdk");
    if (bdk_prop == nullptr || !RNA_property_is_set(&target_ptr, bdk_prop)) {
      return false;
    }
    PointerRNA bdk_ptr = RNA_property_pointer_get(&target_ptr, bdk_prop);
    PropertyRNA* type_prop = RNA_struct_find_property(&bdk_ptr, "type");
    if (!RNA_property_is_set(&bdk_ptr, type_prop)) {
      return false;
    }
    int type_value = RNA_property_enum_get(&bdk_ptr, type_prop);
    return ELEM(type_value, 1); // 1 is `TERRAIN_INFO`
  }

  struct TerrainInfo {
    float scale;
    int2 size;
  };

  static bool get_terrain_info(Object* ob, TerrainInfo& terrain_info) {
    PointerRNA target_ptr = RNA_id_pointer_create(&ob->id);
    PropertyRNA* bdk_prop = RNA_struct_find_property(&target_ptr, "bdk");
    if (bdk_prop == nullptr || !RNA_property_is_set(&target_ptr, bdk_prop)) {
      return false;
    }
    PointerRNA bdk_ptr = RNA_property_pointer_get(&target_ptr, bdk_prop);
    PropertyRNA* terrain_info_prop = RNA_struct_find_property(&bdk_ptr, "terrain_info");
    if (terrain_info_prop == nullptr || !RNA_property_is_set(&bdk_ptr, terrain_info_prop)) {
      return false;
    }
    PointerRNA terrain_info_ptr = RNA_property_pointer_get(&bdk_ptr, terrain_info_prop);

    // Terrain Scale
    PropertyRNA* terrain_scale_prop = RNA_struct_find_property(&terrain_info_ptr, "terrain_scale");
    if (!RNA_property_is_set(&terrain_info_ptr, terrain_scale_prop)) {
      return false;
    }
    terrain_info.scale = RNA_property_float_get(&terrain_info_ptr, terrain_scale_prop);

    // Size X
    PropertyRNA* x_size_prop = RNA_struct_find_property(&terrain_info_ptr, "x_size");
    if (!RNA_property_is_set(&terrain_info_ptr, x_size_prop)) {
      return false;
    }
    terrain_info.size.x = RNA_property_int_get(&terrain_info_ptr, x_size_prop);

    // Size Y
    PropertyRNA* y_size_prop = RNA_struct_find_property(&terrain_info_ptr, "y_size");
    if (!RNA_property_is_set(&terrain_info_ptr, y_size_prop)) {
      return false;
    }
    terrain_info.size.y = RNA_property_int_get(&terrain_info_ptr, y_size_prop);
    return true;
  }

  static void node_geo_exec(GeoNodeExecParams params) {

    using namespace blender::math;

    const Object* self = params.self_object();

    FrustumConfig config;
    config.fov = params.extract_input<float>("FOV");
    config.max_trace_distance = params.extract_input<float>("MaxTraceDistance");
    config.usize = params.extract_input<float>("USize");
    config.vsize = params.extract_input<float>("VSize");
    config.draw_scale = params.extract_input<float>("DrawScale");

    const Frustum frustum = calculate_frustum(config);

    // Frustum
    if (params.output_is_required("Frustum"))
    {
      Mesh* mesh = BKE_mesh_new_nomain(frustum.corners.size(), 12, 0, 0);

      MutableSpan<float3> positions = mesh->vert_positions_for_write();
      MutableSpan<int2> edges = mesh->edges_for_write();

      const std::array<int, 24> edge_indices = {
        0, 1, 1, 2, 2, 3, 3, 0, 4, 5, 5, 6, 6, 7, 7, 4, 0, 4, 1, 5, 2, 6, 3, 7
      };

      for (int i = 0; i < 12; ++i) {
        edges[i][0] = edge_indices[(i * 2) + 0];
        edges[i][1] = edge_indices[(i * 2) + 1];
      }

      positions.copy_from(frustum.corners);

      params.set_output("Frustum", GeometrySet::from_mesh(mesh));
    }

    // Geometry
    if (params.output_is_required("Geometry"))
    {
      Object* target = params.extract_input<Object*>("Target");

      if (target == nullptr) {
        params.set_default_remaining_outputs();
        return;
      }

      const bool transform_space_relative = true;  // TODO: make this a parameter
      const Object* self_object = params.self_object();

      const float4x4 target_matrix = float4x4(target->object_to_world());
      const float4x4 transform = float4x4(self_object->world_to_object()) * target_matrix;

      GeometrySet target_geometry = bke::object_get_evaluated_geometry_set(*target);
      if (transform_space_relative) {
        geometry::transform_geometry(target_geometry, transform);
      }

      if (target_geometry.has_mesh()) {
        const Mesh* target_mesh = target_geometry.get_mesh();
        const Span<float3> vert_positions = target_mesh->vert_positions();
        const float4x4 world_to_object(self->world_to_object());
        const float4x4 object_to_world(self->object_to_world());

        // Get axis aligned bounding box of frustum.
        std::vector<ClipTriangle> tris;

        if (is_object_bdk_terrain_info(target))
        {
          TerrainInfo terrain_info;
          get_terrain_info(target, terrain_info);

          // Figure out the min point of the terrain geometry.
          const float2 terrain_size = float2(terrain_info.size) * terrain_info.scale;
          const float2 half_size = terrain_size * 0.5f;
          const float2 terrain_min = float2(-half_size.x, -half_size.y);

          std::array<float3, 8> frustum_corners = frustum.corners;
          std::transform(frustum.corners.begin(), frustum.corners.end(), frustum_corners.begin(), [&](const float3& v) { return transform_point(object_to_world, v); });
          float3 min = float3(FLT_MAX);
          float3 max = float3(-FLT_MAX);
          for (auto& corner : frustum_corners) {
            minmax_v3v3_v3(&min.x, &max.x, corner);
          }
          min -= float3(FLT_EPSILON);
          max += float3(FLT_EPSILON);
          const float2 frustum_min = float2(min.x, min.y);
          const float2 frustum_max = float2(max.x, max.y);

          const int x_min = std::max(0, static_cast<int>(std::floor((frustum_min.x - terrain_min.x) / terrain_info.scale)));
          const int x_max = std::min(terrain_info.size.x - 1, static_cast<int>(std::ceil((frustum_max.x - terrain_min.x) / terrain_info.scale)));
          const int y_min = std::max(0, static_cast<int>(std::floor((frustum_min.y - terrain_min.y) / terrain_info.scale)));
          const int y_max = std::min(terrain_info.size.y - 1, static_cast<int>(std::ceil((frustum_max.y - terrain_min.y) / terrain_info.scale)));

          const OffsetIndices<int> faces = target_mesh->faces();
          const Span<int> corner_verts = target_mesh->corner_verts();
          for (int y = y_min; y < y_max; ++y) {
            for (int x = x_min; x < x_max; ++x) {
              const int face_index = (y * (terrain_info.size.x - 1)) + x;
              const auto face_corners = faces[face_index];
              if (face_corners.size() < 3) {
                continue;
              }
              for (auto i = 1; i < face_corners.size() - 1; ++i) {
                tris.push_back(ClipTriangle{
                  vert_positions[corner_verts[face_corners[0]]],
                  vert_positions[corner_verts[face_corners[i]]],
                  vert_positions[corner_verts[face_corners[i + 1]]],
                  });
              }
            }
          }
        }
        else
        {
          const Span<int3> corner_tris = target_mesh->corner_tris();
          const Span<int> corner_verts = target_mesh->corner_verts();
          tris.reserve(corner_tris.size());
          for (const auto& corner_tri : corner_tris) {
            tris.push_back(ClipTriangle{
              vert_positions[corner_verts[corner_tri[0]]],
              vert_positions[corner_verts[corner_tri[1]]],
              vert_positions[corner_verts[corner_tri[2]]],
              });
          }
        }

        for (const auto& plane : frustum.planes) {
          tris = clip_triangles(plane, tris);
        }

        const int verts_num = tris.size() * 3;
        const int edges_num = tris.size() * 3;
        const int loops_num = tris.size() * 3;
        const int faces_num = tris.size();
        Mesh* mesh = BKE_mesh_new_nomain(verts_num, edges_num, faces_num, loops_num);

        MutableSpan<float3> positions = mesh->vert_positions_for_write();
        MutableSpan<int2> edges = mesh->edges_for_write();
        MutableSpan<int> corner_verts = mesh->corner_verts_for_write();
        MutableSpan<int> corner_edges = mesh->corner_edges_for_write();
        MutableSpan<int> face_offsets = mesh->face_offsets_for_write();

        for (int face_index = 0; face_index < faces_num; ++face_index) {
          face_offsets[face_index] = face_index * 3;
          const int edge_index = face_index * 3;
          edges[edge_index + 0][0] = edge_index + 0;
          edges[edge_index + 0][1] = edge_index + 1;
          edges[edge_index + 1][0] = edge_index + 1;
          edges[edge_index + 1][1] = edge_index + 2;
          edges[edge_index + 2][0] = edge_index + 2;
          edges[edge_index + 2][1] = edge_index + 0;
        }

        float3* position = positions.begin();
        int j = 0;
        for (const ClipTriangle& tri : tris) {
          *position++ = tri.co[0];
          *position++ = tri.co[1];
          *position++ = tri.co[2];

          corner_verts[j + 0] = j + 0;
          corner_edges[j + 0] = j + 0;
          corner_verts[j + 1] = j + 1;
          corner_edges[j + 1] = j + 1;
          corner_verts[j + 2] = j + 2;
          corner_edges[j + 2] = j + 2;

          j += 3;
        }

        const float clip_far = frustum.planes[5].origin.x;
        const float clip_near = frustum.planes[0].origin.x;
        const float u_size_near = math::length(frustum.corners[0] - frustum.corners[1]);
        const float v_size_near = math::length(frustum.corners[0] - frustum.corners[2]);
        const float u_size_far = math::length(frustum.corners[4] - frustum.corners[5]);
        const float v_size_far = math::length(frustum.corners[4] - frustum.corners[7]);

        MutableAttributeAccessor attributes = mesh->attributes_for_write();

        // UVs
        {
          auto uv_map_id = params.get_output_anonymous_attribute_id_if_needed("UV Map");

          if (uv_map_id) {
            MutableAttributeAccessor attributes = mesh->attributes_for_write();

            SpanAttributeWriter<float2> uv_attribute = attributes.lookup_or_add_for_write_only_span<float2>(uv_map_id.value(), AttrDomain::Corner);
            MutableSpan<float2> uvs = uv_attribute.span;
            int index = 0;
            for (const auto& position : positions) {
              const float fac = (position.x - clip_near) / (clip_far - clip_near);
              // lerp the Y/Z coords based on the T-value and write to UV map
              const float u_size = interpf(u_size_far, u_size_near, fac);
              const float v_size = interpf(v_size_far, v_size_near, fac);
              const float u = 0.5f + position.y / u_size;
              const float v = 0.5f + position.z / v_size;
              uvs[index++] = { u, v };
            }

            uv_attribute.finish();
          }
        }

        // Attenutation
        {
          auto attenuation_id = params.get_output_anonymous_attribute_id_if_needed("Attenuation");

          if (attenuation_id) {
            MutableAttributeAccessor attributes = mesh->attributes_for_write();

            SpanAttributeWriter<float> attenuation_attribute = attributes.lookup_or_add_for_write_only_span<float>(attenuation_id.value(), AttrDomain::Point);
            MutableSpan<float> attenuations = attenuation_attribute.span;

            int index = 0;
            for (const auto& position : positions) {
              attenuations[index++] = 1.0f - math::clamp((position.x - clip_near) / (clip_far - clip_near), 0.f, 1.f);
            }

            attenuation_attribute.finish();
          }
        }

        BKE_id_material_eval_ensure_default_slot(&mesh->id);

        params.set_output("Geometry", GeometrySet::from_mesh(mesh));
      }
    }

    params.set_default_remaining_outputs();
  }

  static void node_register()
  {
    namespace file_ns = blender::nodes::node_geo_bdk_projector_cc;

    static blender::bke::bNodeType ntype;

    geo_node_type_base(&ntype, GEO_NODE_BDK_PROJECTOR, "BDK Projector", NODE_CLASS_GEOMETRY);
    ntype.declare = file_ns::node_declare;
    ntype.geometry_node_execute = file_ns::node_geo_exec;
    blender::bke::node_register_type(&ntype);
  }

  NOD_REGISTER_NODE(node_register);

} // namespace blender::nodes::node_type_geo_bdk_projector
