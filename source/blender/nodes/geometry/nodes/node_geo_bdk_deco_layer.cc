/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_mesh.h"

#include "DNA_pointcloud_types.h"

#include "node_geometry_util.hh"

#include "BKE_pointcloud.hh"

#include "BLI_math_base.hh"
#include "BLI_math_vector.hh"
#include "BLI_task.hh"

using namespace blender;

class unreal_random_number_engine {
public:
  using self_type = unreal_random_number_engine;
  using seed_type = long;
  using result_type = float;

  unreal_random_number_engine() :
    _seed(0) {
  }
  unreal_random_number_engine(const self_type& x) :
    _seed(x._seed) {
  }
  unreal_random_number_engine(seed_type s) :
    _seed(s) {
  }

  void seed() {
    _seed = 0;
    SRandTemp = 1.0f;
  }

  void seed(seed_type s) {
    _seed = s;
  }

  //void seed(SeedSequence q);

  float operator()() {
    _seed = (_seed * 196314165) + 907633515;
    float result;
    *(long*)&result = (*(long*)&SRandTemp & 0xff800000) | (_seed & 0x007fffff);
    return result - 1.0f;
  }

  void discard(size_t z) {
    while (z--) {
      _seed = (_seed * 196314165) + 907633515;
    }
  }

  bool operator==(const self_type& other) {
    return _seed == other._seed;
  }

  bool operator!=(const self_type& other) {
    return _seed != other._seed;
  }

private:
  seed_type _seed = 0;
  float SRandTemp = 1.0f;
};

namespace blender::nodes::node_geo_bdk_deco_layer_cc {

  static void node_declare(NodeDeclarationBuilder& b)
  {
    b.add_input<decl::Geometry>(N_("Terrain"));
    b.add_input<decl::Int>(N_("Heightmap X")).min(0).max(512);
    b.add_input<decl::Int>(N_("Heightmap Y")).min(0).max(512);
    b.add_input<decl::Int>(N_("Sector Size"))
      .default_value(16)
      .min(1)
      .max(32)
      .description(
        N_("The number of quads per terrain sector along a single axis"));
    b.add_input<decl::Int>(N_("Max Per Quad"))
      .default_value(1)
      .min(1)
      .max(2)
      .description(N_("The maximum number of decorations per quad"));
    b.add_input<decl::Int>(N_("Seed")).min(-2147483648).max(2147483647);

    b.add_input<decl::Float>(N_("Offset")); // DecoLayerOffset

    // Density
    b.add_input<decl::Float>(N_("Density Map")).subtype(PROP_FACTOR).min(0).max(1).default_value(1.f).field_on_all();
    b.add_input<decl::Float>(N_("Density Multiplier Min")).subtype(PROP_FACTOR).default_value(1.f);
    b.add_input<decl::Float>(N_("Density Multiplier Max")).subtype(PROP_FACTOR).default_value(1.f);

    // Scale
    b.add_input<decl::Float>(N_("Scale Map")).default_value(1.0f).field_on_all();
    b.add_input<decl::Vector>(N_("Scale Multiplier Min")).default_value(float3(1.f));
    b.add_input<decl::Vector>(N_("Scale Multiplier Max")).default_value(float3(1.f));

    // Color
    b.add_input<decl::Color>(N_("Color Map")).default_value(ColorGeometry4f(1.0f, 1.0f, 1.0f, 1.0f)).field_on_all();

    b.add_input<decl::Bool>(N_("Show On Invisible Terrain"));
    b.add_input<decl::Bool>(N_("Align To Terrain"));
    b.add_input<decl::Bool>(N_("Random Yaw")).default_value(true);
    b.add_input<decl::Bool>(N_("Inverted")); // if the terrain is upside down or not

    b.add_output<decl::Geometry>(N_("Points")).propagate_all();
    b.add_output<decl::Vector>(N_("Rotation")).subtype(PROP_EULER).field_on_all();
    b.add_output<decl::Vector>(N_("Scale")).field_on_all();
  }

  float3 normal_to_euler(const float3& normal) {
    // Find yaw
    float yaw = math::atan2(normal.y, normal.x);
    float pitch = math::atan2(normal.z, math::sqrt(normal.x * normal.x + normal.y * normal.y));
    const float roll = 0.0f;
    pitch -= M_PI_2;
    float3 euler = { roll, -pitch, yaw };
    return euler;
  }

  struct UpdateDecorationsParams
  {
    const Mesh* mesh;
    int seed;
    int max_per_quad;
    int heightmap_x;
    bool show_on_invisible_terrain;
    int sector_size;
    float density_multiplier_min;
    float density_multiplier_max;
    float offset;
    bool inverted;
    bool align_to_terrain;
    bool random_yaw;
    float3 scale_multiplier_min;
    float3 scale_multiplier_max;
    float scale_map;
  };

  static void update_decorations(const UpdateDecorationsParams& params, long sector_index, const VArray<float>& density_map, const VArray<int> material_indices, Vector<float3>& r_positions, Vector<float3>& r_rotations, Vector<float3>& r_scales)
  {
    if (params.mesh == nullptr) {
      return;
    }

    unreal_random_number_engine random(params.seed + sector_index);

    // TODO: need to take into account that the quads can overrun the sector size
    const int quads_per_row = (params.heightmap_x - 1);
    const int quads_per_column = (params.heightmap_x - 1);
    const float density_multiplier_range = params.density_multiplier_max - params.density_multiplier_min;
    //const int quads_per_sector = sector_size * sector_size;
    const int sectors_per_row = static_cast<int>(ceil(static_cast<float>(quads_per_row) / params.sector_size));
    const int sector_x = sector_index % sectors_per_row;
    const int sector_y = sector_index / sectors_per_row;
    const int sector_x_offset = sector_x * params.sector_size;
    const int sector_y_offset = sector_y * params.sector_size;
    const int quads_y = math::min(params.sector_size, quads_per_column - sector_y_offset);
    const int quads_x = math::min(params.sector_size, quads_per_row - sector_x_offset);
    const int vertex_count = params.heightmap_x * params.heightmap_x;
    int quad_index = (quads_per_row * sector_y_offset) + sector_x_offset;
    int quad_row_stride = quads_per_row - quads_x;
    const float3 scale_multiplier_range = params.scale_multiplier_max - params.scale_multiplier_min;

    const Span<float3> vert_positions = params.mesh->vert_positions();

    if (vert_positions.size() != vertex_count) {
      return;
    }

    // Check if the density map has any non-zero values. If it's all zeroes, just skip it entirely.
    // We can't just simply check in the iterator because the state of the random number generator
    // would be divergent from that of the SDK.
    const auto has_non_zero_density = [&]() {
      for (int y = 0; y < quads_y; ++y) {
        const int vertex_y = sector_y_offset + y;
        for (int x = 0; x < quads_x; ++x) {
          const int vertex_x = sector_x_offset + x;
          const int vertex_index = (vertex_y * params.heightmap_x) + vertex_x;
          const float density = density_map[vertex_index];
          if (density != 0.0f) {
            return true;
          }
        }
      }
      return false;
    };

    if (!has_non_zero_density()) {
      return;
    }

    for (int y = 0; y < quads_y; ++y, quad_index += quad_row_stride)
    {
      const int vertex_y = sector_y_offset + y;
      int vertex_index = vertex_y * params.heightmap_x + sector_x_offset;

      for (int x = 0; x < quads_x; ++x, ++quad_index, ++vertex_index)
      {
        const float density = density_map[vertex_index];

        // TODO: a little flimsy, make this index variable in case something changes upstream (or use attributes)
        if (!params.show_on_invisible_terrain && material_indices[quad_index] == 1) {
          continue;
        }

        for (int i = 0; i < params.max_per_quad; ++i) {
          const float density_multiplier = params.density_multiplier_min + density_multiplier_range * random();

          if (random() >= (density * density_multiplier)) {
            continue;
          }

          const float rand_x = random();
          const float rand_y = random();

          float3 normal;
          float3 location;

          if (rand_x > rand_y) {
            // TODO: optimization: we only need to calculate the Z for the direction vectors, the X & Y are always the same
            const float3 dir_x = vert_positions[vertex_index] - vert_positions[vertex_index + 1];
            const float3 dir_y = vert_positions[vertex_index + params.heightmap_x + 1] - vert_positions[vertex_index + 1];
            location = vert_positions[vertex_index + 1] + (dir_x * (1.f - rand_x)) + (dir_y * rand_y);
            normal = math::cross(dir_x, dir_y);
          } else {
            const float3 dir_x = vert_positions[vertex_index] - vert_positions[vertex_index + params.heightmap_x];
            const float3 dir_y = vert_positions[vertex_index + params.heightmap_x + 1] - vert_positions[vertex_index + params.heightmap_x];
            location = vert_positions[vertex_index + params.heightmap_x] + (dir_x * rand_x) + (dir_y * (1.f - rand_y));
            normal = math::cross(dir_x, dir_y);
          }

          normal = math::normalize(normal);

          if (normal.z < 0) {
            normal *= -1.f;
          }

          if (params.inverted) {
            normal *= -1.f;
          }

          if (!params.align_to_terrain) {
            if (params.inverted) {
              normal = float3(0, 0, -1);
            }
            else {
              normal = float3(0, 0, 1);
            }
          }

          float3 rotation = normal_to_euler(normal);
          if (params.random_yaw) {
            rotation[2] = random() * M_PI * 2;
          }

          location += normal * params.offset;
          float3 scale;
          // NOTE: WE don't use the constructor here because the evaluation order of the arguments is not guaranteed.
          scale[0] = params.scale_multiplier_min[0] + (scale_multiplier_range[0] * random()) * params.scale_map;
          scale[1] = params.scale_multiplier_min[1] + (scale_multiplier_range[1] * random()) * params.scale_map;
          scale[2] = params.scale_multiplier_min[2] + (scale_multiplier_range[2] * random()) * params.scale_map;

          r_positions.append(location);
          r_rotations.append(rotation);
          r_scales.append(scale);
        }
      }
    }
  }

  namespace {
  struct AttributeOutputs {
    AnonymousAttributeIDPtr rotation_id;
    AnonymousAttributeIDPtr scale_id;
  };
}  // namespace

  static void node_geo_exec(GeoNodeExecParams params)
  {
    GeometrySet geometry_set = params.get_input<GeometrySet>("Terrain");

    if (!geometry_set.has_mesh()) {
      params.set_default_remaining_outputs();
      return;
    }

    const int heightmap_x = params.get_input<int>("Heightmap X");
    const int heightmap_y = params.get_input<int>("Heightmap Y");
    const int vertex_count = heightmap_x * heightmap_y;

    const Mesh* mesh = geometry_set.get_mesh();

    if (mesh->totvert != vertex_count) {
      params.error_message_add(NodeWarningType::Error, "Incorrect vertex count");
      params.set_default_remaining_outputs();
      return;
    }

    const int sector_size = params.get_input<int>("Sector Size");
    const int sectors_x = static_cast<int>(math::ceil(static_cast<float>(heightmap_x) / sector_size));
    const int sectors_y = static_cast<int>(math::ceil(static_cast<float>(heightmap_y) / sector_size));
    const int sector_count = sectors_x * sectors_y;

    lazy_threading::send_hint();

    auto start = std::chrono::high_resolution_clock().now();

    // Density Map
    Vector<const GeometryComponent*> components = geometry_set.get_components();
    const Field<float> density_map_field = params.get_input<Field<float>>("Density Map");
    VArray<float> density_map;
    for (const GeometryComponent* component : components) {
      const std::optional<AttributeAccessor> attributes = component->attributes();
      if (!attributes.has_value()) {
        continue;
      }
      if (attributes->domain_supported(ATTR_DOMAIN_POINT)) {
        bke::GeometryFieldContext field_context{ *component, ATTR_DOMAIN_POINT };
        const int domain_num = attributes->domain_size(ATTR_DOMAIN_POINT);

        fn::FieldEvaluator data_evaluator{ field_context, domain_num };
        data_evaluator.add(density_map_field);
        data_evaluator.evaluate();
        density_map = std::move(data_evaluator.get_evaluated<float>(0));
      }
    }

    struct Bucket {
      Vector<float3> positions;
      Vector<float3> rotations;
      Vector<float3> scales;
    };
    Vector<Bucket> buckets;

    const size_t sectors_per_bucket = 64;
    const int bucket_count = static_cast<int>(math::ceil(static_cast<float>(sector_count) / sectors_per_bucket));
    buckets.resize(bucket_count);

    int point_count = 0;
    const AttributeAccessor attributes = mesh->attributes();
    const VArray<int> material_indices = *attributes.lookup_or_default<int>("material_index", ATTR_DOMAIN_FACE, 0);

    UpdateDecorationsParams update_params = {};
    update_params.mesh = mesh;
    update_params.seed = static_cast<long>(params.get_input<int>("Seed"));
    update_params.max_per_quad = params.get_input<int>("Max Per Quad");
    update_params.heightmap_x = params.get_input<int>("Heightmap X");
    update_params.show_on_invisible_terrain = params.get_input<bool>("Show On Invisible Terrain");
    update_params.sector_size = params.get_input<int>("Sector Size");
    update_params.density_multiplier_min = params.get_input<float>("Density Multiplier Min");
    update_params.density_multiplier_max = params.get_input<float>("Density Multiplier Max");
    update_params.offset = params.get_input<float>("Offset");
    update_params.inverted = params.get_input<bool>("Inverted");
    update_params.align_to_terrain = params.get_input<bool>("Align To Terrain");
    update_params.random_yaw = params.get_input<bool>("Random Yaw");
    update_params.scale_map = params.get_input<Field<float>>("Scale Map"); // TODO: not right! needs to be gleaned from field
    update_params.scale_multiplier_min = params.get_input<float3>("Scale Multiplier Min");
    update_params.scale_multiplier_max = params.get_input<float3>("Scale Multiplier Max");

    threading::parallel_for_each(IndexRange(bucket_count), [&](const int bucket_index) {
      const int sector_index = bucket_index * sectors_per_bucket;
      Bucket& bucket = buckets[bucket_index];
      for (const int sector_index : IndexRange(sector_index, sectors_per_bucket)) {
        update_decorations(update_params, sector_index, density_map, material_indices, bucket.positions, bucket.rotations, bucket.scales);
        point_count += bucket.positions.size();
      }
    });

    Vector<float3> positions;
    Vector<float3> rotations;
    Vector<float3> scales;

    positions.reserve(point_count);
    rotations.reserve(point_count);
    scales.reserve(point_count);

    for (const auto& bucket : buckets) {
      positions.extend(bucket.positions.as_span());
      rotations.extend(bucket.rotations.as_span());
      scales.extend(bucket.scales.as_span());
    }

    PointCloud* pointcloud = BKE_pointcloud_new_nomain(positions.size());
    bke::MutableAttributeAccessor point_attributes = pointcloud->attributes_for_write();
    bke::SpanAttributeWriter<float3> point_positions =
      point_attributes.lookup_or_add_for_write_only_span<float3>("position", ATTR_DOMAIN_POINT);
    point_positions.span.copy_from(positions);
    point_positions.finish();

    AttributeOutputs attribute_outputs;
    attribute_outputs.rotation_id = params.get_output_anonymous_attribute_id_if_needed("Rotation");
    attribute_outputs.scale_id = params.get_output_anonymous_attribute_id_if_needed("Scale");

    SpanAttributeWriter<float3> rotations_writer;
    SpanAttributeWriter<float3> scales_writer;

    if (attribute_outputs.rotation_id) {
      rotations_writer = point_attributes.lookup_or_add_for_write_only_span<float3>(
        attribute_outputs.rotation_id.get(), ATTR_DOMAIN_POINT);
    }

    if (attribute_outputs.scale_id) {
      scales_writer = point_attributes.lookup_or_add_for_write_only_span<float3>(
        attribute_outputs.scale_id.get(), ATTR_DOMAIN_POINT);
    }

    Map<AttributeIDRef, AttributeKind> attributes_to_propagate;
    geometry_set.gather_attributes_for_propagation({ GeometryComponent::Type::Mesh },
      GeometryComponent::Type::PointCloud,
      false,
      params.get_output_propagation_info("Points"),
      attributes_to_propagate);

    rotations_writer.span.copy_from(rotations);
    rotations_writer.finish();

    scales_writer.span.copy_from(scales);
    scales_writer.finish();

    params.set_output("Points", GeometrySet::from_pointcloud(pointcloud));
  }

  void node_register()
  {
    namespace file_ns = blender::nodes::node_geo_bdk_deco_layer_cc;

    static bNodeType ntype;

    geo_node_type_base(&ntype, GEO_NODE_BDK_DECO_LAYER, "BDK DecoLayer", NODE_CLASS_GEOMETRY);
    ntype.declare = file_ns::node_declare;
    ntype.geometry_node_execute = file_ns::node_geo_exec;
    nodeRegisterType(&ntype);
  }

  NOD_REGISTER_NODE(node_register)

} // namespace blender::nodes::node_geo_bdk_deco_layer_cc
