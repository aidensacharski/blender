/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "node_geometry_util.hh"

#include "BKE_material.h"

#include "DNA_material_types.h"

#include "RNA_access.hh"

using namespace blender;

namespace blender::nodes::node_geo_bdk_material_size_cc {

  static void node_declare(NodeDeclarationBuilder& b)
  {
    b.add_input<decl::Material>(N_("Material"));
    b.add_output<decl::Int>(N_("U")).dependent_field();
    b.add_output<decl::Int>(N_("V")).dependent_field();
  }

  static std::optional<vec2i> get_size_from_bdk_material(Material* material) {
    if (material == nullptr) {
      return std::nullopt;
    }
    PointerRNA material_ptr = RNA_id_pointer_create(&material->id);
    PropertyRNA* bdk_prop = RNA_struct_find_property(&material_ptr, "bdk");

    if (bdk_prop == nullptr || !RNA_property_is_set(&material_ptr, bdk_prop)) {
      return std::nullopt;
    }
    PointerRNA bdk_ptr = RNA_property_pointer_get(&material_ptr, bdk_prop);
    PropertyRNA* size_x_prop = RNA_struct_find_property(&bdk_ptr, "size_x");
    PropertyRNA* size_y_prop = RNA_struct_find_property(&bdk_ptr, "size_y");

    if (!RNA_property_is_set(&bdk_ptr, size_x_prop) || !RNA_property_is_set(&bdk_ptr, size_y_prop)) {
      return std::nullopt;
    }

    const int u = RNA_property_int_get(&bdk_ptr, size_x_prop);
    const int v = RNA_property_int_get(&bdk_ptr, size_y_prop);
    return vec2i{ u, v };
  }

  static void node_geo_exec(GeoNodeExecParams params)
  {
    Material* material = params.get_input<Material*>("Material");

    if (material == nullptr) {
      params.set_default_remaining_outputs();
      return;
    }

    const vec2i size = get_size_from_bdk_material(material).value_or(vec2i());
    params.set_output("U", size.x);
    params.set_output("V", size.y);
  }

  static void node_register()
  {
    namespace file_ns = blender::nodes::node_geo_bdk_material_size_cc;

    static blender::bke::bNodeType ntype;

    geo_node_type_base(&ntype, GEO_NODE_BDK_MATERIAL_SIZE, "BDK Material Size", NODE_CLASS_INPUT);
    ntype.declare = file_ns::node_declare;
    ntype.geometry_node_execute = file_ns::node_geo_exec;
    nodeRegisterType(&ntype);
  }

  NOD_REGISTER_NODE(node_register)

} // namespace blender::nodes::node_geo_bdk_material_size_cc
