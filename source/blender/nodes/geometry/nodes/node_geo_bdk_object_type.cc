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

#include "RNA_access.hh"


namespace blender::nodes::node_geo_bdk_object_type_cc {

  static void node_declare(NodeDeclarationBuilder& b)
  {
    b.add_input<decl::Object>(N_("Object"));
    b.add_output<decl::Bool>(N_("Is BDK Object")).propagate_all();
    b.add_output<decl::Int>(N_("BDK Object Type")).propagate_all();
  }

  static int get_bdk_object_type(Object* ob)
  {
    PointerRNA ob_ptr = RNA_id_pointer_create(&ob->id);
    PropertyRNA* bdk_prop = RNA_struct_find_property(&ob_ptr, "bdk");
    if (bdk_prop == nullptr || !RNA_property_is_set(&ob_ptr, bdk_prop)) {
      return 0;
    }
    PointerRNA bdk_ptr = RNA_property_pointer_get(&ob_ptr, bdk_prop);
    PropertyRNA* type_prop = RNA_struct_find_property(&bdk_ptr, "type");
    if (!RNA_property_is_set(&bdk_ptr, type_prop)) {
      return 0;
    }
    return RNA_property_enum_get(&bdk_ptr, type_prop);
  }

  static void node_geo_exec(GeoNodeExecParams params) {

    using namespace blender::math;

    Object* object = params.extract_input<Object*>("Object");

    if (object == nullptr) {
      params.set_default_remaining_outputs();
      return;
    }

    const int bdk_object_type = get_bdk_object_type(object);

    params.set_output("Is BDK Object", bdk_object_type > 0);
    params.set_output("BDK Object Type", bdk_object_type);
  }

  static void node_register()
  {
    namespace file_ns = blender::nodes::node_geo_bdk_object_type_cc;

    static blender::bke::bNodeType ntype;

    geo_node_type_base(&ntype, GEO_NODE_BDK_OBJECT_TYPE, "BDK Object Type", NODE_CLASS_GEOMETRY);
    ntype.declare = file_ns::node_declare;
    ntype.geometry_node_execute = file_ns::node_geo_exec;
    blender::bke::node_register_type(&ntype);
  }

  NOD_REGISTER_NODE(node_register);

} // namespace blender::nodes::node_type_geo_bdk_object_type_cc
