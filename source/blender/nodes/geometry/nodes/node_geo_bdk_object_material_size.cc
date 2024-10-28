/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_mesh.h"

#include "node_geometry_util.hh"

#include "BKE_material.h"

#include "DNA_material_types.h"

#include "RNA_access.hh"

using namespace blender;

namespace blender::nodes::node_geo_bdk_object_material_size_cc {

  static void node_declare(NodeDeclarationBuilder& b)
  {
    b.add_input<decl::Object>(N_("Object"));
    b.add_input<decl::Int>(N_("Material Index")).implicit_field(implicit_field_inputs::index);
    b.add_output<decl::Int>(N_("U")).dependent_field();
    b.add_output<decl::Int>(N_("V")).dependent_field();
  }

  class SampleFunction : public mf::MultiFunction {
  private:
    Vector<vec2i> sizes;

  public:
    SampleFunction(Vector<vec2i>&& sizes) :
      sizes(std::move(sizes))
    {
      static const mf::Signature signature = []() {
        mf::Signature signature;
        mf::SignatureBuilder builder{ "BDK Obect Material Size", signature };
        builder.single_input<int>("Material Index");
        builder.single_output<int>("U"/*, mf::ParamFlag::SupportsUnusedOutput*/);
        builder.single_output<int>("V"/*, mf::ParamFlag::SupportsUnusedOutput*/);
        return signature;
        }();
        this->set_signature(&signature);
    }

    void call(const IndexMask& mask, mf::Params params, mf::Context /*context*/) const override
    {
      const VArray<int>& material_indices = params.readonly_single_input<int>(0, "Material Index");
      MutableSpan<int> u = params.uninitialized_single_output<int>(1, "U");
      MutableSpan<int> v = params.uninitialized_single_output<int>(2, "V");

      index_mask::masked_fill(u, 0, mask);
      index_mask::masked_fill(v, 0, mask);

      mask.foreach_index_optimized<int>(GrainSize(512), [&](const int j) {
        u[j] = sizes[material_indices[j]].x;
        v[j] = sizes[material_indices[j]].y;
        });
    }
  };

  static void get_size_from_bdk_material(Material* material, int& u, int& v) {
    if (material == nullptr) {
      u = 0;
      v = 0;
      return;
    }
    PointerRNA material_ptr = RNA_id_pointer_create(&material->id);
    PropertyRNA* bdk_prop = RNA_struct_find_property(&material_ptr, "bdk");

    if (bdk_prop == nullptr || !RNA_property_is_set(&material_ptr, bdk_prop)) {
      u = 0;
      v = 0;
      return;
    }
    PointerRNA bdk_ptr = RNA_property_pointer_get(&material_ptr, bdk_prop);

    // U
    PropertyRNA* size_x_prop = RNA_struct_find_property(&bdk_ptr, "size_x");
    if (!RNA_property_is_set(&bdk_ptr, size_x_prop)) {
      u = 0;
    }
    else {
      u = RNA_property_int_get(&bdk_ptr, size_x_prop);
    }

    // V
    PropertyRNA* size_y_prop = RNA_struct_find_property(&bdk_ptr, "size_y");
    if (!RNA_property_is_set(&bdk_ptr, size_y_prop)) {
      v = 0;
    }
    else {
      v = RNA_property_int_get(&bdk_ptr, size_y_prop);
    }
  }

  static void node_geo_exec(GeoNodeExecParams params)
  {
    Object* object = params.get_input<Object*>("Object");

    if (object == nullptr) {
      params.set_default_remaining_outputs();
      return;
    }

    // Populate the materials vector.
    short* materials_length = BKE_object_material_len_p(object);
    if (materials_length == nullptr) {
      params.set_default_remaining_outputs();
      return;
    }
    Vector<vec2i> sizes = Vector<vec2i>{ *materials_length };
    for (short i = 0; i < *materials_length; ++i) {
      int u, v;
      Material* material = BKE_object_material_get(object, i + 1);
      get_size_from_bdk_material(material, u, v);
      sizes[i] = { u, v };
    }

    // Get the material index and do a lookup against the object.
    Field<int> material_index_field = params.get_input<Field<int>>("Material Index");

    auto sample_fn = std::make_unique<SampleFunction>(std::move(sizes));
    auto sample_op = FieldOperation::Create(std::move(sample_fn), { std::move(material_index_field) });

    params.set_output("U", Field<int>(sample_op, 0));
    params.set_output("V", Field<int>(sample_op, 1));
  }

  static void node_register()
  {
    namespace file_ns = blender::nodes::node_geo_bdk_object_material_size_cc;

    static blender::bke::bNodeType ntype;

    geo_node_type_base(&ntype, GEO_NODE_BDK_OBJECT_MATERIAL_SIZE, "BDK Object Material Size", NODE_CLASS_INPUT);
    ntype.declare = file_ns::node_declare;
    ntype.geometry_node_execute = file_ns::node_geo_exec;
    blender::bke::node_register_type(&ntype);
  }

  NOD_REGISTER_NODE(node_register)

} // namespace blender::nodes::node_geo_bdk_object_material_size_cc
