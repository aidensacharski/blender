/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "node_geometry_util.hh"

#include "BLI_math_base.hh"
#include "BLI_math_geom.h"
#include "BLI_math_vector.hh"

#pragma optimize("", off)

using namespace blender;

namespace blender::nodes::node_geo_bdk_line_sphere_intersection_cc {

  static void node_declare(NodeDeclarationBuilder& b)
  {
    b.add_input<decl::Vector>(N_("Line Start"));
    b.add_input<decl::Vector>(N_("Line End"));
    b.add_input<decl::Vector>(N_("Sphere Origin"));
    b.add_input<decl::Float>(N_("Sphere Radius"));
    b.add_output<decl::Int>(N_("Intersection Count"));
    b.add_output<decl::Vector>(N_("Intersection 1"));
    b.add_output<decl::Vector>(N_("Intersection 2"));
  }

  static void node_geo_exec(GeoNodeExecParams params)
  {
    const float3 l1 = params.get_input<float3>("Line Start");
    const float3 l2 = params.get_input<float3>("Line End");
    const float3 sp = params.get_input<float3>("Sphere Origin");
    const float r = params.get_input<float>("Sphere Radius");

    float3 p[2] = {};
    float3 r_p[2] = {};
    int i = isect_line_sphere_v3(l1, l2, sp, r, p[0], p[1], true);

    // Discard intersections that are not on the line segment.
    float length = 0.0f;
    const float3 ldir = math::normalize_and_get_length(l2 - l1, length);
    int k = 0;
    int i2 = i;
    for (int j = 0; j < i2; ++j) {
      const float t = math::dot(ldir, p[j] - l1);
      if (t >= 0.0f && t <= length) {
        r_p[k++] = p[j];
      }
      else {
        --i;
      }
    }

    params.set_output("Intersection 1", r_p[0]);
    params.set_output("Intersection 2", r_p[1]);
    params.set_output("Intersection Count", i);
  }

  static void node_register()
  {
    namespace file_ns = blender::nodes::node_geo_bdk_line_sphere_intersection_cc;

    static bNodeType ntype;

    geo_node_type_base(&ntype, GEO_NODE_BDK_LINE_SPHERE_INTERSECTION, "BDK Line-Sphere Intersection", NODE_CLASS_GEOMETRY);
    ntype.declare = file_ns::node_declare;
    ntype.geometry_node_execute = file_ns::node_geo_exec;
    nodeRegisterType(&ntype);
  }

  NOD_REGISTER_NODE(node_register)

} // namespace blender::nodes::node_geo_bdk_line_sphere_intersection_cc
