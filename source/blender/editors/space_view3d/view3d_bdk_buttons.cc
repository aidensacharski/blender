/* SPDX-FileCopyrightText: 2009 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup spview3d
 */

#include <cfloat>
#include <cmath>
#include <cstdio>
#include <cstring>

#include "DNA_material_types.h"
#include "DNA_mesh_types.h"
#include "DNA_meshdata_types.h"
#include "DNA_object_types.h"
#include "DNA_scene_types.h"

#include "MEM_guardedalloc.h"

#include "BLT_translation.hh"

#include "BLI_array_utils.h"
#include "BLI_blenlib.h"
#include "BLI_math_matrix.h"
#include "BLI_math_vector.h"
#include "BLI_utildefines.h"
#include "BLI_generic_pointer.hh"

#include "BKE_attribute.hh"
#include "BKE_context.hh"
#include "BKE_customdata.hh"
#include "BKE_editmesh.hh"
#include "BKE_layer.hh"
#include "BKE_object.hh"
#include "BKE_report.hh"
#include "BKE_screen.hh"

#include "DEG_depsgraph.hh"

#include "WM_api.hh"
#include "WM_types.hh"

#include "RNA_access.hh"
#include "RNA_prototypes.h"

#include "ED_mesh.hh"
#include "ED_object.hh"
#include "ED_screen.hh"

#include "UI_interface.hh"
#include "UI_resources.hh"

#include "view3d_intern.hh" /* own include */

#pragma optimize("", off)

/* ******************* view3d space & buttons ************** */

enum EPolyFlags {
  PF_Invisible    = 0x00000001,
  PF_TwoSided     = 0x00000100,
  PF_SpecialLit   = 0x00100000,
  PF_Unlit        = 0x00400000,
  PF_FakeBackdrop = 0x00000080,
  PF_Mirror       = 0x20000000,
  PF_Portal       = 0x04000000,
  PF_AntiPortal   = 0x08000000,
};

const int poly_flag_masks[] = {
  PF_AntiPortal,
  PF_FakeBackdrop,
  PF_Invisible,
  PF_Mirror,
  PF_Portal,
  PF_SpecialLit,
  PF_TwoSided,
  PF_Unlit,
};

const char* poly_flag_strings[] = {
  "Anti Portal",
  "Fake Backdrop",
  "Invisible",
  "Mirror",
  "Portal",
  "Special Lit",
  "Two Sided",
  "Unlit",
};

enum {
  BDK_POLY_FLAGS = 0x7853,  // ?
};

/*
static void bdk_view3d_panel_bsp_surface_material(const bContext* C, Panel* panel)
{
  // Gather all the vertex positions of the face.

  // Normal
  blender::float3 normal;
  BM_face_calc_normal(actface, normal);

  // Display the normal.
  std::string normal_string = "Normal: " + std::to_string(normal[0]) + ", " + std::to_string(normal[1]) + ", " + std::to_string(normal[2]);
  uiItemL(panel->layout, normal_string.c_str(), ICON_NONE);

  BMLoop* l_first, * l_iter;
  int i = 0;
  l_iter = l_first = BM_FACE_FIRST_LOOP(actface);
  blender::Vector<blender::float3> positions;
  positions.reserve(actface->len);
  do {
    positions.append(l_iter->v->co);
  } while ((l_iter = l_iter->next) != l_first);

  // Transform the vertex positions to world space.
  blender::Vector<blender::float3> world_positions;
  for (i = 0; i < positions.size(); i++) {
    blender::float3 p;
    mul_v3_m4v3(p, ob->object_to_world, positions[i]);
    world_positions.append(p);
  }

  if (!world_positions.is_empty()) {
    // Get the bounding box of the face in world space.
    blender::float3 min, max;
    min = max = world_positions[0];
    for (i = 1; i < world_positions.size(); i++) {
      minmax_v3v3_v3(min, max, world_positions[i]);
    }

    // Print out the bounding box.
    std::string min_text = "min: " + std::to_string(min[0]) + ", " + std::to_string(min[1]) + ", " + std::to_string(min[2]);
    std::string max_text = "max: " + std::to_string(max[0]) + ", " + std::to_string(max[1]) + ", " + std::to_string(max[2]);
    uiItemL(panel->layout, min_text.c_str(), ICON_NONE);
    uiItemL(panel->layout, max_text.c_str(), ICON_NONE);
  }

  // Get the material at the material index.
  Material* mat = nullptr;
  if (actface->mat_nr >= 0 && actface->mat_nr < me->totcol) {
    mat = me->mat[actface->mat_nr];
  }

  if (mat != nullptr) {
    // Try to get the BDK material properties from the RNA.
    PointerRNA matptr = RNA_id_pointer_create(&mat->id);
    PropertyRNA* bdkprop = RNA_struct_find_property(&matptr, "bdk");

    // Get "size_x" and "size_y" from the BDK material properties.
    if (RNA_property_is_set(&matptr, bdkprop)) {
      PointerRNA bdkptr = RNA_property_pointer_get(&matptr, bdkprop);
      PropertyRNA* size_x_prop = RNA_struct_find_property(&bdkptr, "size_x");
      PropertyRNA* size_y_prop = RNA_struct_find_property(&bdkptr, "size_y");
      if (RNA_property_is_set(&bdkptr, size_x_prop) && RNA_property_is_set(&bdkptr, size_y_prop)) {
        float size_x = RNA_property_float_get(&bdkptr, size_x_prop);
        float size_y = RNA_property_float_get(&bdkptr, size_y_prop);
        // Draw the size_x and size_y properties.
        uiLayout* col = uiLayoutColumn(panel->layout, false);
        uiItemR(panel->layout, &bdkptr, "size_x", UI_ITEM_R_EXPAND, nullptr, ICON_NONE);
        uiItemR(panel->layout, &bdkptr, "size_x", UI_ITEM_R_EXPAND, nullptr, ICON_NONE);
      }
    }
  }
}
*/

const char* bdk_poly_flags_attribute_name = "bdk.poly_flags";
const char* bdk_read_only_attribute_name = "bdk.read_only";

static void bdk_view3d_panel_bsp_surface_poly_flags(const bContext* C, Panel* panel)
{
  const Scene* scene = CTX_data_scene(C);
  ViewLayer* view_layer = CTX_data_view_layer(C);
  BKE_view_layer_synced_ensure(scene, view_layer);
  Object* ob = BKE_view_layer_active_object_get(view_layer);
  BMEditMesh* em = BKE_editmesh_from_object(ob);
  BMesh* bm = em->bm;
  BMFace* actface = bm->act_face;
  Mesh *me = ED_mesh_context(C);

  if (actface == nullptr) {
    return;
  }

  AttributeOwner owner = AttributeOwner::from_id(&me->id);
  CustomDataLayer* poly_flags_layer = BKE_attribute_find(owner, bdk_poly_flags_attribute_name, CD_PROP_INT32, blender::bke::AttrDomain::Face);
  CustomDataLayer* read_only_layer = BKE_attribute_find(owner, bdk_read_only_attribute_name, CD_PROP_BOOL, blender::bke::AttrDomain::Face);
  if (poly_flags_layer == nullptr) {
    return;
  }
  const int face_poly_flags = BM_ELEM_CD_GET_INT(actface, poly_flags_layer->offset);

  uiBlock* block = uiLayoutGetBlock(panel->layout);

  // Disable the block if the face is read-only.
  const bool is_read_only = read_only_layer != nullptr && BM_ELEM_CD_GET_BOOL(actface, read_only_layer->offset);
  uiLayoutSetEnabled(panel->layout, !is_read_only);

  int yi = 0;
  int butw = 128;
  int buth = 20 * UI_SCALE_FAC;

    // Display a button togglable for each poly flag.
  for (int i = 0; i < 8; i++) {
    const int flag_mask = poly_flag_masks[i];
    const bool is_set = (face_poly_flags & flag_mask) != 0;

    auto but = uiDefBut(block, UI_BTYPE_TOGGLE, 0,
      IFACE_(poly_flag_strings[i]),
      0, yi -= buth,
      butw, buth,
      nullptr, 0, 0, nullptr
    );
    UI_but_func_set(but, [C, poly_flags_layer, face_poly_flags, flag_mask](bContext& context) {
      Mesh* me = ED_mesh_context(&context);
      const Scene* scene = CTX_data_scene(C);
      ViewLayer* view_layer = CTX_data_view_layer(C);
      BKE_view_layer_synced_ensure(scene, view_layer);
      Object* ob = BKE_view_layer_active_object_get(view_layer);
      BMEditMesh* em = BKE_editmesh_from_object(ob);
      BMesh* bm = em->bm;
      BMFace* actface = bm->act_face;
      BM_ELEM_CD_SET_INT(actface, poly_flags_layer->offset, face_poly_flags ^ flag_mask);
      EDBMUpdate_Params update{};
      update.calc_looptris = false;
      update.calc_normals = false;
      update.is_destructive = false;
      EDBM_update(me, &update);
    });
    UI_but_func_pushed_state_set(but, [is_set](const uiBut&) {
      return is_set;
    });
  }
}

static bool bdk_view3d_panel_bsp_surface_poly_flags_poll(const bContext* /*C*/, PanelType* /*pt*/)
{
  return true;
}

static void bdk_view3d_panel_bsp_surface(const bContext* /*C*/, Panel* /*panel*/)
{
  // Do nothing, subpanels hold all the functionality.
}

static bool is_object_bsp_brush_or_level(Object* ob)
{
  PointerRNA ob_ptr = RNA_id_pointer_create(&ob->id);
  PropertyRNA* bdk_prop = RNA_struct_find_property(&ob_ptr, "bdk");
  if (bdk_prop == nullptr || !RNA_property_is_set(&ob_ptr, bdk_prop)) {
    return false;
  }
  PointerRNA bdk_ptr = RNA_property_pointer_get(&ob_ptr, bdk_prop);
  PropertyRNA* type_prop = RNA_struct_find_property(&bdk_ptr, "type");
  if (!RNA_property_is_set(&bdk_ptr, type_prop)) {
    return false;
  }
  int type_value = RNA_property_enum_get(&bdk_ptr, type_prop);
  return ELEM(type_value, 3, 5); // 3 IS BSP_BRUSH. 5 is LEVEL. Without a non-const bContext, we can't query the enum value.
}

static bool bdk_view3d_panel_bsp_surface_poll(const bContext* C, PanelType* /*pt*/)
{
  const Scene* scene = CTX_data_scene(C);
  ViewLayer* view_layer = CTX_data_view_layer(C);
  BKE_view_layer_synced_ensure(scene, view_layer);
  Object* ob = BKE_view_layer_active_object_get(view_layer);
  if (ob == nullptr || ob->type != OB_MESH) {
    return false;
  }

  if (!is_object_bsp_brush_or_level(ob)) {
    return false;
  }

  BMEditMesh* em = BKE_editmesh_from_object(ob);
  BMesh* bm = em ? em->bm : nullptr;
  return em != nullptr && em->selectmode & SCE_SELECT_FACE && bm->totfacesel > 0;
}

void bdk_view3d_buttons_register(ARegionType* art)
{
  PanelType* pt;

  pt = static_cast<PanelType*>(MEM_callocN(sizeof(PanelType), "bdk spacetype view3d panel object"));
  STRNCPY(pt->idname, "BDK_PT_bsp_surface");
  STRNCPY(pt->label, N_("Surface"));
  STRNCPY(pt->category, "BDK");
  STRNCPY(pt->translation_context, BLT_I18NCONTEXT_DEFAULT_BPYRNA);
  pt->space_type = SPACE_VIEW3D;
  pt->draw = bdk_view3d_panel_bsp_surface;
  pt->poll = bdk_view3d_panel_bsp_surface_poll;
  BLI_addtail(&art->paneltypes, pt);

  PanelType* pt_surface = pt;

  pt = static_cast<PanelType*>(MEM_callocN(sizeof(PanelType), "bdk spacetype view3d panel object"));
  STRNCPY(pt->idname, "BDK_PT_bsp_surface_poly_flags");
  STRNCPY(pt->label, N_("Flags"));
  STRNCPY(pt->category, "BDK");
  STRNCPY(pt->translation_context, BLT_I18NCONTEXT_DEFAULT_BPYRNA);
  STRNCPY(pt->parent_id, "BDK_pt_bsp_surface");
  pt->parent = pt_surface;
  pt->space_type = SPACE_VIEW3D;
  pt->draw = bdk_view3d_panel_bsp_surface_poly_flags;
  pt->poll = bdk_view3d_panel_bsp_surface_poly_flags_poll;
  BLI_addtail(&pt_surface->children, BLI_genericNodeN(pt));
  BLI_addtail(&art->paneltypes, pt);
}
