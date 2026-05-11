"""Sidebar UI for the ModdingTool add-on."""

from __future__ import annotations

import bpy
from bpy.types import Panel

from . import native


class MT_PT_main_panel(Panel):
    bl_label = "ModdingTool"
    bl_idname = "MT_PT_main_panel"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "ModdingTool"

    def draw(self, context):
        layout = self.layout
        layout.label(text=native.version(), icon="PLUGIN")


class MT_PT_collision_panel(Panel):
    bl_label = "Collision Mesh"
    bl_idname = "MT_PT_collision_panel"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "ModdingTool"
    bl_parent_id = "MT_PT_main_panel"

    def draw(self, context):
        layout = self.layout
        col = layout.column(align=True)
        col.label(text="Generates a low-poly collision proxy")
        col.label(text="from each selected mesh.")

        op = layout.operator("moddingtool.generate_collision", text="Generate From Selected", icon="MESH_ICOSPHERE")
        op.voxel_size = 0.0
        op.relative_voxel_size = 0.05
        op.pin_boundary = True
        op.uniform_remesh = False
        op.remesh_edge_length = 0.0
        op.remesh_relative_edge_length = 0.05
        op.remesh_iterations = 5
        op.remesh_protect_boundary = True
        op.apply_modifiers = True
        op.suffix = "_collision"


class MT_PT_bake_panel(Panel):
    bl_label = "Bake Textures"
    bl_idname = "MT_PT_bake_panel"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "ModdingTool"
    bl_parent_id = "MT_PT_main_panel"

    def draw(self, context):
        layout = self.layout
        col = layout.column(align=True)
        col.label(text="Selected = HD source mesh(es)")
        col.label(text="Active = LOD bake target")

        op = layout.operator("moddingtool.bake_diffuse", text="Bake Diffuse HD -> LOD", icon="RENDER_STILL")
        op.resolution = 1024
        op.margin = 16
        op.cage_extrusion = 0.05
        op.max_ray_distance = 0.1


class MT_PT_decal_panel(Panel):
    bl_label = "Glue Decal"
    bl_idname = "MT_PT_decal_panel"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "ModdingTool"
    bl_parent_id = "MT_PT_main_panel"

    def draw(self, context):
        layout = self.layout
        col = layout.column(align=True)
        col.label(text="Select edges along the seam,")
        col.label(text="then build an L-shaped decal.")

        op = layout.operator("moddingtool.generate_glue_decal", text="Build Decal From Edges", icon="MOD_MESHDEFORM")
        op.wall_height = 0.5
        op.floor_extent = 0.3
        op.bias = 0.001
        op.flip_outward = False
        op.even_thickness = True
        op.create_uvs = True
        op.quadify = True
        op.suffix = "_decal"


class MT_PT_blend_decal_panel(Panel):
    bl_label = "Blend Decal"
    bl_idname = "MT_PT_blend_decal_panel"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "ModdingTool"
    bl_parent_id = "MT_PT_main_panel"

    def draw(self, context):
        layout = self.layout
        col = layout.column(align=True)
        col.label(text="Select edges along a ground seam,")
        col.label(text="then build a flat blend ribbon.")

        op = layout.operator("moddingtool.generate_blend_decal", text="Build Blend Decal From Edges", icon="MOD_MASK")
        op.width = 0.5
        op.center_offset = 0.0
        op.bias = 0.001
        op.flip_across = False
        op.even_thickness = True
        op.create_uvs = True
        op.quadify = True
        op.suffix = "_blend"


_classes = (MT_PT_main_panel, 
            MT_PT_collision_panel, 
            MT_PT_bake_panel,
            MT_PT_decal_panel, 
            MT_PT_blend_decal_panel)


def register():
    for c in _classes:
        bpy.utils.register_class(c)


def unregister():
    for c in reversed(_classes):
        bpy.utils.unregister_class(c)
