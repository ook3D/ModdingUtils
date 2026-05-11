from __future__ import annotations

import math

import bpy
import bmesh
from bpy.props import (
    BoolProperty,
    FloatProperty,
    FloatVectorProperty,
    IntProperty,
    StringProperty,
)
from bpy.types import Operator
from mathutils import Matrix, Vector

from . import native


def _decompose_for_props(m: Matrix) -> tuple[tuple, tuple, tuple]:
    loc, rot_q, scale = m.decompose()
    return (
        (loc.x, loc.y, loc.z),
        (rot_q.w, rot_q.x, rot_q.y, rot_q.z),
        (scale.x, scale.y, scale.z),
    )


def _compose_from_props(loc3, quat4, scale3) -> Matrix:
    from mathutils import Quaternion
    q = Quaternion((quat4[0], quat4[1], quat4[2], quat4[3]))
    return Matrix.LocRotScale(Vector(loc3), q, Vector(scale3))


def _apply_world_transform(decal_obj: bpy.types.Object, world: Matrix) -> None:
    loc, rot_q, scale = world.decompose()
    decal_obj.parent = None
    decal_obj.matrix_parent_inverse = Matrix.Identity(4)
    mode = decal_obj.rotation_mode
    if mode == "QUATERNION":
        decal_obj.rotation_quaternion = rot_q
    elif mode == "AXIS_ANGLE":
        axis, angle = rot_q.to_axis_angle()
        decal_obj.rotation_axis_angle = (angle, axis.x, axis.y, axis.z)
    else:
        decal_obj.rotation_euler = rot_q.to_euler(mode)
    decal_obj.location = loc
    decal_obj.scale = scale


def _quadify_mesh(me: bpy.types.Mesh) -> None:
    bm = bmesh.new()
    try:
        bm.from_mesh(me)
        bmesh.ops.join_triangles(
            bm,
            faces=bm.faces[:],
            cmp_seam=False,
            cmp_sharp=False,
            cmp_uvs=False,
            cmp_vcols=False,
            cmp_materials=False,
            angle_face_threshold=math.radians(40.0),
            angle_shape_threshold=math.radians(40.0),
        )
        bm.to_mesh(me)
        me.update()
    finally:
        bm.free()


def _triangulated_world_mesh(obj: bpy.types.Object, apply_modifiers: bool, depsgraph: bpy.types.Depsgraph):
    src = obj.evaluated_get(depsgraph) if apply_modifiers else obj
    me = src.to_mesh()
    try:
        bm = bmesh.new()
        try:
            bm.from_mesh(me)
            bmesh.ops.triangulate(bm, faces=bm.faces[:])
            mat = obj.matrix_world

            verts = [0.0] * (len(bm.verts) * 3)
            bm.verts.ensure_lookup_table()
            for i, v in enumerate(bm.verts):
                wv = mat @ v.co
                verts[i * 3] = wv.x
                verts[i * 3 + 1] = wv.y
                verts[i * 3 + 2] = wv.z

            tris = [0] * (len(bm.faces) * 3)
            for i, f in enumerate(bm.faces):
                tris[i * 3] = f.verts[0].index
                tris[i * 3 + 1] = f.verts[1].index
                tris[i * 3 + 2] = f.verts[2].index
            return verts, tris
        finally:
            bm.free()
    finally:
        src.to_mesh_clear()


def _build_blender_mesh(name: str, verts, tris) -> bpy.types.Mesh:
    me = bpy.data.meshes.new(name)
    me.from_pydata([Vector(v) for v in verts], [], list(tris))
    me.update(calc_edges=True)
    return me


class MT_OT_generate_collision(Operator):
    bl_idname = "moddingtool.generate_collision"
    bl_label = "Generate Collision Mesh"
    bl_description = ("Build a low poly collision proxy for each selected mesh")
    bl_options = {"REGISTER", "UNDO"}

    voxel_size: FloatProperty(
        name="Voxel Size",
        description="Absolute voxel size in world units. 0 = use Relative.",
        default=0.0,
        min=0.0,
        soft_max=10.0,
        unit="LENGTH",
    )
    relative_voxel_size: FloatProperty(
        name="Relative Voxel Size",
        description="Voxel size as a fraction of the bounding box diagonal",
        default=0.05,
        min=0.001,
        max=0.5,
        precision=3,
    )
    pin_boundary: BoolProperty(
        name="Pin Boundary",
        description="Cells touching an open mesh edge collapse to the average "
                    "of just the boundary verts in that cell, holding the "
                    "silhouette in place instead of pulling it inward",
        default=True,
    )
    uniform_remesh: BoolProperty(
        name="Uniform Remesh",
        description="Run native isotropic triangle remeshing on the collision "
                    "mesh: split/collapse/flip until edge lengths are roughly "
                    "uniform and valences are regular",
        default=False,
    )
    remesh_edge_length: FloatProperty(
        name="Target Edge Length",
        description="Target edge length in world units. 0 = use Relative.",
        default=0.0,
        min=0.0,
        soft_max=10.0,
        unit="LENGTH",
    )
    remesh_relative_edge_length: FloatProperty(
        name="Relative Edge Length",
        description="Target edge length as a fraction of the bounding box "
                    "diagonal (used when Target Edge Length is 0)",
        default=0.05,
        min=0.001,
        max=0.5,
        precision=3,
    )
    remesh_iterations: IntProperty(
        name="Iterations",
        description="Number of split/collapse/flip/smooth passes",
        default=5,
        min=1,
        max=50,
    )
    remesh_protect_boundary: BoolProperty(
        name="Protect Boundary (Remesh)",
        description="Freeze boundary vertices during remeshing so the "
                    "silhouette doesn't drift",
        default=True,
    )
    apply_modifiers: BoolProperty(
        name="Apply Modifiers",
        description="Use the modifier-evaluated mesh as input",
        default=True,
    )
    suffix: StringProperty(
        name="Suffix",
        description="Suffix appended to the source object name for the collision output",
        default="_collision",
    )

    @classmethod
    def poll(cls, context):
        return any(o.type == "MESH" for o in context.selected_objects)

    def execute(self, context):
        sources = [o for o in context.selected_objects if o.type == "MESH"]
        if not sources:
            self.report({"WARNING"}, "No mesh objects selected")
            return {"CANCELLED"}

        collision_params = native.CollisionParams(
            voxel_size=float(self.voxel_size),
            relative_voxel_size=float(self.relative_voxel_size),
            pin_boundary=1 if self.pin_boundary else 0,
        )
        remesh_params = native.RemeshParams(
            target_edge_length=float(self.remesh_edge_length),
            relative_edge_length=float(self.remesh_relative_edge_length),
            iterations=int(self.remesh_iterations),
            protect_boundary=1 if self.remesh_protect_boundary else 0,
        )

        depsgraph = context.evaluated_depsgraph_get()
        created = []
        total_in_tris = 0

        for obj in sources:
            try:
                verts, tris = _triangulated_world_mesh(obj, self.apply_modifiers, depsgraph)
            except Exception as e:
                self.report({"ERROR"}, f"Failed to read '{obj.name}': {e}")
                continue

            if not verts or not tris:
                self.report({"WARNING"}, f"'{obj.name}' has no geometry, skipping")
                continue
            total_in_tris += len(tris) // 3
            try:
                with native.Mesh() as src_mesh:
                    src_mesh.set_vertices(verts)
                    src_mesh.set_triangles(tris)
                    with native.generate_collision(src_mesh, collision_params) as out_mesh:
                        if self.uniform_remesh:
                            native.remesh_isotropic(out_mesh, remesh_params)
                        out_verts = out_mesh.get_vertices()
                        out_tris  = out_mesh.get_triangles()
            except Exception as e:
                self.report({"ERROR"}, f"Native pipeline failed for '{obj.name}': {e}")
                continue

            if not out_verts or not out_tris:
                self.report({"WARNING"},
                            f"Collision generation produced empty result for '{obj.name}' "
                            f"(try a smaller voxel size)")
                continue

            new_name = f"{obj.name}{self.suffix}"
            me = _build_blender_mesh(new_name, out_verts, out_tris)
            new_obj = bpy.data.objects.new(new_name, me)
            new_obj.matrix_world = Matrix.Identity(4)
            new_obj.display_type = "WIRE"
            new_obj.show_in_front = True
            (obj.users_collection[0] if obj.users_collection else context.collection).objects.link(new_obj)
            created.append(new_obj)

        if not created:
            return {"CANCELLED"}

        bpy.ops.object.select_all(action="DESELECT")
        for o in created:
            o.select_set(True)
        context.view_layer.objects.active = created[-1]

        final_tris = sum(len(o.data.polygons) for o in created)
        self.report({"INFO"},
                    f"Built {len(created)} collision mesh(es): "
                    f"{total_in_tris} tris → {final_tris} tris")
        return {"FINISHED"}

def _get_or_create_bake_image(name: str, resolution: int) -> bpy.types.Image:
    img = bpy.data.images.get(name)
    if img is None:
        img = bpy.data.images.new(name, width=resolution, height=resolution, alpha=True)
    elif img.size[0] != resolution or img.size[1] != resolution:
        img.scale(resolution, resolution)
    return img


def _ensure_baked_material(obj: bpy.types.Object, image: bpy.types.Image) -> bpy.types.Material:
    if not obj.data.materials:
        mat = bpy.data.materials.new(name=f"{obj.name}_baked")
        mat.use_nodes = True
        obj.data.materials.append(mat)
    else:
        mat = obj.active_material or obj.data.materials[0]
        if not mat.use_nodes:
            mat.use_nodes = True

    nodes = mat.node_tree.nodes
    links = mat.node_tree.links

    bsdf = next((n for n in nodes if n.type == "BSDF_PRINCIPLED"), None)
    if bsdf is None:
        bsdf = nodes.new("ShaderNodeBsdfPrincipled")
        bsdf.location = (0, 0)

    output = next((n for n in nodes if n.type == "OUTPUT_MATERIAL"), None)
    if output is None:
        output = nodes.new("ShaderNodeOutputMaterial")
        output.location = (300, 0)

    if not any(l.from_node == bsdf and l.to_node == output for l in links):
        links.new(bsdf.outputs["BSDF"], output.inputs["Surface"])

    img_node = next((n for n in nodes if n.type == "TEX_IMAGE" and n.image == image), None)
    if img_node is None:
        img_node = nodes.new("ShaderNodeTexImage")
        img_node.location = (-300, 0)
        img_node.image = image

    base_color = bsdf.inputs.get("Base Color")
    if base_color is not None:
        already_linked = any(
            l.from_node == img_node and l.to_socket == base_color for l in links
        )
        if not already_linked:
            for l in list(links):
                if l.to_socket == base_color:
                    links.remove(l)
            links.new(img_node.outputs["Color"], base_color)

    for n in nodes:
        n.select = False
    img_node.select = True
    nodes.active = img_node

    return mat


class MT_OT_bake_diffuse(Operator):
    bl_idname = "moddingtool.bake_diffuse"
    bl_label = "Bake Diffuse HD → LOD"
    bl_description = ("Bake the diffuse color from selected HD meshes onto the "
                      "active LOD mesh's UVs (Cycles, color pass only). The "
                      "result lives in the .blend as an image; a basic "
                      "Principled BSDF material is wired up automatically.")
    bl_options = {"REGISTER", "UNDO"}

    resolution: IntProperty(
        name="Resolution",
        description="Square pixel size of the baked image",
        default=1024,
        min=16,
        soft_max=8192,
        subtype="PIXEL",
    )
    margin: IntProperty(
        name="Margin",
        description="Pixel padding around UV islands to prevent seam bleed",
        default=16,
        min=0,
        soft_max=64,
    )
    cage_extrusion: FloatProperty(
        name="Cage Extrusion",
        description="Inflate the LOD outward by this distance when ray-casting "
                    "into the HD source",
        default=0.05,
        min=0.0,
        soft_max=1.0,
        unit="LENGTH",
    )
    max_ray_distance: FloatProperty(
        name="Max Ray Distance",
        description="Maximum distance to look for the HD surface from the LOD",
        default=0.1,
        min=0.0,
        soft_max=10.0,
        unit="LENGTH",
    )

    @classmethod
    def poll(cls, context):
        a = context.active_object
        return a is not None and a.type == "MESH" and len(context.selected_objects) >= 2

    def execute(self, context):
        lod = context.active_object
        if lod is None or lod.type != "MESH":
            self.report({"ERROR"}, "Active object must be a mesh (the LOD bake target)")
            return {"CANCELLED"}

        hd = [o for o in context.selected_objects if o.type == "MESH" and o is not lod]
        if not hd:
            self.report({"ERROR"},
                        "Select one or more HD source meshes plus the LOD as active")
            return {"CANCELLED"}

        if not lod.data.uv_layers:
            self.report({"ERROR"},
                        f"LOD mesh '{lod.name}' has no UV map; unwrap it before baking")
            return {"CANCELLED"}

        scene = context.scene
        original_engine = scene.render.engine
        try:
            if original_engine != "CYCLES":
                scene.render.engine = "CYCLES"

            img_name = f"{lod.name}_diffuse"
            image = _get_or_create_bake_image(img_name, int(self.resolution))
            _ensure_baked_material(lod, image)

            bake = scene.render.bake
            bake.use_selected_to_active = True
            bake.cage_extrusion = float(self.cage_extrusion)
            bake.max_ray_distance = float(self.max_ray_distance)
            bake.margin = int(self.margin)
            bake.use_clear = True
            bake.target = "IMAGE_TEXTURES"
            bake.use_pass_direct = False
            bake.use_pass_indirect = False
            bake.use_pass_color = True

            for o in context.view_layer.objects:
                o.select_set(False)
            for o in hd:
                o.select_set(True)
            lod.select_set(True)
            context.view_layer.objects.active = lod

            override = {
                "active_object": lod,
                "object": lod,
                "selected_objects": hd + [lod],
                "selected_editable_objects": hd + [lod],
                "view_layer": context.view_layer,
                "scene": scene,
            }
            try:
                with context.temp_override(**override):
                    result = bpy.ops.object.bake(
                        "EXEC_DEFAULT",
                        type="DIFFUSE",
                        pass_filter={"COLOR"},
                        use_selected_to_active=True,
                    )
            except RuntimeError as e:
                self.report({"ERROR"}, f"Bake failed: {e}")
                return {"CANCELLED"}

            if "FINISHED" not in result:
                self.report({"ERROR"}, f"Bake operator returned {result}")
                return {"CANCELLED"}

            self.report({"INFO"},
                        f"Baked diffuse from {len(hd)} HD mesh(es) into "
                        f"'{img_name}' ({self.resolution}×{self.resolution}) "
                        f"on '{lod.name}'")
            return {"FINISHED"}
        finally:
            if scene.render.engine != original_engine:
                scene.render.engine = original_engine


class MT_OT_generate_glue_decal(Operator):
    bl_idname = "moddingtool.generate_glue_decal"
    bl_label = "Generate Glue Decal"
    bl_description = ("Build an L-shaped decal strip from selected edges. "
                      "Each edge needs at least one wall-like face attached; "
                      "the strip extends up the wall and out along the floor "
                      "to hide the seam.")
    bl_options = {"REGISTER", "UNDO"}

    wall_height: FloatProperty(
        name="Wall Height",
        description="How far up the wall the decal extends",
        default=0.5,
        min=0.0,
        soft_max=5.0,
        unit="LENGTH",
    )
    floor_extent: FloatProperty(
        name="Floor Extent",
        description="How far along the floor the decal extends, away from the wall",
        default=0.3,
        min=0.0,
        soft_max=5.0,
        unit="LENGTH",
    )
    bias: FloatProperty(
        name="Surface Bias",
        description="Tiny offset away from the wall and up from the floor to "
                    "prevent Z-fighting",
        default=0.001,
        min=0.0,
        soft_max=0.05,
        unit="LENGTH",
        precision=4,
    )
    flip_outward: BoolProperty(
        name="Flip Outward Direction",
        description="Reverse the assumed outward direction (use if the decal "
                    "appears on the wrong side of the wall)",
        default=False,
    )
    even_thickness: BoolProperty(
        name="Even Thickness",
        description="Miter the strip at corners so its perpendicular width "
                    "stays constant across joints (instead of narrowing on "
                    "the bevel)",
        default=True,
    )
    create_uvs: BoolProperty(
        name="Generate UVs",
        description="Build a UV map: U along the edge length, V across the L",
        default=True,
    )
    quadify: BoolProperty(
        name="Tris → Quads",
        description="Merge generated triangle pairs into quads after building "
                    "the decal mesh",
        default=True,
    )
    suffix: StringProperty(
        name="Suffix",
        description="Suffix for the new decal object",
        default="_decal",
    )
    source_name: StringProperty(options={"HIDDEN"})
    source_loc: FloatVectorProperty(size=3, options={"HIDDEN"})
    source_rot: FloatVectorProperty(size=4, options={"HIDDEN"}, default=(1.0, 0.0, 0.0, 0.0))
    source_scale: FloatVectorProperty(size=3, options={"HIDDEN"}, default=(1.0, 1.0, 1.0))
    _vertical_dot_threshold = 0.7

    @classmethod
    def poll(cls, context):
        a = context.active_object
        return a is not None and a.type == "MESH"

    def invoke(self, context, event):
        a = context.active_object
        if a and a.type == "MESH":
            self.source_name = a.name
            loc, rot, scale = _decompose_for_props(a.matrix_world)
        else:
            self.source_name = ""
            loc, rot, scale = (0, 0, 0), (1, 0, 0, 0), (1, 1, 1)
        self.source_loc = loc
        self.source_rot = rot
        self.source_scale = scale
        return self.execute(context)

    def execute(self, context):
        src = bpy.data.objects.get(self.source_name) if self.source_name else None
        if src is None:
            src = context.active_object
        if src is None or src.type != "MESH":
            self.report({"ERROR"}, "Active object must be a mesh")
            return {"CANCELLED"}
        src_world = _compose_from_props(self.source_loc, self.source_rot, self.source_scale)

        was_edit = (src.mode == "EDIT")
        if was_edit:
            bm = bmesh.from_edit_mesh(src.data)
            owns_bm = False
        else:
            bm = bmesh.new()
            bm.from_mesh(src.data)
            owns_bm = True

        try:
            selected_edges = [e for e in bm.edges if e.select]
            if not selected_edges:
                self.report({"WARNING"},
                            "No edges selected on the active mesh")
                return {"CANCELLED"}

            mw = src_world
            mn = mw.to_3x3().inverted_safe().transposed()
            up = Vector((0.0, 0.0, 1.0))
            sign = -1.0 if self.flip_outward else 1.0

            vert_idx_remap: dict[int, int] = {}
            vertices_xyz: list[float] = []
            edges_ab: list[int] = []
            wall_normals_xyz: list[float] = []
            skipped = 0

            for e in selected_edges:
                wall_normal_world = None
                best_dot = 1.0
                for f in e.link_faces:
                    n_local = f.normal
                    if n_local.length_squared == 0.0:
                        continue
                    n_world = (mn @ n_local).normalized()
                    d = abs(n_world.dot(up))
                    if d < best_dot:
                        best_dot = d
                        wall_normal_world = n_world

                if wall_normal_world is None or best_dot > self._vertical_dot_threshold:
                    skipped += 1
                    continue

                O = wall_normal_world - up * wall_normal_world.dot(up)
                if O.length < 1e-5:
                    skipped += 1
                    continue
                O = (O.normalized()) * sign

                ab = []
                for bv in (e.verts[0], e.verts[1]):
                    bidx = bv.index
                    out_idx = vert_idx_remap.get(bidx)
                    if out_idx is None:
                        out_idx = len(vertices_xyz) // 3
                        vert_idx_remap[bidx] = out_idx
                        wp = mw @ bv.co
                        vertices_xyz.extend((wp.x, wp.y, wp.z))
                    ab.append(out_idx)

                edges_ab.extend(ab)
                wall_normals_xyz.extend((O.x, O.y, O.z))

            if not edges_ab:
                self.report({"WARNING"},
                            f"No wall-like faces found on selected edges "
                            f"({skipped} skipped)")
                return {"CANCELLED"}

            params = native.DecalParams(
                wall_height=float(self.wall_height),
                floor_extent=float(self.floor_extent),
                bias=float(self.bias),
                up_x=0.0, up_y=0.0, up_z=1.0,
                even_thickness=1 if self.even_thickness else 0,
            )

            try:
                with native.generate_glue_decal(
                    vertices_xyz, edges_ab, wall_normals_xyz, params
                ) as out_mesh:
                    out_verts = out_mesh.get_vertices()
                    out_tris  = out_mesh.get_triangles()
                    out_uvs   = out_mesh.get_uvs()
            except Exception as ex:
                self.report({"ERROR"}, f"Native decal generation failed: {ex}")
                return {"CANCELLED"}

            if not out_verts or not out_tris:
                self.report({"WARNING"},
                            "Decal generation produced no geometry")
                return {"CANCELLED"}

            mw_inv = mw.inverted_safe()
            local_verts = [tuple(mw_inv @ Vector(v)) for v in out_verts]

            decal_name = f"{src.name}{self.suffix}"
            me = bpy.data.meshes.new(decal_name)
            me.from_pydata(local_verts, [], list(out_tris))
            me.update(calc_edges=True)

            if self.create_uvs and out_uvs and len(out_uvs) == len(out_tris) * 3:
                uv_layer = me.uv_layers.new(name="UVMap")
                for i, uv in enumerate(out_uvs):
                    uv_layer.data[i].uv = uv

            if self.quadify:
                _quadify_mesh(me)

            obj = bpy.data.objects.new(decal_name, me)
            coll = src.users_collection[0] if src.users_collection else context.collection
            coll.objects.link(obj)
            _apply_world_transform(obj, src_world)

            used = len(selected_edges) - skipped
            self.report({"INFO"},
                        f"Built '{decal_name}' from {used} edge(s) "
                        f"({skipped} skipped, {len(out_tris)} tris, "
                        f"{len(vertices_xyz)//3} unique source verts)")
            return {"FINISHED"}
        finally:
            if owns_bm:
                bm.free()


class MT_OT_generate_blend_decal(Operator):
    bl_idname = "moddingtool.generate_blend_decal"
    bl_label = "Generate Blend Decal"
    bl_description = ("Build a flat ribbon strip along selected edges, used to "
                      "fade between two ground materials at a seam. The strip "
                      "lies on the horizontal plane and straddles each edge.")
    bl_options = {"REGISTER", "UNDO"}

    width: FloatProperty(
        name="Width",
        description="Total width of the blend ribbon, perpendicular to the edge",
        default=0.5,
        min=0.0,
        soft_max=5.0,
        unit="LENGTH",
    )
    center_offset: FloatProperty(
        name="Center Offset",
        description="Lateral offset across the strip; 0 = centered on the edge. "
                    "Positive shifts toward the +across direction (V=0 side).",
        default=0.0,
        soft_min=-2.0,
        soft_max=2.0,
        unit="LENGTH",
    )
    bias: FloatProperty(
        name="Surface Bias",
        description="Tiny vertical offset above the ground to prevent Z-fighting",
        default=0.001,
        min=0.0,
        soft_max=0.05,
        unit="LENGTH",
        precision=4,
    )
    flip_across: BoolProperty(
        name="Flip Across Direction",
        description="Reverse the across direction (swaps which side of the "
                    "edge is V=0 vs V=1)",
        default=False,
    )
    even_thickness: BoolProperty(
        name="Even Thickness",
        description="Miter the strip at corners so its perpendicular width "
                    "stays constant across joints",
        default=True,
    )
    create_uvs: BoolProperty(
        name="Generate UVs",
        description="Build a UV map: U along the edge length, V across the width",
        default=True,
    )
    quadify: BoolProperty(
        name="Tris → Quads",
        description="Merge generated triangle pairs into quads after building "
                    "the decal mesh",
        default=True,
    )
    suffix: StringProperty(
        name="Suffix",
        description="Suffix for the new decal object",
        default="_blend",
    )
    source_name: StringProperty(options={"HIDDEN"})
    source_loc: FloatVectorProperty(size=3, options={"HIDDEN"})
    source_rot: FloatVectorProperty(size=4, options={"HIDDEN"}, default=(1.0, 0.0, 0.0, 0.0))
    source_scale: FloatVectorProperty(size=3, options={"HIDDEN"}, default=(1.0, 1.0, 1.0))

    @classmethod
    def poll(cls, context):
        a = context.active_object
        return a is not None and a.type == "MESH"

    def invoke(self, context, event):
        a = context.active_object
        if a and a.type == "MESH":
            self.source_name = a.name
            loc, rot, scale = _decompose_for_props(a.matrix_world)
        else:
            self.source_name = ""
            loc, rot, scale = (0, 0, 0), (1, 0, 0, 0), (1, 1, 1)
        self.source_loc = loc
        self.source_rot = rot
        self.source_scale = scale
        return self.execute(context)

    def execute(self, context):
        src = bpy.data.objects.get(self.source_name) if self.source_name else None
        if src is None:
            src = context.active_object
        if src is None or src.type != "MESH":
            self.report({"ERROR"}, "Active object must be a mesh")
            return {"CANCELLED"}
        src_world = _compose_from_props(self.source_loc, self.source_rot, self.source_scale)

        was_edit = (src.mode == "EDIT")
        if was_edit:
            bm = bmesh.from_edit_mesh(src.data)
            owns_bm = False
        else:
            bm = bmesh.new()
            bm.from_mesh(src.data)
            owns_bm = True

        try:
            selected_edges = [e for e in bm.edges if e.select]
            if not selected_edges:
                self.report({"WARNING"}, "No edges selected on the active mesh")
                return {"CANCELLED"}

            mw = src_world
            up = Vector((0.0, 0.0, 1.0))

            vert_idx_remap: dict[int, int] = {}
            vertices_xyz: list[float] = []
            edges_ab: list[int] = []
            edge_across_xyz: list[float] = []
            skipped = 0

            for e in selected_edges:
                wa = mw @ e.verts[0].co
                wb = mw @ e.verts[1].co
                tangent = wb - wa
                if tangent.length < 1e-6:
                    skipped += 1
                    continue
                across = up.cross(tangent)
                if across.length < 1e-5:
                    skipped += 1
                    continue
                across = across.normalized()

                ab = []
                for bv, wp in ((e.verts[0], wa), (e.verts[1], wb)):
                    bidx = bv.index
                    out_idx = vert_idx_remap.get(bidx)
                    if out_idx is None:
                        out_idx = len(vertices_xyz) // 3
                        vert_idx_remap[bidx] = out_idx
                        vertices_xyz.extend((wp.x, wp.y, wp.z))
                    ab.append(out_idx)

                edges_ab.extend(ab)
                edge_across_xyz.extend((across.x, across.y, across.z))

            if not edges_ab:
                self.report({"WARNING"},
                            f"No usable edges (all degenerate or vertical, "
                            f"{skipped} skipped)")
                return {"CANCELLED"}

            EN = len(edges_ab) // 2
            vert_to_local_edges: dict[int, list[int]] = {}
            for li in range(EN):
                a = edges_ab[li * 2]
                b = edges_ab[li * 2 + 1]
                vert_to_local_edges.setdefault(a, []).append(li)
                vert_to_local_edges.setdefault(b, []).append(li)

            visited = [False] * EN
            sign_globally = -1.0 if self.flip_across else 1.0
            for seed in range(EN):
                if visited[seed]:
                    continue
                visited[seed] = True
                stack = [seed]
                while stack:
                    e = stack.pop()
                    ex = edge_across_xyz[e * 3]
                    ey = edge_across_xyz[e * 3 + 1]
                    ez = edge_across_xyz[e * 3 + 2]
                    for v in (edges_ab[e * 2], edges_ab[e * 2 + 1]):
                        for n in vert_to_local_edges[v]:
                            if visited[n]:
                                continue
                            visited[n] = True
                            nx = edge_across_xyz[n * 3]
                            ny = edge_across_xyz[n * 3 + 1]
                            nz = edge_across_xyz[n * 3 + 2]
                            if ex * nx + ey * ny + ez * nz < 0.0:
                                edge_across_xyz[n * 3]     = -nx
                                edge_across_xyz[n * 3 + 1] = -ny
                                edge_across_xyz[n * 3 + 2] = -nz
                            stack.append(n)

            if sign_globally < 0.0:
                for k in range(len(edge_across_xyz)):
                    edge_across_xyz[k] = -edge_across_xyz[k]

            params = native.BlendDecalParams(
                width=float(self.width),
                center_offset=float(self.center_offset),
                bias=float(self.bias),
                up_x=0.0, up_y=0.0, up_z=1.0,
                even_thickness=1 if self.even_thickness else 0,
            )

            try:
                with native.generate_blend_decal(
                    vertices_xyz, edges_ab, edge_across_xyz, params
                ) as out_mesh:
                    out_verts = out_mesh.get_vertices()
                    out_tris  = out_mesh.get_triangles()
                    out_uvs   = out_mesh.get_uvs()
            except Exception as ex:
                self.report({"ERROR"}, f"Native blend-decal generation failed: {ex}")
                return {"CANCELLED"}

            if not out_verts or not out_tris:
                self.report({"WARNING"}, "Blend decal produced no geometry")
                return {"CANCELLED"}

            mw_inv = mw.inverted_safe()
            local_verts = [tuple(mw_inv @ Vector(v)) for v in out_verts]

            decal_name = f"{src.name}{self.suffix}"
            me = bpy.data.meshes.new(decal_name)
            me.from_pydata(local_verts, [], list(out_tris))
            me.update(calc_edges=True)

            if self.create_uvs and out_uvs and len(out_uvs) == len(out_tris) * 3:
                uv_layer = me.uv_layers.new(name="UVMap")
                for i, uv in enumerate(out_uvs):
                    uv_layer.data[i].uv = uv

            if self.quadify:
                _quadify_mesh(me)

            obj = bpy.data.objects.new(decal_name, me)
            coll = src.users_collection[0] if src.users_collection else context.collection
            coll.objects.link(obj)
            _apply_world_transform(obj, src_world)

            used = len(selected_edges) - skipped
            self.report({"INFO"},
                        f"Built '{decal_name}' from {used} edge(s) "
                        f"({skipped} skipped, {len(out_tris)} tris, "
                        f"{len(vertices_xyz)//3} unique source verts)")
            return {"FINISHED"}
        finally:
            if owns_bm:
                bm.free()


_classes = (MT_OT_generate_collision, MT_OT_bake_diffuse,
            MT_OT_generate_glue_decal, MT_OT_generate_blend_decal)


def register():
    for c in _classes:
        bpy.utils.register_class(c)


def unregister():
    for c in reversed(_classes):
        bpy.utils.unregister_class(c)
