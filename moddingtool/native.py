from __future__ import annotations

import ctypes
import os
import sys
from ctypes import (
    POINTER,
    Structure,
    c_char_p,
    c_float,
    c_int32,
    c_void_p,
)

def _library_path() -> str:
    here = os.path.dirname(os.path.abspath(__file__))
    if sys.platform == "win32":
        name = "moddingtool.dll"
    elif sys.platform == "darwin":
        name = "libmoddingtool.dylib"
    else:
        name = "libmoddingtool.so"
    return os.path.join(here, "bin", name)


def _load() -> ctypes.CDLL:
    path = _library_path()
    if not os.path.isfile(path):
        raise FileNotFoundError(
            f"moddingtool native library not found at: {path}\n"
            "Build it first (run build.bat at the project root)."
        )
    return ctypes.CDLL(path)


_lib = _load()

MT_OK          = 0
MT_ERR_NULL    = 1
MT_ERR_INVALID = 2
MT_ERR_EMPTY   = 3
MT_ERR_OOM     = 4

_STATUS_NAMES = {
    MT_OK: "OK",
    MT_ERR_NULL: "NULL pointer",
    MT_ERR_INVALID: "invalid argument",
    MT_ERR_EMPTY: "empty mesh",
    MT_ERR_OOM: "out of memory",
}


def status_str(s: int) -> str:
    return _STATUS_NAMES.get(int(s), f"unknown status {s}")

class CollisionParams(Structure):
    _fields_ = [
        ("voxel_size",          c_float),
        ("relative_voxel_size", c_float),
        ("pin_boundary",        c_int32),
    ]


class RemeshParams(Structure):
    _fields_ = [
        ("target_edge_length",   c_float),
        ("relative_edge_length", c_float),
        ("iterations",           c_int32),
        ("protect_boundary",     c_int32),
    ]


class DecalParams(Structure):
    _fields_ = [
        ("wall_height",    c_float),
        ("floor_extent",   c_float),
        ("bias",           c_float),
        ("up_x",           c_float),
        ("up_y",           c_float),
        ("up_z",           c_float),
        ("even_thickness", c_int32),
    ]


class BlendDecalParams(Structure):
    _fields_ = [
        ("width",          c_float),
        ("center_offset",  c_float),
        ("bias",           c_float),
        ("up_x",           c_float),
        ("up_y",           c_float),
        ("up_z",           c_float),
        ("even_thickness", c_int32),
    ]


_lib.mt_version.argtypes = []
_lib.mt_version.restype = c_char_p

_lib.mt_mesh_create.argtypes = []
_lib.mt_mesh_create.restype = c_void_p

_lib.mt_mesh_destroy.argtypes = [c_void_p]
_lib.mt_mesh_destroy.restype = None

_lib.mt_mesh_set_vertices.argtypes = [c_void_p, POINTER(c_float), c_int32]
_lib.mt_mesh_set_vertices.restype = c_int32

_lib.mt_mesh_set_triangles.argtypes = [c_void_p, POINTER(c_int32), c_int32]
_lib.mt_mesh_set_triangles.restype = c_int32

_lib.mt_mesh_vertex_count.argtypes = [c_void_p]
_lib.mt_mesh_vertex_count.restype = c_int32

_lib.mt_mesh_triangle_count.argtypes = [c_void_p]
_lib.mt_mesh_triangle_count.restype = c_int32

_lib.mt_mesh_get_vertices.argtypes = [c_void_p, POINTER(c_float)]
_lib.mt_mesh_get_vertices.restype = c_int32

_lib.mt_mesh_get_triangles.argtypes = [c_void_p, POINTER(c_int32)]
_lib.mt_mesh_get_triangles.restype = c_int32

_lib.mt_collision_params_default.argtypes = []
_lib.mt_collision_params_default.restype = CollisionParams

_lib.mt_generate_collision.argtypes = [c_void_p, POINTER(CollisionParams), POINTER(c_int32)]
_lib.mt_generate_collision.restype = c_void_p

_lib.mt_remesh_params_default.argtypes = []
_lib.mt_remesh_params_default.restype = RemeshParams

_lib.mt_remesh_isotropic.argtypes = [c_void_p, POINTER(RemeshParams)]
_lib.mt_remesh_isotropic.restype = c_int32

_lib.mt_mesh_uv_count.argtypes = [c_void_p]
_lib.mt_mesh_uv_count.restype = c_int32

_lib.mt_mesh_get_uvs.argtypes = [c_void_p, POINTER(c_float)]
_lib.mt_mesh_get_uvs.restype = c_int32

_lib.mt_decal_params_default.argtypes = []
_lib.mt_decal_params_default.restype = DecalParams

_lib.mt_generate_glue_decal.argtypes = [
    POINTER(c_float), c_int32,
    POINTER(c_int32), c_int32,
    POINTER(c_float),
    POINTER(DecalParams),
    POINTER(c_int32),
]
_lib.mt_generate_glue_decal.restype = c_void_p

_lib.mt_blend_decal_params_default.argtypes = []
_lib.mt_blend_decal_params_default.restype = BlendDecalParams

_lib.mt_generate_blend_decal.argtypes = [
    POINTER(c_float), c_int32,
    POINTER(c_int32), c_int32,
    POINTER(c_float),
    POINTER(BlendDecalParams),
    POINTER(c_int32),
]
_lib.mt_generate_blend_decal.restype = c_void_p


def version() -> str:
    return _lib.mt_version().decode("utf-8", errors="replace")


def default_params() -> CollisionParams:
    return _lib.mt_collision_params_default()


class Mesh:
    """RAII wrapper around an opaque MTMesh*."""

    __slots__ = ("_handle",)

    def __init__(self, handle: int | None = None):
        if handle is None:
            handle = _lib.mt_mesh_create()
            if not handle:
                raise MemoryError("mt_mesh_create returned NULL")
        self._handle = c_void_p(handle)

    def close(self) -> None:
        if self._handle:
            _lib.mt_mesh_destroy(self._handle)
            self._handle = c_void_p(0)

    def __del__(self):
        self.close()

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()

    @property
    def handle(self) -> c_void_p:
        return self._handle

    def set_vertices(self, flat_xyz: "list[float] | tuple[float, ...]") -> None:
        n = len(flat_xyz) // 3
        arr = (c_float * (n * 3))(*flat_xyz)
        s = _lib.mt_mesh_set_vertices(self._handle, arr, n)
        if s != MT_OK:
            raise RuntimeError(f"mt_mesh_set_vertices failed: {status_str(s)}")

    def set_triangles(self, flat_indices: "list[int] | tuple[int, ...]") -> None:
        n = len(flat_indices) // 3
        arr = (c_int32 * (n * 3))(*flat_indices)
        s = _lib.mt_mesh_set_triangles(self._handle, arr, n)
        if s != MT_OK:
            raise RuntimeError(f"mt_mesh_set_triangles failed: {status_str(s)}")

    @property
    def vertex_count(self) -> int:
        return int(_lib.mt_mesh_vertex_count(self._handle))

    @property
    def triangle_count(self) -> int:
        return int(_lib.mt_mesh_triangle_count(self._handle))

    def get_vertices(self) -> list[tuple[float, float, float]]:
        n = self.vertex_count
        if n == 0:
            return []
        buf = (c_float * (n * 3))()
        s = _lib.mt_mesh_get_vertices(self._handle, buf)
        if s != MT_OK:
            raise RuntimeError(f"mt_mesh_get_vertices failed: {status_str(s)}")
        return [(buf[i * 3], buf[i * 3 + 1], buf[i * 3 + 2]) for i in range(n)]

    def get_triangles(self) -> list[tuple[int, int, int]]:
        n = self.triangle_count
        if n == 0:
            return []
        buf = (c_int32 * (n * 3))()
        s = _lib.mt_mesh_get_triangles(self._handle, buf)
        if s != MT_OK:
            raise RuntimeError(f"mt_mesh_get_triangles failed: {status_str(s)}")
        return [(buf[i * 3], buf[i * 3 + 1], buf[i * 3 + 2]) for i in range(n)]

    @property
    def uv_loop_count(self) -> int:
        return int(_lib.mt_mesh_uv_count(self._handle))

    def get_uvs(self) -> list[tuple[float, float]]:
        """Return per-loop UVs in triangle-corner order, or [] if none stored."""
        n = self.uv_loop_count
        if n == 0:
            return []
        buf = (c_float * (n * 2))()
        s = _lib.mt_mesh_get_uvs(self._handle, buf)
        if s != MT_OK:
            raise RuntimeError(f"mt_mesh_get_uvs failed: {status_str(s)}")
        return [(buf[i * 2], buf[i * 2 + 1]) for i in range(n)]


def generate_collision(input_mesh: Mesh, params: CollisionParams | None = None) -> Mesh:
    if params is None:
        params = default_params()
    status = c_int32(MT_OK)
    handle = _lib.mt_generate_collision(input_mesh.handle, ctypes.byref(params), ctypes.byref(status))
    if not handle:
        raise RuntimeError(f"mt_generate_collision failed: {status_str(status.value)}")
    return Mesh(handle=handle)


def default_remesh_params() -> RemeshParams:
    return _lib.mt_remesh_params_default()


def remesh_isotropic(mesh: Mesh, params: RemeshParams | None = None) -> None:
    if params is None:
        params = default_remesh_params()
    s = _lib.mt_remesh_isotropic(mesh.handle, ctypes.byref(params))
    if s != MT_OK:
        raise RuntimeError(f"mt_remesh_isotropic failed: {status_str(s)}")


def default_decal_params() -> DecalParams:
    return _lib.mt_decal_params_default()


def generate_glue_decal(vertices_xyz: "list[float] | tuple[float, ...]",
                        edges_ab: "list[int] | tuple[int, ...]",
                        wall_normals_xyz: "list[float] | tuple[float, ...]",
                        params: DecalParams | None = None) -> Mesh:
    if params is None:
        params = default_decal_params()

    vn = len(vertices_xyz) // 3
    en = len(edges_ab) // 2
    if vn == 0 or en == 0:
        raise ValueError("generate_glue_decal: empty input")
    if len(wall_normals_xyz) // 3 != en:
        raise ValueError("generate_glue_decal: one wall normal per edge required")

    v_arr = (c_float * (vn * 3))(*vertices_xyz)
    e_arr = (c_int32 * (en * 2))(*edges_ab)
    n_arr = (c_float * (en * 3))(*wall_normals_xyz)

    status = c_int32(MT_OK)
    handle = _lib.mt_generate_glue_decal(
        v_arr, vn,
        e_arr, en,
        n_arr,
        ctypes.byref(params),
        ctypes.byref(status),
    )
    if not handle:
        raise RuntimeError(f"mt_generate_glue_decal failed: {status_str(status.value)}")
    return Mesh(handle=handle)


def default_blend_decal_params() -> BlendDecalParams:
    return _lib.mt_blend_decal_params_default()


def generate_blend_decal(vertices_xyz: "list[float] | tuple[float, ...]",
                         edges_ab: "list[int] | tuple[int, ...]",
                         edge_across_xyz: "list[float] | tuple[float, ...]",
                         params: BlendDecalParams | None = None) -> Mesh:
    if params is None:
        params = default_blend_decal_params()

    vn = len(vertices_xyz) // 3
    en = len(edges_ab) // 2
    if vn == 0 or en == 0:
        raise ValueError("generate_blend_decal: empty input")
    if len(edge_across_xyz) // 3 != en:
        raise ValueError("generate_blend_decal: one across vector per edge required")

    v_arr = (c_float * (vn * 3))(*vertices_xyz)
    e_arr = (c_int32 * (en * 2))(*edges_ab)
    a_arr = (c_float * (en * 3))(*edge_across_xyz)

    status = c_int32(MT_OK)
    handle = _lib.mt_generate_blend_decal(
        v_arr, vn,
        e_arr, en,
        a_arr,
        ctypes.byref(params),
        ctypes.byref(status),
    )
    if not handle:
        raise RuntimeError(f"mt_generate_blend_decal failed: {status_str(status.value)}")
    return Mesh(handle=handle)
