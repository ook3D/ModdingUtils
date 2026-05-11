bl_info = {
    "name": "ModdingTool",
    "author": "ook3d",
    "version": (0, 1, 0),
    "blender": (5, 0, 0),
    "location": "View3D > Sidebar > ModdingTool",
    "description": "Modding utilities for gta 5 modding",
    "category": "Object",
}

from . import native  # noqa: F401  (DLL loaded eagerly for clearer errors)
from . import operators
from . import panel


def register():
    operators.register()
    panel.register()


def unregister():
    panel.unregister()
    operators.unregister()
