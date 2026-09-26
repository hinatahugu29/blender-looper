# SPDX-License-Identifier: GPL-3.0-or-later

bl_info = {
    "name": "Looper",
    "author": "hinatahugu29",
    "version": (0, 4, 0),
    "blender": (4, 2, 0),
    "location": "Edit Mode > Nパネル(Looper) / Alt+R / Ctrl+E / 右クリック",
    "description": "メッシュの形状を保ったままループ辺をスライドさせる",
    "doc_url": "https://github.com/hinatahugu29/blender-looper",
    "tracker_url": "https://github.com/hinatahugu29/blender-looper/issues",
    "category": "Mesh",
}

import bpy

from . import ops
from . import modal
from . import panels

addon_keymaps = []


def _draw_entries(layout):
    layout.separator()
    layout.operator(modal.MESH_OT_looper_rotate.bl_idname,
                    text="Looper Rotate")
    layout.operator(ops.MESH_OT_looper_flatten.bl_idname,
                    text="Flatten Loop")
    layout.operator(ops.MESH_OT_looper_fix_to_rails.bl_idname,
                    text="Fix Loop to Rails")


def _menu_func(self, context):
    _draw_entries(self.layout)


def _context_menu_func(self, context):
    if context.tool_settings.mesh_select_mode[1]:  # 辺モードのときだけ
        _draw_entries(self.layout)


def register():
    for cls in ops.classes + modal.classes + panels.classes:
        bpy.utils.register_class(cls)
    panels.register_props()

    bpy.types.VIEW3D_MT_edit_mesh_edges.append(_menu_func)
    bpy.types.VIEW3D_MT_edit_mesh_context_menu.append(_context_menu_func)

    kc = bpy.context.window_manager.keyconfigs.addon
    if kc:
        km = kc.keymaps.new(name="Mesh", space_type='EMPTY')
        kmi = km.keymap_items.new(modal.MESH_OT_looper_rotate.bl_idname,
                                  'R', 'PRESS', alt=True)
        addon_keymaps.append((km, kmi))


def unregister():
    for km, kmi in addon_keymaps:
        km.keymap_items.remove(kmi)
    addon_keymaps.clear()

    bpy.types.VIEW3D_MT_edit_mesh_context_menu.remove(_context_menu_func)
    bpy.types.VIEW3D_MT_edit_mesh_edges.remove(_menu_func)

    panels.unregister_props()
    for cls in reversed(ops.classes + modal.classes + panels.classes):
        bpy.utils.unregister_class(cls)
