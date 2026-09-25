bl_info = {
    "name": "Looper",
    "author": "",
    "version": (0, 2, 0),
    "blender": (4, 2, 0),
    "location": "Edit Mode > Ctrl+E / 右クリック / Alt+R",
    "description": "ループ辺を元メッシュの形状を保ったままスライドさせる",
    "category": "Mesh",
}

import bpy

from . import ops
from . import modal

addon_keymaps = []


def _menu_func(self, context):
    layout = self.layout
    layout.separator()
    layout.operator(modal.MESH_OT_looper_rotate.bl_idname,
                    text="Looper Rotate")
    layout.operator(ops.MESH_OT_looper_fix_to_rails.bl_idname,
                    text="Fix Loop to Rails")


def _context_menu_func(self, context):
    if context.tool_settings.mesh_select_mode[1]:  # 辺モードのときだけ
        layout = self.layout
        layout.separator()
        layout.operator(modal.MESH_OT_looper_rotate.bl_idname,
                        text="Looper Rotate")
        layout.operator(ops.MESH_OT_looper_fix_to_rails.bl_idname,
                        text="Fix Loop to Rails")


def register():
    for cls in ops.classes + modal.classes:
        bpy.utils.register_class(cls)

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

    for cls in reversed(ops.classes + modal.classes):
        bpy.utils.unregister_class(cls)
