# SPDX-License-Identifier: GPL-3.0-or-later

import bpy
from bpy.props import BoolProperty, EnumProperty, PointerProperty

from . import ops
from . import modal


class LooperSettings(bpy.types.PropertyGroup):
    mode: EnumProperty(name="Mode", items=ops.MODE_ITEMS, default='PLANE')
    extend: BoolProperty(
        name="Extend Past Neighbors",
        description=ops.EXTEND_DESC,
        default=True,
    )


class VIEW3D_PT_looper(bpy.types.Panel):
    bl_label = "Looper"
    bl_idname = "VIEW3D_PT_looper"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = "Looper"

    @classmethod
    def poll(cls, context):
        return context.mode == 'EDIT_MESH'

    def draw(self, context):
        st = context.scene.looper
        layout = self.layout

        # -- 設定 ----------------------------------------------------------
        col = layout.column(align=True)
        col.label(text="設定")
        box = col.box()
        box.prop(st, "mode", text="")
        box.prop(st, "extend")

        # -- 動かす --------------------------------------------------------
        col = layout.column(align=True)
        col.label(text="動かす")
        sub = col.column(align=True)
        sub.scale_y = 1.4
        op = sub.operator(modal.MESH_OT_looper_rotate.bl_idname,
                          text="Rotate Loop", icon='FILE_REFRESH')
        op.mode = st.mode
        op.extend = st.extend

        # -- 揃える --------------------------------------------------------
        col = layout.column(align=True)
        col.label(text="揃える")
        op = col.operator(ops.MESH_OT_looper_flatten.bl_idname,
                          text="Flatten Loop", icon='MOD_LATTICE')
        op.extend = st.extend

        # -- 直す ----------------------------------------------------------
        col = layout.column(align=True)
        col.label(text="直す")
        op = col.operator(ops.MESH_OT_looper_fix_to_rails.bl_idname,
                          text="Fix Loop to Rails", icon='SNAP_ON')
        op.mode = st.mode
        op.extend = st.extend

        layout.separator()
        box = layout.box()
        box.scale_y = 0.8
        for line in ("Rotate は Alt+R でも起動します",
                     "X/Y/Z 軸拘束  Shift 精密",
                     "P モード切替  E 越境切替"):
            box.label(text=line)


classes = (LooperSettings, VIEW3D_PT_looper)


def register_props():
    bpy.types.Scene.looper = PointerProperty(type=LooperSettings)


def unregister_props():
    if hasattr(bpy.types.Scene, "looper"):
        del bpy.types.Scene.looper
