# SPDX-License-Identifier: GPL-3.0-or-later

import bpy
import bmesh
from bpy.props import BoolProperty, EnumProperty

from . import core

MODE_ITEMS = [
    ('PLANE', "Plane Section",
     "ループ全体を1枚の平面として扱い、レールとの交点を取る。"
     "断面が必ず平面になり、コーンなら正しい楕円になる"),
    ('NEAREST', "Nearest Point",
     "頂点ごとに独立して最も近いレール位置へ寄せる。"
     "非平面のループでも動くが、断面は平面にならない"),
]


class MESH_OT_looper_fix_to_rails(bpy.types.Operator):
    """移動・回転させたループ辺を、元のエッジ（レール）上に引き戻す"""
    bl_idname = "mesh.looper_fix_to_rails"
    bl_label = "Fix Loop to Rails"
    bl_options = {'REGISTER', 'UNDO'}

    mode: EnumProperty(name="Mode", items=MODE_ITEMS, default='PLANE')
    extend: BoolProperty(
        name="Extend Past Neighbors",
        description="隣のループを越えてレールを辿る。OFF なら直近の1区間に制限",
        default=True,
    )

    @classmethod
    def poll(cls, context):
        ob = context.object
        return (ob is not None and ob.type == 'MESH'
                and context.mode == 'EDIT_MESH')

    def execute(self, context):
        ob = context.edit_object
        bm = bmesh.from_edit_mesh(ob.data)

        if self.mode == 'PLANE':
            res = core.fix_selection_to_plane(bm, extend=self.extend)
            if res.get("error") == "no_plane":
                self.report({'WARNING'}, "平面をフィットできませんでした")
                return {'CANCELLED'}
        else:
            res = core.fix_selection_to_rails(bm, extend=self.extend)

        if res.get("error") == "no_selection":
            self.report({'WARNING'}, "頂点が選択されていません")
            return {'CANCELLED'}
        if res["moved"] == 0:
            self.report({'WARNING'},
                        "レールを特定できませんでした（選択範囲が広すぎるか、"
                        "隣接ループが存在しません）")
            return {'CANCELLED'}

        bmesh.update_edit_mesh(ob.data, loop_triangles=False, destructive=False)

        msg = f"{res['moved']} 頂点を補正"
        if res["skipped"]:
            msg += f"（{res['skipped']} 頂点はスキップ）"
        self.report({'INFO'}, msg)
        return {'FINISHED'}


classes = (MESH_OT_looper_fix_to_rails,)
