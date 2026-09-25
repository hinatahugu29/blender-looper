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

EXTEND_DESC = "隣のループを越えてレールを辿る。OFF なら直近の1区間に制限"

# Ctrl 押下中に何へスナップするか。Blender の磁石アイコンと同じ考え方で、
# 「Ctrl = スナップ」は固定し、スナップ先だけを設定で切り替える。
SNAP_ITEMS = [
    ('INCREMENT', "Increment",
     "角度を一定の刻みに丸める（既定 5°、Shift 併用で 1°）"),
    ('FACE', "Face",
     "カーソルの下にある面の法線にループを揃える。"
     "マウスは面を指すのに使うので、回転の積算は止まる"),
    ('LOOP', "Loop",
     "カーソルの下にあるエッジループの平面にループを揃える。"
     "面に平行にするのとは別物で、段差や縁取りの傾きを合わせるときに使う"),
]


def poll_edit_mesh(context):
    ob = context.object
    return (ob is not None and ob.type == 'MESH'
            and context.mode == 'EDIT_MESH')


def _report_result(op, res):
    """core の戻り値を UI レポートへ変換する。返り値は operator の返り値。"""
    if res.get("error") == "no_selection":
        op.report({'WARNING'}, "頂点が選択されていません")
        return {'CANCELLED'}
    if res.get("error") == "no_plane":
        op.report({'WARNING'}, "平面をフィットできませんでした")
        return {'CANCELLED'}
    if res["moved"] == 0:
        op.report({'WARNING'},
                  "レールを特定できませんでした（選択範囲が広すぎるか、"
                  "隣接ループが存在しません）")
        return {'CANCELLED'}

    msg = f"{res['moved']} 頂点を補正"
    if res["skipped"]:
        msg += f"（{res['skipped']} 頂点はスキップ）"
    op.report({'INFO'}, msg)
    return {'FINISHED'}


class MESH_OT_looper_fix_to_rails(bpy.types.Operator):
    """移動・回転させたループ辺を、元のエッジ（レール）上に引き戻す"""
    bl_idname = "mesh.looper_fix_to_rails"
    bl_label = "Fix Loop to Rails"
    bl_options = {'REGISTER', 'UNDO'}

    mode: EnumProperty(name="Mode", items=MODE_ITEMS, default='PLANE')
    extend: BoolProperty(name="Extend Past Neighbors",
                         description=EXTEND_DESC, default=True)

    @classmethod
    def poll(cls, context):
        return poll_edit_mesh(context)

    def execute(self, context):
        ob = context.edit_object
        bm = bmesh.from_edit_mesh(ob.data)

        if self.mode == 'PLANE':
            res = core.fix_selection_to_plane(bm, extend=self.extend)
        else:
            res = core.fix_selection_to_rails(bm, extend=self.extend)

        ret = _report_result(self, res)
        if ret == {'FINISHED'}:
            bmesh.update_edit_mesh(ob.data, loop_triangles=False,
                                   destructive=False)
        return ret


class MESH_OT_looper_flatten(bpy.types.Operator):
    """選択ループを、そのループ自身のベストフィット平面へ揃えて平らにする"""
    bl_idname = "mesh.looper_flatten"
    bl_label = "Flatten Loop"
    bl_options = {'REGISTER', 'UNDO'}

    extend: BoolProperty(name="Extend Past Neighbors",
                         description=EXTEND_DESC, default=True)

    @classmethod
    def poll(cls, context):
        return poll_edit_mesh(context)

    def execute(self, context):
        ob = context.edit_object
        bm = bmesh.from_edit_mesh(ob.data)
        res = core.fix_selection_to_plane(bm, extend=self.extend)
        ret = _report_result(self, res)
        if ret == {'FINISHED'}:
            bmesh.update_edit_mesh(ob.data, loop_triangles=False,
                                   destructive=False)
        return ret


classes = (MESH_OT_looper_fix_to_rails, MESH_OT_looper_flatten)
