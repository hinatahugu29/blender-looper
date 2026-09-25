# SPDX-License-Identifier: GPL-3.0-or-later

"""ループを既存辺に沿わせたまま回転させるモーダルオペレータ。

ピボット = ループの重心 / 軸 = ビュー方向（標準の R と同じ感覚）
"""

import math

import bpy
import bmesh
from bpy.props import BoolProperty
from bpy_extras import view3d_utils
from mathutils import Vector

from . import core
from . import ops

_NUM_CHARS = {
    'ZERO': '0', 'ONE': '1', 'TWO': '2', 'THREE': '3', 'FOUR': '4',
    'FIVE': '5', 'SIX': '6', 'SEVEN': '7', 'EIGHT': '8', 'NINE': '9',
    'NUMPAD_0': '0', 'NUMPAD_1': '1', 'NUMPAD_2': '2', 'NUMPAD_3': '3',
    'NUMPAD_4': '4', 'NUMPAD_5': '5', 'NUMPAD_6': '6', 'NUMPAD_7': '7',
    'NUMPAD_8': '8', 'NUMPAD_9': '9',
    'PERIOD': '.', 'NUMPAD_PERIOD': '.',
    'MINUS': '-', 'NUMPAD_MINUS': '-',
}


class MESH_OT_looper_rotate(bpy.types.Operator):
    """既存の辺に沿わせたまま、選択ループを回転させる"""
    bl_idname = "mesh.looper_rotate"
    bl_label = "Looper Rotate"
    bl_options = {'REGISTER', 'UNDO', 'GRAB_CURSOR', 'BLOCKING'}

    mode: bpy.props.EnumProperty(
        name="Mode", items=ops.MODE_ITEMS, default='PLANE')
    extend: BoolProperty(
        name="Extend Past Neighbors",
        description="隣のループを越えてレールを辿る",
        default=True,
    )
    angle: bpy.props.FloatProperty(
        name="Angle", subtype='ANGLE', default=0.0,
    )
    axis_local: bpy.props.FloatVectorProperty(
        name="Axis", size=3, default=(0.0, 0.0, 1.0), options={'HIDDEN'},
    )

    @classmethod
    def poll(cls, context):
        ob = context.object
        return (ob is not None and ob.type == 'MESH'
                and context.mode == 'EDIT_MESH'
                and context.space_data is not None
                and context.space_data.type == 'VIEW_3D')

    # -- 内部 -------------------------------------------------------------
    def _screen_angle(self, event):
        """3Dビュー領域を基準にした、ピボットまわりのマウス角度。

        Nパネルのボタンから起動した場合、マウスは UI 領域の上にあって
        event.mouse_region_* が 3D ビュー基準にならないので、
        ウィンドウ座標から 3D ビュー領域の原点を引いて求める。
        """
        if self.pivot_2d is None:
            return 0.0
        mx = event.mouse_x - self.region.x
        my = event.mouse_y - self.region.y
        return math.atan2(my - self.pivot_2d.y, mx - self.pivot_2d.x)

    def _local_axis(self):
        if self.axis_lock is None:
            world = self.rv3d.view_rotation @ Vector((0.0, 0.0, 1.0))
        else:
            world = Vector((0.0, 0.0, 0.0))
            world['XYZ'.index(self.axis_lock)] = 1.0
        local = self.mat3_inv @ world
        if local.length_squared < 1e-12:
            return Vector((0.0, 0.0, 1.0))
        return local.normalized()

    def _apply(self, context):
        axis = self._local_axis()
        self.axis_local = axis
        mat = core.rotation_matrix_about(axis, self.angle, self.cache.pivot)
        self.cache.apply_matrix(mat, extend=self.extend, mode=self.mode)
        self.bm.normal_update()
        bmesh.update_edit_mesh(self.me, loop_triangles=False, destructive=False)
        self._header(context)

    def _header(self, context):
        if self.num_buf:
            a = f"角度: [{self.num_buf}]"
        else:
            a = f"角度: {math.degrees(self.angle):.2f}°"
        axis = self.axis_lock if self.axis_lock else "ビュー"
        ext = "ON" if self.extend else "OFF"
        m = "平面断面" if self.mode == "PLANE" else "最近点"
        context.area.header_text_set(
            f"{a}  軸: {axis}  モード(P): {m}  越境(E): {ext}   "
            f"[X/Y/Z 軸拘束  Shift 精密  数値入力  LMB/Enter 確定  Esc/RMB 中止]")

    def _finish(self, context):
        context.area.header_text_set(None)
        context.window.cursor_modal_restore()

    # -- 実行 -------------------------------------------------------------
    def invoke(self, context, event):
        ob = context.edit_object
        self.me = ob.data
        self.bm = bmesh.from_edit_mesh(self.me)

        sel = [v for v in self.bm.verts if v.select and not v.hide]
        if not sel:
            self.report({'WARNING'}, "頂点が選択されていません")
            return {'CANCELLED'}

        self.cache = core.RailCache(sel)
        if self.cache.railless == len(sel):
            self.report({'WARNING'}, "レールを特定できませんでした")
            return {'CANCELLED'}
        if self.cache.railless:
            self.report({'INFO'},
                        f"{self.cache.railless} 頂点はレール無しのため素の回転")
        if self.mode == 'PLANE' and not self.cache.has_plane:
            self.mode = 'NEAREST'
            self.report({'INFO'}, "平面をフィットできないため最近点モード")

        self.rv3d = context.space_data.region_3d
        self.region = next(
            (r for r in context.area.regions if r.type == "WINDOW"),
            context.region)
        self.mat3_inv = ob.matrix_world.to_3x3().inverted()

        pivot_world = ob.matrix_world @ self.cache.pivot
        self.pivot_2d = view3d_utils.location_3d_to_region_2d(
            self.region, self.rv3d, pivot_world)

        self.angle = 0.0
        self.axis_lock = None
        self.num_buf = ""
        self.last_raw = self._screen_angle(event)

        context.window.cursor_modal_set('CROSSHAIR')
        self._apply(context)
        context.window_manager.modal_handler_add(self)
        return {'RUNNING_MODAL'}

    def modal(self, context, event):
        ev, val = event.type, event.value

        if ev == 'MOUSEMOVE':
            raw = self._screen_angle(event)
            if not self.num_buf:
                d = raw - self.last_raw
                # -pi..pi へ巻き戻す
                d = (d + math.pi) % (2 * math.pi) - math.pi
                self.angle += d * (0.1 if event.shift else 1.0)
                self._apply(context)
            self.last_raw = raw
            return {'RUNNING_MODAL'}

        if val == 'PRESS':
            if ev in _NUM_CHARS:
                self.num_buf += _NUM_CHARS[ev]
                try:
                    self.angle = math.radians(float(self.num_buf))
                except ValueError:
                    pass
                self._apply(context)
                return {'RUNNING_MODAL'}

            if ev == 'BACK_SPACE' and self.num_buf:
                self.num_buf = self.num_buf[:-1]
                try:
                    self.angle = math.radians(float(self.num_buf or 0))
                except ValueError:
                    pass
                self._apply(context)
                return {'RUNNING_MODAL'}

            if ev in {'X', 'Y', 'Z'}:
                self.axis_lock = None if self.axis_lock == ev else ev
                self._apply(context)
                return {'RUNNING_MODAL'}

            if ev == 'E':
                self.extend = not self.extend
                self._apply(context)
                return {'RUNNING_MODAL'}

            if ev == 'P':
                if self.mode == 'PLANE':
                    self.mode = 'NEAREST'
                elif self.cache.has_plane:
                    self.mode = 'PLANE'
                else:
                    self.report({'WARNING'}, "平面をフィットできません")
                self._apply(context)
                return {'RUNNING_MODAL'}

            if ev in {'LEFTMOUSE', 'RET', 'NUMPAD_ENTER'}:
                self._finish(context)
                return {'FINISHED'}

            if ev in {'RIGHTMOUSE', 'ESC'}:
                self.cache.restore()
                self.bm.normal_update()
                bmesh.update_edit_mesh(self.me, loop_triangles=False,
                                       destructive=False)
                self._finish(context)
                return {'CANCELLED'}

        return {'RUNNING_MODAL'}

    def execute(self, context):
        """F9 / リドゥ用（角度をプロパティから再適用）。"""
        ob = context.edit_object
        self.me = ob.data
        self.bm = bmesh.from_edit_mesh(self.me)
        sel = [v for v in self.bm.verts if v.select and not v.hide]
        if not sel:
            return {'CANCELLED'}
        cache = core.RailCache(sel)
        mat = core.rotation_matrix_about(Vector(self.axis_local[:]),
                                         self.angle, cache.pivot)
        cache.apply_matrix(mat, extend=self.extend, mode=self.mode)
        self.bm.normal_update()
        bmesh.update_edit_mesh(self.me, loop_triangles=False, destructive=False)
        return {'FINISHED'}


classes = (MESH_OT_looper_rotate,)
