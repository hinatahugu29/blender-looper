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

# Blender 標準のトランスフォームに合わせた刻み幅。
# Shift は通常「精密モード」だが、Ctrl 併用時は細かい刻みに意味を振り替える。
SNAP_STEP = math.radians(5.0)
SNAP_STEP_FINE = math.radians(1.0)

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
    snap_type: bpy.props.EnumProperty(
        name="Snap", items=ops.SNAP_ITEMS, default='INCREMENT',
        description="Ctrl を押している間のスナップ先",
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

    def _pick_face_normal(self, event):
        """カーソル下の面の法線（ローカル空間）。外していれば None。"""
        if self.bvh is None:
            return None
        co = (event.mouse_x - self.region.x, event.mouse_y - self.region.y)
        origin = view3d_utils.region_2d_to_origin_3d(self.region, self.rv3d, co)
        direction = view3d_utils.region_2d_to_vector_3d(self.region, self.rv3d, co)
        # レイをローカル空間へ持ち込む。ここに留まる限り、法線を
        # 逆転置行列で変換する必要がない（非一様スケールでも壊れない）
        o = self.mw_inv @ origin
        d = (self.mw_inv.to_3x3() @ direction).normalized()
        hit = self.bvh.ray_cast(o, d)
        return hit[1] if hit and hit[1] is not None else None

    def _update_face_snap(self, event):
        """カーソル下の面へ揃える回転を求めて face_rot に入れる。

        姿勢を直接当てるのではなく (軸, 角度) へ逆算して持つので、
        確定後のリドゥも通常の回転とまったく同じ経路に乗る。
        """
        normal = self._pick_face_normal(event)
        if normal is None or self.cache.normal0 is None:
            return
        lock = self._local_axis() if self.axis_lock else None
        res = core.rotation_to_normal(self.cache.normal0, normal, axis=lock)
        if res is not None:
            self.face_rot = res

    def _snap_step(self):
        return SNAP_STEP_FINE if self.snap_fine else SNAP_STEP

    def _effective_angle(self):
        """実際に適用する角度。

        self.angle にはマウスが積算した生の角度を保持したまま、スナップ中だけ
        ここで丸める。こうしておくと Ctrl を離した瞬間に生の角度へ滑らかに
        戻り、スナップ解除で値が飛ばない。
        """
        if self.snapping and not self.num_buf:
            return core.snap_angle(self.angle, self._snap_step())
        return self.angle

    @property
    def _face_snapping(self):
        return self.snapping and self.snap_type == 'FACE'

    def _current_rotation(self):
        """今フレーム適用する (ローカル軸, 角度)。"""
        if self._face_snapping and self.face_rot is not None:
            return self.face_rot
        return self._local_axis(), self._effective_angle()

    def _apply(self, context):
        axis, angle = self._current_rotation()
        self.axis_local = axis
        mat = core.rotation_matrix_about(axis, angle, self.cache.pivot)
        self.cache.apply_matrix(mat, extend=self.extend, mode=self.mode)
        self.bm.normal_update()
        bmesh.update_edit_mesh(self.me, loop_triangles=False, destructive=False)
        self._header(context)

    def _header(self, context):
        if self.num_buf:
            a = f"角度: [{self.num_buf}]"
        else:
            a = f"角度: {math.degrees(self._current_rotation()[1]):.2f}°"
        axis = self.axis_lock if self.axis_lock else "ビュー"

        if self._face_snapping:
            state = "面に整列" if self.face_rot else "面を指してください"
            head = (f"[スナップ｜{state}]  {a}   "
                    f"軸拘束: {self.axis_lock or 'なし'}   "
                    f"[Ctrl を離すと通常の回転へ]")
        elif self.snapping:
            # スナップ中は状態が変わったことを一目で分かるようにし、
            # 常時ヒントは引っ込める（項目数を増やさない）
            step = math.degrees(self._snap_step())
            head = (f"[スナップ {step:.0f}° 刻み]  {a}   軸: {axis}   "
                    f"[Shift でさらに細かく  Ctrl を離すと通常の回転へ]")
        else:
            ext = "ON" if self.extend else "OFF"
            m = "平面断面" if self.mode == "PLANE" else "最近点"
            head = (f"{a}   軸: {axis}   "
                    f"[X/Y/Z 拘束  Ctrl スナップ  Shift 精密  "
                    f"Enter 確定  Esc 中止]"
                    f"      P:{m}  E:{ext}")

        clamped = self.cache.last_clamped
        if clamped:
            head += f"      ※ {clamped} 頂点がレール端で止まり平面に届いていません"
        context.area.header_text_set(head)

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
        self.snapping = False
        self.snap_fine = False
        self.face_rot = None
        self.mw_inv = ob.matrix_world.inverted()
        self.bvh = core.build_snap_bvh(self.bm, sel)
        self.last_raw = self._screen_angle(event)

        context.window.cursor_modal_set('CROSSHAIR')
        self._apply(context)
        context.window_manager.modal_handler_add(self)
        return {'RUNNING_MODAL'}

    def modal(self, context, event):
        ev, val = event.type, event.value

        # 修飾キーは毎イベントで状態を引き直す。押下・解除イベントを
        # 取りこぼしても状態がずれない。
        ctrl, shift = event.ctrl, event.shift
        if ctrl != self.snapping or (ctrl and shift != self.snap_fine):
            self.snapping, self.snap_fine = ctrl, shift
            if not ctrl:
                # Blender 標準のスナップと同じく、離したら素の回転に戻る
                self.face_rot = None
            elif self.snap_type == 'FACE':
                self._update_face_snap(event)
            # 面スナップ中はマウスが回転を駆動しないので、基準を取り直さないと
            # Ctrl を離した瞬間にその間の移動量が一気に効いてループが飛ぶ
            self.last_raw = self._screen_angle(event)
            self._apply(context)
        else:
            self.snap_fine = shift

        if ev == 'MOUSEMOVE':
            raw = self._screen_angle(event)
            if self._face_snapping:
                self._update_face_snap(event)   # マウスは面を指す役に回る
                self._apply(context)
            elif not self.num_buf:
                d = raw - self.last_raw
                # -pi..pi へ巻き戻す
                d = (d + math.pi) % (2 * math.pi) - math.pi
                # Ctrl 併用時の Shift は「細かい刻み」なので減速はしない
                fine = event.shift and not event.ctrl
                self.angle += d * (0.1 if fine else 1.0)
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
                if self._face_snapping:
                    # 拘束が変われば「その軸で最も近づく角度」も変わる
                    self.face_rot = None
                    self._update_face_snap(event)
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
                # 確定した姿勢を実値として残す。そうしないと F9 のリドゥが
                # スナップ前の生の角度で再計算してしまう。
                axis, angle = self._current_rotation()
                self.axis_local, self.angle = axis, angle
                self.snapping = False
                self.face_rot = None
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
