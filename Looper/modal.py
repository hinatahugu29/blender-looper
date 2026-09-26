# SPDX-License-Identifier: GPL-3.0-or-later

"""ループを既存辺に沿わせたまま回転させるモーダルオペレータ。

ピボット = ループの重心 / 軸 = ビュー方向（標準の R と同じ感覚）
"""

import math

import bpy
import bmesh
from bpy.props import BoolProperty
import gpu
from gpu_extras.batch import batch_for_shader
from bpy_extras import view3d_utils
from mathutils import Vector

from . import core
from . import ops

# Blender 標準のトランスフォームに合わせた刻み幅。
# Shift は通常「精密モード」だが、Ctrl 併用時は細かい刻みに意味を振り替える。
SNAP_STEP = math.radians(5.0)
SNAP_STEP_FINE = math.radians(1.0)

# 絶対角度の基準面。拘束軸まわりに測るので、軸と平行でない法線を選ぶ。
# X / Y 軸まわりなら「XY 平面（法線 Z）を 0 度」、Z 軸まわりだけは
# Z を基準にできないので YZ 平面（法線 X）を 0 度とする。
# 軸の色は Blender の慣習に合わせる。文字を読まなくても何に拘束中か分かる。
_AXIS_COLOR = {
    'X': (1.0, 0.25, 0.35, 0.9),
    'Y': (0.5, 0.85, 0.2, 0.9),
    'Z': (0.2, 0.55, 1.0, 0.9),
}
_SNAP_COLOR = (1.0, 0.7, 0.1, 1.0)      # 拾ったスナップ先
_CLAMP_COLOR = (1.0, 0.3, 0.1, 1.0)     # 平面に届かなかった頂点

_ABS_REFERENCE = {
    'X': Vector((0.0, 0.0, 1.0)),
    'Y': Vector((0.0, 0.0, 1.0)),
    'Z': Vector((1.0, 0.0, 0.0)),
}

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

    def _cursor_ray(self, event):
        """カーソルから伸びるレイを、オブジェクトのローカル空間で返す。"""
        co = (event.mouse_x - self.region.x, event.mouse_y - self.region.y)
        origin = view3d_utils.region_2d_to_origin_3d(self.region, self.rv3d, co)
        direction = view3d_utils.region_2d_to_vector_3d(self.region, self.rv3d, co)
        # レイをローカル空間へ持ち込む。ここに留まる限り、面の法線を
        # 逆転置行列で変換する必要がない（非一様スケールでも壊れない）
        return (self.mw_inv @ origin,
                (self.mw_inv.to_3x3() @ direction).normalized())

    def _update_face_snap(self, event):
        """カーソル下のスナップ先へ揃える回転を求めて face_rot に入れる。

        姿勢を直接当てるのではなく (軸, 角度) へ逆算して持つので、
        確定後のリドゥも通常の回転とまったく同じ経路に乗る。
        """
        origin, direction = self._cursor_ray(event)
        rot, info, preview = core.snap_rotation_from_ray(
            self.bvh, self.snap_faces, origin, direction,
            self.cache.normal0, snap_type=self.snap_type, moving=self.moving,
            axis=self._local_axis() if self.axis_lock else None)
        self.snap_info = info
        self.snap_preview = [(self.mw @ a, self.mw @ b) for a, b in preview]
        if rot is not None:
            self.face_rot = rot

    def _absolute_angle(self, angle=None):
        """今のループ平面が、拘束軸まわりで何度傾いているか（ワールド基準）。

        軸拘束が無いときは基準が視点依存になって意味を失うので None。
        angle を渡すと「その相対角度まで回したときの絶対角度」を返す。
        """
        if self.axis_lock is None or self.cache.normal0 is None:
            return None
        if angle is None:
            angle = self._current_rotation()[1]
        axis_local, _ = self._current_rotation()
        local = (core.rotation_matrix_about(axis_local, angle, self.cache.pivot)
                 .to_3x3() @ self.cache.normal0)
        # 法線のローカル→ワールドは逆転置行列。ここを 3x3 で済ませると
        # 非一様スケールのオブジェクトで角度がずれる。
        world = (self.mat3_nrm @ local).normalized()
        axis_world = Vector((0.0, 0.0, 0.0))
        axis_world['XYZ'.index(self.axis_lock)] = 1.0
        return core.absolute_plane_angle(world, axis_world,
                                         _ABS_REFERENCE[self.axis_lock])

    def _relative_for_absolute(self, target_abs):
        """絶対角度 target_abs にするために必要な、開始姿勢からの相対角度。

        単純な引き算では非一様スケールのオブジェクトで外れる（絶対角度が
        回転量に対して加算的でなくなる）ので、反復して解く。加算的な
        通常のケースでは 1 回で厳密に決まる。
        """
        return core.solve_angle_for(
            lambda a: self._absolute_angle(angle=a), target_abs)

    def _num_is_absolute(self):
        """数値入力を絶対角度として解釈するか。

        角度は 1 自由度、法線は 2 自由度なので、絶対指定は「どの軸まわりか」が
        決まっていないと姿勢が定まらない。拘束が無いときは常に相対。
        """
        return self.num_absolute and self.axis_lock is not None

    def _commit_num(self):
        """数値バッファを角度へ反映する。"""
        try:
            val = math.radians(float(self.num_buf or 0))
        except ValueError:
            return                      # 入力途中の "-" や "." はまだ数にならない
        if self._num_is_absolute():
            rel = self._relative_for_absolute(val)
            if rel is not None:
                self.angle = rel
                return
        self.angle = val

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
        """カーソルでスナップ先を指す種別か（＝マウスが回転を駆動しない）。"""
        return self.snapping and self.snap_type in {'FACE', 'LOOP'}

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
        if self.region:
            self.region.tag_redraw()    # スナップ先の表示だけが変わる場合もある
        self._header(context)

    # -- 3D ビューへの描画 -------------------------------------------------
    def _draw_3d(self):
        """拾ったスナップ先・拘束軸・届かなかった頂点を描く。

        テキストで「12 頂点のループ」と言われても、それが狙ったループかは
        確かめようがない。見えれば一発で分かる。
        """
        try:
            shader = gpu.shader.from_builtin('UNIFORM_COLOR')
            gpu.state.blend_set('ALPHA')
            gpu.state.depth_test_set('NONE')   # 陰に隠れても見せる

            if self.snap_preview:
                gpu.state.line_width_set(3.0)
                pts = [p for seg in self.snap_preview for p in seg]
                shader.uniform_float("color", _SNAP_COLOR)
                batch_for_shader(shader, 'LINES', {"pos": pts}).draw(shader)

            if self.axis_lock:
                gpu.state.line_width_set(1.5)
                shader.uniform_float("color", _AXIS_COLOR[self.axis_lock])
                batch_for_shader(shader, 'LINES',
                                 {"pos": self._axis_line()}).draw(shader)

            clamped = self.cache.clamped_co
            if clamped:
                gpu.state.point_size_set(9.0)
                shader.uniform_float("color", _CLAMP_COLOR)
                pts = [self.mw @ co for co in clamped]
                batch_for_shader(shader, 'POINTS', {"pos": pts}).draw(shader)
        finally:
            gpu.state.line_width_set(1.0)
            gpu.state.point_size_set(1.0)
            gpu.state.depth_test_set('LESS_EQUAL')
            gpu.state.blend_set('NONE')

    def _axis_line(self):
        """拘束軸をピボットを通る線分として返す（ワールド座標）。"""
        pivot = self.mw @ self.cache.pivot
        d = Vector((0.0, 0.0, 0.0))
        d['XYZ'.index(self.axis_lock)] = 1.0
        return [pivot - d * self.axis_len, pivot + d * self.axis_len]

    def _snap_label(self):
        return {'INCREMENT': "刻み", 'FACE': "面", 'LOOP': "ループ"}[
            self.snap_type]

    def _header(self, context):
        """ヘッダには「毎フレーム変わる値」と「今の状態」だけを置く。

        キー一覧はステータスバーへ回す（Blender 標準の R / G と同じ構成）。
        不変のヒントが値と同じ行に並んでいると、一番読みたい角度が埋もれる。
        """
        if self.num_buf:
            label = "絶対" if self._num_is_absolute() else "回転"
            a = f"{label} [{self.num_buf}]"
        else:
            # 符号と桁で幅が動くと数字が左右に揺れて読みづらいので幅を固定する
            a = f"回転 {math.degrees(self._current_rotation()[1]):+8.2f}°"

        abs_a = self._absolute_angle()
        if abs_a is not None:
            ref = "YZ" if self.axis_lock == 'Z' else "XY"
            a += f"   絶対 {math.degrees(abs_a):+8.2f}° ({ref})"
            # 数値入力が絶対と相対のどちらで解釈されるかは、打ち始める前に
            # 見えている必要がある。A を押しても無反応に見えるのが一番困る。
            a += f"   数値:{'絶対' if self.num_absolute else '相対'}"

        axis = f"軸 {self.axis_lock}" if self.axis_lock else "軸 ビュー"

        if self._face_snapping:
            what = self._snap_label()
            state = self.snap_info or f"{what}を指してください"
            head = f"[{what}スナップ] {state}   {a}   {axis}"
        elif self.snapping:
            step = math.degrees(self._snap_step())
            head = f"[刻み {step:.0f}°]   {a}   {axis}"
        else:
            ext = "越境ON" if self.extend else "越境OFF"
            m = "平面断面" if self.mode == "PLANE" else "最近点"
            head = (f"{a}   {axis}   Ctrl:{self._snap_label()}"
                    f"   {m} · {ext}")

        clamped = self.cache.last_clamped
        if clamped:
            head += f"   ※ {clamped} 頂点が平面に届いていません"
        context.area.header_text_set(head)
        self._status(context)

    def _status(self, context):
        """キー一覧はステータスバーへ。状態が変わったときだけ書き換える。"""
        if self._face_snapping:
            keys = ("カーソルで対象を指す", "S スナップ先", "X/Y/Z 軸拘束",
                    "Ctrl を離すと通常の回転")
        elif self.snapping:
            keys = ("Shift 1°刻み", "S スナップ先", "X/Y/Z 軸拘束",
                    "Ctrl を離すと通常の回転")
        else:
            keys = ("X/Y/Z 軸拘束", "Ctrl スナップ", "S スナップ先",
                    "Shift 精密", "数値 角度入力",
                    *((f"A 数値入力を{'相対' if self.num_absolute else '絶対'}へ",)
                      if self.axis_lock else ()),
                    "P モード", "E 越境", "Enter 確定", "Esc 中止")
        text = "  |  ".join(keys)
        if text != self.status_text:
            self.status_text = text
            context.workspace.status_text_set(text)

    def _finish(self, context):
        # ハンドラが残ると描画され続けて Blender の再起動が要る。
        # 確定・中止・例外のどの経路からも必ずここを通すこと。
        if self._handle is not None:
            bpy.types.SpaceView3D.draw_handler_remove(self._handle, 'WINDOW')
            self._handle = None
        context.area.header_text_set(None)
        context.workspace.status_text_set(None)
        context.window.cursor_modal_restore()
        if self.region:
            self.region.tag_redraw()

    # -- 実行 -------------------------------------------------------------
    def invoke(self, context, event):
        # キーマップ（Alt+R）はプロパティを渡してこないので、そのままだと
        # N パネルの設定が一切効かず、常に既定値で起動してしまう。
        # 起動経路によらずパネルを唯一の設定元にする。
        st = getattr(context.scene, "looper", None)
        if st is not None:
            self.mode = st.mode
            self.extend = st.extend
            self.snap_type = st.snap_type

        self._handle = None
        self.region = None

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
        self.snap_info = ""
        self.snap_preview = []
        self.status_text = None
        self.mw = ob.matrix_world.copy()
        dims = ob.dimensions
        self.axis_len = max(dims.x, dims.y, dims.z, 1.0) * 2.0
        self.num_absolute = True
        self.moving = set(sel)
        self.mw_inv = ob.matrix_world.inverted()
        # 法線をローカル→ワールドへ移すのは逆転置行列
        self.mat3_nrm = ob.matrix_world.to_3x3().inverted().transposed()
        built = core.build_snap_bvh(self.bm, sel)
        self.bvh, self.snap_faces = built if built else (None, [])
        self.last_raw = self._screen_angle(event)

        context.window.cursor_modal_set('CROSSHAIR')
        self._handle = bpy.types.SpaceView3D.draw_handler_add(
            self._draw_3d, (), 'WINDOW', 'POST_VIEW')
        self._apply(context)
        context.window_manager.modal_handler_add(self)
        return {'RUNNING_MODAL'}

    def modal(self, context, event):
        # 途中で例外が出てもハンドラを残さない。残すと描画され続ける。
        try:
            return self._modal(context, event)
        except Exception:
            import traceback
            traceback.print_exc()
            self.cache.restore()
            bmesh.update_edit_mesh(self.me, loop_triangles=False,
                                   destructive=False)
            self._finish(context)
            self.report({'ERROR'}, "Looper Rotate が中断しました（コンソール参照）")
            return {'CANCELLED'}

    def _modal(self, context, event):
        ev, val = event.type, event.value

        # 修飾キーは毎イベントで状態を引き直す。押下・解除イベントを
        # 取りこぼしても状態がずれない。
        ctrl, shift = event.ctrl, event.shift
        if ctrl != self.snapping or (ctrl and shift != self.snap_fine):
            self.snapping, self.snap_fine = ctrl, shift
            if not ctrl:
                # Blender 標準のスナップと同じく、離したら素の回転に戻る
                self.face_rot = None
                self.snap_info = ""
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
                self._commit_num()
                self._apply(context)
                return {'RUNNING_MODAL'}

            if ev == 'BACK_SPACE' and self.num_buf:
                self.num_buf = self.num_buf[:-1]
                self._commit_num()
                self._apply(context)
                return {'RUNNING_MODAL'}

            if ev == 'S':
                # スナップ先はモーダル中に変えたくなる。パネルまで戻らせない。
                ids = [i[0] for i in ops.SNAP_ITEMS]
                self.snap_type = ids[(ids.index(self.snap_type) + 1) % len(ids)]
                self.face_rot = None
                self.snap_info = ""
                if self._face_snapping:
                    self._update_face_snap(event)
                self._apply(context)
                return {'RUNNING_MODAL'}

            if ev == 'A' and self.axis_lock:
                self.num_absolute = not self.num_absolute
                self._commit_num()
                self._apply(context)
                return {'RUNNING_MODAL'}

            if ev in {'X', 'Y', 'Z'}:
                self.axis_lock = None if self.axis_lock == ev else ev
                if self._face_snapping:
                    # 拘束が変われば「その軸で最も近づく角度」も変わる
                    self.face_rot = None
                    self._update_face_snap(event)
                # 絶対角度は拘束軸まわりで測るので、軸が変われば意味が変わる
                self._commit_num()
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
