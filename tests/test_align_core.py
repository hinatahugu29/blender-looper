"""apply_plane と姿勢→回転量の逆算のヘッドレステスト。

blender --background --factory-startup --python tests/test_align_core.py
"""
import os, sys, math
import bpy, bmesh
from mathutils import Vector, Matrix

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from Looper import core                                        # noqa: E402
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from test_core import make_cone, loop_at_z, check, FAILS, report  # noqa: E402


def planarity(verts):
    pts = [v.co.copy() for v in verts]
    n = core.fit_plane_normal(pts)
    c = sum(pts, Vector()) / len(pts)
    return max(abs((p - c).dot(n)) for p in pts)


# -- 逆算 ------------------------------------------------------------------

def test_rotation_to_normal_free():
    print("[test] 自由回転の逆算が目標法線に一致する")
    n0 = Vector((0.0, 0.0, 1.0))
    for target in (Vector((0.3, -0.2, 1.0)), Vector((1.0, 0.0, 0.2)),
                   Vector((0.0, 0.9, 0.4))):
        ax, ang = core.rotation_to_normal(n0, target)
        got = Matrix.Rotation(ang, 4, ax).to_3x3() @ n0
        err = (got - target.normalized()).length
        check(err < 1e-5, f"逆算した回転で法線が一致 (誤差 {err:.3e})")


def test_rotation_to_normal_sign():
    print("[test] 法線の符号反転で 180 度回らない")
    n0 = Vector((0.0, 0.0, 1.0))
    ax, ang = core.rotation_to_normal(n0, Vector((0.0, 0.05, -1.0)))
    check(abs(ang) < math.radians(10.0),
          f"-Z 寄りの目標でも小さな回転 ({math.degrees(ang):.2f}°)")


def test_rotation_to_normal_constrained():
    print("[test] 軸拘束付きの逆算")
    n0 = Vector((0.0, 0.0, 1.0))
    axis = Vector((1.0, 0.0, 0.0))
    # X 軸まわりなら YZ 平面内の目標にはぴたりと届く
    target = Vector((0.0, 1.0, 1.0))
    ax, ang = core.rotation_to_normal(n0, target, axis=axis)
    got = Matrix.Rotation(ang, 4, ax).to_3x3() @ n0
    check((got - target.normalized()).length < 1e-6,
          "拘束軸上の目標には厳密に一致")

    # 軸に平行な目標は原理的に到達不能 → None
    check(core.rotation_to_normal(n0, axis, axis=axis) is None,
          "到達不能な目標は None を返す")

    # 到達不能でない範囲では「最も近づく角度」になっているか（数値的に確認）
    target = Vector((0.6, 0.6, 0.5))
    ax, ang = core.rotation_to_normal(n0, target, axis=axis)
    t = target.normalized()

    def dist(a):
        return ((Matrix.Rotation(a, 4, ax).to_3x3() @ n0) - t).length

    best = min((dist(ang + d) for d in
                (-0.2, -0.05, -0.01, 0.0, 0.01, 0.05, 0.2)))
    check(abs(dist(ang) - best) < 1e-9, "拘束下で最も近い角度を選んでいる")


def test_snap_angle():
    print("[test] 角度の刻み")
    step = math.radians(5.0)
    for deg, want in ((0.0, 0.0), (2.4, 0.0), (2.6, 5.0), (47.0, 45.0),
                      (-47.0, -45.0), (183.0, 185.0)):
        got = math.degrees(core.snap_angle(math.radians(deg), step))
        check(abs(got - want) < 1e-9, f"{deg}° → {got:.0f}° (期待 {want:.0f}°)")


# -- apply_plane -----------------------------------------------------------

def test_apply_plane_matches_matrix():
    print("[test] apply_plane と apply_matrix(PLANE) が一致する")
    bm = make_cone()
    loop = loop_at_z(bm, 0.0)
    ang = math.radians(20.0)
    axis = Vector((1.0, 0.0, 0.0))

    c1 = core.RailCache(loop)
    mat = core.rotation_matrix_about(axis, ang, c1.pivot)
    c1.apply_matrix(mat, mode='PLANE')
    via_matrix = [v.co.copy() for v in loop]
    c1.restore()

    c2 = core.RailCache(loop)
    normal = (mat.to_3x3() @ c2.normal0)
    c2.apply_plane(normal, mat @ c2.pivot)
    via_plane = [v.co.copy() for v in loop]
    c2.restore()

    err = max((a - b).length for a, b in zip(via_matrix, via_plane))
    check(err < 1e-12, f"両経路の結果が一致 (差 {err:.3e})")


def test_apply_plane_aligns_to_target():
    print("[test] 目標法線を渡すとその向きに揃う")
    bm = make_cone()
    loop = loop_at_z(bm, 0.0)
    cache = core.RailCache(loop)

    target = Vector((0.0, 0.35, 1.0)).normalized()
    clamped = cache.apply_plane(target, cache.pivot)
    check(clamped == 0, f"全頂点がレールと交差 (クランプ {clamped})")

    flat = planarity(loop)
    check(flat < 1e-6, f"結果が平面に乗っている (最大ズレ {flat:.3e})")

    got = core.fit_plane_normal([v.co.copy() for v in loop])
    if got.dot(target) < 0:
        got = -got
    check((got - target).length < 1e-5,
          f"法線が目標と一致 (誤差 {(got - target).length:.3e})")
    cache.restore()


def test_apply_plane_reports_clamping():
    print("[test] 届かない平面はクランプとして報告される")
    bm = make_cone()
    loop = loop_at_z(bm, 0.0)
    cache = core.RailCache(loop)
    # レールからはるか遠くの平面 → 交差しようがない
    far = cache.pivot + Vector((0.0, 0.0, 100.0))
    clamped = cache.apply_plane(Vector((0.0, 0.0, 1.0)), far)
    check(clamped == len(loop), f"全頂点がクランプ扱い ({clamped}/{len(loop)})")
    check(cache.last_clamped == clamped, "last_clamped に記録される")
    cache.restore()


def test_align_roundtrip():
    print("[test] 逆算した回転で apply_matrix しても同じ姿勢になる")
    bm = make_cone()
    loop = loop_at_z(bm, 0.0)
    cache = core.RailCache(loop)
    target = Vector((0.2, 0.3, 1.0)).normalized()

    ax, ang = core.rotation_to_normal(cache.normal0, target)
    mat = core.rotation_matrix_about(ax, ang, cache.pivot)
    cache.apply_matrix(mat, mode='PLANE')
    got = core.fit_plane_normal([v.co.copy() for v in loop])
    if got.dot(target) < 0:
        got = -got
    check((got - target).length < 1e-5,
          f"逆算 → 行列 → 射影で目標姿勢 (誤差 {(got - target).length:.3e})")
    cache.restore()


# -- 面スナップ経路 --------------------------------------------------------

def test_snap_bvh_excludes_moving_faces():
    print("[test] BVH が動かす面を除外している")
    bm = make_cone()
    loop = loop_at_z(bm, 0.0)
    built = core.build_snap_bvh(bm, loop)
    check(built is not None, "BVH が作れる")
    bvh, faces = built

    moving = set(loop)
    excluded = [f for f in bm.faces if moving.intersection(f.verts)]
    check(len(excluded) > 0, f"除外対象の面が存在する ({len(excluded)} 枚)")

    # 除外した面の重心へ、その面の法線方向から撃つ → 当たってはいけない
    hits = 0
    for f in excluded:
        c = f.calc_center_median()
        if bvh.ray_cast(c + f.normal * 0.5, -f.normal, 0.45)[1] is not None:
            hits += 1
    check(hits == 0, f"動かす面には当たらない (ヒット {hits} 件)")

    # 動かさない面（上下のキャップ）にはちゃんと当たる
    keep = faces
    c = keep[0].calc_center_median()
    n = keep[0].normal
    check(bvh.ray_cast(c + n * 0.5, -n, 1.0)[1] is not None,
          "動かさない面には当たる")


def test_face_snap_pipeline():
    print("[test] 面の法線を拾う → 逆算 → 射影 で面と平行になる")
    bm = make_cone()
    loop = loop_at_z(bm, 0.0)
    bvh, faces = core.build_snap_bvh(bm, loop)
    cache = core.RailCache(loop)

    # 側面の一枚を「カーソル下の面」に見立てて法線を拾う
    moving = set(loop)
    side = next(f for f in bm.faces
                if not moving.intersection(f.verts) and abs(f.normal.z) < 0.9)
    c = side.calc_center_median()
    hit = bvh.ray_cast(c + side.normal * 0.5, -side.normal, 1.0)
    picked = hit[1]
    check(picked is not None, "レイキャストで法線を拾えた")

    res = core.rotation_to_normal(cache.normal0, picked)
    check(res is not None, "法線から回転量を逆算できた")

    # 円錐側面の法線はほぼ水平で、そこへ揃えると目標平面がレールとほぼ平行に
    # なる＝原理的に交差しない。結果が平面に乗らないこと自体は正しいので、
    # ここで守りたいのは「黙って間違わない」こと。
    ax, ang = res
    cache.apply_matrix(core.rotation_matrix_about(ax, ang, cache.pivot),
                       mode='PLANE')
    got = core.fit_plane_normal([v.co.copy() for v in loop])
    if got.dot(picked) < 0:
        got = -got
    aligned = (got - picked.normalized()).length < 1e-4
    check(aligned or cache.last_clamped > 0,
          f"揃うか、届かないならクランプを報告する "
          f"(揃った={aligned} クランプ={cache.last_clamped})")
    cache.restore()

    # 届く範囲の目標なら、拾った法線から厳密に揃うこと
    target = cache.normal0.lerp(picked.normalized()
                                if picked.dot(cache.normal0) > 0
                                else -picked.normalized(), 0.3).normalized()
    ax, ang = core.rotation_to_normal(cache.normal0, target)
    cache.apply_matrix(core.rotation_matrix_about(ax, ang, cache.pivot),
                       mode='PLANE')
    check(cache.last_clamped == 0,
          f"届く範囲ではクランプしない ({cache.last_clamped})")
    got = core.fit_plane_normal([v.co.copy() for v in loop])
    if got.dot(target) < 0:
        got = -got
    err = (got - target).length
    check(err < 1e-4, f"拾った法線の方向へ厳密に揃う (誤差 {err:.3e})")
    cache.restore()


def test_face_snap_constrained():
    print("[test] 軸拘束中の面スナップは拘束を破らない")
    bm = make_cone()
    loop = loop_at_z(bm, 0.0)
    cache = core.RailCache(loop)
    axis = Vector((1.0, 0.0, 0.0))
    target = Vector((0.4, 0.5, 0.8))

    res = core.rotation_to_normal(cache.normal0, target, axis=axis)
    check(res is not None, "拘束付きでも回転量が求まる")
    ax, ang = res
    check((ax - axis).length < 1e-9, "返る軸が拘束軸そのもの")

    mat = core.rotation_matrix_about(ax, ang, cache.pivot)
    cache.apply_matrix(mat, mode='PLANE')
    got = core.fit_plane_normal([v.co.copy() for v in loop])
    # X 軸まわりの回転なので、法線の X 成分は元のまま（＝0）のはず
    check(abs(got.x) < 1e-5, f"拘束軸成分が変化していない (x={got.x:.3e})")
    cache.restore()


# -- 絶対角度 --------------------------------------------------------------

REF_Z = Vector((0.0, 0.0, 1.0))
AX_X = Vector((1.0, 0.0, 0.0))


def test_absolute_angle_of_flat_loop():
    print("[test] 水平なループの絶対角度は 0 度")
    bm = make_cone()
    loop = loop_at_z(bm, 0.0)
    cache = core.RailCache(loop)
    a = core.absolute_plane_angle(cache.normal0, AX_X, REF_Z)
    check(a is not None and abs(math.degrees(a)) < 1e-4,
          f"XY 平面上のループは 0° ({math.degrees(a):.4f}°)")


def test_absolute_angle_tracks_rotation():
    print("[test] 回した角度がそのまま絶対角度になる")
    bm = make_cone()
    loop = loop_at_z(bm, 0.0)
    cache = core.RailCache(loop)
    for deg in (5.0, 12.5, -20.0, 30.0):
        mat = core.rotation_matrix_about(AX_X, math.radians(deg), cache.pivot)
        cache.apply_matrix(mat, mode='PLANE')
        n = core.fit_plane_normal([v.co.copy() for v in loop])
        a = math.degrees(core.absolute_plane_angle(n, AX_X, REF_Z))
        check(abs(a - deg) < 1e-3, f"{deg:+.1f}° 回すと絶対 {a:+.3f}°")
        cache.restore()


def test_absolute_target_is_reached():
    print("[test] 絶対角度の指定どおりに傾く")
    bm = make_cone()
    loop = loop_at_z(bm, 0.0)
    cache = core.RailCache(loop)

    # 開始姿勢をわざと傾けておき、そこから「絶対 25 度にする」
    start = math.radians(8.0)
    cache.apply_matrix(core.rotation_matrix_about(AX_X, start, cache.pivot),
                       mode='PLANE')
    cache.restore()

    base = core.absolute_plane_angle(cache.normal0, AX_X, REF_Z)
    target = math.radians(25.0)
    rel = target - base                      # modal._relative_for_absolute と同じ式
    cache.apply_matrix(core.rotation_matrix_about(AX_X, rel, cache.pivot),
                       mode='PLANE')
    n = core.fit_plane_normal([v.co.copy() for v in loop])
    got = math.degrees(core.absolute_plane_angle(n, AX_X, REF_Z))
    check(abs(got - 25.0) < 1e-3, f"絶対 25° 指定 → {got:.4f}°")
    check(cache.last_clamped == 0, f"クランプなし ({cache.last_clamped})")
    cache.restore()


def test_absolute_angle_folds_to_quarter_turn():
    print("[test] 絶対角度は -90°..+90° に畳まれる")
    n = Matrix.Rotation(math.radians(100.0), 4, AX_X).to_3x3() @ REF_Z
    a = math.degrees(core.absolute_plane_angle(n, AX_X, REF_Z))
    check(abs(a - (-80.0)) < 1e-3,
          f"100° 傾いた平面は -80° と等価 ({a:.3f}°)")

    # 畳んだあとも、その角度で実際に同じ姿勢になること
    back = Matrix.Rotation(math.radians(a), 4, AX_X).to_3x3() @ REF_Z
    check(min((back - n).length, (back + n).length) < 1e-5,
          "畳んだ角度で同じ平面が再現できる")


def test_absolute_angle_undefined_on_parallel_axis():
    print("[test] 軸と法線が平行なら測れない")
    check(core.absolute_plane_angle(REF_Z, REF_Z, REF_Z) is None,
          "基準と軸が同じ向きなら None")


# -- ループ整列 ------------------------------------------------------------

def test_walk_edge_loop_closed():
    print("[test] 閉じたエッジループを一周たどれる")
    bm = make_cone(segments=12, cuts=3)
    loop = loop_at_z(bm, 0.0)
    target = set(loop)
    # ループ内の辺を1本選び、そこから一周できるか
    edge = next(e for e in bm.edges
                if e.verts[0] in target and e.verts[1] in target)
    verts = core.walk_edge_loop(edge)
    check(len(verts) == len(loop),
          f"12 頂点すべてを辿れた ({len(verts)}/{len(loop)})")
    check(set(verts) == target, "辿った頂点が元のループと一致")


def test_loop_plane_at_finds_neighbour_loop():
    print("[test] 隣のループの平面を拾える")
    bm = make_cone(segments=12, cuts=3)
    moving = loop_at_z(bm, 0.0)
    moving_set = set(moving)

    # 隣のループ（動かさない方）に接する面を1枚とって、その上の
    # 「隣ループ寄り」の位置を指したことにする
    neighbour = [v for v in bm.verts
                 if v not in moving_set and abs(v.co.z - 0.5) < 1e-4]
    check(len(neighbour) == 12, f"隣ループが 12 頂点ある ({len(neighbour)})")
    nb_set = set(neighbour)

    face = next(f for f in bm.faces
                if not moving_set.intersection(f.verts)
                and len(nb_set.intersection(f.verts)) >= 2)
    co = sum((v.co for v in face.verts if v in nb_set), Vector()) / 2.0

    res = core.loop_plane_at(face, co, blocked=moving_set)
    check(res is not None, "ループ平面が求まった")
    normal, center, n = res
    check(n == 12, f"隣ループ 12 頂点を拾った ({n})")
    check(abs(abs(normal.z) - 1.0) < 1e-5,
          f"水平なループなので法線は Z ({normal.z:.6f})")
    check(abs(center.z - 0.5) < 1e-4,
          f"重心が隣ループの高さ ({center.z:.4f})")


def test_loop_plane_rejects_moving_loop():
    print("[test] 動かしているループは拾わない")
    bm = make_cone(segments=12, cuts=3)
    moving = loop_at_z(bm, 0.0)
    moving_set = set(moving)
    # 動かすループの辺を直接指しても、blocked なので拒否されるべき
    edge = next(e for e in bm.edges
                if e.verts[0] in moving_set and e.verts[1] in moving_set)
    fake_face = edge.link_faces[0]
    co = (edge.verts[0].co + edge.verts[1].co) / 2.0
    check(core.loop_plane_at(fake_face, co, blocked=moving_set) is None,
          "自分自身のループなら None（参照が循環する）")


def test_loop_align_pipeline():
    print("[test] ループの平面へ揃える")
    bm = make_cone(segments=12, cuts=3)
    moving = loop_at_z(bm, 0.0)
    moving_set = set(moving)

    # 隣のループをあらかじめ傾けておき、その傾きに合わせられるか見る
    nb = [v for v in bm.verts
          if v not in moving_set and abs(v.co.z - 0.5) < 1e-4]
    nb_cache = core.RailCache(nb)
    tilt = core.rotation_matrix_about(Vector((1.0, 0.0, 0.0)),
                                      math.radians(15.0), nb_cache.pivot)
    nb_cache.apply_matrix(tilt, mode='PLANE')
    want = core.fit_plane_normal([v.co.copy() for v in nb])

    cache = core.RailCache(moving)
    face = next(f for f in bm.faces
                if not moving_set.intersection(f.verts)
                and len(set(nb).intersection(f.verts)) >= 2)
    co = sum((v.co for v in face.verts if v in set(nb)), Vector()) / 2.0
    res = core.loop_plane_at(face, co, blocked=moving_set)
    check(res is not None, "傾けた隣ループの平面を拾えた")
    picked = res[0]

    ax, ang = core.rotation_to_normal(cache.normal0, picked)
    cache.apply_matrix(core.rotation_matrix_about(ax, ang, cache.pivot),
                       mode='PLANE')
    check(cache.last_clamped == 0, f"クランプなし ({cache.last_clamped})")

    got = core.fit_plane_normal([v.co.copy() for v in moving])
    if got.dot(want) < 0:
        got = -got
    err = (got - want).length
    check(err < 1e-4, f"隣ループと平行になった (誤差 {err:.3e})")
    check(abs(math.degrees(core.absolute_plane_angle(
        got, Vector((1.0, 0.0, 0.0)), Vector((0.0, 0.0, 1.0)))) - 15.0) < 1e-2,
        "絶対角度でも 15° になっている")
    cache.restore()


def run_all():
    for fn in (test_rotation_to_normal_free, test_rotation_to_normal_sign,
               test_rotation_to_normal_constrained, test_snap_angle,
               test_apply_plane_matches_matrix, test_apply_plane_aligns_to_target,
               test_apply_plane_reports_clamping, test_align_roundtrip,
               test_snap_bvh_excludes_moving_faces, test_face_snap_pipeline,
               test_face_snap_constrained,
               test_absolute_angle_of_flat_loop,
               test_absolute_angle_tracks_rotation,
               test_absolute_target_is_reached,
               test_absolute_angle_folds_to_quarter_turn,
               test_absolute_angle_undefined_on_parallel_axis,
               test_walk_edge_loop_closed,
               test_loop_plane_at_finds_neighbour_loop,
               test_loop_plane_rejects_moving_loop,
               test_loop_align_pipeline):
        fn()


if __name__ == "__main__":
    run_all()
    report()
