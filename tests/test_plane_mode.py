"""平面断面モード vs 最近点モードのヘッドレステスト。"""
import os, sys, math
import bpy, bmesh
from mathutils import Vector

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from Looper import core                                        # noqa: E402
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from test_core import make_cone, loop_at_z, check, FAILS       # noqa: E402


def planarity(verts):
    """ループがどれだけ平面から外れているか（最大絶対距離）。"""
    pts = [v.co.copy() for v in verts]
    n = core.fit_plane_normal(pts)
    c = sum(pts, Vector()) / len(pts)
    return max(abs((p - c).dot(n)) for p in pts)


def rotated(bm, loop, deg, axis, mode, extend=True):
    cache = core.RailCache(loop)
    mat = core.rotation_matrix_about(Vector(axis), math.radians(deg), cache.pivot)
    cache.apply_matrix(mat, extend=extend, mode=mode)
    return cache


def test_plane_stays_planar():
    print("[test] PLANE は平面を保つ / NEAREST は崩れる")
    for deg in (10, 25, 40):
        bmp = make_cone(segments=24, cuts=3)
        bmn = make_cone(segments=24, cuts=3)
        zs = sorted({round(v.co.z, 5) for v in bmp.verts})
        lp = loop_at_z(bmp, zs[2]); ln = loop_at_z(bmn, zs[2])
        rotated(bmp, lp, deg, (0, 1, 0), 'PLANE')
        rotated(bmn, ln, deg, (0, 1, 0), 'NEAREST')
        pp, pn = planarity(lp), planarity(ln)
        check(pp < 1e-6, f"{deg}°: PLANE の非平面度 {pp:.3e}")
        check(pn > pp * 10, f"{deg}°: NEAREST は非平面 {pn:.3e} (PLANE {pp:.3e})")
        bmp.free(); bmn.free()


def test_plane_stays_on_cone_surface():
    """平面性テストと合わせて「平面 ∩ 円錐 = 楕円」を保証する。

    真円錐（radius2=0）では、母線上の点は必ず r / (z_apex - z) = 一定 を
    満たす。全頂点がこれを満たす ⇔ 円錐面上に厳密に乗っている。
    """
    print("[test] PLANE の断面頂点は円錐面上に厳密に乗る")
    bm = bmesh.new()
    bmesh.ops.create_cone(bm, cap_ends=True, cap_tris=False, segments=48,
                          radius1=1.0, radius2=0.0, depth=2.0)
    side = [e for e in bm.edges if abs(e.verts[0].co.z - e.verts[1].co.z) > 1e-6]
    bmesh.ops.subdivide_edges(bm, edges=side, cuts=3, use_grid_fill=False)
    bm.verts.ensure_lookup_table()
    apex_z = max(v.co.z for v in bm.verts)
    zs = sorted({round(v.co.z, 5) for v in bm.verts})
    loop = loop_at_z(bm, zs[2])

    for mode, tol in (('PLANE', 2e-6), ('NEAREST', None)):
        for v, o, _p, _c in core.RailCache(loop).entries:
            v.co = o
        rotated(bm, loop, 25, (0, 1, 0), mode)
        ks = [math.hypot(v.co.x, v.co.y) / (apex_z - v.co.z) for v in loop]
        spread = max(ks) - min(ks)
        if tol is not None:
            check(spread < tol, f"{mode}: r/(z_apex-z) のばらつき {spread:.3e}")
        # 半径が一様でない＝円ではない（＝楕円になっている）
        rs = [math.hypot(v.co.x, v.co.y) for v in loop]
        if mode == 'PLANE':
            check(max(rs) - min(rs) > 1e-2,
                  f"円ではない (r {min(rs):.4f}〜{max(rs):.4f})")
    bm.free()


def test_plane_no_dead_vertices():
    print("[test] PLANE では手前/奥の頂点も死なない")
    bmp = make_cone(segments=24, cuts=3)
    bmn = make_cone(segments=24, cuts=3)
    zs = sorted({round(v.co.z, 5) for v in bmp.verts})
    lp = loop_at_z(bmp, zs[2]); ln = loop_at_z(bmn, zs[2])
    op = [v.co.copy() for v in lp]; on = [v.co.copy() for v in ln]
    rotated(bmp, lp, 25, (0, 1, 0), 'PLANE')
    rotated(bmn, ln, 25, (0, 1, 0), 'NEAREST')
    # Y 軸回転なので、Y 軸上（手前/奥）の頂点が「動かない側」
    mp = min((v.co - o).length for v, o in zip(lp, op))
    mn = min((v.co - o).length for v, o in zip(ln, on))
    check(mn < 1e-4, f"NEAREST: 最小移動量 {mn:.3e}（ほぼ死んでいる）")
    check(mp < 1e-4, f"PLANE: 回転軸上の頂点はやはり不動 {mp:.3e}")
    # 軸から離れた頂点の移動量の分布で比べる
    dp = sorted((v.co - o).length for v, o in zip(lp, op))
    dn = sorted((v.co - o).length for v, o in zip(ln, on))
    med_p, med_n = dp[len(dp) // 2], dn[len(dn) // 2]
    check(med_p > med_n, f"PLANE の移動量中央値 {med_p:.4f} > NEAREST {med_n:.4f}")
    bmp.free(); bmn.free()


def test_plane_linear_response():
    print("[test] PLANE は指令角度に対して線形（飽和しない）")
    ratios = {}
    for mode in ('PLANE', 'NEAREST'):
        out = []
        for deg in (10, 20, 30, 40):
            bm = make_cone(segments=24, cuts=3)
            zs = sorted({round(v.co.z, 5) for v in bm.verts})
            loop = loop_at_z(bm, zs[2])
            cache = rotated(bm, loop, deg, (0, 1, 0), mode)
            n = core.fit_plane_normal([v.co.copy() for v in loop])
            # 実際に傾いた角度を法線と Z 軸のなす角で測る
            ang = math.degrees(math.acos(min(1.0, abs(n.dot(Vector((0, 0, 1)))))))
            out.append(ang / deg)
            bm.free()
        ratios[mode] = out
        print("      " + mode + " 実効/指令 = " +
              ", ".join(f"{r:.3f}" for r in out))
    sp = max(ratios['PLANE']) - min(ratios['PLANE'])
    sn = max(ratios['NEAREST']) - min(ratios['NEAREST'])
    check(sp < 0.02, f"PLANE の比のブレ {sp:.4f}（一定＝線形）")
    check(sn > sp, f"NEAREST の比のブレ {sn:.4f} > PLANE {sp:.4f}（飽和）")


def test_plane_volume_preserved():
    print("[test] PLANE でも体積は保たれる")
    bm = make_cone(segments=24, cuts=4)
    v0 = bm.calc_volume(signed=False)
    zs = sorted({round(v.co.z, 5) for v in bm.verts})
    loop = loop_at_z(bm, zs[2])
    cache = core.RailCache(loop)
    worst = 0.0
    for deg in range(-40, 41, 4):
        mat = core.rotation_matrix_about(Vector((0, 1, 0)),
                                         math.radians(deg), cache.pivot)
        cache.apply_matrix(mat, extend=True, mode='PLANE')
        worst = max(worst, abs(bm.calc_volume(signed=False) - v0) / v0)
    check(worst < 1e-6, f"体積誤差 最大 {worst:.3e}")
    bm.free()


def test_plane_no_drift():
    print("[test] PLANE もドリフトしない")
    bm = make_cone(cuts=3)
    zs = sorted({round(v.co.z, 5) for v in bm.verts})
    loop = loop_at_z(bm, zs[2])
    cache = core.RailCache(loop)
    for i in range(200):
        a = math.radians(30.0 * math.sin(i * 0.05))
        cache.apply_matrix(core.rotation_matrix_about(
            Vector((0, 1, 0)), a, cache.pivot), mode='PLANE')
    cache.apply_matrix(core.rotation_matrix_about(
        Vector((0, 1, 0)), 0.0, cache.pivot), mode='PLANE')
    back = max((v.co - o).length for v, o, _p, _c in cache.entries)
    check(back < 1e-6, f"角度0に戻すと元位置に一致 (差 {back:.3e})")
    bm.free()


def test_fit_plane_normal():
    print("[test] 平面フィットの精度")
    import random
    random.seed(0)
    n_true = Vector((0.3, -0.5, 0.81)).normalized()
    u = n_true.cross(Vector((1, 0, 0))).normalized()
    w = n_true.cross(u).normalized()
    pts = []
    for i in range(40):
        t = i * 0.157
        pts.append(u * math.cos(t) * 2.0 + w * math.sin(t) * 0.7
                   + Vector((1, 2, 3)))
    n = core.fit_plane_normal(pts)
    d = min((n - n_true).length, (n + n_true).length)
    check(d < 1e-5, f"法線の誤差 {d:.3e}")


for fn in (test_plane_stays_planar, test_plane_stays_on_cone_surface,
           test_plane_no_dead_vertices, test_plane_linear_response,
           test_plane_volume_preserved, test_plane_no_drift,
           test_fit_plane_normal):
    fn()

print("\n" + "=" * 50)
if FAILS:
    print(f"FAILED: {len(FAILS)}")
    for f in FAILS:
        print("  - " + f)
    sys.exit(1)
print("ALL PASSED")
