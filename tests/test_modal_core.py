"""モーダルの計算部分（RailCache）のヘッドレステスト。"""
import os, sys, math
import bpy, bmesh
from mathutils import Vector, Matrix

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from Looper import core          # noqa: E402
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from test_core import (make_cone, loop_at_z, dist_to_segment,  # noqa: E402
                       rail_anchors, check, FAILS)


def test_cache_matches_postfix():
    print("[test] モーダル計算 == 事後補正の結果")
    bm1 = make_cone(cuts=3); bm2 = make_cone(cuts=3)
    zs = sorted({round(v.co.z, 5) for v in bm1.verts})
    l1 = loop_at_z(bm1, zs[2]); l2 = loop_at_z(bm2, zs[2])

    cache = core.RailCache(l1)
    axis = Vector((0, 1, 0))
    mat = core.rotation_matrix_about(axis, math.radians(25), cache.pivot)
    cache.apply_matrix(mat, extend=True)

    for v in l2:                       # 従来ルート: 崩して → 補正
        v.co = mat @ v.co
    core.fix_verts_to_rails(bm2, l2, extend=True)

    d = max((a.co - b.co).length for a, b in zip(l1, l2))
    check(d < 1e-9, f"両ルートの差 {d:.3e}")
    bm1.free(); bm2.free()


def test_no_drift_over_frames():
    print("[test] 200フレーム分ドラッグしても誤差が溜まらない")
    bm = make_cone(cuts=3)
    zs = sorted({round(v.co.z, 5) for v in bm.verts})
    loop = loop_at_z(bm, zs[2])
    blocked = set(loop)
    anchors = {v.index: rail_anchors(v, blocked) for v in loop}
    cache = core.RailCache(loop)
    axis = Vector((0, 1, 0))
    for i in range(200):               # 少しずつ角度を動かす＝ドラッグ相当
        a = math.radians(30.0 * math.sin(i * 0.05))
        cache.apply_matrix(core.rotation_matrix_about(axis, a, cache.pivot))
    off = max(dist_to_segment(v.co, *anchors[v.index]) for v in loop)
    check(off < 1e-6, f"レールからのズレ {off:.3e}")

    cache.apply_matrix(core.rotation_matrix_about(axis, 0.0, cache.pivot))
    back = max((v.co - o).length for v, o, _p, _c in cache.entries)
    check(back < 1e-9, f"角度0に戻すと元位置に一致 (差 {back:.3e})")
    bm.free()


def test_cancel_restore():
    print("[test] Esc 相当の restore で完全復元")
    bm = make_cone(cuts=3)
    zs = sorted({round(v.co.z, 5) for v in bm.verts})
    loop = loop_at_z(bm, zs[2])
    before = [v.co.copy() for v in loop]
    cache = core.RailCache(loop)
    cache.apply_matrix(core.rotation_matrix_about(
        Vector((1, 0, 0)), math.radians(40), cache.pivot))
    cache.restore()
    d = max((v.co - b).length for v, b in zip(loop, before))
    check(d < 1e-12, f"復元差 {d:.3e}")
    bm.free()


def test_volume_stable_during_drag():
    print("[test] ドラッグ中ずっと体積が保たれる")
    bm = make_cone(segments=24, cuts=4)
    v0 = bm.calc_volume(signed=False)
    zs = sorted({round(v.co.z, 5) for v in bm.verts})
    loop = loop_at_z(bm, zs[2])
    cache = core.RailCache(loop)
    worst = 0.0
    for deg in range(-40, 41, 2):
        cache.apply_matrix(core.rotation_matrix_about(
            Vector((0, 1, 0)), math.radians(deg), cache.pivot))
        worst = max(worst, abs(bm.calc_volume(signed=False) - v0) / v0)
    # 1e-6 は float32 の頂点座標精度による下限（実質ゼロ）
    check(worst < 1e-6, f"-40°〜+40° 全域での体積誤差 最大 {worst:.3e}")
    bm.free()


def test_pivot_is_centroid():
    print("[test] ピボット = ループ重心")
    bm = make_cone(cuts=3)
    zs = sorted({round(v.co.z, 5) for v in bm.verts})
    loop = loop_at_z(bm, zs[2])
    cache = core.RailCache(loop)
    expect = sum((v.co for v in loop), Vector()) / len(loop)
    check((cache.pivot - expect).length < 1e-9, f"pivot {tuple(round(x,4) for x in cache.pivot)}")
    bm.free()


for fn in (test_cache_matches_postfix, test_no_drift_over_frames,
           test_cancel_restore, test_volume_stable_during_drag,
           test_pivot_is_centroid):
    fn()

print("\n" + "=" * 50)
if FAILS:
    print(f"FAILED: {len(FAILS)}")
    for f in FAILS:
        print("  - " + f)
    sys.exit(1)
print("ALL PASSED")
