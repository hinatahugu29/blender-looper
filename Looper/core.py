# SPDX-License-Identifier: GPL-3.0-or-later

"""レール投影のコアロジック。

bpy に依存せず bmesh / mathutils だけで完結させる（ヘッドレステスト用）。

考え方
------
ループカットで挿入された頂点 v は、必ず「上下の隣接頂点 a, b を結ぶ線分」の上に
乗っている。ユーザーが v を回転・移動させても a, b は動いていないので、
線分 (a, b) は編集後のメッシュからそのまま復元できる。
v をこの線分へ最近点射影すれば、元の面形状が厳密に戻る。

extend=True のときは、a / b の先へエッジリングを辿って折れ線を伸ばし、
隣のループを越えてスライドできるようにする。
"""

import math

from mathutils import Vector
from mathutils.bvhtree import BVHTree

MAX_WALK = 256


def _other(edge, vert):
    return edge.verts[1] if edge.verts[0] is vert else edge.verts[0]


def _ring_next(vert, edge):
    """vert を挟んで edge の「向かい側」の辺を返す（エッジループ継続）。

    valence 4 の通常頂点でのみ成立。継続できなければ None。
    """
    if len(vert.link_edges) != 4:
        return None
    faces = set(edge.link_faces)
    cands = [e for e in vert.link_edges
             if e is not edge and not (set(e.link_faces) & faces)]
    return cands[0] if len(cands) == 1 else None


def _walk(vert, edge, blocked):
    """vert から edge 方向へ辿り、通過した頂点を順に返す。

    blocked（＝動かす対象の頂点集合）に当たったらそこで止める。
    """
    out = []
    cur_v, cur_e = vert, edge
    seen = {vert}
    for _ in range(MAX_WALK):
        nxt = _other(cur_e, cur_v)
        if nxt in seen or nxt in blocked:
            break
        out.append(nxt)
        seen.add(nxt)
        cur_e = _ring_next(nxt, cur_e)
        if cur_e is None:
            break
        cur_v = nxt
    return out


def _pick_rail_edges(vert, blocked):
    """vert からレールとして使える辺のペアを選ぶ。

    blocked に含まれない隣接頂点への辺が候補。3本以上あるときは
    最も向きが反平行なペア（＝一番「筋が通った」レール）を選ぶ。
    """
    cands = [e for e in vert.link_edges if _other(e, vert) not in blocked]
    if len(cands) < 2:
        return None
    if len(cands) == 2:
        return cands[0], cands[1]

    best, best_dot = None, 2.0
    dirs = []
    for e in cands:
        d = (_other(e, vert).co - vert.co)
        if d.length_squared < 1e-16:
            continue
        dirs.append((e, d.normalized()))
    for i in range(len(dirs)):
        for j in range(i + 1, len(dirs)):
            dot = dirs[i][1].dot(dirs[j][1])
            if dot < best_dot:
                best_dot, best = dot, (dirs[i][0], dirs[j][0])
    return best


def _closest_on_polyline(points, co, segments=None):
    """折れ線 points 上で co に最も近い点を返す。

    segments を渡すとその index の区間だけを対象にする。
    """
    rng = segments if segments is not None else range(len(points) - 1)
    best_p, best_d = None, float('inf')
    for i in rng:
        a, b = points[i], points[i + 1]
        ab = b - a
        L2 = ab.length_squared
        t = 0.0 if L2 < 1e-16 else max(0.0, min(1.0, (co - a).dot(ab) / L2))
        p = a + ab * t
        d = (co - p).length_squared
        if d < best_d:
            best_d, best_p = d, p
    return best_p


def build_rail(vert, blocked, extend=True):
    """vert のレール折れ線と、中央区間（元の線分）の index を返す。

    戻り値: (points, center_index) / 作れなければ None
    """
    pair = _pick_rail_edges(vert, blocked)
    if pair is None:
        return None
    e_a, e_b = pair
    v_a, v_b = _other(e_a, vert), _other(e_b, vert)

    if extend:
        back = _walk(vert, e_a, blocked)   # v_a, その先...
        fwd = _walk(vert, e_b, blocked)    # v_b, その先...
    else:
        back, fwd = [v_a], [v_b]

    points = [v.co.copy() for v in reversed(back)] + [v.co.copy() for v in fwd]
    center = len(back) - 1  # points[center] == v_a, points[center+1] == v_b
    return points, center


def fix_verts_to_rails(bm, verts, extend=True):
    """verts をそれぞれのレール上へ射影する。

    verts 自身は互いにアンカーにならない（＝blocked 扱い）。
    戻り値: dict(moved=int, skipped=int, max_delta=float)
    """
    blocked = set(verts)
    results = []
    skipped = 0

    # 全頂点の新座標を先に計算してから書き戻す（順序依存を避ける）
    for v in verts:
        rail = build_rail(v, blocked, extend=extend)
        if rail is None:
            skipped += 1
            continue
        points, center = rail
        if len(points) < 2:
            skipped += 1
            continue
        if extend:
            new_co = _closest_on_polyline(points, v.co)
        else:
            new_co = _closest_on_polyline(points, v.co, segments=[center])
        results.append((v, new_co))

    max_delta = 0.0
    for v, co in results:
        max_delta = max(max_delta, (co - v.co).length)
        v.co = co

    return {"moved": len(results), "skipped": skipped, "max_delta": max_delta}


def fix_selection_to_rails(bm, extend=True):
    """bm の選択頂点をレール上へ射影する。"""
    sel = [v for v in bm.verts if v.select and not v.hide]
    if not sel:
        return {"moved": 0, "skipped": 0, "max_delta": 0.0, "error": "no_selection"}
    return fix_verts_to_rails(bm, sel, extend=extend)


# --------------------------------------------------------------------------
# 平面断面モード
# --------------------------------------------------------------------------

def fit_plane_normal(points, squarings=12):
    """点群に最小二乗で平面をフィットし、その法線を返す。

    共分散行列の最小固有ベクトル＝法線。固有値分解の代わりに
    (trace*I - M) に対する累乗法で最小固有ベクトルを取り出す。
    ループの頂点順序を必要としないので、任意の選択に使える。

    単純な反復だと細長いループ（固有値が離れている）で収束が遅いので、
    行列を squarings 回二乗して B^(2^squarings) を作る。
    """
    n = len(points)
    if n < 3:
        return None
    c = Vector((0.0, 0.0, 0.0))
    for p in points:
        c += p
    c /= n

    m = [[0.0] * 3 for _ in range(3)]
    for p in points:
        d = p - c
        for i in range(3):
            for j in range(3):
                m[i][j] += d[i] * d[j]

    tr = m[0][0] + m[1][1] + m[2][2]
    if tr < 1e-20:
        return None
    # B = tr*I - M は M の最小固有ベクトルを最大固有ベクトルとして持つ
    b = [[(tr if i == j else 0.0) - m[i][j] for j in range(3)] for i in range(3)]

    def _norm(mat):
        mx = max(abs(mat[i][j]) for i in range(3) for j in range(3))
        if mx < 1e-30:
            return None
        return [[mat[i][j] / mx for j in range(3)] for i in range(3)]

    b = _norm(b)
    if b is None:
        return None
    for _ in range(squarings):
        sq = [[sum(b[i][k] * b[k][j] for k in range(3)) for j in range(3)]
              for i in range(3)]
        sq = _norm(sq)
        if sq is None:
            break
        b = sq

    # 収束後の各列は最大固有ベクトルに揃うので、一番ノルムが大きい列を採る
    cols = [Vector((b[0][j], b[1][j], b[2][j])) for j in range(3)]
    best = max(cols, key=lambda c: c.length_squared)
    if best.length < 1e-20:
        return None
    return best.normalized()


def intersect_rail_plane_ex(points, center, ref_co, normal, plane_pt,
                            extend=True):
    """レール折れ線と平面の交点を (座標, 交差したか) で返す。

    交点が複数あるときは ref_co に最も近いものを採用（安定性のため
    ref_co には invoke 時の元座標を渡す）。
    交点が無ければ、平面に最も近い端点へクランプして False を返す。
    クランプは「目標平面まで届かなかった」＝結果が平面に乗らないことを
    意味するので、呼び出し側はこれを数えてユーザーに知らせること。
    """
    rng = range(len(points) - 1) if extend else [center]
    s = [(p - plane_pt).dot(normal) for p in points]

    best, best_d = None, float('inf')
    for i in rng:
        s0, s1 = s[i], s[i + 1]
        if s0 == 0.0 and s1 == 0.0:
            continue
        if (s0 > 0.0) == (s1 > 0.0) and s0 != 0.0 and s1 != 0.0:
            continue                      # 符号が変わらない＝交差なし
        denom = s0 - s1
        if abs(denom) < 1e-20:
            continue
        t = max(0.0, min(1.0, s0 / denom))
        p = points[i] + (points[i + 1] - points[i]) * t
        d = (p - ref_co).length_squared
        if d < best_d:
            best_d, best = d, p
    if best is not None:
        return best, True

    # 交点なし: レールの端から出てしまったので端点へクランプ
    idx = list(rng)
    cand = [idx[0], idx[-1] + 1]
    clamped = min((points[i] for i in cand),
                  key=lambda p: abs((p - plane_pt).dot(normal)))
    return clamped, False


def intersect_rail_plane(points, center, ref_co, normal, plane_pt, extend=True):
    """交点の座標だけを返す薄いラッパ。"""
    return intersect_rail_plane_ex(points, center, ref_co, normal, plane_pt,
                                   extend)[0]


def fix_selection_to_plane(bm, extend=True):
    """選択ループに平面をフィットし、各レールとの交点へ移動させる。"""
    sel = [v for v in bm.verts if v.select and not v.hide]
    if not sel:
        return {"moved": 0, "skipped": 0, "max_delta": 0.0, "error": "no_selection"}

    normal = fit_plane_normal([v.co for v in sel])
    if normal is None:
        return {"moved": 0, "skipped": 0, "max_delta": 0.0, "error": "no_plane"}
    plane_pt = sum((v.co for v in sel), Vector()) / len(sel)

    blocked = set(sel)
    results, skipped, clamped = [], 0, 0
    for v in sel:
        rail = build_rail(v, blocked, extend=extend)
        if rail is None or len(rail[0]) < 2:
            skipped += 1
            continue
        points, center = rail
        co, hit = intersect_rail_plane_ex(points, center, v.co,
                                          normal, plane_pt, extend)
        if not hit:
            clamped += 1
        results.append((v, co))

    max_delta = 0.0
    for v, co in results:
        max_delta = max(max_delta, (co - v.co).length)
        v.co = co
    return {"moved": len(results), "skipped": skipped, "max_delta": max_delta,
            "clamped": clamped}


# --------------------------------------------------------------------------
# モーダル用: レールを事前キャッシュして毎フレーム再射影する
# --------------------------------------------------------------------------

class RailCache:
    """invoke 時に1回だけ作る。アンカーは動かないので使い回せる。

    entries: [(vert, orig_co, points|None, center)]
    """

    __slots__ = ("entries", "pivot", "railless", "normal0", "last_clamped")

    def __init__(self, verts, extend_build=True):
        self.last_clamped = 0
        blocked = set(verts)
        self.entries = []
        self.railless = 0
        acc = Vector((0.0, 0.0, 0.0))
        for v in verts:
            rail = build_rail(v, blocked, extend=extend_build)
            if rail is None or len(rail[0]) < 2:
                self.entries.append((v, v.co.copy(), None, 0))
                self.railless += 1
            else:
                points, center = rail
                self.entries.append((v, v.co.copy(), points, center))
            acc += v.co
        self.pivot = acc / len(verts) if verts else Vector()
        self.normal0 = fit_plane_normal([e[1] for e in self.entries])

    @property
    def has_plane(self):
        return self.normal0 is not None

    def restore(self):
        for v, orig, _p, _c in self.entries:
            v.co = orig.copy()

    def apply_plane(self, normal, plane_pt, extend=True, fallback_mat=None):
        """目標平面 (normal, plane_pt) を直接指定して、各レールとの交点へ移す。

        行列を経由しないので、「この面の法線に揃える」のように回転量ではなく
        姿勢そのものが決まっている操作は、これを直接呼べばよい。

        レールを持たない頂点は fallback_mat があればそれを適用し、
        無ければ元位置のまま（平面だけでは行き先が決まらないため）。

        戻り値: 平面と交差できず端点へクランプした頂点数。
        0 でなければ結果は平面に乗っていないので、呼び出し側で知らせること。
        """
        n = normal.normalized()
        clamped = 0
        for v, orig, points, center in self.entries:
            if points is None:
                v.co = (fallback_mat @ orig) if fallback_mat else orig.copy()
                continue
            co, hit = intersect_rail_plane_ex(points, center, orig,
                                              n, plane_pt, extend)
            if not hit:
                clamped += 1
            v.co = co
        self.last_clamped = clamped
        return clamped

    def apply_matrix(self, mat, extend=True, mode='NEAREST'):
        """orig 座標に mat を適用してからレールへ乗せ直す。

        累積ではなく毎回 orig から計算するので誤差が溜まらない。

        mode='NEAREST' : 頂点ごとに独立して最近点射影
        mode='PLANE'   : ループ全体を1枚の平面として扱い、レールとの交点を取る
        """
        if mode == 'PLANE' and self.normal0 is not None:
            # 行列は「目標平面を作る手段」でしかないので、ここで平面に還元する
            self.apply_plane((mat.to_3x3() @ self.normal0), mat @ self.pivot,
                             extend=extend, fallback_mat=mat)
            return 0.0

        self.last_clamped = 0
        max_off = 0.0
        for v, orig, points, center in self.entries:
            target = mat @ orig
            if points is None:
                v.co = target
                continue
            if extend:
                co = _closest_on_polyline(points, target)
            else:
                co = _closest_on_polyline(points, target, segments=[center])
            max_off = max(max_off, (target - co).length)
            v.co = co
        return max_off


def rotation_matrix_about(axis, angle, pivot):
    """pivot を中心に axis まわりに angle 回転する 4x4 行列。"""
    from mathutils import Matrix
    R = Matrix.Rotation(angle, 4, axis.normalized())
    return Matrix.Translation(pivot) @ R @ Matrix.Translation(-pivot)


# --------------------------------------------------------------------------
# 姿勢 → 回転量の逆算（スナップ用）
# --------------------------------------------------------------------------

def snap_angle(angle, step):
    """angle を step 刻みへ丸める。"""
    if step <= 0.0:
        return angle
    return round(angle / step) * step


def _project_out(vec, axis):
    """axis 成分を抜いた（axis に垂直な平面へ射影した）ベクトル。"""
    return vec - axis * vec.dot(axis)


def rotation_to_normal(normal0, target, axis=None):
    """normal0 を target の向きへ揃える回転を (axis, angle) で返す。

    姿勢を直接指定するスナップでも、結果を常に (軸, 角度) に還元しておくと、
    確定後のリドゥもヘッダ表示もスナップ解除後の継続も、通常の回転と
    まったく同じ経路に乗る。スナップを特別な状態にしないための逆算。

    平面の法線は符号が反転しても同じ平面を表すので、常に ±target のうち
    近い方を採る。そうしないと、ほぼ揃っているのに 180 度回る事故が起きる。

    axis を渡すとその軸まわりの回転だけに制限し、target に最も近づく角度を
    返す（軸拘束中のスナップ）。決められなければ None。
    """
    n0 = normal0.normalized()
    t = target.normalized()
    if t.dot(n0) < 0.0:
        t = -t

    if axis is None:
        ax, ang = n0.rotation_difference(t).to_axis_angle()
        if ax.length_squared < 1e-16:
            return None
        return ax.normalized(), ang

    ax = axis.normalized()
    p0 = _project_out(n0, ax)
    pt = _project_out(t, ax)
    if p0.length_squared < 1e-12 or pt.length_squared < 1e-12:
        return None            # 軸と法線がほぼ平行＝この軸では近づけられない
    p0.normalize()
    pt.normalize()
    ang = math.atan2(ax.dot(p0.cross(pt)), p0.dot(pt))
    # ±pi ずれても同じ平面なので、回転量が小さい方を選ぶ
    if ang > math.pi / 2:
        ang -= math.pi
    elif ang < -math.pi / 2:
        ang += math.pi
    return ax, ang


def build_snap_bvh(bm, moving):
    """面スナップ用の BVH を作る。moving に触れる面は除外する。

    自分自身に揃えようとすると参照が循環するし、その面は操作中に動いて
    しまう。除外したあとに残る面はどれも動かないので、座標を一度写し取れば
    操作中ずっと使い回せる（毎フレーム再構築は不要）。

    戻り値: (BVHTree, 面のリスト) / 対象の面が1枚も無ければ None
    面のリストは ray_cast が返すインデックスから元の面を引くためのもの。
    """
    moving = set(moving)
    bm.verts.index_update()
    coords = [v.co.copy() for v in bm.verts]
    faces = [f for f in bm.faces
             if not f.hide and not moving.intersection(f.verts)]
    if not faces:
        return None
    polys = [[v.index for v in f.verts] for f in faces]
    return BVHTree.FromPolygons(coords, polys, all_triangles=False), faces


def absolute_plane_angle(normal, axis, reference):
    """axis まわりで測った、平面の絶対的な傾き。

    reference を 0 度として、normal がそこから何度傾いているかを返す。
    「XY 平面を 0 度として X 軸まわりに何度」は reference=Z, axis=X。

    rotation_to_normal と同じ ± の畳み込みが入るので、戻り値は必ず
    -90°..+90° に収まる。平面は 180 度回すと元に戻るので、これが
    「傾き」として自然な範囲になる。

    軸と法線がほぼ平行で測りようがなければ None。
    """
    res = rotation_to_normal(reference, normal, axis=axis)
    return None if res is None else res[1]


def walk_edge_loop(edge, limit=MAX_WALK):
    """edge を含むエッジループ上の頂点を返す。

    valence 4 の頂点で「面を共有しない向かい側の辺」へ進む＝レール延長に
    使っている _ring_next と同じ規則。あちらはレール方向のループを辿り、
    こちらは狙ったループそのものを辿る、という違いだけ。

    平面フィットは頂点順序を必要としないので、順序は保証しない。
    """
    verts = {edge.verts[0], edge.verts[1]}
    seen = {edge}
    for i in (0, 1):
        cur_e, cur_v = edge, edge.verts[i]
        for _ in range(limit):
            nxt_e = _ring_next(cur_v, cur_e)
            if nxt_e is None or nxt_e in seen:
                break
            seen.add(nxt_e)
            cur_v = _other(nxt_e, cur_v)
            verts.add(cur_v)
            cur_e = nxt_e
    return list(verts)


def closest_edge_in_face(face, co):
    """face の辺のうち co に最も近いものを返す。"""
    best, best_d = None, float('inf')
    for e in face.edges:
        p = _closest_on_polyline([e.verts[0].co, e.verts[1].co], co)
        d = (co - p).length_squared
        if d < best_d:
            best_d, best = d, e
    return best


def loop_plane_at(face, co, blocked=()):
    """face 上の co に最も近い辺のエッジループに平面をフィットする。

    blocked（＝動かす頂点）がループに含まれていたら None を返す。
    動いているループに揃えようとすると参照が循環する。

    戻り値: (法線, ループ重心, 頂点数) / 求まらなければ None
    """
    edge = closest_edge_in_face(face, co)
    if edge is None:
        return None
    verts = walk_edge_loop(edge)
    if len(verts) < 3:
        return None
    blocked = set(blocked)
    if blocked and blocked.intersection(verts):
        return None
    pts = [v.co.copy() for v in verts]
    normal = fit_plane_normal(pts)
    if normal is None:
        return None
    center = sum(pts, Vector()) / len(pts)
    return normal, center, len(pts)
