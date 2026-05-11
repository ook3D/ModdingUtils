#include "remesh.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace mt {

namespace {

struct EdgeKey 
{
    int32_t a, b; // a < b
    bool operator==(const EdgeKey& o) const noexcept { return a == o.a && b == o.b; }
};

struct EdgeHash 
{
    size_t operator()(const EdgeKey& k) const noexcept 
    {
        return static_cast<uint64_t>(static_cast<uint32_t>(k.a)) * 0x9E3779B97F4A7C15ull ^ static_cast<uint64_t>(static_cast<uint32_t>(k.b));
    }
};

inline EdgeKey edge_key(int32_t a, int32_t b) 
{
    return a < b ? EdgeKey{a, b} : EdgeKey{b, a};
}

inline float length_sq(const Vec3& v) { return dot(v, v); }
inline float length(const Vec3& v)    { return std::sqrt(dot(v, v)); }

inline Vec3 normalize_or_zero(const Vec3& v) 
{
    float L2 = dot(v, v);
    if (L2 <= 0.0f) return {0, 0, 0};
    return v * (1.0f / std::sqrt(L2));
}

struct DynMesh {
    std::vector<Vec3> verts;
    std::vector<uint8_t> v_alive;
    std::vector<uint8_t> v_boundary;
    std::vector<std::vector<int32_t>> v_tris;

    std::vector<Tri> tris;
    std::vector<uint8_t> t_alive;

    void build(const Mesh& m, bool detect_boundary);
    void compact(Mesh& out) const;

    int32_t add_vertex(const Vec3& p, bool boundary);
    int32_t add_triangle(int32_t a, int32_t b, int32_t c);
    void kill_triangle(int32_t t);

    void prune_vertex_tris(int32_t v);

    int32_t opposite_vertex(int32_t t, int32_t a, int32_t b) const;
    int  find_edge_tris(int32_t a, int32_t b, int32_t out[2]) const;
    int32_t valence(int32_t v) const;

    int split_long_edges(float Lmax);
    int collapse_short_edges(float Lmin, float Lmax, bool protect_boundary);
    int equalize_valences();
    void tangential_smoothing(float lambda);
};

void DynMesh::build(const Mesh& m, bool detect_boundary) 
{
    verts = m.vertices;
    v_alive.assign(verts.size(), 1);
    v_boundary.assign(verts.size(), 0);
    v_tris.assign(verts.size(), {});

    tris.reserve(m.triangles.size());
    t_alive.reserve(m.triangles.size());
    const int32_t VN = static_cast<int32_t>(verts.size());
    for (const Tri& t : m.triangles) 
    {
        if (t[0] < 0 || t[1] < 0 || t[2] < 0) continue;
        if (t[0] >= VN || t[1] >= VN || t[2] >= VN) continue;
        if (t[0] == t[1] || t[1] == t[2] || t[0] == t[2]) continue;
        int32_t idx = static_cast<int32_t>(tris.size());
        tris.push_back(t);
        t_alive.push_back(1);
        v_tris[static_cast<size_t>(t[0])].push_back(idx);
        v_tris[static_cast<size_t>(t[1])].push_back(idx);
        v_tris[static_cast<size_t>(t[2])].push_back(idx);
    }

    if (detect_boundary) 
    {
        std::unordered_map<EdgeKey, int, EdgeHash> ec;
        ec.reserve(tris.size() * 3);
        for (size_t i = 0; i < tris.size(); ++i) 
        {
            if (!t_alive[i]) continue;
            const Tri& t = tris[i];
            for (int k = 0; k < 3; ++k) 
            {
                ec[edge_key(t[k], t[(k + 1) % 3])] += 1;
            }
        }
        for (const auto& kv : ec) 
        {
            if (kv.second == 1) 
            {
                v_boundary[static_cast<size_t>(kv.first.a)] = 1;
                v_boundary[static_cast<size_t>(kv.first.b)] = 1;
            }
        }
    }
}

void DynMesh::compact(Mesh& out) const 
{
    out.clear();
    std::vector<int32_t> remap(verts.size(), -1);
    for (size_t i = 0; i < verts.size(); ++i) 
    {
        if (!v_alive[i]) continue;
        remap[i] = static_cast<int32_t>(out.vertices.size());
        out.vertices.push_back(verts[i]);
    }
    for (size_t i = 0; i < tris.size(); ++i) 
    {
        if (!t_alive[i]) continue;
        const Tri& t = tris[i];
        int32_t a = remap[t[0]], b = remap[t[1]], c = remap[t[2]];
        if (a < 0 || b < 0 || c < 0) continue;
        if (a == b || b == c || a == c) continue;
        out.triangles.push_back({a, b, c});
    }
}

int32_t DynMesh::add_vertex(const Vec3& p, bool boundary) 
{
    int32_t idx = static_cast<int32_t>(verts.size());
    verts.push_back(p);
    v_alive.push_back(1);
    v_boundary.push_back(boundary ? 1 : 0);
    v_tris.emplace_back();
    return idx;
}

int32_t DynMesh::add_triangle(int32_t a, int32_t b, int32_t c) 
{
    int32_t idx = static_cast<int32_t>(tris.size());
    tris.push_back({a, b, c});
    t_alive.push_back(1);
    v_tris[static_cast<size_t>(a)].push_back(idx);
    v_tris[static_cast<size_t>(b)].push_back(idx);
    v_tris[static_cast<size_t>(c)].push_back(idx);
    return idx;
}

void DynMesh::kill_triangle(int32_t t)
{
    if (t < 0 || static_cast<size_t>(t) >= tris.size()) return;
    t_alive[static_cast<size_t>(t)] = 0;
}

void DynMesh::prune_vertex_tris(int32_t v) 
{
    auto& list = v_tris[static_cast<size_t>(v)];
    list.erase(std::remove_if(list.begin(), list.end(), [&](int32_t t) 
    {
        if (t < 0 || static_cast<size_t>(t) >= tris.size()) return true;
        if (!t_alive[static_cast<size_t>(t)]) return true;
        const Tri& tr = tris[static_cast<size_t>(t)];
        return tr[0] != v && tr[1] != v && tr[2] != v;
    }),
    list.end());
}

int32_t DynMesh::opposite_vertex(int32_t t, int32_t a, int32_t b) const 
{
    const Tri& tr = tris[static_cast<size_t>(t)];
    for (int k = 0; k < 3; ++k) 
    {
        if (tr[k] != a && tr[k] != b) return tr[k];
    }
    return -1;
}

int DynMesh::find_edge_tris(int32_t a, int32_t b, int32_t out[2]) const 
{
    int n = 0;
    for (int32_t t : v_tris[static_cast<size_t>(a)]) 
    {
        if (t < 0 || static_cast<size_t>(t) >= tris.size()) continue;
        if (!t_alive[static_cast<size_t>(t)]) continue;
        const Tri& tr = tris[static_cast<size_t>(t)];
        bool has_a = tr[0] == a || tr[1] == a || tr[2] == a;
        bool has_b = tr[0] == b || tr[1] == b || tr[2] == b;
        if (has_a && has_b) 
        {
            if (n < 2) out[n++] = t;
            else { ++n; break; } // non-manifold
        }
    }
    return n;
}

int32_t DynMesh::valence(int32_t v) const 
{
    std::unordered_set<int32_t> nbrs;
    for (int32_t t : v_tris[static_cast<size_t>(v)]) 
    {
        if (t < 0 || static_cast<size_t>(t) >= tris.size()) continue;
        if (!t_alive[static_cast<size_t>(t)]) continue;
        const Tri& tr = tris[static_cast<size_t>(t)];
        for (int k = 0; k < 3; ++k) if (tr[k] != v) nbrs.insert(tr[k]);
    }
    return static_cast<int32_t>(nbrs.size());
}

int DynMesh::split_long_edges(float Lmax) 
{
    const float Lmax2 = Lmax * Lmax;

    std::unordered_set<EdgeKey, EdgeHash> seen;
    std::vector<EdgeKey> queue;
    queue.reserve(tris.size() * 3);
    for (size_t i = 0; i < tris.size(); ++i) 
    {
        if (!t_alive[i]) continue;
        const Tri& tr = tris[i];
        for (int k = 0; k < 3; ++k) 
        {
            EdgeKey ek = edge_key(tr[k], tr[(k + 1) % 3]);
            if (seen.insert(ek).second) queue.push_back(ek);
        }
    }

    int splits = 0;
    for (const EdgeKey& ek : queue) 
    {
        int32_t a = ek.a, b = ek.b;
        if (!v_alive[static_cast<size_t>(a)] || !v_alive[static_cast<size_t>(b)]) continue;
        if (length_sq(verts[a] - verts[b]) <= Lmax2) continue;

        int32_t inc[2];
        int n = find_edge_tris(a, b, inc);
        if (n == 0 || n > 2) continue; // non manifold or already gone

        Vec3 mid = (verts[a] + verts[b]) * 0.5f;
        bool m_boundary = (n == 1) || (v_boundary[static_cast<size_t>(a)] && v_boundary[static_cast<size_t>(b)]);
        int32_t m = add_vertex(mid, m_boundary);

        for (int i = 0; i < n; ++i) 
        {
            int32_t t = inc[i];
            int32_t c = opposite_vertex(t, a, b);
            const Tri& tr = tris[static_cast<size_t>(t)];

            bool ab_forward = false;
            for (int k = 0; k < 3; ++k) 
            {
                if (tr[k] == a && tr[(k + 1) % 3] == b) { ab_forward = true; break; }
            }

            kill_triangle(t);
            if (ab_forward)
            {
                add_triangle(a, m, c);
                add_triangle(m, b, c);
            } else 
            {
                add_triangle(b, m, c);
                add_triangle(m, a, c);
            }
        }
        prune_vertex_tris(a);
        prune_vertex_tris(b);
        ++splits;
    }
    return splits;
}

int DynMesh::collapse_short_edges(float Lmin, float Lmax, bool protect_boundary)
 {
    const float Lmin2 = Lmin * Lmin;
    const float Lmax2 = Lmax * Lmax;

    std::unordered_set<EdgeKey, EdgeHash> seen;
    std::vector<EdgeKey> queue;
    queue.reserve(tris.size() * 3);
    for (size_t i = 0; i < tris.size(); ++i) 
    {
        if (!t_alive[i]) continue;
        const Tri& tr = tris[i];
        for (int k = 0; k < 3; ++k) 
        {
            EdgeKey ek = edge_key(tr[k], tr[(k + 1) % 3]);
            if (seen.insert(ek).second) queue.push_back(ek);
        }
    }

    int collapses = 0;
    for (const EdgeKey& ek : queue) 
    {
        int32_t a = ek.a, b = ek.b;
        if (!v_alive[static_cast<size_t>(a)] || !v_alive[static_cast<size_t>(b)]) continue;
        if (length_sq(verts[a] - verts[b]) >= Lmin2) continue;

        const bool ba = v_boundary[static_cast<size_t>(a)] != 0;
        const bool bb = v_boundary[static_cast<size_t>(b)] != 0;
        if (protect_boundary && (ba || bb)) continue;

        int32_t inc[2];
        int n = find_edge_tris(a, b, inc);
        if (n == 0 || n > 2) continue;

        int32_t keep = a, drop = b;
        if (bb && !ba) { keep = b; drop = a; }
        Vec3 new_pos = (verts[keep] + verts[drop]) * 0.5f;
        if (v_boundary[static_cast<size_t>(keep)]) new_pos = verts[keep];
        
        std::vector<int32_t> drop_tris;
        for (int32_t t : v_tris[static_cast<size_t>(drop)]) 
        {
            if (t < 0 || static_cast<size_t>(t) >= tris.size()) continue;
            if (!t_alive[static_cast<size_t>(t)]) continue;
            drop_tris.push_back(t);
        }

        auto sorted_tri_key = [](int32_t x, int32_t y, int32_t z) 
        {
            if (x > y) std::swap(x, y);
            if (y > z) std::swap(y, z);
            if (x > y) std::swap(x, y);
            return std::array<int32_t, 3>{x, y, z};
        };

        struct ArrHash 
        {
            size_t operator()(const std::array<int32_t, 3>& a) const noexcept {
                size_t h = static_cast<uint32_t>(a[0]) * 73856093u;
                h ^= static_cast<uint32_t>(a[1]) * 19349663u + 0x9e3779b9 + (h << 6) + (h >> 2);
                h ^= static_cast<uint32_t>(a[2]) * 83492791u + 0x9e3779b9 + (h << 6) + (h >> 2);
                return h;
            }
        };
        std::unordered_set<std::array<int32_t, 3>, ArrHash> proposed;
        bool ok = true;

        std::vector<Tri> new_tris;
        new_tris.reserve(drop_tris.size());
        for (int32_t t : drop_tris) 
        {
            const Tri& tr = tris[static_cast<size_t>(t)];
            int32_t na = (tr[0] == drop) ? keep : tr[0];
            int32_t nb = (tr[1] == drop) ? keep : tr[1];
            int32_t nc = (tr[2] == drop) ? keep : tr[2];
            if (na == nb || nb == nc || na == nc) continue;
            new_tris.push_back({na, nb, nc});

            auto key = sorted_tri_key(na, nb, nc);
            if (!proposed.insert(key).second) { ok = false; break; }
        }
        if (!ok) continue;

        if (ok) 
        {
            const Vec3 kp = new_pos;
            for (const Tri& tr : new_tris) 
            {
                for (int k = 0; k < 3; ++k) 
                {
                    int32_t u = tr[k], v = tr[(k + 1) % 3];
                    if (u == keep || v == keep)
                    {
                        const Vec3& pu = (u == keep) ? kp : verts[u];
                        const Vec3& pv = (v == keep) ? kp : verts[v];
                        if (length_sq(pu - pv) > Lmax2) { ok = false; break; }
                    }
                }
                if (!ok) break;
            }
        }
        if (!ok) continue;

        if (ok) 
        {
            for (size_t i = 0; i < drop_tris.size(); ++i) 
            {
                int32_t t = drop_tris[i];
                const Tri& old_tr = tris[static_cast<size_t>(t)];
                bool has_keep = (old_tr[0] == keep || old_tr[1] == keep || old_tr[2] == keep);
                if (has_keep) continue; // these collapse to a line

                Vec3 op0 = verts[old_tr[0]];
                Vec3 op1 = verts[old_tr[1]];
                Vec3 op2 = verts[old_tr[2]];
                Vec3 old_n = cross(op1 - op0, op2 - op0);

                Vec3 np0 = (old_tr[0] == drop) ? new_pos : verts[old_tr[0]];
                Vec3 np1 = (old_tr[1] == drop) ? new_pos : verts[old_tr[1]];
                Vec3 np2 = (old_tr[2] == drop) ? new_pos : verts[old_tr[2]];
                Vec3 new_n = cross(np1 - np0, np2 - np0);

                if (dot(old_n, new_n) <= 0.0f) { ok = false; break; }
            }
        }
        if (!ok) continue;

        for (int32_t t : drop_tris) kill_triangle(t);
        for (const Tri& tr : new_tris) add_triangle(tr[0], tr[1], tr[2]);
        verts[keep] = new_pos;
        v_alive[static_cast<size_t>(drop)] = 0;
        v_tris[static_cast<size_t>(drop)].clear();
        prune_vertex_tris(keep);
        ++collapses;
    }
    return collapses;
}

int DynMesh::equalize_valences() 
{
    auto ideal = [&](int32_t v) 
    {
        return v_boundary[static_cast<size_t>(v)] ? 4 : 6;
    };

    std::unordered_set<EdgeKey, EdgeHash> seen;
    std::vector<EdgeKey> queue;
    queue.reserve(tris.size() * 3);
    for (size_t i = 0; i < tris.size(); ++i) 
    {
        if (!t_alive[i]) continue;
        const Tri& tr = tris[i];
        for (int k = 0; k < 3; ++k)
        {
            EdgeKey ek = edge_key(tr[k], tr[(k + 1) % 3]);
            if (seen.insert(ek).second) queue.push_back(ek);
        }
    }

    int flips = 0;
    for (const EdgeKey& ek : queue) 
    {
        int32_t a = ek.a, b = ek.b;
        if (!v_alive[static_cast<size_t>(a)] || !v_alive[static_cast<size_t>(b)]) continue;

        int32_t inc[2];
        int n = find_edge_tris(a, b, inc);
        if (n != 2) continue; // boundary or non manifold

        int32_t c = opposite_vertex(inc[0], a, b);
        int32_t d = opposite_vertex(inc[1], a, b);
        if (c < 0 || d < 0 || c == d) continue;

        int va = valence(a), vb = valence(b), vc = valence(c), vd = valence(d);
        int ia = ideal(a), ib = ideal(b), ic = ideal(c), id = ideal(d);
        int before = std::abs(va - ia) + std::abs(vb - ib) + std::abs(vc - ic) + std::abs(vd - id);
        int after  = std::abs(va - 1 - ia) + std::abs(vb - 1 - ib) + std::abs(vc + 1 - ic) + std::abs(vd + 1 - id);
        if (after >= before) continue;

        Vec3 n0_old = cross(verts[b] - verts[a], verts[c] - verts[a]);
        Vec3 n1_old = cross(verts[a] - verts[b], verts[d] - verts[b]);
        Vec3 n0_new = cross(verts[d] - verts[c], verts[a] - verts[c]);
        Vec3 n1_new = cross(verts[c] - verts[d], verts[b] - verts[d]);
        if (dot(n0_old, n0_new) <= 0.0f) continue;
        if (dot(n1_old, n1_new) <= 0.0f) continue;

        bool ab_in_t0 = false;
        {
            const Tri& tr = tris[static_cast<size_t>(inc[0])];
            for (int k = 0; k < 3; ++k) 
            {
                if (tr[k] == a && tr[(k + 1) % 3] == b) { ab_in_t0 = true; break; }
            }
        }

        kill_triangle(inc[0]);
        kill_triangle(inc[1]);
        if (ab_in_t0) 
        {
            add_triangle(a, d, c);
            add_triangle(b, c, d);
        } else
        {
            add_triangle(a, c, d);
            add_triangle(b, d, c);
        }
        prune_vertex_tris(a);
        prune_vertex_tris(b);
        prune_vertex_tris(c);
        prune_vertex_tris(d);
        ++flips;
    }
    return flips;
}

void DynMesh::tangential_smoothing(float lambda) 
{
    std::vector<Vec3> new_pos = verts;

    for (size_t v = 0; v < verts.size(); ++v) 
    {
        if (!v_alive[v]) continue;
        if (v_boundary[v]) continue;

        std::unordered_set<int32_t> nbrs;
        Vec3 normal{0, 0, 0};
        for (int32_t t : v_tris[v]) 
        {
            if (t < 0 || static_cast<size_t>(t) >= tris.size()) continue;
            if (!t_alive[static_cast<size_t>(t)]) continue;
            const Tri& tr = tris[static_cast<size_t>(t)];
            bool has = false;
            for (int k = 0; k < 3; ++k) if (tr[k] == static_cast<int32_t>(v)) has = true;
            if (!has) continue;
            for (int k = 0; k < 3; ++k) 
            {
                if (tr[k] != static_cast<int32_t>(v)) nbrs.insert(tr[k]);
            }
            Vec3 fn = cross(verts[tr[1]] - verts[tr[0]], verts[tr[2]] - verts[tr[0]]);
            normal = normal + fn;
        }
        if (nbrs.empty()) continue;

        Vec3 centroid{0, 0, 0};
        for (int32_t nb : nbrs) centroid = centroid + verts[nb];
        centroid = centroid * (1.0f / static_cast<float>(nbrs.size()));

        Vec3 delta = centroid - verts[v];
        normal = normalize_or_zero(normal);
        if (dot(normal, normal) > 0.0f) 
        {
            float dn = dot(delta, normal);
            delta = delta - normal * dn;
        }
        new_pos[v] = verts[v] + delta * lambda;
    }
    verts.swap(new_pos);
}

} // namespace

bool remesh_isotropic(Mesh& mesh, const MTRemeshParams& params) 
{
    if (mesh.empty()) return false;

    float L = params.target_edge_length;
    if (!(L > 0.0f)) 
    {
        Vec3 mn, mx;
        mesh.bounds(mn, mx);
        Vec3 d = mx - mn;
        float diag = std::sqrt(dot(d, d));
        float rel = params.relative_edge_length > 0.0f ? params.relative_edge_length : 0.05f;
        L = diag * rel;
    }
    if (!(L > 0.0f) || !std::isfinite(L)) return false;

    const float Lmin = (4.0f / 5.0f) * L;
    const float Lmax = (4.0f / 3.0f) * L;
    const bool protect_boundary = params.protect_boundary != 0;
    const int iterations = params.iterations > 0 ? params.iterations : 5;

    DynMesh dm;
    dm.build(mesh, protect_boundary);

    for (int it = 0; it < iterations; ++it) 
    {
        dm.split_long_edges(Lmax);
        dm.collapse_short_edges(Lmin, Lmax, protect_boundary);
        for (int k = 0; k < 3; ++k) dm.equalize_valences();
        dm.tangential_smoothing(0.5f);
    }

    Mesh out;
    dm.compact(out);
    if (out.empty()) return false;
    mesh = std::move(out);
    return true;
}

} // namespace mt
