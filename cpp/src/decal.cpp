#include "decal.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

namespace mt {

namespace {

inline Vec3 normalize_or_zero(const Vec3& v) 
{
    float L2 = dot(v, v);
    if (L2 <= 0.0f) return {0, 0, 0};
    return v * (1.0f / std::sqrt(L2));
}

inline float length(const Vec3& v) 
{ 
    return std::sqrt(dot(v, v)); 
}

} // namespace

bool generate_glue_decal(const std::vector<Vec3>& src_verts,
                         const std::vector<std::pair<int32_t, int32_t>>& edges,
                         const std::vector<Vec3>& wall_normals,
                         const MTDecalParams& params,
                         Mesh& out)
{
    out.clear();
    if (src_verts.empty() || edges.empty()) return false;
    if (wall_normals.size() != edges.size()) return false;

    Vec3 up = normalize_or_zero(Vec3{params.up_x, params.up_y, params.up_z});
    if (dot(up, up) == 0.0f) up = {0.0f, 0.0f, 1.0f};

    const float wall_h = params.wall_height;
    const float floor_w = params.floor_extent;
    const float bias = params.bias;
    const size_t VN = src_verts.size();
    const size_t EN = edges.size();

    std::vector<Vec3>    edge_outward(EN, {0, 0, 0});
    std::vector<uint8_t> edge_valid(EN, 0);
    for (size_t i = 0; i < EN; ++i) 
    {
        Vec3 n = wall_normals[i];
        Vec3 oh = n - up * dot(n, up);
        if (dot(oh, oh) < 1e-10f) continue; // face nearly horizontal
        edge_outward[i] = normalize_or_zero(oh);
        edge_valid[i] = 1;
    }

    std::vector<Vec3> avg_out(VN, {0, 0, 0});
    std::vector<int32_t> v_degree(VN, 0);
    for (size_t i = 0; i < EN; ++i) 
    {
        if (!edge_valid[i]) continue;
        int32_t a = edges[i].first;
        int32_t b = edges[i].second;
        if (a < 0 || b < 0 || static_cast<size_t>(a) >= VN || static_cast<size_t>(b) >= VN) continue;
        avg_out[a] = avg_out[a] + edge_outward[i];
        avg_out[b] = avg_out[b] + edge_outward[i];
        v_degree[a] += 1;
        v_degree[b] += 1;
    }

    const bool  miter = params.even_thickness != 0;
    const float k_max_miter = 5.0f;

    struct VertTriple { int32_t top, corner, out; };
    std::vector<VertTriple> v_triples(VN, {-1, -1, -1});

    for (size_t i = 0; i < VN; ++i) 
    {
        if (v_degree[i] == 0) continue;
        const Vec3& sum = avg_out[i];
        float sumlen = std::sqrt(dot(sum, sum));
        if (sumlen <= 0.0f) continue;
        Vec3 O = sum * (1.0f / sumlen);

        float scale = 1.0f;
        if (miter) 
        {
            scale = static_cast<float>(v_degree[i]) / sumlen;
            if (scale > k_max_miter) scale = k_max_miter;
            if (!std::isfinite(scale) || scale < 1.0f) scale = 1.0f;
        }

        const Vec3& A = src_verts[i];
        Vec3 vt = A + up * wall_h + O * (bias * scale);
        Vec3 vc = A + O * (bias * scale) + up * bias;
        Vec3 vo = A + O * (floor_w * scale) + up * bias;

        v_triples[i].top = static_cast<int32_t>(out.vertices.size());
        out.vertices.push_back(vt);
        v_triples[i].corner = static_cast<int32_t>(out.vertices.size());
        out.vertices.push_back(vc);
        v_triples[i].out = static_cast<int32_t>(out.vertices.size());
        out.vertices.push_back(vo);
    }

    const float v_corner_frac = wall_h / std::max(wall_h + floor_w, 1e-6f);

    out.uvs.reserve(EN * 4 * 3 * 2); // 4 tris * 3 loops * 2 floats per edge

    auto push_uv = [&](float u, float v)
    {
        out.uvs.push_back(u);
        out.uvs.push_back(v);
    };

    for (size_t i = 0; i < EN; ++i) 
    {
        if (!edge_valid[i]) continue;
        int32_t a = edges[i].first;
        int32_t b = edges[i].second;
        if (a < 0 || b < 0 || static_cast<size_t>(a) >= VN || static_cast<size_t>(b) >= VN) continue;
        const VertTriple& At = v_triples[a];
        const VertTriple& Bt = v_triples[b];
        if (At.top < 0 || Bt.top < 0) continue;

        const Vec3& Oedge = edge_outward[i]; // for winding sanity check

        // Wall strip
        Tri w1 = {At.top, Bt.top, Bt.corner};
        Tri w2 = {At.top, Bt.corner, At.corner};
        {
            const Vec3& p0 = out.vertices[w1[0]];
            const Vec3& p1 = out.vertices[w1[1]];
            const Vec3& p2 = out.vertices[w1[2]];
            Vec3 n = cross(p1 - p0, p2 - p0);
            if (dot(n, Oedge) < 0.0f) 
            {
                std::swap(w1[1], w1[2]);
                std::swap(w2[1], w2[2]);
            }
        }

        // Floor strip
        Tri f1 = {At.corner, Bt.corner, Bt.out};
        Tri f2 = {At.corner, Bt.out, At.out};
        {
            const Vec3& p0 = out.vertices[f1[0]];
            const Vec3& p1 = out.vertices[f1[1]];
            const Vec3& p2 = out.vertices[f1[2]];
            Vec3 n = cross(p1 - p0, p2 - p0);
            if (dot(n, up) < 0.0f) {
                std::swap(f1[1], f1[2]);
                std::swap(f2[1], f2[2]);
            }
        }

        const float uA = 0.0f;
        const float uB = length(src_verts[b] - src_verts[a]);
        struct UVRow { float u; float v; };
        auto uv_for_idx = [&](int32_t idx) -> UVRow {
            if      (idx == At.top)    return {uA, 0.0f};
            else if (idx == At.corner) return {uA, v_corner_frac};
            else if (idx == At.out)    return {uA, 1.0f};
            else if (idx == Bt.top)    return {uB, 0.0f};
            else if (idx == Bt.corner) return {uB, v_corner_frac};
            else                       return {uB, 1.0f}; // == Bt.out
        };

        const Tri to_emit[4] = {w1, w2, f1, f2};
        for (const Tri& tr : to_emit) 
        {
            out.triangles.push_back(tr);
            for (int k = 0; k < 3; ++k) 
            {
                UVRow uv = uv_for_idx(tr[k]);
                push_uv(uv.u, uv.v);
            }
        }
    }

    return !out.triangles.empty();
}

} // namespace mt
