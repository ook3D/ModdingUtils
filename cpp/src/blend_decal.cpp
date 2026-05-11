#include "blend_decal.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

namespace mt {

namespace 
{

    inline Vec3 normalize_or_zero(const Vec3& v) 
    {
        float L2 = dot(v, v);
        if (L2 <= 0.0f) return {0, 0, 0};
        return v * (1.0f / std::sqrt(L2));
    }

    inline float length(const Vec3& v) { return std::sqrt(dot(v, v)); }

} // namespace

bool generate_blend_decal(const std::vector<Vec3>& src_verts,
                          const std::vector<std::pair<int32_t, int32_t>>& edges,
                          const std::vector<Vec3>& edge_across,
                          const MTBlendDecalParams& params,
                          Mesh& out)
{
    out.clear();
    if (src_verts.empty() || edges.empty()) return false;
    if (edge_across.size() != edges.size()) return false;

    Vec3 up = normalize_or_zero(Vec3{params.up_x, params.up_y, params.up_z});
    if (dot(up, up) == 0.0f) up = {0.0f, 0.0f, 1.0f};

    const float width      = params.width;
    const float center_off = params.center_offset;
    const float bias       = params.bias;
    const float half_w     = 0.5f * width;
    const size_t VN        = src_verts.size();
    const size_t EN        = edges.size();

    std::vector<Vec3>    e_across(EN, {0, 0, 0});
    std::vector<uint8_t> e_valid(EN, 0);
    for (size_t i = 0; i < EN; ++i) 
    {
        Vec3 a = edge_across[i];
        Vec3 ah = a - up * dot(a, up);
        if (dot(ah, ah) < 1e-10f) continue;
        e_across[i] = normalize_or_zero(ah);
        e_valid[i]  = 1;
    }

    std::vector<Vec3>    avg_across(VN, {0, 0, 0});
    std::vector<int32_t> v_degree(VN, 0);
    for (size_t i = 0; i < EN; ++i)
    {
        if (!e_valid[i]) continue;
        int32_t a = edges[i].first;
        int32_t b = edges[i].second;
        if (a < 0 || b < 0 || static_cast<size_t>(a) >= VN || static_cast<size_t>(b) >= VN) continue;
        avg_across[a] = avg_across[a] + e_across[i];
        avg_across[b] = avg_across[b] + e_across[i];
        v_degree[a] += 1;
        v_degree[b] += 1;
    }

    const bool  miter = params.even_thickness != 0;
    const float k_max_miter = 5.0f;

    struct VertTriple { int32_t left, center, right; };
    std::vector<VertTriple> v_trips(VN, {-1, -1, -1});

    for (size_t i = 0; i < VN; ++i)
    {
        if (v_degree[i] == 0) continue;
        const Vec3& sum = avg_across[i];
        float sumlen = std::sqrt(dot(sum, sum));
        if (sumlen <= 0.0f) continue;
        Vec3 X = sum * (1.0f / sumlen);

        float scale = 1.0f;
        if (miter)
        {
            scale = static_cast<float>(v_degree[i]) / sumlen;
            if (scale > k_max_miter) scale = k_max_miter;
            if (!std::isfinite(scale) || scale < 1.0f) scale = 1.0f;
        }

        const Vec3& A = src_verts[i];
        const float center_x = center_off * scale;
        const float right_off = (center_off + half_w) * scale;
        const float left_off  = (center_off - half_w) * scale;
        Vec3 vR = A + X * right_off + up * bias;
        Vec3 vC = A + X * center_x  + up * bias;
        Vec3 vL = A + X * left_off  + up * bias;

        v_trips[i].right  = static_cast<int32_t>(out.vertices.size());
        out.vertices.push_back(vR);
        v_trips[i].center = static_cast<int32_t>(out.vertices.size());
        out.vertices.push_back(vC);
        v_trips[i].left   = static_cast<int32_t>(out.vertices.size());
        out.vertices.push_back(vL);
    }

    out.uvs.reserve(EN * 4 * 3 * 2); // 4 tris * 3 loops * 2 floats per edge

    auto push_uv = [&](float u, float v) 
    {
        out.uvs.push_back(u);
        out.uvs.push_back(v);
    };

    for (size_t i = 0; i < EN; ++i) 
    {
        if (!e_valid[i]) continue;
        int32_t a = edges[i].first;
        int32_t b = edges[i].second;
        if (a < 0 || b < 0 || static_cast<size_t>(a) >= VN || static_cast<size_t>(b) >= VN) continue;
        const VertTriple& At = v_trips[a];
        const VertTriple& Bt = v_trips[b];
        if (At.center < 0 || Bt.center < 0) continue;

        Tri r1 = {At.center, Bt.center, Bt.right};
        Tri r2 = {At.center, Bt.right,  At.right};
        Tri l1 = {At.left,   Bt.left,   Bt.center};
        Tri l2 = {At.left,   Bt.center, At.center};

        {
            const Vec3& p0 = out.vertices[r1[0]];
            const Vec3& p1 = out.vertices[r1[1]];
            const Vec3& p2 = out.vertices[r1[2]];
            Vec3 n = cross(p1 - p0, p2 - p0);
            if (dot(n, up) < 0.0f)
            {
                std::swap(r1[1], r1[2]);
                std::swap(r2[1], r2[2]);
                std::swap(l1[1], l1[2]);
                std::swap(l2[1], l2[2]);
            }
        }

        const float uA = 0.0f;
        const float uB = length(src_verts[b] - src_verts[a]);
        struct UVRow { float u; float v; };
        auto uv_for_idx = [&](int32_t idx) -> UVRow 
        {
            if      (idx == At.left)   return {uA, 0.0f};
            else if (idx == At.center) return {uA, 0.5f};
            else if (idx == At.right)  return {uA, 1.0f};
            else if (idx == Bt.left)   return {uB, 0.0f};
            else if (idx == Bt.center) return {uB, 0.5f};
            else                       return {uB, 1.0f};
        };

        const Tri to_emit[4] = {r1, r2, l1, l2};
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
