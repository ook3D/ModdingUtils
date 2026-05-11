#include "collision.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace mt {

namespace {

struct CellKey 
{
    int32_t x, y, z;
    bool operator==(const CellKey& o) const noexcept 
    {
        return x == o.x && y == o.y && z == o.z;
    }
};

struct CellHash 
{
    size_t operator()(const CellKey& k) const noexcept 
    {
        size_t h = static_cast<uint32_t>(k.x) * 73856093u;
        h ^= static_cast<uint32_t>(k.y) * 19349663u + 0x9e3779b9 + (h << 6) + (h >> 2);
        h ^= static_cast<uint32_t>(k.z) * 83492791u + 0x9e3779b9 + (h << 6) + (h >> 2);
        return h;
    }
};

struct EdgeKey 
{
    int32_t a, b; // a < b
    bool operator==(const EdgeKey& o) const noexcept 
    { 
        return a == o.a && b == o.b;
    }
};
struct EdgeHash 
{
    size_t operator()(const EdgeKey& k) const noexcept
    {
        return static_cast<uint64_t>(static_cast<uint32_t>(k.a)) * 0x9E3779B97F4A7C15ull ^ static_cast<uint64_t>(static_cast<uint32_t>(k.b));
    }
};

struct TriKey 
{
    int32_t a, b, c; // sorted ascending
    bool operator==(const TriKey& o) const noexcept 
    {
        return a == o.a && b == o.b && c == o.c;
    }
};
struct TriHash 
{
    size_t operator()(const TriKey& k) const noexcept 
    {
        size_t h = static_cast<uint32_t>(k.a) * 73856093u;
        h ^= static_cast<uint32_t>(k.b) * 19349663u + 0x9e3779b9 + (h << 6) + (h >> 2);
        h ^= static_cast<uint32_t>(k.c) * 83492791u + 0x9e3779b9 + (h << 6) + (h >> 2);
        return h;
    }
};

inline EdgeKey edge_key(int32_t a, int32_t b) 
{
    return a < b ? EdgeKey{a, b} : EdgeKey{b, a};
}

inline TriKey sorted_tri(int32_t a, int32_t b, int32_t c) 
{
    if (a > b) std::swap(a, b);
    if (b > c) std::swap(b, c);
    if (a > b) std::swap(a, b);
    return {a, b, c};
}

float resolve_voxel_size(const Mesh& input, const MTCollisionParams& params) 
{
    if (params.voxel_size > 0.0f) return params.voxel_size;
    Vec3 mn, mx;
    input.bounds(mn, mx);
    Vec3 d = mx - mn;
    float diag = std::sqrt(dot(d, d));
    float rel = params.relative_voxel_size > 0.0f ? params.relative_voxel_size : 0.05f;
    float v = diag * rel;
    if (!(v > 0.0f) || !std::isfinite(v)) v = 1.0f;
    return v;
}

} // namespace

bool generate_collision(const Mesh& input, const MTCollisionParams& params, Mesh& out) 
{
    out.clear();
    if (input.empty()) return false;

    const float vs = resolve_voxel_size(input, params);
    const float inv_vs = 1.0f / vs;
    const bool  pin_boundary = params.pin_boundary != 0;

    const size_t VN = input.vertices.size();
    const size_t TN = input.triangles.size();

    std::vector<uint8_t> is_boundary(VN, 0);
    if (pin_boundary) 
    {
        std::unordered_map<EdgeKey, int32_t, EdgeHash> edge_count;
        edge_count.reserve(TN * 3);
        for (const Tri& tri : input.triangles) 
        {
            if (tri[0] < 0 || tri[1] < 0 || tri[2] < 0) continue;
            if (static_cast<size_t>(tri[0]) >= VN || static_cast<size_t>(tri[1]) >= VN || static_cast<size_t>(tri[2]) >= VN) continue;
            for (int k = 0; k < 3; ++k) 
            {
                EdgeKey ek = edge_key(tri[k], tri[(k + 1) % 3]);
                edge_count[ek] += 1;
            }
        }
        for (const auto& kv : edge_count) 
        {
            if (kv.second == 1) 
            {
                is_boundary[static_cast<size_t>(kv.first.a)] = 1;
                is_boundary[static_cast<size_t>(kv.first.b)] = 1;
            }
        }
    }

    struct Cell 
    {
        Vec3    sum_all       {0, 0, 0};
        int32_t count_all     {0};
        Vec3    sum_boundary  {0, 0, 0};
        int32_t count_boundary{0};
    };

    std::unordered_map<CellKey, int32_t, CellHash> cell_to_idx;
    cell_to_idx.reserve(VN);
    std::vector<Cell> cells;
    cells.reserve(VN / 4 + 16);
    std::vector<int32_t> in_to_out(VN);

    for (size_t i = 0; i < VN; ++i) 
    {
        const Vec3& v = input.vertices[i];
        CellKey key{
            static_cast<int32_t>(std::floor(v.x * inv_vs)),
            static_cast<int32_t>(std::floor(v.y * inv_vs)),
            static_cast<int32_t>(std::floor(v.z * inv_vs)),
        };
        auto it = cell_to_idx.find(key);
        int32_t out_idx;
        if (it == cell_to_idx.end()) 
        {
            out_idx = static_cast<int32_t>(cells.size());
            cell_to_idx.emplace(key, out_idx);
            cells.emplace_back();
        } else 
        {
            out_idx = it->second;
        }
        Cell& c = cells[out_idx];
        c.sum_all = c.sum_all + v;
        c.count_all += 1;
        if (is_boundary[i]) 
        {
            c.sum_boundary = c.sum_boundary + v;
            c.count_boundary += 1;
        }
        in_to_out[i] = out_idx;
    }

    out.vertices.resize(cells.size());
    for (size_t i = 0; i < cells.size(); ++i) 
    {
        const Cell& c = cells[i];
        if (pin_boundary && c.count_boundary > 0) 
        {
            out.vertices[i] = c.sum_boundary * (1.0f / static_cast<float>(c.count_boundary));
        } else 
        {
            out.vertices[i] = c.sum_all * (1.0f / static_cast<float>(c.count_all));
        }
    }

    std::unordered_set<TriKey, TriHash> seen;
    seen.reserve(TN);
    out.triangles.reserve(TN);
    const int32_t vcount = static_cast<int32_t>(VN);
    for (const Tri& t : input.triangles) 
    {
        if (t[0] < 0 || t[1] < 0 || t[2] < 0) continue;
        if (t[0] >= vcount || t[1] >= vcount || t[2] >= vcount) continue;
        int32_t a = in_to_out[static_cast<size_t>(t[0])];
        int32_t b = in_to_out[static_cast<size_t>(t[1])];
        int32_t c = in_to_out[static_cast<size_t>(t[2])];
        if (a == b || b == c || a == c) continue;
        TriKey k = sorted_tri(a, b, c);
        if (!seen.insert(k).second) continue;
        out.triangles.push_back({a, b, c});
    }

    if (!out.triangles.empty()) 
    {
        std::vector<int32_t> remap(out.vertices.size(), -1);
        std::vector<Vec3> compacted;
        compacted.reserve(out.vertices.size());
        for (Tri& t : out.triangles) {
            for (int& idx : t) 
            {
                if (remap[idx] < 0) 
                {
                    remap[idx] = static_cast<int32_t>(compacted.size());
                    compacted.push_back(out.vertices[idx]);
                }
                idx = remap[idx];
            }
        }
        out.vertices = std::move(compacted);
    }

    return !out.empty();
}

} // namespace mt
