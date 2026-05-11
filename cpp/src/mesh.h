#pragma once

#include <array>
#include <cstdint>
#include <vector>

namespace mt {

struct Vec3 
{
    float x = 0, y = 0, z = 0;

    Vec3 operator+(const Vec3& o) const { return {x + o.x, y + o.y, z + o.z}; }
    Vec3 operator-(const Vec3& o) const { return {x - o.x, y - o.y, z - o.z}; }
    Vec3 operator*(float s) const { return {x * s, y * s, z * s}; }
};

inline Vec3 cross(const Vec3& a, const Vec3& b) 
{
    return {a.y * b.z - a.z * b.y,
            a.z * b.x - a.x * b.z,
            a.x * b.y - a.y * b.x};
}

inline float dot(const Vec3& a, const Vec3& b) 
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

using Tri = std::array<int32_t, 3>;

struct Mesh 
{
    std::vector<Vec3> vertices;
    std::vector<Tri> triangles;
    std::vector<float> uvs;

    void clear() { vertices.clear(); triangles.clear(); uvs.clear(); }
    bool empty() const { return vertices.empty() || triangles.empty(); }

    void bounds(Vec3& mn, Vec3& mx) const;
};

} // namespace mt

struct MTMesh 
{
    mt::Mesh data;
};
