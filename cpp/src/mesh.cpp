#include "mesh.h"

#include <limits>

namespace mt {

void Mesh::bounds(Vec3& mn, Vec3& mx) const 
{
    if (vertices.empty()) 
    {
        mn = mx = Vec3{0, 0, 0};
        return;
    }
    constexpr float inf = std::numeric_limits<float>::infinity();
    mn = { inf,  inf,  inf};
    mx = {-inf, -inf, -inf};
    for (const Vec3& v : vertices) 
    {
        if (v.x < mn.x) mn.x = v.x;
        if (v.y < mn.y) mn.y = v.y;
        if (v.z < mn.z) mn.z = v.z;
        if (v.x > mx.x) mx.x = v.x;
        if (v.y > mx.y) mx.y = v.y;
        if (v.z > mx.z) mx.z = v.z;
    }
}

} // namespace mt
