#pragma once

#include "mesh.h"
#include "../include/moddingtool.h"

#include <cstdint>
#include <utility>
#include <vector>

namespace mt {

bool generate_glue_decal(const std::vector<Vec3>& vertices,
                         const std::vector<std::pair<int32_t, int32_t>>& edges,
                         const std::vector<Vec3>& wall_normals,
                         const MTDecalParams& params,
                         Mesh& out);

} // namespace mt
