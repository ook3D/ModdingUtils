#pragma once

#include "mesh.h"
#include "../include/moddingtool.h"

#include <cstdint>
#include <utility>
#include <vector>

namespace mt {

bool generate_blend_decal(const std::vector<Vec3>& vertices,
                          const std::vector<std::pair<int32_t, int32_t>>& edges,
                          const std::vector<Vec3>& edge_across,
                          const MTBlendDecalParams& params,
                          Mesh& out);

} // namespace mt
