#pragma once

#include "mesh.h"
#include "../include/moddingtool.h"

namespace mt {
    
bool generate_collision(const Mesh& input, const MTCollisionParams& params, Mesh& out);

} // namespace mt
