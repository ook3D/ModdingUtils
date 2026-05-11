#include "../include/moddingtool.h"
#include "mesh.h"
#include "collision.h"
#include "remesh.h"
#include "decal.h"
#include "blend_decal.h"

#include <cstring>
#include <new>
#include <vector>
#include <utility>

extern "C" {

MT_API const char* mt_version(void) 
{
    return "moddingtool 0.1.0";
}

MT_API MTMesh* mt_mesh_create(void) 
{
    return new (std::nothrow) MTMesh();
}

MT_API void mt_mesh_destroy(MTMesh* mesh) 
{
    delete mesh;
}

MT_API MTStatus mt_mesh_set_vertices(MTMesh* mesh, const float* vertices, int32_t vertex_count)
{
    if (!mesh) return MT_ERR_NULL;
    if (vertex_count < 0) return MT_ERR_INVALID;
    if (vertex_count > 0 && !vertices) return MT_ERR_NULL;

    try 
    {
        mesh->data.vertices.resize(static_cast<size_t>(vertex_count));
    } catch (const std::bad_alloc&) 
    {
        return MT_ERR_OOM;
    }
    if (vertex_count > 0) 
    {
        std::memcpy(mesh->data.vertices.data(), vertices, sizeof(float) * 3u * static_cast<size_t>(vertex_count));
    }
    return MT_OK;
}

MT_API MTStatus mt_mesh_set_triangles(MTMesh* mesh, const int32_t* triangles, int32_t triangle_count) 
{
    if (!mesh) return MT_ERR_NULL;
    if (triangle_count < 0) return MT_ERR_INVALID;
    if (triangle_count > 0 && !triangles) return MT_ERR_NULL;

    try
    {
        mesh->data.triangles.resize(static_cast<size_t>(triangle_count));
    } catch (const std::bad_alloc&) 
    {
        return MT_ERR_OOM;
    }
    if (triangle_count > 0) 
    {
        std::memcpy(mesh->data.triangles.data(), triangles, sizeof(int32_t) * 3u * static_cast<size_t>(triangle_count));
    }
    return MT_OK;
}

MT_API int32_t mt_mesh_vertex_count(const MTMesh* mesh)
{
    return mesh ? static_cast<int32_t>(mesh->data.vertices.size()) : 0;
}

MT_API int32_t mt_mesh_triangle_count(const MTMesh* mesh) 
{
    return mesh ? static_cast<int32_t>(mesh->data.triangles.size()) : 0;
}

MT_API MTStatus mt_mesh_get_vertices(const MTMesh* mesh, float* out_vertices)
{
    if (!mesh || !out_vertices) return MT_ERR_NULL;
    if (mesh->data.vertices.empty()) return MT_OK;
    std::memcpy(out_vertices, mesh->data.vertices.data(), sizeof(float) * 3u * mesh->data.vertices.size());
    return MT_OK;
}

MT_API MTStatus mt_mesh_get_triangles(const MTMesh* mesh, int32_t* out_triangles) 
{
    if (!mesh || !out_triangles) return MT_ERR_NULL;
    if (mesh->data.triangles.empty()) return MT_OK;
    std::memcpy(out_triangles, mesh->data.triangles.data(), sizeof(int32_t) * 3u * mesh->data.triangles.size());
    return MT_OK;
}

MT_API int32_t mt_mesh_uv_count(const MTMesh* mesh) 
{
    if (!mesh) return 0;
    return static_cast<int32_t>(mesh->data.uvs.size() / 2u);
}

MT_API MTStatus mt_mesh_get_uvs(const MTMesh* mesh, float* out_uvs) 
{
    if (!mesh || !out_uvs) return MT_ERR_NULL;
    if (mesh->data.uvs.empty()) return MT_OK;
    std::memcpy(out_uvs, mesh->data.uvs.data(), sizeof(float) * mesh->data.uvs.size());
    return MT_OK;
}

MT_API MTCollisionParams mt_collision_params_default(void) 
{
    MTCollisionParams p{};
    p.voxel_size          = 0.0f;   // derive from relative
    p.relative_voxel_size = 0.05f;  // 5% of bbox diagonal
    p.pin_boundary        = 1;
    return p;
}

MT_API MTRemeshParams mt_remesh_params_default(void) 
{
    MTRemeshParams p{};
    p.target_edge_length   = 0.0f;   // derive from relative
    p.relative_edge_length = 0.05f;
    p.iterations           = 5;
    p.protect_boundary     = 1;
    return p;
}

MT_API MTStatus mt_remesh_isotropic(MTMesh* mesh, const MTRemeshParams* params) 
{
    if (!mesh) return MT_ERR_NULL;
    if (mesh->data.empty()) return MT_ERR_EMPTY;

    MTRemeshParams p = params ? *params : mt_remesh_params_default();
    if (!mt::remesh_isotropic(mesh->data, p)) return MT_ERR_INVALID;
    return MT_OK;
}

MT_API MTDecalParams mt_decal_params_default(void) 
{
    MTDecalParams p{};
    p.wall_height    = 0.5f;
    p.floor_extent   = 0.3f;
    p.bias           = 0.001f;
    p.up_x           = 0.0f;
    p.up_y           = 0.0f;
    p.up_z           = 1.0f;
    p.even_thickness = 1;
    return p;
}

MT_API MTMesh* mt_generate_glue_decal(const float* vertices, int32_t vertex_count,
                                      const int32_t* edges_ab, int32_t edge_count,
                                      const float* wall_normals,
                                      const MTDecalParams* params,
                                      MTStatus* out_status) 
{
    auto set_status = [&](MTStatus s) { if (out_status) *out_status = s; };

    if (!vertices || !edges_ab || !wall_normals) { set_status(MT_ERR_NULL); return nullptr; }
    if (vertex_count <= 0 || edge_count <= 0)    { set_status(MT_ERR_EMPTY); return nullptr; }

    std::vector<mt::Vec3> verts(static_cast<size_t>(vertex_count));
    for (int32_t i = 0; i < vertex_count; ++i) 
    {
        verts[static_cast<size_t>(i)] = {
            vertices[i * 3 + 0],
            vertices[i * 3 + 1],
            vertices[i * 3 + 2],
        };
    }

    std::vector<std::pair<int32_t, int32_t>> es(static_cast<size_t>(edge_count));
    std::vector<mt::Vec3> wns(static_cast<size_t>(edge_count));
    for (int32_t i = 0; i < edge_count; ++i)
    {
        es[static_cast<size_t>(i)] = {
            edges_ab[i * 2 + 0],
            edges_ab[i * 2 + 1],
        };
        wns[static_cast<size_t>(i)] = {
            wall_normals[i * 3 + 0],
            wall_normals[i * 3 + 1],
            wall_normals[i * 3 + 2],
        };
    }

    MTDecalParams p = params ? *params : mt_decal_params_default();

    MTMesh* result = new (std::nothrow) MTMesh();
    if (!result) { set_status(MT_ERR_OOM); return nullptr; }

    if (!mt::generate_glue_decal(verts, es, wns, p, result->data)) 
    {
        delete result;
        set_status(MT_ERR_INVALID);
        return nullptr;
    }
    set_status(MT_OK);
    return result;
}

MT_API MTBlendDecalParams mt_blend_decal_params_default(void) 
{
    MTBlendDecalParams p{};
    p.width          = 0.5f;
    p.center_offset  = 0.0f;
    p.bias           = 0.001f;
    p.up_x           = 0.0f;
    p.up_y           = 0.0f;
    p.up_z           = 1.0f;
    p.even_thickness = 1;
    return p;
}

MT_API MTMesh* mt_generate_blend_decal(const float* vertices, int32_t vertex_count,
                                       const int32_t* edges_ab, int32_t edge_count,
                                       const float* edge_across,
                                       const MTBlendDecalParams* params,
                                       MTStatus* out_status) 
{
    auto set_status = [&](MTStatus s) { if (out_status) *out_status = s; };

    if (!vertices || !edges_ab || !edge_across) { set_status(MT_ERR_NULL); return nullptr; }
    if (vertex_count <= 0 || edge_count <= 0)   { set_status(MT_ERR_EMPTY); return nullptr; }

    std::vector<mt::Vec3> verts(static_cast<size_t>(vertex_count));
    for (int32_t i = 0; i < vertex_count; ++i) 
    {
        verts[static_cast<size_t>(i)] = {
            vertices[i * 3 + 0],
            vertices[i * 3 + 1],
            vertices[i * 3 + 2],
        };
    }

    std::vector<std::pair<int32_t, int32_t>> es(static_cast<size_t>(edge_count));
    std::vector<mt::Vec3> ax(static_cast<size_t>(edge_count));
    for (int32_t i = 0; i < edge_count; ++i) 
    {
        es[static_cast<size_t>(i)] = {
            edges_ab[i * 2 + 0],
            edges_ab[i * 2 + 1],
        };
        ax[static_cast<size_t>(i)] = {
            edge_across[i * 3 + 0],
            edge_across[i * 3 + 1],
            edge_across[i * 3 + 2],
        };
    }

    MTBlendDecalParams p = params ? *params : mt_blend_decal_params_default();

    MTMesh* result = new (std::nothrow) MTMesh();
    if (!result) { set_status(MT_ERR_OOM); return nullptr; }

    if (!mt::generate_blend_decal(verts, es, ax, p, result->data)) 
    {
        delete result;
        set_status(MT_ERR_INVALID);
        return nullptr;
    }
    set_status(MT_OK);
    return result;
}

MT_API MTMesh* mt_generate_collision(const MTMesh* input, const MTCollisionParams* params, MTStatus* out_status) 
{
    auto set_status = [&](MTStatus s) { if (out_status) *out_status = s; };

    if (!input) { set_status(MT_ERR_NULL); return nullptr; }
    if (input->data.empty()) { set_status(MT_ERR_EMPTY); return nullptr; }

    MTCollisionParams p = params ? *params : mt_collision_params_default();

    MTMesh* result = new (std::nothrow) MTMesh();
    if (!result) { set_status(MT_ERR_OOM); return nullptr; }

    if (!mt::generate_collision(input->data, p, result->data)) 
    {
        delete result;
        set_status(MT_ERR_INVALID);
        return nullptr;
    }
    set_status(MT_OK);
    return result;
}

} // extern "C"
