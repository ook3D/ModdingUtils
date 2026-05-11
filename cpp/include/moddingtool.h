#ifndef MODDINGTOOL_H
#define MODDINGTOOL_H

#include <stdint.h>

#if defined(_WIN32)
    #if defined(MT_BUILDING_DLL)
        #define MT_API __declspec(dllexport)
    #else
        #define MT_API __declspec(dllimport)
    #endif
#else
    #define MT_API __attribute__((visibility("default")))
#endif

#ifdef __cplusplus
extern "C" {
#endif

typedef struct MTMesh MTMesh;

typedef enum MTStatus {
    MT_OK = 0,
    MT_ERR_NULL = 1,
    MT_ERR_INVALID = 2,
    MT_ERR_EMPTY = 3,
    MT_ERR_OOM = 4
} MTStatus;

MT_API const char* mt_version(void);

MT_API MTMesh*  mt_mesh_create(void);
MT_API void     mt_mesh_destroy(MTMesh* mesh);

MT_API MTStatus mt_mesh_set_vertices(MTMesh* mesh, const float* vertices, int32_t vertex_count);
MT_API MTStatus mt_mesh_set_triangles(MTMesh* mesh, const int32_t* triangles, int32_t triangle_count);

MT_API int32_t  mt_mesh_vertex_count(const MTMesh* mesh);
MT_API int32_t  mt_mesh_triangle_count(const MTMesh* mesh);
MT_API MTStatus mt_mesh_get_vertices(const MTMesh* mesh, float* out_vertices);
MT_API MTStatus mt_mesh_get_triangles(const MTMesh* mesh, int32_t* out_triangles);

MT_API int32_t  mt_mesh_uv_count(const MTMesh* mesh);
MT_API MTStatus mt_mesh_get_uvs(const MTMesh* mesh, float* out_uvs);

typedef struct MTCollisionParams {
    float voxel_size;
    float relative_voxel_size;
    int32_t pin_boundary;
} MTCollisionParams;

MT_API MTCollisionParams mt_collision_params_default(void);

MT_API MTMesh* mt_generate_collision(const MTMesh* input, const MTCollisionParams* params, MTStatus* out_status);

typedef struct MTRemeshParams {
    float target_edge_length;
    float relative_edge_length;
    int32_t iterations;
    int32_t protect_boundary;
} MTRemeshParams;

MT_API MTRemeshParams mt_remesh_params_default(void);

MT_API MTStatus mt_remesh_isotropic(MTMesh* mesh, const MTRemeshParams* params);

typedef struct MTDecalParams {
    float wall_height;
    float floor_extent;
    float bias;
    float up_x, up_y, up_z;
    int32_t even_thickness;
} MTDecalParams;

MT_API MTDecalParams mt_decal_params_default(void);

MT_API MTMesh* mt_generate_glue_decal(const float* vertices, int32_t vertex_count,
                                      const int32_t* edges_ab, int32_t edge_count,
                                      const float* wall_normals,
                                      const MTDecalParams* params,
                                      MTStatus* out_status);

typedef struct MTBlendDecalParams {
    float width;
    float center_offset;
    float bias;
    float up_x, up_y, up_z;
    int32_t even_thickness;
} MTBlendDecalParams;

MT_API MTBlendDecalParams mt_blend_decal_params_default(void);

MT_API MTMesh* mt_generate_blend_decal(const float* vertices, int32_t vertex_count,
                                       const int32_t* edges_ab, int32_t edge_count,
                                       const float* edge_across,
                                       const MTBlendDecalParams* params,
                                       MTStatus* out_status);

#ifdef __cplusplus
}
#endif

#endif /* MODDINGTOOL_H */
