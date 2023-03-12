#pragma once

namespace VHACD {

    /**
    * This enumeration determines how the voxels as filled to create a solid
    * object. The default should be 'FLOOD_FILL' which generally works fine
    * for closed meshes. However, if the mesh is not watertight, then using
    * RAYCAST_FILL may be preferable as it will determine if a voxel is part
    * of the interior of the source mesh by raycasting around it.
    *
    * Finally, there are some cases where you might actually want a convex
    * decomposition to treat the source mesh as being hollow. If that is the
    * case you can pass in 'SURFACE_ONLY' and then the convex decomposition
    * will converge only onto the 'skin' of the surface mesh.
    */
    enum class FillMode
    {
        FLOOD_FILL, // This is the default behavior, after the voxelization step it uses a flood fill to determine 'inside'
        // from 'outside'. However, meshes with holes can fail and create hollow results.
        SURFACE_ONLY, // Only consider the 'surface', will create 'skins' with hollow centers.
        RAYCAST_FILL, // Uses raycasting to determine inside from outside.
    };
}
