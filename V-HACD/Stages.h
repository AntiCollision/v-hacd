#pragma once

enum class Stages
{
    COMPUTE_BOUNDS_OF_INPUT_MESH,
    REINDEXING_INPUT_MESH,
    CREATE_RAYCAST_MESH,
    VOXELIZING_INPUT_MESH,
    BUILD_INITIAL_CONVEX_HULL,
    PERFORMING_DECOMPOSITION,
    INITIALIZING_CONVEX_HULLS_FOR_MERGING,
    COMPUTING_COST_MATRIX,
    MERGING_CONVEX_HULLS,
    FINALIZING_RESULTS,
    NUM_STAGES
};