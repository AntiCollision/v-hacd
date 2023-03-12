#pragma once

// This class represents a collection of voxels, the convex hull
// which surrounds them, and a triangle mesh representation of those voxels
class VoxelHull
{
public:

    // This method constructs a new VoxelHull based on a plane split of the parent
    // convex hull
    VoxelHull(const VoxelHull& parent,
        SplitAxis axis,
        uint32_t splitLoc);

    // Here we construct the initial convex hull around the
    // entire voxel set
    VoxelHull(Volume& voxels,
        const IVHACD::Parameters& params,
        VHACDCallbacks* callbacks);

    ~VoxelHull() = default;

    // Helper method to refresh the min/max voxel bounding region
    void MinMaxVoxelRegion(const Voxel& v);

    void BuildRaycastMesh();

    // We now compute the convex hull relative to a triangle mesh generated 
    // from the voxels
    void ComputeConvexHull();

    // Returns true if this convex hull should be considered done
    bool IsComplete();


    // Convert a voxel position into it's correct double precision location
    VHACD::Vect3 GetPoint(const int32_t x,
        const int32_t y,
        const int32_t z,
        const double scale,
        const VHACD::Vect3& bmin) const;

    // Sees if we have already got an index for this voxel position.
    // If the voxel position has already been indexed, we just return
    // that index value.
    // If not, then we convert it into the floating point position and
    // add it to the index map
    uint32_t GetVertexIndex(const VHACD::Vector3<uint32_t>& p);

    // This method will convert the voxels into an actual indexed triangle mesh of boxes
    // This serves two purposes.
    // The primary purpose is so that when we compute a convex hull it considered all of the points
    // for each voxel, not just the center point. If you don't do this, then the hulls don't fit the
    // mesh accurately enough.
    // The second reason we convert it into a triangle mesh is so that we can do raycasting against it
    // to search for the best splitting plane fairly quickly. That algorithm will be discussed in the 
    // method which computes the best splitting plane.
    void BuildVoxelMesh();

    // Convert a single voxel position into an actual 3d box mesh comprised
    // of 12 triangles
    void AddVoxelBox(const Voxel& v);

    // Add the triangle represented by these 3 indices into the 'box' set of vertices
    // to the output mesh
    void AddTri(const std::array<VHACD::Vector3<uint32_t>, 8>& box,
        uint32_t i1,
        uint32_t i2,
        uint32_t i3);

    // Here we convert from voxel space to a 3d position, index it, and add
    // the triangle positions and indices for the output mesh
    void AddTriangle(const VHACD::Vector3<uint32_t>& p1,
        const VHACD::Vector3<uint32_t>& p2,
        const VHACD::Vector3<uint32_t>& p3);

    // When computing the split plane, we start by simply 
    // taking the midpoint of the longest side. However,
    // we can also search the surface and look for the greatest
    // spot of concavity and use that as the split location.
    // This will make the convex decomposition more efficient
    // as it will tend to cut across the greatest point of
    // concavity on the surface.
    SplitAxis ComputeSplitPlane(uint32_t& location);

    VHACD::Vect3 GetPosition(const VHACD::Vector3<int32_t>& ip) const;

    double Raycast(const VHACD::Vector3<int32_t>& p1,
        const VHACD::Vector3<int32_t>& p2) const;

    bool FindConcavity(uint32_t idx,
        uint32_t& splitLoc);

    // Finding the greatest area of concavity..
    bool FindConcavityX(uint32_t& splitLoc);

    // Finding the greatest area of concavity..
    bool FindConcavityY(uint32_t& splitLoc);

    // Finding the greatest area of concavity..
    bool FindConcavityZ(uint32_t& splitLoc);

    // This operation is performed in a background thread.
    // It splits the voxels by a plane
    void PerformPlaneSplit();

    // Used only for debugging. Saves the voxelized mesh to disk
    // Optionally saves the original source mesh as well for comparison
    void SaveVoxelMesh(const SimpleMesh& inputMesh,
        bool saveVoxelMesh,
        bool saveSourceMesh);

    void SaveOBJ(const char* fname,
        const VoxelHull* h);

    void SaveOBJ(const char* fname);

private:
    void WriteOBJ(FILE* fph,
        const std::vector<VHACD::Vertex>& vertices,
        const std::vector<VHACD::Triangle>& indices,
        uint32_t baseIndex);
public:

    SplitAxis               m_axis{ SplitAxis::X_AXIS_NEGATIVE };
    Volume* m_voxels{ nullptr }; // The voxelized data set
    double                  m_voxelScale{ 0 };   // Size of a single voxel
    double                  m_voxelScaleHalf{ 0 }; // 1/2 of the size of a single voxel
    VHACD::BoundsAABB       m_voxelBounds;
    VHACD::Vect3            m_voxelAdjust;       // Minimum coordinates of the voxel space, with adjustment
    uint32_t                m_depth{ 0 };        // How deep in the recursion of the binary tree this hull is
    uint32_t                m_index{ 0 };        // Each convex hull is given a unique id to distinguish it from the others
    double                  m_volumeError{ 0 };  // The percentage error from the convex hull volume vs. the voxel volume
    double                  m_voxelVolume{ 0 };  // The volume of the voxels
    double                  m_hullVolume{ 0 };   // The volume of the enclosing convex hull

    std::unique_ptr<IVHACD::ConvexHull> m_convexHull{ nullptr }; // The convex hull which encloses this set of voxels.
    std::vector<Voxel>                  m_surfaceVoxels;     // The voxels which are on the surface of the source mesh.
    std::vector<Voxel>                  m_newSurfaceVoxels;  // Voxels which are on the surface as a result of a plane split
    std::vector<Voxel>                  m_interiorVoxels;    // Voxels which are part of the interior of the hull

    std::unique_ptr<VoxelHull>          m_hullA{ nullptr }; // hull resulting from one side of the plane split
    std::unique_ptr<VoxelHull>          m_hullB{ nullptr }; // hull resulting from the other side of the plane split

    // Defines the coordinates this convex hull comprises within the voxel volume
    // of the entire source
    VHACD::Vector3<uint32_t>                    m_1{ 0 };
    VHACD::Vector3<uint32_t>                    m_2{ 0 };
    AABBTree                                    m_AABBTree;
    std::unordered_map<uint32_t, uint32_t>      m_voxelIndexMap; // Maps from a voxel coordinate space into a vertex index space
    std::vector<VHACD::Vertex>                  m_vertices;
    std::vector<VHACD::Triangle>                m_indices;
    static uint32_t                             m_voxelHullCount;
    IVHACD::Parameters                          m_params;
    VHACDCallbacks* m_callbacks{ nullptr };
};

uint32_t VoxelHull::m_voxelHullCount = 0;

