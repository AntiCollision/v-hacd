#pragma once
#include <stdint.h>
#include <vector>
#include <functional>
#include "Vertex.h"
#include "Triangle.h"
#include "Vector3.h"
#include "FillMode.h"

namespace VHACD {
    typedef VHACD::Vector3<double> Vect3;

    class IVHACD
    {
    public:
        /**
        * This optional pure virtual interface is used to notify the caller of the progress
        * of convex decomposition as well as a signal when it is complete when running in
        * a background thread
        */
        class IUserCallback
        {
        public:
            virtual ~IUserCallback() {};

            /**
            * Notifies the application of the current state of the convex decomposition operation
            *
            * @param overallProgress : Total progress from 0-100%
            * @param stageProgress : Progress of the current stage 0-100%
            * @param stage : A text description of the current stage we are in
            * @param operation : A text description of what operation is currently being performed.
            */
            virtual void Update(const double overallProgress,
                const double stageProgress,
                const char* const stage,
                const char* operation) = 0;

            // This is an optional user callback which is only called when running V-HACD asynchronously.
            // This is a callback performed to notify the user that the
            // convex decomposition background process is completed. This call back will occur from
            // a different thread so the user should take that into account.
            virtual void NotifyVHACDComplete()
            {
            }
        };

        /**
        * Optional user provided pure virtual interface to be notified of warning or informational messages
        */
        class IUserLogger
        {
        public:
            virtual ~IUserLogger() {};
            virtual void Log(const char* const msg) = 0;
        };

        /**
        * An optional user provided pure virtual interface to perform a background task.
        * This was added by Danny Couture at Epic as they wanted to use their own
        * threading system instead of the standard library version which is the default.
        */
        class IUserTaskRunner
        {
        public:
            virtual ~IUserTaskRunner() {};
            virtual void* StartTask(std::function<void()> func) = 0;
            virtual void JoinTask(void* Task) = 0;
        };

        /**
        * A simple class that represents a convex hull as a triangle mesh with
        * double precision vertices. Polygons are not currently provided.
        */
        class ConvexHull
        {
        public:
            std::vector<VHACD::Vertex>      m_points;
            std::vector<VHACD::Triangle>    m_triangles;

            double                          m_volume{ 0 };          // The volume of the convex hull
            VHACD::Vect3                    m_center{ 0, 0, 0 };    // The centroid of the convex hull
            uint32_t                        m_meshId{ 0 };          // A unique id for this convex hull
            VHACD::Vect3            mBmin;                  // Bounding box minimum of the AABB
            VHACD::Vect3            mBmax;                  // Bounding box maximum of the AABB
        };

        /**
        * This class provides the parameters controlling the convex decomposition operation
        */
        class Parameters
        {
        public:
            IUserCallback* m_callback{ nullptr };            // Optional user provided callback interface for progress
            IUserLogger* m_logger{ nullptr };              // Optional user provided callback interface for log messages
            IUserTaskRunner* m_taskRunner{ nullptr };          // Optional user provided interface for creating tasks
            uint32_t            m_maxConvexHulls{ 64 };         // The maximum number of convex hulls to produce
            uint32_t            m_resolution{ 400000 };         // The voxel resolution to use
            double              m_minimumVolumePercentErrorAllowed{ 1 }; // if the voxels are within 1% of the volume of the hull, we consider this a close enough approximation
            uint32_t            m_maxRecursionDepth{ 10 };        // The maximum recursion depth
            bool                m_shrinkWrap{ true };             // Whether or not to shrinkwrap the voxel positions to the source mesh on output
            FillMode            m_fillMode{ FillMode::FLOOD_FILL }; // How to fill the interior of the voxelized mesh
            uint32_t            m_maxNumVerticesPerCH{ 64 };    // The maximum number of vertices allowed in any output convex hull
            bool                m_asyncACD{ true };             // Whether or not to run asynchronously, taking advantage of additional cores
            uint32_t            m_minEdgeLength{ 2 };           // Once a voxel patch has an edge length of less than 4 on all 3 sides, we don't keep recursing
            bool                m_findBestPlane{ false };       // Whether or not to attempt to split planes along the best location. Experimental feature. False by default.
        };

        /**
        * Will cause the convex decomposition operation to be canceled early. No results will be produced but the background operation will end as soon as it can.
        */
        virtual void Cancel() = 0;

        /**
        * Compute a convex decomposition of a triangle mesh using float vertices and the provided user parameters.
        *
        * @param points : The vertices of the source mesh as floats in the form of X1,Y1,Z1,  X2,Y2,Z2,.. etc.
        * @param countPoints : The number of vertices in the source mesh.
        * @param triangles : The indices of triangles in the source mesh in the form of I1,I2,I3, ....
        * @param countTriangles : The number of triangles in the source mesh
        * @param params : The convex decomposition parameters to apply
        * @return : Returns true if the convex decomposition operation can be started
        */
        virtual bool Compute(const float* const points,
            const uint32_t countPoints,
            const uint32_t* const triangles,
            const uint32_t countTriangles,
            const Parameters& params) = 0;

        /**
        * Compute a convex decomposition of a triangle mesh using double vertices and the provided user parameters.
        *
        * @param points : The vertices of the source mesh as floats in the form of X1,Y1,Z1,  X2,Y2,Z2,.. etc.
        * @param countPoints : The number of vertices in the source mesh.
        * @param triangles : The indices of triangles in the source mesh in the form of I1,I2,I3, ....
        * @param countTriangles : The number of triangles in the source mesh
        * @param params : The convex decomposition parameters to apply
        * @return : Returns true if the convex decomposition operation can be started
        */
        virtual bool Compute(const double* const points,
            const uint32_t countPoints,
            const uint32_t* const triangles,
            const uint32_t countTriangles,
            const Parameters& params) = 0;

        /**
        * Returns the number of convex hulls that were produced.
        *
        * @return : Returns the number of convex hulls produced, or zero if it failed or was canceled
        */
        virtual uint32_t GetNConvexHulls() const = 0;

        /**
        * Retrieves one of the convex hulls in the solution set
        *
        * @param index : Which convex hull to retrieve
        * @param ch : The convex hull descriptor to return
        * @return : Returns true if the convex hull exists and could be retrieved
        */
        virtual bool GetConvexHull(const uint32_t index,
            ConvexHull& ch) const = 0;

        /**
        * Releases any memory allocated by the V-HACD class
        */
        virtual void Clean() = 0; // release internally allocated memory

        /**
        * Releases this instance of the V-HACD class
        */
        virtual void Release() = 0; // release IVHACD

        // Will compute the center of mass of the convex hull decomposition results and return it
        // in 'centerOfMass'.  Returns false if the center of mass could not be computed.
        virtual bool ComputeCenterOfMass(double centerOfMass[3]) const = 0;

        // In synchronous mode (non-multi-threaded) the state is always 'ready'
        // In asynchronous mode, this returns true if the background thread is not still actively computing
        // a new solution.  In an asynchronous config the 'IsReady' call will report any update or log
        // messages in the caller's current thread.
        virtual bool IsReady() const
        {
            return true;
        }

        /**
        * At the request of LegionFu : out_look@foxmail.com
        * This method will return which convex hull is closest to the source position.
        * You can use this method to figure out, for example, which vertices in the original
        * source mesh are best associated with which convex hull.
        *
        * @param pos : The input 3d position to test against
        *
        * @return : Returns which convex hull this position is closest to.
        */
        virtual uint32_t findNearestConvexHull(const double pos[3],
            double& distanceToHull) = 0;

    protected:
        virtual ~IVHACD()
        {
        }
    };
}
