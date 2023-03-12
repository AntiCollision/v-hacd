#pragma once

class VHACDAsyncImpl : public VHACD::IVHACD,
    public VHACD::IVHACD::IUserCallback,
    VHACD::IVHACD::IUserLogger,
    VHACD::IVHACD::IUserTaskRunner
{
public:
    VHACDAsyncImpl() = default;

    ~VHACDAsyncImpl() override;

    void Cancel() override final;

    bool Compute(const float* const points,
        const uint32_t countPoints,
        const uint32_t* const triangles,
        const uint32_t countTriangles,
        const Parameters& params) override final;

    bool Compute(const double* const points,
        const uint32_t countPoints,
        const uint32_t* const triangles,
        const uint32_t countTriangles,
        const Parameters& params) override final;

    bool GetConvexHull(const uint32_t index,
        VHACD::IVHACD::ConvexHull& ch) const override final;

    uint32_t GetNConvexHulls() const override final;

    void Clean() override final; // release internally allocated memory

    void Release() override final; // release IVHACD

    // Will compute the center of mass of the convex hull decomposition results and return it
    // in 'centerOfMass'.  Returns false if the center of mass could not be computed.
    bool ComputeCenterOfMass(double centerOfMass[3]) const override;

    bool IsReady() const override final;

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
    uint32_t findNearestConvexHull(const double pos[3],
        double& distanceToHull) override final;

    void Update(const double overallProgress,
        const double stageProgress,
        const char* const stage,
        const char* operation) override final;

    void Log(const char* const msg) override final;

    void* StartTask(std::function<void()> func) override;

    void JoinTask(void* Task) override;

    bool Compute(const Parameters params);

    bool ComputeNow(const std::vector<VHACD::Vertex>& points,
        const std::vector<VHACD::Triangle>& triangles,
        const Parameters& _desc);

    // As a convenience for the calling application we only send it update and log messages from it's own main
    // thread.  This reduces the complexity burden on the caller by making sure it only has to deal with log
    // messages in it's main application thread.
    void ProcessPendingMessages() const;

private:
    VHACD::VHACDImpl                m_VHACD;
    std::vector<VHACD::Vertex>      m_vertices;
    std::vector<VHACD::Triangle>    m_indices;
    VHACD::IVHACD::IUserCallback* m_callback{ nullptr };
    VHACD::IVHACD::IUserLogger* m_logger{ nullptr };
    VHACD::IVHACD::IUserTaskRunner* m_taskRunner{ nullptr };
    void* m_task{ nullptr };
    std::atomic<bool>               m_running{ false };
    std::atomic<bool>               m_cancel{ false };

    // Thread safe caching mechanism for messages and update status.
    // This is so that caller always gets messages in his own thread
    // Member variables are marked as 'mutable' since the message dispatch function
    // is called from const query methods.
    mutable std::mutex              m_messageMutex;
    mutable std::vector<LogMessage> m_messages;
    mutable std::atomic<bool>       m_haveMessages{ false };
};
