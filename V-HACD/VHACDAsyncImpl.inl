#pragma once

VHACDAsyncImpl::~VHACDAsyncImpl()
{
    Cancel();
}

void VHACDAsyncImpl::Cancel()
{
    m_cancel = true;
    m_VHACD.Cancel();

    if (m_task)
    {
        m_taskRunner->JoinTask(m_task); // Wait for the thread to fully exit before we delete the instance
        m_task = nullptr;
    }
    m_cancel = false; // clear the cancel semaphore
}

bool VHACDAsyncImpl::Compute(const float* const points,
    const uint32_t countPoints,
    const uint32_t* const triangles,
    const uint32_t countTriangles,
    const Parameters& params)
{
    m_vertices.reserve(countPoints);
    for (uint32_t i = 0; i < countPoints; ++i)
    {
        m_vertices.emplace_back(points[i * 3 + 0],
            points[i * 3 + 1],
            points[i * 3 + 2]);
    }

    m_indices.reserve(countTriangles);
    for (uint32_t i = 0; i < countTriangles; ++i)
    {
        m_indices.emplace_back(triangles[i * 3 + 0],
            triangles[i * 3 + 1],
            triangles[i * 3 + 2]);
    }

    return Compute(params);
}

bool VHACDAsyncImpl::Compute(const double* const points,
    const uint32_t countPoints,
    const uint32_t* const triangles,
    const uint32_t countTriangles,
    const Parameters& params)
{
    // We need to copy the input vertices and triangles into our own buffers so we can operate
    // on them safely from the background thread.
    // Can't be local variables due to being asynchronous
    m_vertices.reserve(countPoints);
    for (uint32_t i = 0; i < countPoints; ++i)
    {
        m_vertices.emplace_back(points[i * 3 + 0],
            points[i * 3 + 1],
            points[i * 3 + 2]);
    }

    m_indices.reserve(countTriangles);
    for (uint32_t i = 0; i < countTriangles; ++i)
    {
        m_indices.emplace_back(triangles[i * 3 + 0],
            triangles[i * 3 + 1],
            triangles[i * 3 + 2]);
    }

    return Compute(params);
}

bool VHACDAsyncImpl::GetConvexHull(const uint32_t index,
    VHACD::IVHACD::ConvexHull& ch) const
{
    return m_VHACD.GetConvexHull(index,
        ch);
}

uint32_t VHACDAsyncImpl::GetNConvexHulls() const
{
    ProcessPendingMessages();
    return m_VHACD.GetNConvexHulls();
}

void VHACDAsyncImpl::Clean()
{
    Cancel();
    m_VHACD.Clean();
}

void VHACDAsyncImpl::Release()
{
    delete this;
}

bool VHACDAsyncImpl::ComputeCenterOfMass(double centerOfMass[3]) const
{
    bool ret = false;

    centerOfMass[0] = 0;
    centerOfMass[1] = 0;
    centerOfMass[2] = 0;

    if (IsReady())
    {
        ret = m_VHACD.ComputeCenterOfMass(centerOfMass);
    }
    return ret;
}

bool VHACDAsyncImpl::IsReady() const
{
    ProcessPendingMessages();
    return !m_running;
}

uint32_t VHACDAsyncImpl::findNearestConvexHull(const double pos[3],
    double& distanceToHull)
{
    uint32_t ret = 0; // The default return code is zero

    distanceToHull = 0;
    // First, make sure that we have valid and completed results
    if (IsReady())
    {
        ret = m_VHACD.findNearestConvexHull(pos, distanceToHull);
    }

    return ret;
}

void VHACDAsyncImpl::Update(const double overallProgress,
    const double stageProgress,
    const char* const stage,
    const char* operation)
{
    m_messageMutex.lock();
    LogMessage m;
    m.m_operation = std::string(operation);
    m.m_overallProgress = overallProgress;
    m.m_stageProgress = stageProgress;
    m.m_stage = std::string(stage);
    m_messages.push_back(m);
    m_haveMessages = true;
    m_messageMutex.unlock();
}

void VHACDAsyncImpl::Log(const char* const msg)
{
    m_messageMutex.lock();
    LogMessage m;
    m.m_operation = std::string(msg);
    m_haveMessages = true;
    m_messages.push_back(m);
    m_messageMutex.unlock();
}

void* VHACDAsyncImpl::StartTask(std::function<void()> func)
{
    return new std::thread(func);
}

void VHACDAsyncImpl::JoinTask(void* Task)
{
    std::thread* t = static_cast<std::thread*>(Task);
    t->join();
    delete t;
}

bool VHACDAsyncImpl::Compute(Parameters params)
{
    Cancel(); // if we previously had a solution running; cancel it.

    m_taskRunner = params.m_taskRunner ? params.m_taskRunner : this;
    params.m_taskRunner = m_taskRunner;

    m_running = true;
    m_task = m_taskRunner->StartTask([this, params]() {
        ComputeNow(m_vertices,
        m_indices,
        params);
    // If we have a user provided callback and the user did *not* call 'cancel' we notify him that the
    // task is completed. However..if the user selected 'cancel' we do not send a completed notification event.
    if (params.m_callback && !m_cancel)
    {
        params.m_callback->NotifyVHACDComplete();
    }
    m_running = false;
        });
    return true;
}

bool VHACDAsyncImpl::ComputeNow(const std::vector<VHACD::Vertex>& points,
    const std::vector<VHACD::Triangle>& triangles,
    const Parameters& _desc)
{
    uint32_t ret = 0;

    Parameters desc;
    m_callback = _desc.m_callback;
    m_logger = _desc.m_logger;

    desc = _desc;
    // Set our intercepting callback interfaces if non-null
    desc.m_callback = _desc.m_callback ? this : nullptr;
    desc.m_logger = _desc.m_logger ? this : nullptr;

    // If not task runner provided, then use the default one
    if (desc.m_taskRunner == nullptr)
    {
        desc.m_taskRunner = this;
    }

    bool ok = m_VHACD.Compute(points,
        triangles,
        desc);
    if (ok)
    {
        ret = m_VHACD.GetNConvexHulls();
    }

    return ret ? true : false;
}

void VHACDAsyncImpl::ProcessPendingMessages() const
{
    if (m_cancel)
    {
        return;
    }
    if (m_haveMessages)
    {
        m_messageMutex.lock();
        for (auto& i : m_messages)
        {
            if (i.m_overallProgress == -1)
            {
                if (m_logger)
                {
                    m_logger->Log(i.m_operation.c_str());
                }
            }
            else if (m_callback)
            {
                m_callback->Update(i.m_overallProgress,
                    i.m_stageProgress,
                    i.m_stage.c_str(),
                    i.m_operation.c_str());
            }
        }
        m_messages.clear();
        m_haveMessages = false;
        m_messageMutex.unlock();
    }
}

IVHACD* CreateVHACD_ASYNC()
{
    VHACDAsyncImpl* m = new VHACDAsyncImpl;
    return static_cast<IVHACD*>(m);
}