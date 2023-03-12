#pragma once
class VHACDCallbacks
{
public:
    virtual void ProgressUpdate(Stages stage,
        double stageProgress,
        const char* operation) = 0;
    virtual bool IsCanceled() const = 0;

    virtual ~VHACDCallbacks() = default;
};
