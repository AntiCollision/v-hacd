#pragma once
class LogMessage
{
public:
    double  m_overallProgress{ double(-1.0) };
    double  m_stageProgress{ double(-1.0) };
    std::string m_stage;
    std::string m_operation;
};