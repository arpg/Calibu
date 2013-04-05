#pragma once

#include <ceres/ceres.h>

enum eSplineCostType
{
    eSplineCostTypeNone = 0,
    eSplineCostTypeIMU = 1,
    eSplineCostTypeVicon = 2,
    eSplineCostTypeCamera = 4,
    eSplineCostTypePenalty = 8,
    eSplineCostTypeAll = 0xffffffff
};

class CostFunctionAndParams
    : public ceres::CostFunction
{
public:     
    virtual ~CostFunctionAndParams()
    {
    }    
    
    std::vector<double*>& Params()
    {
        return m_params;
    }
    
    ceres::LossFunction*& Loss()
    {
        return m_loss_func;
    }    
    
    eSplineCostType& CostType()
    {
        return m_cost_type;
    }    
    
protected:
    std::vector<double*> m_params;
    ceres::LossFunction* m_loss_func;
    eSplineCostType m_cost_type;    
};

class TimedCost : public CostFunctionAndParams
{
public:
    TimedCost() 
        : m_time_u(0)
    {        
    }
    
    virtual ~TimedCost()
    {
    }
    
    inline double& Time_u() {
        return m_time_u;
    }
    
protected:
    double m_time_u;
};
