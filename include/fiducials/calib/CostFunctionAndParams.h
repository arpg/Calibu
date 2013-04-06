#pragma once

#include <ceres/ceres.h>

namespace fiducials
{

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
        
protected:
    std::vector<double*> m_params;
    ceres::LossFunction* m_loss_func;
};

}
