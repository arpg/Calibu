/* 
   This file is part of the Calibu Project.
   https://robotics.gwu.edu/git/calibu

   Copyright (C) 2013 George Washington University,
                      Steven Lovegrove,

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

   http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
 */

#pragma once

#include <type_traits>

#include <ceres/ceres.h>
#include "CeresExtra.h"

namespace ceres
{

// Sum group of variadic compile time values.
template <int... PS>
struct SumParams;

template<int P>
struct SumParams<P>{
    static const int value = P;
};

template <int P, int... PS>
struct SumParams<P,PS...>
{
    static const int value = P + SumParams<PS...>::value;
};

// Sum first N variadic compile time values
template<int... ALL>
struct SumParamsN;

template<int N, int P, int... PS>
struct SumParamsN<N,P,PS...>
{
    static const int value = P + std::conditional<N==1, SumParams<0>, SumParamsN<N-1,PS...> >::value; 
};

// Take Nth variadic compile time value
template<int... ALL>
struct ParamsN;

template<int N, int P, int... PS>
struct ParamsN<N,P,PS...>
{
    static const int value = std::conditional<N==0, SumParams<P>, ParamsN<N-1,PS...> >::value; 
};

template <typename JetT, typename T, unsigned int Start, unsigned int... Blocks>
struct Take1stOrderPart;

template <typename JetT, typename T, unsigned int Start>
struct Take1stOrderPart<JetT,T,Start>
{
    // Base case - do nothing
    static inline void go(const int, const JetT*, T** ){}
};


template <typename JetT, typename T, unsigned int Start, unsigned int FirstBlock, unsigned int... Blocks>
struct Take1stOrderPart<JetT,T,Start,FirstBlock,Blocks...>
{
    static inline void go(const int M, const JetT *src, T** dst)
    {
        if(dst[0]) {
            internal::Take1stOrderPart<JetT,T,Start,FirstBlock>(M, src, dst[0]);
        }
        Take1stOrderPart<JetT,T,Start+FirstBlock,Blocks...>::go(M,src,dst+1);
    }
};


template <typename CostBase, typename Derived, unsigned int NumResiduals, unsigned int... Blocks>
class AutoDiffArrayCostFunction :
        public CostBase
{ 
public:    
    
    static const unsigned int num_residuals = NumResiduals;
    static const unsigned int num_blocks = sizeof...(Blocks);
    static const unsigned int num_params = SumParams<Blocks...>::value;
    const unsigned int num_params_in_block[num_blocks] = {Blocks...};
    
    typedef double T;
    typedef Jet<T, num_params> JetT;
    
    AutoDiffArrayCostFunction()
    {
        // Set up residuals / params in base class
        CostBase::set_num_residuals(num_residuals);
        
        for(unsigned int p=0; p < num_blocks; p++) {
            CostBase::mutable_parameter_block_sizes()->push_back( num_params_in_block[p] );            
        }
    }
    
    virtual ~AutoDiffArrayCostFunction() {}
    
    inline const Derived& derived() const {
        return *static_cast<const Derived*>(this);
    }
    
    virtual bool Evaluate(T const* const* parameters,
                          T* residuals,
                          T** jacobians) const
    {
        if (!jacobians) {
            return derived().template Evaluate<T>(parameters, residuals);
        }else{
            JetT jet_residuals[num_residuals];
            
            JetT x[num_params];
            int jet_start[num_blocks];
            JetT* jet_parameters[num_blocks];
            
            int jetb = 0;
            for(unsigned int b=0; b < num_blocks; ++b) {
                jet_start[b] = jetb;
                jet_parameters[b] = x + jet_start[b];
                ceres::internal::Make1stOrderPerturbation<JetT,double>(jetb, num_params_in_block[b], parameters[b], jet_parameters[b] );
                jetb += num_params_in_block[b];
            }
            
            if(!derived().template Evaluate<JetT>(jet_parameters, jet_residuals)) {
                return false;
            }
            
            internal::Take0thOrderPart(num_residuals, jet_residuals, residuals);
            
            Take1stOrderPart<JetT, T, 0, Blocks...>::go(num_residuals, jet_residuals, jacobians);
            
            return true;
        }
    }
};

}
