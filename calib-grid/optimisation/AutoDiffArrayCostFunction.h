#pragma once

#include <type_traits>

#include <ceres/ceres.h>
#include "EigenCeresJetNumTraits.h"

namespace ceres
{

// Sum group of variadic compile time values.
template <int... PS> struct SumParams;
template<int P> struct SumParams<P>{
    static const int sum = P;
};
template <int P, int... PS> struct SumParams<P,PS...> {
    static const int sum = P + SumParams<PS...>::sum;
};

// Sum first N variadic compile time values
template<int N, int P, int... PS> struct SumParamsN{
    static const int sum = P + std::conditional<N==1, SumParams<0>, SumParamsN<N-1,PS...> >::sum; 
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
    static const unsigned int num_params = SumParams<Blocks...>::sum;
    const unsigned int block_params[num_blocks] = {Blocks...};

    typedef double T;
    typedef Jet<T, num_params> JetT;
    
    AutoDiffArrayCostFunction()
    {
        // Set up residuals / params in base class
        CostBase::set_num_residuals(num_residuals);
        
        for(unsigned int p=0; p < num_blocks; p++) {
            CostBase::mutable_parameter_block_sizes()->push_back( block_params[p] );            
        }
    }
    
    virtual ~AutoDiffArrayCostFunction() {}

    inline const Derived& derived() const {
        return *static_cast<const Derived*>(this);
    }

    virtual bool Evaluate(T const* const* parameters,
                        T* residuals,
                        T** jacobians) const {
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
            ceres::internal::Make1stOrderPerturbation<JetT,double>(jetb, block_params[b], parameters[b], jet_parameters[b] );
            jetb += block_params[b];
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
