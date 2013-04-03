/* This file is part of the fiducials Project.
 * https://github.com/stevenlovegrove/fiducials
 *
 * Copyright (c) 2011 Steven Lovegrove
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use,
 * copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following
 * conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 */

#pragma once

#include <algorithm>
#include <vector>
#include <limits>

namespace fiducials {

template<typename Model, int minimum_set_size, typename Data>
class Ransac
{
public:
  typedef double(*CostFunction)(const Model& model, int element_index, Data);
  typedef Model(*ModelFunction)(const std::vector<int>& element_indices, Data);

  Ransac(ModelFunction mf, CostFunction cf, Data data );

  Model Compute(unsigned int num_elements, std::vector<int>& inliers, int iterations, double max_datum_fit_error, unsigned int min_consensus_size );

protected:
  void SelectCandidates(std::vector<int>& set, int num_elements);

  CostFunction cf;
  ModelFunction mf;
  Data data;
};

template<typename Model, int minimum_set_size, typename Data>
inline Ransac<Model,minimum_set_size,Data>::Ransac(Ransac::ModelFunction mf, Ransac::CostFunction cf, Data data )
{
  this->mf = mf;
  this->cf = cf;
  this->data = data;
}

template<typename Model, int minimum_set_size, typename Data>
void Ransac<Model,minimum_set_size,Data>::SelectCandidates(std::vector<int>& set, int num_elements)
{
  while( set.size() < minimum_set_size )
  {
    const int i = rand() % num_elements;
    if( find(set.begin(),set.end(), i) == set.end() )
      set.push_back(i);
  }
}

template<typename Model, int minimum_set_size, typename Data>
inline Model Ransac<Model,minimum_set_size,Data>::Compute(unsigned int num_elements, std::vector<int>& inliers, int iterations, double max_datum_fit_error, unsigned int min_consensus_size )
{
  // http://en.wikipedia.org/wiki/RANSAC
  if( num_elements < minimum_set_size )
    return Model();

  Model best_model;
  std::vector<int> best_consensus_set;
  double best_error = std::numeric_limits<double>::max();

  for( int k=0; k < iterations; ++k )
  {
    std::vector<int> maybe_inliers;
    SelectCandidates(maybe_inliers, num_elements);
    Model maybe_model = (*mf)(maybe_inliers,data);
    std::vector<int> consensus_set(maybe_inliers);

    for( unsigned int i=0; i<num_elements; ++i )
    {
      if( find(maybe_inliers.begin(),maybe_inliers.end(),i) == maybe_inliers.end() )
      {
        const double err = (*cf)(maybe_model,i,data);
        if( err < max_datum_fit_error )
          consensus_set.push_back(i);
      }
    }

    if( consensus_set.size() >= min_consensus_size )
    {
      Model better_model = (*mf)(consensus_set,data);
      double this_error = 0;
      for( unsigned int i=0; i< consensus_set.size(); ++i )
      {
        double err = (*cf)(better_model,consensus_set[i],data);
        this_error += err*err;
      }
      this_error /= consensus_set.size();
      if( this_error < best_error )
      {
        best_model = better_model;
        best_consensus_set = consensus_set;
        best_error = this_error;
      }
    }
  }

  inliers = best_consensus_set;
  return best_model;
}

}
