/*
   This file is part of the Calibu Project.
   https://github.com/arpg/Calibu

   Copyright (C) 2013 University of Colorado Boulder
                      Steven Lovegrove,
                      Dorian Galvez-Lopez

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

#include <calibu/target/TargetViconGridDot.h>

#include <cmath>
#include <fstream>
#include <vector>
#include <opencv2/opencv.hpp>

namespace calibu {

TargetViconGridDot::TargetViconGridDot(TargetGridDot grid, ViconLayout layout)
  : TargetGridDot(grid)
{
  const float W = static_cast<float>(grid_size(0));
  const float H = static_cast<float>(grid_size(1));
  const float H2 = floor(H / 2.f);

  // Note: the 3 first points are the ones that define the coordinate frame
  // of the vixon markers
  // {col, row}
  switch(layout)
  {
  case EShapeAlignBottom:
    m_markers = { {-1, -1}, {W, -1},   {-1, H2}, {-1, H-1}, {W, H-1} };
    break;
  default: // EShapeAlignRight
    m_markers = { {-1, -1}, {W-1, -1}, {-1, H2}, {-1, H},   {W-1, H} };
    break;
  }

  for(const auto& marker : m_markers)
    tpts2d.emplace_back(Eigen::Vector2d(grid_spacing * marker.x,
                                        grid_spacing * marker.y));

  // Vicon frame ("m" means marker):
  // X = vector(m[0] to m[1])
  // Z = X cross_product vector(m[0] to m[2])
  // Y = Z cross_product X
}

bool TargetViconGridDot::FindViconTarget(
    const std::vector<Conic, Eigen::aligned_allocator<Conic> >& grid_conics,
    const std::vector<int>& grid_conics_map,
    const std::vector<Conic, Eigen::aligned_allocator<Conic> >& new_conics,
    std::vector<int>& vicon_map)
{
  vicon_map.clear();
  vicon_map.resize(new_conics.size(), -1);

  const unsigned int index_offset = grid_conics.size();
  for( size_t i=0; i < new_conics.size(); ++i )
    vs.emplace_back(Vertex(index_offset + i, new_conics[i]));

  if(new_conics.empty() || grid_conics.size() < 4 ||
     grid_conics.size() != grid_conics_map.size())
    return false;

  // estimate target grid rectangle
  std::vector<cv::Point2f> im_dots, target_dots;
  for(size_t i = 0; i < grid_conics_map.size(); ++i) {
    if(grid_conics_map[i] != -1) {
      const Eigen::Vector2d& c = grid_conics[i].center;
      const int row = grid_conics_map[i] / grid_size(0);
      const int col = grid_conics_map[i] % grid_size(0);
      target_dots.push_back(cv::Point2f(col, row));
      im_dots.push_back(cv::Point2f(c(0), c(1)));
    }
  }

  cv::Mat H = cv::findHomography(target_dots, im_dots, cv::noArray());
  if(H.empty()) return false;

  // get position of vicon markers in the image
  std::vector<cv::Point2f> im_markers;
  cv::perspectiveTransform(m_markers, im_markers, H);

  // associate conics
  const double MAX_SQ_D = 10*10;
  std::vector<std::pair<int, double>> associations;
  // [marker idx] -> <conic idx, sqd>
  associations.reserve(im_markers.size());

  for(size_t i = 0; i < im_markers.size(); ++i) {
    int best_conic = -1;
    double lowest_sq_dist = MAX_SQ_D;

    for(size_t j = 0; j < new_conics.size(); ++j) {
      const Conic& c = new_conics[j];
      double sq_dist =
          (c.center(0) - im_markers[i].x) * (c.center(0) - im_markers[i].x) +
          (c.center(1) - im_markers[i].y) * (c.center(1) - im_markers[i].y);
      if(sq_dist < lowest_sq_dist) {
        lowest_sq_dist = sq_dist;
        best_conic = j;
      }
    }

    if(best_conic != -1) {
      // check aliasing
      auto it = std::find_if(associations.begin(), associations.end(),
                             [best_conic](const std::pair<int, double>& p)
      {
        return (p.first == best_conic);
      });

      if(it != associations.end()) {
        if(lowest_sq_dist < it->second)
          it->first = -1;
        else
          best_conic = -1;
      }
    }
    associations.push_back(std::make_pair(best_conic, lowest_sq_dist));
  }

  // copy indices
  for(size_t i = 0; i < associations.size(); ++i) {
    if(associations[i].first != -1) {
      vicon_map[associations[i].first] = i;

      Vertex& v = vs[index_offset + associations[i].first];
      v.value = 1;
      v.pg = Eigen::Vector2i(m_markers[associations[i].first].x,
          m_markers[associations[i].first].y);
    }
  }

  /*
  // ### debug
  cv::Mat im = cv::imread("test.png", 1);
  for(size_t i = 0; i < grid_conics.size(); ++i) {
    cv::Scalar color = vicon_map[i] != -1 ? cv::Scalar(200, 200, 200) :
                                            cv::Scalar(0,   0, 255);
    cv::circle(im, cv::Point2f(grid_conics[i].center(0),
                               grid_conics[i].center(1)),
               5, color);
  }
  for(cv::Point2f& m : im_markers) {
    cv::circle(im, m, 7, cv::Scalar(255, 0, 0));
  }
  for(size_t i = 0; i < new_conics.size(); ++i) {
    cv::Scalar color = vicon_map[i] != -1 ? cv::Scalar(0, 255, 0) :
                                            cv::Scalar(0,   0, 255);
    cv::circle(im, cv::Point2f(new_conics[i].center(0),
                               new_conics[i].center(1)),
               3, color);
  }
  cv::imwrite("test.png", im);
  // ###
  */

  // consider it found only if all the markers are found
  return std::all_of(associations.begin(), associations.end(),
                     [](const std::pair<int, double>& assoc)
  {
    return assoc.first != -1;
  });
}

void TargetViconGridDot::SaveSVG(std::string filename, double rad0,
                                 double rad1) const
{
  const double R = std::max(rad0, rad1);
  const double rad = 8.5; // mm, vicon dot rad
  const double bg_rad = (grid_spacing - R) * 0.9 * 1e3; // mm, background rad
  const bool draw_crosses = true;

  double left, right, top, bottom;
  left = right = top = bottom = 0;

  if (!m_markers.empty()) {
    float minr, maxr, minc, maxc;
    minr = maxr = m_markers[0].y;
    minc = maxc = m_markers[0].x;
    for(const cv::Point2f& p : m_markers) {
      minr = std::min(minr, p.y);
      maxr = std::max(maxr, p.y);
      minc = std::min(minc, p.x);
      maxc = std::max(maxc, p.x);
    }
    const int gaps = ( draw_crosses ? 1 : 0 );
    left = (-minc - gaps) * grid_spacing + bg_rad / 1e3 - R;
    right = (maxc - grid_size(0) + 1) * grid_spacing + bg_rad / 1e3 - R;
    top = (-minr - gaps) * grid_spacing + bg_rad / 1e3 - R;
    bottom = (maxr - grid_size(1) + 1) * grid_spacing + bg_rad / 1e3 - R;
  }

  std::ofstream f(filename);
  SaveSVGHead(f);
  SaveSVGBody(f, rad0, rad1, draw_crosses, left, top, right, bottom);

  for(const cv::Point2f& p : m_markers) {
    double cx = (left + R + p.x * grid_spacing) * 1e3;
    double cy = (top  + R + p.y * grid_spacing) * 1e3;

    if(draw_crosses) {
      cx += grid_spacing * 1e3;
      cy += grid_spacing * 1e3;
    }

    // bg
    const double x = cx - bg_rad;
    const double y = cy - bg_rad;
    const double len = bg_rad * 2;
    f << "<rect x=\"" << x << "mm\" y=\"" << y << "mm\" fill=\"black\""
      << " width=\"" << len << "mm\" height=\"" << len << "mm\" />"
      << std::endl;

    // circle
    f << "<circle cx=\"" << cx << "mm\" cy=\"" << cy << "mm\" fill=\"white\""
      << " r=\"" << rad << "mm\" stroke=\"black\" stroke-width=\"1\" />"
      << std::endl;

    f << "<line x1=\"" << cx - rad << "mm\" x2=\""
      << cx + rad << "mm\" y1=\"" << cy << "mm\" y2=\""
      << cy << "mm\" stroke=\"black\" stroke-width=\"1\"/>" << std::endl
      << "<line x1=\"" << cx << "mm\" x2=\"" << cx << "mm\" y1=\""
      << cy - rad << "mm\" y2=\"" << cy + rad << "mm\" "
         "stroke=\"black\" stroke-width=\"1\"/>" << std::endl;
  }

  SaveSVGEnd(f);
}

} // namespace calibu
