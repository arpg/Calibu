/* 
   This file is part of the Calibu Project.
   https://robotics.gwu.edu/git/calibu

   Copyright (C) 2013 George Washington University,
                      Steven Lovegrove

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

#include <Eigen/Eigen>
#include <sophus/se2.hpp>
#include <random>
#include <iostream>
#include <array>
#include <signal.h>
#include <fstream>

namespace calibu
{

void SaveEPS(
    std::string filename, const Eigen::MatrixXi& M,
    const Eigen::Vector2d& offset, double grid_spacing,
    double rad0, double rad1, double pts_per_unit
);

Eigen::MatrixXi MakePattern(int r, int c, uint32_t seed = 0);

std::array<Eigen::MatrixXi,4> MakePatternGroup(int r, int c, uint32_t seed);

int HammingDistance(const Eigen::MatrixXi& M, const Eigen::MatrixXi& m, int r, int c);

int NumExactMatches(const Eigen::MatrixXi& M, const Eigen::MatrixXi& m, int& best_score, int& best_r, int& best_c);

int NumExactMatches(const std::array<Eigen::MatrixXi,4>& PG, const Eigen::MatrixXi& m, int& best_score, int& best_g, int& best_r, int& best_c);

int AutoCorrelation(const std::array<Eigen::MatrixXi,4>& PG, int minr=2, int minc=2 );

int AutoCorrelationMinArea(const std::array<Eigen::MatrixXi,4>& PG );

int SeedScore(uint32_t seed, int r, int c);

uint32_t FindBestSeed(int r, int c, bool& should_run);

void PrintPattern(const Eigen::MatrixXi& M);

}
