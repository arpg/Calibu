/* This file is part of the calibu Project.
 * https://github.com/stevenlovegrove/calibu
 *
 * Copyright (C) 2013  Steven Lovegrove, 
 *                     George Washington University
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

#include <Eigen/Eigen>
#include <Sophus/se2.hpp>
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
