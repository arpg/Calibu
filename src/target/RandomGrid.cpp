/* 
   This file is part of the Calibu Project.
   https://robotics.gwu.edu/git/calibu

   Copyright (C) 2013 George Washington University

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

#include <calibu/target/RandomGrid.h>

namespace calibu
{

void SaveEPS(
    std::string filename, const Eigen::MatrixXi& M,
    const Eigen::Vector2d& offset, double grid_spacing,
    double rad0, double rad1, double pts_per_unit
) {
    const double border = 3*rad1;
    const Eigen::Vector2d border2d(border,border);
    const Eigen::Vector2d max_pts(
                pts_per_unit * ((M.cols()-1) * grid_spacing + 2*border),
                pts_per_unit * ((M.rows()-1) * grid_spacing + 2*border)
                );
    
    std::ofstream f;
    f.open(filename.c_str());
    f << "%!PS-Adobe EPSF-3.0" << std::endl;
    f << "%%Creator: FiducialCalibrationTarget" << std::endl;
    f << "%%Title: Calibration Target" << std::endl;
    f << "%%Origin: 0 0" << std::endl;
    f << "%%BoundingBox: 0 0 " << max_pts[0] << " " << max_pts[1] << std::endl;
    f << std::endl;
    
    for( int r=0; r<M.rows(); ++r ) {
        for( int c=0; c<M.cols(); ++c) {
            const double rad_pts = pts_per_unit * ((M(r,c) == 1) ? rad1 : rad0);
            const Eigen::Vector2d p_pts = pts_per_unit* (offset + border2d + grid_spacing * Eigen::Vector2d(c,r));
            f << p_pts[0] << " " << max_pts[1] - p_pts[1] << " "
                   << rad_pts << " 0 360 arc closepath" << std::endl
                   << "0.0 setgray fill" << std::endl
                   << std::endl;            
        }
    }
    
    f.close();
}

Eigen::MatrixXi MakePattern(int r, int c, uint32_t seed )
{
    Eigen::MatrixXi M(r,c);
    
    std::mt19937 rng(seed);
    std::uniform_int_distribution<uint32_t> uint_dist1(0,1);
    
    for(int r=0; r < M.rows(); ++r) {
        for(int c=0; c < M.cols(); ++c) {
            M(r,c) = uint_dist1(rng);
        }
    }
    return M;
}

std::array<Eigen::MatrixXi,4> MakePatternGroup(int r, int c, uint32_t seed)
{
    std::array<Eigen::MatrixXi,4> patterns;
    const Eigen::MatrixXi& M = patterns[0];

    patterns[0] = MakePattern(r,c,seed);
    patterns[1] = Eigen::MatrixXi(M.cols(),M.rows());
    patterns[2] = Eigen::MatrixXi(M.rows(),M.cols());
    patterns[3] = Eigen::MatrixXi(M.cols(),M.rows());
    
    for(int r=0; r < M.rows(); ++r) {
        for(int c=0; c < M.cols(); ++c) {
            patterns[0](r,c) = M(r,c);
            patterns[1](M.cols()-c-1,r) = M(r,c);
            patterns[2](M.rows()-r-1, M.cols()-c-1) = M(r,c);
            patterns[3](c,M.rows()-r-1) = M(r,c);
        }
    }
    return patterns;
}

int HammingDistance(const Eigen::MatrixXi& M, const Eigen::MatrixXi& m, int r, int c)
{
    int sum = 0;
    for(int mr=0; mr<m.rows(); ++mr) {
        for(int mc=0; mc<m.cols(); ++mc) {
            const int vm = m(mr,mc);
            const int Mr = mr+r;
            const int Mc = mc+c;
            if(vm >=0 && 0 <= Mr && Mr < M.rows() && 0 <= Mc && Mc < M.cols() ) {
                sum += abs( M(Mr,Mc) - vm);
            }
        }
    } 
    return sum;
}


int NumExactMatches(const Eigen::MatrixXi& M, const Eigen::MatrixXi& m)
{
    const Eigen::Vector2i num( 1+M.rows() - m.rows(), 1+M.cols() - m.cols());
    
    int num_zeros = 0;
    
    for(int r=0; r < num(0); ++r ) {
        for(int c=0; c < num(1); ++c) {
            const Eigen::Vector2i t(r,c);
            const int corr = HammingDistance(M, m, r,c );
            if(corr==0)  ++num_zeros;
        }
    }
    
    return num_zeros;
}

int NumExactMatches(const std::array<Eigen::MatrixXi,4>& PG, const Eigen::MatrixXi& m)
{
    int num_zeroes = 0;
    
    for(int i=0; i<4; ++i) {
        num_zeroes += NumExactMatches(PG[i], m);
    }
    
    return num_zeroes;
}

int AutoCorrelation(const std::array<Eigen::MatrixXi,4>& PG, int minr, int minc )
{
    const Eigen::MatrixXi& M = PG[0];
    const int R = PG[0].rows();
    const int C = PG[0].cols();
    
    int num_bad_matches = 0;
    
    // For all sizes
    for(int nr = minr; nr < R; ++nr ) {
        for(int nc = minc; nc < C; ++nc ) {
            const int MR = R - nr-1;
            const int MC = C - nc-1;
            // For all offsets
            for(int r=0; r < MR; ++r) {
                for(int c=0; c < MC; ++c ) {
                    const Eigen::MatrixXi m = M.block(r,c,nr,nc);
                    // Don't count the known good match (-1)
                    num_bad_matches += NumExactMatches(PG, m) - 1;
                }
            }
        }
    }
    return num_bad_matches;
}

int AutoCorrelationMinArea(const std::array<Eigen::MatrixXi,4>& PG )
{
    const Eigen::MatrixXi& M = PG[0];
    const int R = PG[0].rows();
    const int C = PG[0].cols();

    int min_area = 0;
    
    // For all sizes
    for(int nr = 2; nr < R; ++nr ) {
        for(int nc = 2; nc < C; ++nc ) {
            const int MR = R - nr-1;
            const int MC = C - nc-1;
            // For all offsets
            for(int r=0; r < MR; ++r) {
                for(int c=0; c < MC; ++c ) {
                    const Eigen::MatrixXi m = M.block(r,c,nr,nc);
                    // Don't count the known good match (-1)
                    if(NumExactMatches(PG, m) > 1) {
                        min_area = std::max(min_area, nr*nc+1 );
                    }
                }
            }
        }
    }
    return min_area;
}


int SeedScore(uint32_t seed, int r, int c)
{
    return AutoCorrelationMinArea(MakePatternGroup(r,c,seed));
}

uint32_t FindBestSeed(int r, int c, bool& should_run) {
    
    uint32_t best_seed = 0;
    int best_score = std::numeric_limits<int>::max();
    
    for(uint32_t seed=0; should_run; ++seed) {
        const int score = SeedScore(seed,r,c);
        if(score < best_score) {
            best_seed = seed;
            best_score = score;
        }
    }
    return best_seed;
}

void PrintPattern(const Eigen::MatrixXi& M)
{
    for(int r=0; r< M.rows(); ++r) {
        for(int c=0; c<M.cols(); ++c) {
            const int v = M(r,c);
            const char b = (v == -1) ? 'x' : '0'+v;
            std::cout << b << ' ';
        }
        std::cout << '\n';
    }
}

}
