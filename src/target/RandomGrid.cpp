/*
   This file is part of the Calibu Project.
   https://github.com/gwu-robotics/Calibu

   Copyright (C) 2013 George Washington University,
                      Steven Lovegrove,
                      Gabe Sibley

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
#include <calibu/utils/StreamOperatorsEigen.h>

namespace calibu
{

void SaveEPS(
    std::string filename, const Eigen::MatrixXi& M,
    const Eigen::Vector2d& offset, double grid_spacing,
    double rad0, double rad1, double pts_per_unit,
    unsigned char id
)
{
    const double border = 3*rad1;
    const Eigen::Vector2d border2d(border,border);
    const Eigen::Vector2d max_pts(
                pts_per_unit * ((M.cols()-1) * grid_spacing + 2*border),
                pts_per_unit * ((M.rows()-1) * grid_spacing + 2*border)
                );

    std::ofstream f;
    f.open(filename.c_str());
    f << "%!PS-Adobe EPSF-3.0" << std::endl;
    f << "%%Creator: CalibuCalibrationTarget" << std::endl;
    f << "%%Title: Calibration Target" << std::endl;
    f << "%%Origin: 0 0" << std::endl;
    // usletter BoundingBox is 0, 0, 612, 792
    f << "%%BoundingBox: 0 0 " << max_pts[0] << " " << max_pts[1] << std::endl;
    f << std::endl;
    f << "270 rotate 0 " << -max_pts[0] << " 0 translate" << std::endl;

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

    double r = grid_spacing*( M.rows()+2.5 );
    double dx =  (grid_spacing*(M.cols()-1))/8;
    double hw = (pts_per_unit*dx)/2;
    Eigen::Vector2d base( dx/2.0, 0 );
    for( int c = 0; c < 8; c++ ){
        if( id & 1<<c ){
            const Eigen::Vector2d p =
                pts_per_unit*( offset + base + border2d + Eigen::Vector2d(dx*c,r));
            f   << "newpath\n"
                <<  p[0]-hw <<" " <<  p[1]-hw << " moveto\n"
                <<  p[0]+hw <<" " <<  p[1]-hw << " lineto\n"
                <<  p[0]+hw <<" " <<  p[1]+2*hw << " lineto\n"
                <<  p[0]-hw <<" " <<  p[1]+2*hw << " lineto\n"
                <<  "closepath\n"
                <<  " 0.0 setgray fill\n";
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

std::array<Eigen::MatrixXi, 4> MakePatternGroup(int r, int c, uint32_t seed)
{
  return FillGroup(MakePattern(r, c, seed));
}

std::array<Eigen::MatrixXi, 4> FillGroup(const Eigen::MatrixXi& m)
{
  std::array<Eigen::MatrixXi, 4> patterns;
  patterns[0] = m;

  // Found in this awesome answer http://stackoverflow.com/a/3488737/505049
  patterns[1] = m.transpose().colwise().reverse().eval();  // Rotate 90 CW
  patterns[2] = m.reverse().eval();  // Rotate 180. Not in the S.O. post
  patterns[3] = m.transpose().rowwise().reverse().eval();  // Rotate 270 CW
  return patterns;
}

int HammingDistance(const Eigen::MatrixXi& M, const Eigen::MatrixXi& m, int r, int c)
{
    int diff = 0;
    for(int mr=0; mr<m.rows(); ++mr) {
        for(int mc=0; mc<m.cols(); ++mc) {
            const int vm = m(mr,mc);
            if(vm >=0) {
                const int Mr = mr+r;
                const int Mc = mc+c;
                if(vm >=0 && 0 <= Mr && Mr < M.rows() && 0 <= Mc && Mc < M.cols() ) {
                    diff += abs( M(Mr,Mc) - vm);
                }else{
                    diff += 1;
                }
            }
        }
    }
    return diff;
}


int NumExactMatches(const Eigen::MatrixXi& M, const Eigen::MatrixXi& m, int& best_score, int& best_r, int& best_c)
{
    const int border = std::min((int)std::min(m.rows(),m.cols())-2, 2);
    best_score = std::numeric_limits<int>::max();
    const Eigen::Vector2i rcmax( 2*border + M.rows() - m.rows(), 2*border + M.cols() - m.cols());
    int num_zeros = 0;
    for(int r=-border; r < rcmax(0); ++r ) {
        for(int c=-border; c < rcmax(1); ++c) {
            const int hd = HammingDistance(M, m, r,c );
            if(hd < best_score) {
                best_score = hd;
                best_r = r;
                best_c = c;
            }
            if(hd==0) {
                ++num_zeros;
            }
        }
    }

    return num_zeros;
}

int NumExactMatches(const std::array<Eigen::MatrixXi,4>& PG, const Eigen::MatrixXi& m, int& best_score, int& best_g, int& best_r, int& best_c)
{
    best_score = std::numeric_limits<int>::max();
    int num_exact = 0;

    for(int g=0; g<4; ++g) {
        int pgr, pgc, pgs;
        num_exact += NumExactMatches(PG[g], m, pgs,pgr,pgc);
        if(pgs < best_score) {
            best_score = pgs;
            best_g = g;
            best_r = pgr;
            best_c = pgc;
        }
    }

    return num_exact;
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
                    int bs,bg,br,bc;
                    num_bad_matches += NumExactMatches(PG, m, bs,bg,br,bc) - 1;
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
                    int bs,bg,br,bc;
                    if(NumExactMatches(PG, m, bs,bg,br,bc) > 1) {
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
            std::cout << "*Seed " << seed << ": score:" << score << std::endl;
        }else{
            std::cout << " Seed " << seed << std::endl;
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
        std::cout << std::endl;
    }
}

}
