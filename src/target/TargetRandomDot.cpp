/* 
   This file is part of the Calibu Project.
   http://robotics.gwu.edu/git/?p=calibu

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

#include <calibu/target/TargetRandomDot.h>

#include <algorithm>
#include <iostream>
#include <fstream>

#include <sophus/se2.hpp>

#include <calibu/target/Hungarian.h>
#include <calibu/conics/Conic.h>
#include <calibu/pose/Ransac.h>
#include <calibu/utils/Utils.h>

using namespace std;
using namespace Eigen;

namespace calibu {

// Forwards
Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor>
DistanceMatrix(const std::vector<Eigen::Vector2d >& pts );

template<int R,int C,typename P>
void SortRows(Eigen::Matrix<P,R,C,Eigen::RowMajor>& M)
{
    for( int r=0; r < M.rows(); ++r )
        std::sort(&(M(r,0)), &(M(r,M.cols()-1))+1);
}

TargetRandomDot::TargetRandomDot()
    : seed(0), dt(NULL)
{
    srand(seed);
}

void TargetRandomDot::SetSeed(int s )
{
    seed = s;
    srand(seed);
}

Matrix<double,Dynamic,Dynamic,RowMajor> DistanceMatrix(const vector<Vector2d >& pts )
{
    Matrix<double,Dynamic,Dynamic,RowMajor> D(pts.size(),pts.size());
    for( unsigned int j=0; j < pts.size(); ++j )
        for( unsigned int i=0; i < pts.size(); ++i )
            D(j,i) = (pts[i] - pts[j]).norm();
    return D;
}

void TargetRandomDot::Clear()
{
    // remove existing
    if( dt ) delete dt;
    tpts.clear();
    tpts_reflected.clear();
    tpts3d.clear();
}

void TargetRandomDot::InitialiseFrom2DPts()
{
    for(vector<Vector2d >::const_iterator i = tpts.begin(); i != tpts.end(); ++i )
    {
        const Vector2d& p = *i;
        tpts3d.push_back(Vector3d(p[0],p[1],0));
        tpts_reflected.push_back(Vector2d(-p[0],p[1]));
    }
    
    // Construct Distance matrix
    dt = new Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor>(DistanceMatrix(tpts));
    SortRows(*dt);
    
    //  cout << "Target generated with " << tpts.size() << " circles." << endl;
}


void TargetRandomDot::LoadPattern( std::string filename, double radius, double scale )
{
    Clear();
    
    this->radius = radius * scale;
    
    size[0] = numeric_limits<double>::min();
    size[1] = numeric_limits<double>::min();
    
    ifstream f;
    f.open(filename.c_str());
    while( !f.eof() )
    {
        Vector2d p;
        double z;
        f >> p[0];
        f >> p[1];
        f >> z;
        p *= scale;
        size[0] = max(size[0],p[0]);
        size[1] = max(size[1],p[1]);
        tpts.push_back(p);
    }
    
    InitialiseFrom2DPts();
}

void TargetRandomDot::GenerateCircular(unsigned int max_circles, double radius, double min_distance, double border, const Vector2d& size)
{
    assert(size[0] == size[1]);
    
    Clear();
    
    const int MAX_FAILS = 1000;
    
    this->size = size;
    this->radius = radius;
    
    const double md2 = min_distance * min_distance;
    
    int fails = 0;
    
    // Generate circles
    
    const double big_circle_radius = size[0]/2 - border - radius;
    
    while( tpts.size() < max_circles && fails < MAX_FAILS )
    {
        const double x = ((double)rand() * 2 * big_circle_radius / (double)RAND_MAX) - big_circle_radius;
        const double y = (signbit((int)rand() - (int)RAND_MAX/2) ? 1 : -1) * sqrt(big_circle_radius*big_circle_radius - x*x);
        
        Vector2d p = Vector2d(
                    radius + border + big_circle_radius + x,
                    radius + border + big_circle_radius + y
                    );
        
        bool good = true;
        for( unsigned int i=0; i<tpts.size(); ++i )
        {
            if( (p - tpts[i]).squaredNorm() < md2 )
            {
                good = false;
                break;
            }
        }
        if( good )
        {
            tpts.push_back(p);
            fails = 0;
        }else{
            ++fails;
        }
    }
    
    InitialiseFrom2DPts();
}

void TargetRandomDot::GenerateEmptyCircle(unsigned int max_circles, double radius, double min_distance, double border, double clear_radius, const Vector2d& size)
{
    Clear();
    
    const int MAX_FAILS = 1000;
    
    this->size = size;
    this->radius = radius;
    
    const Vector2d center = size / 2;
    
    const double md2 = min_distance * min_distance;
    
    int fails = 0;
    
    // Generate circles
    
    while( tpts.size() < max_circles && fails < MAX_FAILS )
    {
        Vector2d p = Vector2d(
                (radius+border) + (double)rand() * ((double)size[0] - 2*(radius+border)) / (double)RAND_MAX,
                (radius+border) + (double)rand() * ((double)size[1] - 2*(radius+border)) / (double)RAND_MAX
                );
        
        bool good = ( p - center).norm() > clear_radius;
        
        for( unsigned int i=0; i<tpts.size(); ++i )
        {
            if( (p - tpts[i]).squaredNorm() < md2 )
            {
                good = false;
                break;
            }
        }
        if( good )
        {
            tpts.push_back(p);
            fails = 0;
        }else{
            ++fails;
        }
    }
    
    InitialiseFrom2DPts();
}

void TargetRandomDot::GenerateRandom(unsigned int max_circles, double radius, double min_distance, double border, const Vector2d& size)
{
    Clear();
    
    const int MAX_FAILS = 1000;
    
    this->size = size;
    this->radius = radius;
    
    const double md2 = min_distance * min_distance;
    
    int fails = 0;
    
    // Generate circles
    
    while( tpts.size() < max_circles && fails < MAX_FAILS )
    {
        Vector2d p = Vector2d(
                    (radius+border) + (double)rand() * ((double)size[0] - 2*(radius+border)) / (double)RAND_MAX,
                (radius+border) + (double)rand() * ((double)size[1] - 2*(radius+border)) / (double)RAND_MAX
                );
        bool good = true;
        for( unsigned int i=0; i<tpts.size(); ++i )
        {
            if( (p - tpts[i]).squaredNorm() < md2 )
            {
                good = false;
                break;
            }
        }
        if( good )
        {
            tpts.push_back(p);
            fails = 0;
        }else{
            ++fails;
        }
    }
    
    InitialiseFrom2DPts();
}

void TargetRandomDot::SaveEPS(string filename, float points_per_unit)
{
    const float ppu = points_per_unit;
    ofstream f;
    f.open(filename.c_str());
    f << "%!PS-Adobe EPSF-3.0" << endl;
    f << "%%Creator: CalibuCalibrationTarget" << endl;
    f << "%%Title: Calibration Target" << endl;
    f << "%%Origin: 0 0" << endl;
    f << "%%BoundingBox: 0 0 " << size[0]*ppu << " " << size[1]*ppu << endl;
    f << "% seed: " << seed << endl;
    f << "% radius: " << radius*ppu << endl;
    f << endl;
    
    for( unsigned int i=0; i<tpts.size(); ++i )
    {
        Vector2d& p = tpts[i];
        f << p[0]*ppu << " " << size[1]*ppu - p[1]*ppu << " " << radius*ppu << " 0 360 arc closepath" << endl
                                                       << "0.0 setgray fill" << endl
                                                       << endl;
    }
    
    f.close();
}

void TargetRandomDot::SaveRotatedEPS(string filename, float points_per_unit)
{
    const float ppu = points_per_unit;
    ofstream f;
    f.open(filename.c_str());
    f << "%!PS-Adobe EPSF-3.0" << endl;
    f << "%%Creator: CalibuCalibrationTarget" << endl;
    f << "%%Title: Calibration Target" << endl;
    f << "%%Origin: 0 0" << endl;
    f << "%%BoundingBox: 0 0 " << size[1]*ppu << " " << size[0]*ppu << endl;  
    f << "% seed: " << seed << endl;
    f << "% radius: " << radius << endl;
    f << endl;
    
    for( unsigned int i=0; i<tpts.size(); ++i )
    {
        Vector2d& p = tpts[i];
        f << p[1]*ppu << " " << p[0]*ppu << " " << radius*ppu << " 0 360 arc closepath" << endl
                      << "0.0 setgray fill" << endl
                      << endl;
    }
    
    f.close();
}

bool TargetRandomDot::LoadEPS( std::string filename, float points_per_unit )
{
    Clear();
    
    size[0] = numeric_limits<double>::min();
    size[1] = numeric_limits<double>::min();
    
    std::ifstream f;
    f.open(filename.c_str());
    
    if (f.is_open()) {
        while ( f.good() ) {
            std::string line;
            std::getline(f,line);
            if(line[0] == '%') {
                // comment
            }else if(line.size() > 2) {
                std::stringstream ss(line);
                float px, py, pr;
                ss >> px;
                ss >> py;
                ss >> pr;
                
                // Extract circle
                this->radius = pr / points_per_unit;
                const Vector2d p = Vector2d(px/points_per_unit, py/points_per_unit);
                size[0] = max(size[0],p[0]);
                size[1] = max(size[1],p[1]);
                tpts.push_back(p);
                
                // Consume colour line
                std::getline(f,line);
            }
        }
        f.close();
        
        InitialiseFrom2DPts();
        cout << "Loaded " << tpts.size() << " circles (radius: " << CircleRadius() << ")" << endl;    
        return true;
    }else{
        return false;
    }
}

struct RansacMatchData
{
    RansacMatchData(const vector<Vector2d >& m, const vector<Vector2d >& t, vector<int>& ml)
        :mpts(m),tpts(t),mpts_label(ml)
    {}
    
    const vector<Vector2d >& mpts;
    const vector<Vector2d >& tpts;
    vector<int>& mpts_label;
};

double RansacMatchCostFunction( const Sophus::SE2d& T, int i, RansacMatchData* data )
{
    const Vector2d p = T * data->mpts[i];
    return (data->tpts[data->mpts_label[i]] - p).norm();
}

Sophus::SE2d PoseFromCorrespondences(const vector<const Eigen::Vector2d *>& a, const vector<const Eigen::Vector2d *>& b)
{
    int n = a.size();
    double x1 = 0.0, x2 = 0.0, y1 = 0.0, y2 = 0.0, xx = 0.0, yy = 0.0, xy = 0.0, yx = 0.0;
    
    for (int i=0; i<n; ++i)
    {
        const Eigen::Vector2d & p1 = *a[i];
        const Eigen::Vector2d & p2 = *b[i];
        
        x1 += p1[0];
        x2 += p2[0];
        y1 += p1[1];
        y2 += p2[1];
        xx += p1[0]*p2[0];
        yy += p1[1]*p2[1];
        xy += p1[0]*p2[1];
        yx += p1[1]*p2[0];
    }
    
    double N = (double)n;
    
    double Sxx = xx - x1*x2/N; // calculate S
    double Syy = yy - y1*y2/N;
    double Sxy = xy - x1*y2/N;
    double Syx = yx - y1*x2/N;
    
    double xm1 = x1/N; // calculate means
    double xm2 = x2/N;
    double ym1 = y1/N;
    double ym2 = y2/N;
    
    double yaw = atan2(Sxy-Syx, Sxx+Syy);
    
    // calculate pose
    return Sophus::SE2d(
                Sophus::SO2d(yaw),
                Eigen::Vector2d(
                    xm2 - (xm1*cos(yaw) - ym1*sin(yaw)),
                    ym2 - (xm1*sin(yaw) + ym1*cos(yaw))
                    )
                );
}

Sophus::SE2d RansacMatchModelFunction( const std::vector<int>& indices, RansacMatchData* data )
{
    vector<const Vector2d* > mp;
    vector<const Vector2d* > tp;
    for( unsigned int k=0; k<indices.size(); ++k )
    {
        const int i = indices[k];
        mp.push_back( &data->mpts[i] );
        tp.push_back( &data->tpts[data->mpts_label[i]] );
    }
    return PoseFromCorrespondences(mp,tp);
}

Sophus::SE2d RansacMatchCorrespondences(
        const vector<Vector2d >& measurement,
        const vector<Vector2d >& target,
        vector<int>& measurement_label,
        int iterations,
        double max_point_fit_error,
        int min_consensus_size
        )
{
    RansacMatchData rmd(measurement,target,measurement_label);
    Ransac<Sophus::SE2d,2,RansacMatchData*> ransac( &RansacMatchModelFunction, &RansacMatchCostFunction, &rmd);
    
    vector<int> inliers;
    Sophus::SE2d T = ransac.Compute(
                measurement.size(),inliers,iterations,
                max_point_fit_error,min_consensus_size
                );
    
    // Remove outliers from map.
    for( unsigned int i=0; i<measurement.size(); ++i )
    {
        if( find(inliers.begin(),inliers.end(),i) == inliers.end() )
            measurement_label[i] = -1;
    }
    return T;
}

double DescriptorDist( const VectorXd& m, const VectorXd& t, int neighbours )
{
    const int dsize = std::min((int)m.rows(),neighbours);
    return ( m.head(dsize) - t.head(dsize) ).norm();
}

int ClosestPoint( const vector<Vector2d >& t, const Vector2d& p )
{
    double best_d = numeric_limits<double>::max();
    int best_i = -1;
    
    for( unsigned int i=0; i < t.size(); ++i )
    {
        const double d = (t[i] - p).norm();
        if( d < best_d )
        {
            best_i = i;
            best_d = d;
        }
    }
    
    return best_i;
}

void ClosestPoints( const vector<Vector2d >& a, const vector<Vector2d >& b, vector<int>& a_map)
{
    for( unsigned i=0; i<a.size(); ++i )
    {
        a_map[i] = ClosestPoint(b,a[i]);
    }
}

void MutualClosest( const vector<Vector2d >& a, const vector<Vector2d >& b, vector<int>& a_map)
{
    vector<int> b_map(b.size(),-1);
    ClosestPoints(b,a,b_map);
    
    for( unsigned i=0; i<a.size(); ++i )
    {
        const int ai_close = ClosestPoint(b,a[i]);
        const int ai_close_close = ai_close >= 0 ? b_map[ai_close] : -1;
        a_map[i] = ai_close_close == (int)i ? ai_close : -1;
    }
}

void TargetRandomDot::Match(
        const vector<Vector2d >& measurement,
        vector<int>& measurement_label, int match_neighbours
        ) const
{
    Matrix<double,Dynamic,Dynamic,RowMajor> dm = DistanceMatrix(measurement);
    SortRows(dm);
    Match(dm,measurement_label,match_neighbours);
}

void TargetRandomDot::Match(
        const Matrix<double,Dynamic,Dynamic,RowMajor>& sorted_measurement_distance_matrix,
        std::vector<int>& measurement_label, int match_neighbours
        ) const
{
    const Matrix<double,Dynamic,Dynamic,RowMajor>& dm = sorted_measurement_distance_matrix;
    const size_t msize = dm.rows();
    
    // Create cost matrix and padd with zeroes
    const size_t size = max(msize, tpts.size());
    Eigen::Matrix<double,Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> cost(size,size);
    cost.setZero();
    
    // Assign measurement to target association cost
    for( unsigned int j = 0; j < msize; ++j )
        for( unsigned int i = 0; i < tpts.size(); ++i )
            cost(j,i) = DescriptorDist( dm.row(j), (*dt).row(i), match_neighbours );
    
    double* c_cost[cost.rows()];
    for( int i=0; i<cost.rows(); ++i )
        c_cost[i] = &cost(i,0);
    
    hungarian_problem_t hp;
    hungarian_init(&hp,c_cost,cost.rows(),cost.cols() );
    hungarian_solve(&hp);
    
    for( unsigned int j=0; j<msize; ++j ) {
        const int label = hp.row_to_col_map[j];
        measurement_label[j] = label < (int)tpts.size() ? label : -1;
    }
    
    hungarian_free(&hp);
}

Vector2d Mean( vector<Vector2d >& pts )
{
    Vector2d sum = Vector2d::Zero();
    for( unsigned int i=0; i<pts.size(); ++i )
    {
        sum = sum + pts[i];
    }
    return sum / pts.size();
}


double RansacHomogCostFunction( const Matrix3d& H_tm, int i, RansacMatchData* data )
{
    const Vector2d& m = data->mpts[i];
    const Vector2d& t = data->tpts[data->mpts_label[i]];
    const Vector2d m_t = Project( (Vector3d)(H_tm * Unproject(m)) );
    return (m_t - t).squaredNorm();
}

Matrix3d RansacHomogModelFunction( const std::vector<int>& indices, RansacMatchData* data )
{
    std::vector<Vector2d > mpts;
    std::vector<Vector2d > tpts;
    
    for( unsigned int i=0; i< indices.size(); ++i )
    {
        const int mi = indices[i];
        const Vector2d& m = data->mpts[mi];
        const Vector2d& t = data->tpts[data->mpts_label[mi]];
        mpts.push_back(m);
        tpts.push_back(t);
    }
    
    return EstimateH_ba(mpts,tpts);
}

bool TargetRandomDot::FindTarget(
        const Sophus::SE3d& T_cw,
        const CameraModelInterface& cam,
        const ImageProcessing& images,
        const vector<Conic>& conics,
        vector<int>& conics_target_map
        )
{
    // We have conic centers, and projected centers. Try to match
    
    const IRectangle img_rect( 0,0,cam.Width(),cam.Height());
    
    vector<Vector2d > vis_t;
    vector<int> vis_t_map;
    for( unsigned int i=0; i < tpts3d.size(); ++i )
    {
        const Vector2d t = cam.ProjectMap(T_cw * tpts3d[i] );
        if( img_rect.Contains(t) )
        {
            vis_t.push_back(t);
            vis_t_map.push_back(i);
        }
    }
    
    vector<Vector2d > m;
    for( unsigned i=0; i<conics.size(); ++i )
        m.push_back(conics[i].center);
    
    vector<int> m_map(m.size(),-1);
    MutualClosest(m,vis_t,m_map);
    
    for(unsigned i=0; i<m.size(); ++i )
        conics_target_map[i] = m_map[i] >= 0 ? vis_t_map[m_map[i]] : -1;
    
    return true;
}

Vector3d nd_b(const Sophus::SE3d& T_ba, const Vector3d& n_a)
{
    const Vector3d n_b = T_ba.so3() * n_a;
    const double d_b = 1 - T_ba.translation().dot(n_b);
    return n_b / d_b;
}

Eigen::Vector3d IntersectCamFeaturePlane(
        const Eigen::Vector2d& p, 
        const CameraModelInterface & cam,
        const Sophus::SE3d& T_wk,
        const Eigen::Vector4d& N_w
        )
{
    const Vector3d nd_k = nd_b(T_wk.inverse(), Project(N_w));
    const Vector3d kinvp = cam.UnmapUnproject(p);
    const double denom = nd_k.dot(kinvp);
    if( denom !=0 ) {
        const Vector3d r_k = -kinvp / denom;
        const Vector3d r_w = T_wk * r_k;
        return r_w;
    }else{
        assert(false);
        return Vector3d();
    }
}

bool TargetRandomDot::FindTarget(
        const CameraModelInterface& cam,
        const ImageProcessing& images,
        const vector<Conic>& conics,
        vector<int>& conics_target_map
        )
{
    // Compute metric positions in 2D
    const vector<Eigen::Vector2d >& tpts = this->tpts;
    vector<Vector2d >  mpts;
    
    pair<Vector3d,Matrix3d > plane = PlaneFromConics(conics,radius,cam.K(), params.plane_inlier_thresh);
    const Vector4d N_w = Unproject(plane.first) / (plane.first).norm();
    
    if( !is_finite(N_w) )
        return false;
    
    for( unsigned int i=0; i< conics.size(); ++i )
    {
        const Vector2d ud = conics[i].center;
        const Vector3d p3d = IntersectCamFeaturePlane(ud,cam,Sophus::SE3d(),N_w);
        const Vector2d pnorm = (plane.second.transpose() * p3d).head<2>();
        mpts.push_back(pnorm);
    }
    
    if( mpts.size() >= 2 )
    {
        Matrix<double,Dynamic,Dynamic,RowMajor> ellipse_dm = DistanceMatrix(mpts);
        SortRows(ellipse_dm);
        
        // Match using Hungarian method
        Match(ellipse_dm,conics_target_map,params.match_neighbours);
        
        Sophus::SE2d T_tm;
        
        // Perform RANSAC to remove outliers
        if( params.ransac_its )
        {
            T_tm = RansacMatchCorrespondences(
                        mpts,tpts,conics_target_map,params.ransac_its,
                        params.ransac_max_inlier_err_mm, params.ransac_min_pts
                        );
        }
        
        // Given inliers and 2D transform, find more correspondences
        for( unsigned int i=0; i< mpts.size(); ++i )
        {
            // For each point without a match
            if( conics_target_map[i] < 0 )
            {
                // find closest point
                const Vector2d m_t = T_tm * mpts[i];
                const int t = ClosestPoint(tpts, m_t );
                if( t >= 0 ) {
                    assert( t >= 0 && t < (int)tpts.size() );
                    
                    const double d = (m_t - tpts[t]).norm();
                    
                    // check error is small
                    if( d < params.ransac_max_inlier_err_mm )
                    {
                        // check target circle hasn't already been matched
                        if( find(conics_target_map.begin(),conics_target_map.end(),t) == conics_target_map.end() )
                        {
                            conics_target_map[i] = t;
                        }
                    }
                }
            }
        }
        
    }
    
    return true;
}

bool TargetRandomDot::FindTarget(
        const ImageProcessing& images,
        const std::vector<Conic>& conics,
        std::vector<int>& ellipse_target_map
        )
{
    return false;
}


}
