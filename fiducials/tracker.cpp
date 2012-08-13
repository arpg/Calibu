#include "tracker.h"

#include "adaptive_threshold.h"
#include "label.h"
#include "find_conics.h"
#include "pnp.h"

using namespace std;
//using namespace pangolin;
using namespace Eigen;

Tracker::Tracker(int w, int h)
    : w(w), h(w),
      intI(new float[w*h]), dI(new boost::array<float,2>[w*h]), lI(new short[w*h]), tI(new unsigned char[w*h]),
    last_good(0), good_frames(0)
{

}

bool Tracker::ProcessFrame(LinearCamera& cam, unsigned char* I)
{
    gradient<>(cam.width(), cam.height(), I, dI.get() );
    integral_image(cam.width(), cam.height(), I, intI.get() );
    return ProcessFrame(cam, I, dI.get(), intI.get());
}

bool Tracker::ProcessFrame(LinearCamera& cam, unsigned char* I, boost::array<float,2>* dI, float* intI)
{
    const int w = cam.width();
    const int h = cam.height();

    const float at_threshold = 1.0;
    const int at_window = w/3;
    const float conic_min_area = 25;
    const float conic_max_area = 4E4;
    const float conic_min_density = 0.4;
    const float conic_min_aspect = 0.1;
    const int target_match_neighbours = 10;
    const int target_ransac_its = 100;
    const int target_ransac_min_pts = 5;
    const float target_ransac_max_inlier_err_mm = 15;
    const float target_plane_inlier_thresh = 1.5;
    const double robust_4pt_inlier_tol = 0.006;
    const int robust_4pt_its = 200;

    const double max_rms = 1.2;
    const double max_mmps = 1500;

    double rms = 0;

//    static Var<float> at_threshold("ui.Adap Threshold",1.0,0,1.0);
//    static Var<int> at_window("ui.Adapt Window",w/3,1,200);
//    static Var<float> conic_min_area("ui.Conic min area",25, 0, 1E5);
//    static Var<float> conic_max_area("ui.Conic max area",4E4, 0, 1E5);
//    static Var<float> conic_min_density("ui.Conic min density",0.4, 0, 1.0);
//    static Var<float> conic_min_aspect("ui.Conic min aspect",0.1, 0, 1.0);
//    static Var<int> target_match_neighbours("ui.Match Descriptor Neighbours",10, 5, 20);
//    static Var<int> target_ransac_its("ui.Ransac Its", 100, 20, 500);
//    static Var<int> target_ransac_min_pts("ui.Ransac Min Pts", 5, 5, 10);
//    static Var<float> target_ransac_max_inlier_err_mm("ui.Ransac Max Inlier Err (mm)", 15, 0, 50);
//    static Var<float> target_plane_inlier_thresh("ui.Plane inlier thresh", 1.5, 0.1, 10);
//    static Var<double> rms("ui.RMS", 0);
//    static Var<double> max_rms("ui.max RMS", 1.0, 0.01, 10);
//    static Var<double> robust_4pt_inlier_tol("ui.Ransac 4Pt Inlier", 0.006, 1E-3, 1E-2);
//    static Var<int>    robust_4pt_its("ui.Ransac 4pt its", 200, 10, 500);
//    static Var<double> max_mmps("ui.max mms per sec", 1500, 100, 2000);

    // Threshold and label image
    AdaptiveThreshold(w, h, I,intI,tI.get(),at_threshold,at_window,(unsigned char)0,(unsigned char)255);
    vector<PixelClass> labels;
    Label(w,h,tI.get(),lI.get(),labels);

    // Find candidate regions for conics
    candidates.clear();
    FindCandidateConicsFromLabels(
                w,h,labels,candidates,
                conic_min_area,conic_max_area,conic_min_density, conic_min_aspect
                );

    // Find conic parameters
    conics.clear();
    FindConics(w,h,candidates,dI,conics );

    // Generate map and point structures
    conics_target_map.clear();
    conics_target_map.resize(conics.size(),-1);
    vector<Vector2d > ellipses;
    for( size_t i=0; i < conics.size(); ++i )
        ellipses.push_back(conics[i].center);

    // Find target given approximate pose T_cw
    target.FindTarget( T_hw, cam, conics, conics_target_map );

    // Update pose given correspondences
    T_hw = FindPose(cam,target.circles3D(),ellipses,conics_target_map,robust_4pt_inlier_tol,robust_4pt_its);
    rms = ReprojectionErrorRMS(cam,T_hw,target.circles3D(),ellipses,conics_target_map);

    int inliers =0;
    for( int i=0; i < conics_target_map.size(); ++i)
        if( conics_target_map[i] >=0 ) inliers++;

    if( !isfinite((double)rms) || rms > max_rms || (good_frames < 5 && inliers < 6) )
    {
        // Undistort Conics
        vector<Conic> conics_camframe;
        for( unsigned int i=0; i<conics.size(); ++i )
            conics_camframe.push_back(UnmapConic(conics[i],cam));

        // Find target given (approximately) undistorted conics
        const static LinearCamera idcam(-1,-1,1,1,0,0);
        target.FindTarget(
                    idcam,conics_camframe, conics_target_map, target_match_neighbours,
                    target_ransac_its, target_ransac_min_pts, target_ransac_max_inlier_err_mm,
                    target_plane_inlier_thresh
                    );

        // Estimate camera pose relative to target coordinate system
        T_hw = FindPose(cam,target.circles3D(),ellipses,conics_target_map,robust_4pt_inlier_tol,robust_4pt_its);
        rms = ReprojectionErrorRMS(cam,T_hw,target.circles3D(),ellipses,conics_target_map);
    }

    if( isfinite((double)rms) && rms < max_rms )
    {
        // seconds since good
        const double t = std::difftime(clock(),last_good) / (double) CLOCKS_PER_SEC;
        const double dx = (T_hw.inverse().translation() - T_gw.inverse().translation()).norm();
        if( dx / t < max_mmps || last_good == 0) {
            good_frames++;
        }else{
            good_frames = 0;
        }
        if( good_frames > 5 )
        {
            T_gw = T_hw;
            last_good = clock();
        }
    }else{
        good_frames = 0;
    }

    return good_frames >= 5;
}

int Tracker::NumVisibleFeatures() const
{
    int seen = 0;
    for(int i=0; i< conics_target_map.size(); ++i )
    {
        if(conics_target_map[i] >= 0 ) {
            seen++;
        }
    }
    return seen;
}


Eigen::Matrix<double,3,Eigen::Dynamic> Tracker::TargetPattern3D() const
{
    Eigen::MatrixXd pattern3d = Eigen::MatrixXd(3, target.circles().size() );
    for(size_t i=0; i < target.circles().size(); ++i )
    {
        pattern3d.col(i) = target.circles3D()[i];
    }
    return pattern3d;
}

Eigen::Matrix<double,2,Eigen::Dynamic> Tracker::TargetPatternObservations() const
{
    // Reverse map (target -> conic from conic -> target)
    vector<int> target_conics_map(target.circles().size(), -1);
    for( int i=0; i < conics_target_map.size(); ++i ) {
        if(conics_target_map[i] >= 0) {
            target_conics_map[conics_target_map[i]] = i;
        }
    }

    // Generate list of visible indices
    int unseen = 0;
    std::vector<short int> visibleCircles;
    for( int i=0; i < target_conics_map.size(); ++i ) {
        if( target_conics_map[i] != -1 ) {
            visibleCircles.push_back(i);
        }else{
            ++unseen;
        }
    }

    Eigen::Matrix<double,2,Eigen::Dynamic> obs = Eigen::MatrixXd(2, target.circles().size() );
    for(size_t i=0; i < target.circles().size(); ++i ) {
        const int c = target_conics_map[i];
        if(c >= 0 ) {
            Eigen::Vector2d circ = conics[c].center;
            obs.col(i) <<  circ[0] , circ[1];
        }else{
            obs.col(i) << NAN, NAN;
        }
    }
    return obs;
}

