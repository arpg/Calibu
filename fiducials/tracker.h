#ifndef TRACKER_H
#define TRACKER_H

#include <memory>
#include <sophus/se3.hpp>
#include <ctime>

#include "label.h"
#include "conics.h"
#include "target.h"

struct Tracker
{
//public:
    Tracker(int w, int h);

    bool ProcessFrame(LinearCamera& cam, unsigned char *I);
    bool ProcessFrame(LinearCamera& cam, unsigned char* I, boost::array<float, 2>* dI, float* intI);

    // Return number of visible features
    int NumVisibleFeatures() const;

    // Return indices of visible features
    std::vector<short int> VisibleFeatures() const;

    // Return matrix containing 3D points of pattern, a column per circle.
    Eigen::Matrix<double,3,Eigen::Dynamic> TargetPattern3D() const;

    // Return matrix containing 2D observations, a column per circle.in target pattern
    // Unobserved circles contain (NaN,NaN)'
    Eigen::Matrix<double,2,Eigen::Dynamic> TargetPatternObservations() const;

//protected:
    // Fiducial Target
    Target target;

    // Images
    int w, h;
    std::auto_ptr<float> intI;
    std::auto_ptr<boost::array<float,2> > dI;
    std::auto_ptr<short> lI;
    std::auto_ptr<unsigned char> tI;

    // Hypothesis conics
    std::vector<PixelClass> candidates;
    std::vector<Conic> conics;
    std::vector<int> conics_target_map;

    // Last good pose
    Sophus::SE3d T_gw;
    std::clock_t last_good;
    int good_frames;

    // Pose hypothesis
    Sophus::SE3d T_hw;
};

#endif // TRACKER_H
