#ifndef TRACKER_H
#define TRACKER_H

#include <sophus/se3.h>
#include <ctime>
#include <cvd/image.h>
#include <cvd/rgb.h>

#include "label.h"
#include "conics.h"
#include "target.h"

struct Tracker
{
//public:
    Tracker(const CVD::ImageRef& video_size);

    bool ProcessFrame(LinearCamera& cam, CVD::Image<CVD::byte>& I);
    bool ProcessFrame(LinearCamera& cam, CVD::Image<CVD::byte>& I, CVD::Image<float[2]>& dI, CVD::Image<float>& intI);

//protected:
    // Fiducial Target
    Target target;

    // Images
    int w, h;
    CVD::Image<float> intI;
    CVD::Image<float[2]> dI;
    CVD::Image<short> lI;
    CVD::Image<CVD::byte> tI;

    // Hypothesis conics
    std::vector<PixelClass> candidates;
    std::vector<Conic> conics;
    std::vector<int> conics_target_map;

    // Last good pose
    Sophus::SE3 T_gw;
    std::clock_t last_good;
    int good_frames;

    // Pose hypothesis
    Sophus::SE3 T_hw;

    // Tracking Settings

};

#endif // TRACKER_H
