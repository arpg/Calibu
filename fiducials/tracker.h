#ifndef TRACKER_H
#define TRACKER_H

#include <memory>
#include <sophus/se3.h>
#include <ctime>

#include "label.h"
#include "conics.h"
#include "target.h"

struct Tracker
{
//public:
    Tracker(int w, int h);

    bool ProcessFrame(LinearCamera& cam, unsigned char *I);
    bool ProcessFrame(LinearCamera& cam, unsigned char* I, std::array<float, 2>* dI, float* intI);

//protected:
    // Fiducial Target
    Target target;

    // Images
    int w, h;
    std::unique_ptr<float> intI;
    std::unique_ptr<std::array<float,2> > dI;
    std::unique_ptr<short> lI;
    std::unique_ptr<unsigned char> tI;

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
