#ifndef PNP_H
#define PNP_H

#include <vector>
#include <TooN/se3.h>
#include "camera.h"

void PoseFromPoints(
    const LinearCamera& cam,
    const std::vector<TooN::Vector<3> >& pts3d,
    const std::vector<TooN::Vector<2> >& pts2d,
    const std::vector<int>& map2d_3d,
    TooN::SE3<>& T_cw,
    bool use_guess = false
    );

double ReprojectionErrorRMS(
    const AbstractCamera& cam,
    const TooN::SE3<>& T_cw,
    const std::vector<TooN::Vector<3> >& pts3d,
    const std::vector<TooN::Vector<2> >& pts2d,
    const std::vector<int>& map2d_3d
    );

#endif // PNP_H
