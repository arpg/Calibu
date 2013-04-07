#include <fiducials/target/TargetGridDot.h>

namespace fiducials {

TargetGridDot::TargetGridDot(double grid_spacing, Eigen::Vector2i grid_size, Eigen::Vector2i grid_center)
    : grid_spacing(grid_spacing), grid_size(grid_size), grid_center(grid_center)
{
    // Create cached grid coordinates
    
    tpts2d.resize(grid_size(0) * grid_size(1));
    tpts3d.resize(grid_size(0) * grid_size(1));
    
    for(int r=0; r< grid_size(1); ++r) {
        for(int c=0; c< grid_size(0); ++c) {
            Eigen::Vector2i p = Eigen::Vector2i(c,r) - grid_center;
            tpts2d[r*grid_size(0)+c] = grid_spacing * Eigen::Vector2d(p(0), p(1));
            tpts3d[r*grid_size(0)+c] = grid_spacing * Eigen::Vector3d(p(0), p(1), 0);
        }
    }    
}

std::vector<std::vector<Dist> > ClosestPoints( const std::vector<Eigen::Vector2d>& pts)
{
    std::vector<std::vector<Dist> > ret;

    // Set size of arrays
    ret.resize(pts.size());
    for(size_t p1=0; p1 < pts.size(); ++p1)  ret[p1].resize(pts.size());
    
    // Compute distances between all points
    for(size_t p1=0; p1 < pts.size(); ++p1)
    {
        ret[p1][p1] = Dist{p1,0};
        // Distance relation is symmetric
        for(size_t p2=p1+1; p2 < pts.size(); ++p2 )
        {
            const double dist = (pts[p1] - pts[p2]).norm();
            ret[p1][p2] = Dist{p2, dist};
            ret[p2][p1] = Dist{p1, dist};
        }
    }

    // sort distances
    for(size_t p1=0; p1 < pts.size(); ++p1) {
        std::sort(ret[p1].begin(), ret[p1].end() );
    }
    
    return ret;
}

double SignedArea(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2, const Eigen::Vector2d& p3 )
{
    return p1(0) * (p2(1) - p3(1)) + p2(0) * (p3(1) - p1(1)) + p3(0) * (p1(1) - p2(1));
}

double NormArea(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2, const Eigen::Vector2d& p3 )
{
    // Compute signed area
    const double area = SignedArea(p1,p2,p3);
    const double len = (p3-p1).norm();
    return std::abs(area) / (len*len);
}

bool LineGroupIsInvalid(const LineGroup& lg)
{
    return lg.k == 2;
}

std::vector<Opposite> FindOpposites(const std::vector<Eigen::Vector2d>& pts, const std::vector<Dist>& closest, double thresh_dist, double thresh_area)
{
    std::vector<Opposite> ret;
    
    const size_t max_neigh = 5;
    
    if(closest.size() < max_neigh) 
        return ret;
    
    const size_t c = closest[0].i;
    const Eigen::Vector2d cpt = pts[c];
    
    const double max_dist = 3 * closest[2].dist;
    
    std::array<bool,max_neigh> used;
    used.fill(false);
            
    // Filter possible pairs
    for(size_t n1 = 1; n1 < max_neigh; ++n1 ) {
        const double d1 = closest[n1].dist;

        for(size_t n2 = n1+1; n2 < max_neigh; ++n2 ) {
            const double d2 = closest[n2].dist;
            
            // Check distances aren't much further than closest
            if( d1 < max_dist && d2 < max_dist )
            {
                
                // Check distances are similar
                if( std::abs((d2 - d1)/ d1) < thresh_dist )
                {
                    // Check points are colinear with center
                    const size_t c1 = closest[n1].i;
                    const size_t c2 = closest[n2].i;
                    if( NormArea(pts[c1],cpt,pts[c2]) < thresh_area )
                    {
                        // Check no aliasing exists between matched
                        if(used[n1] || used[n2]) {
                            // ambigous pairs, bail
                            ret.clear();
                            return ret;
                        }
                        
                        used[n1] = true;
                        used[n2] = true;
                        ret.push_back( Opposite{c, c1, c2} );
                    }
                }
            }
        }
    }
                
    return ret;
}

double AngleDist(double a1, double a2)
{
    double dist = a1 - a2;
    while(dist < 0) dist += 2*M_PI;
    while(dist > 2*M_PI) dist -= 2*M_PI;
    if(dist > M_PI) dist = -(2*M_PI - dist);
    return dist;
}

double LineAngleDist(double a1, double a2)
{
    double dist = a1 - a2;
    while(dist < 0) dist += M_PI;
    while(dist > M_PI) dist -= M_PI;
    if(dist > M_PI/2) dist = -(M_PI - dist);
    return dist;
}

//////////////////////////////////////////////////////////////
// returns a score for whether or not the blob pointed to by
// op1 and op2 could work as a center cross
double GetCenterCrossScore(const Opposite& op1,
                           const Opposite& op2,
                           const std::vector<Conic>& conics,
                           const unsigned char* img, int w, int h,
//                           cv::Mat& image,
                           const double areaThreshold, //< area threshold in percent
                           const double maxAreaThreshold, //< area threshold in percent
                           const double innerRadiusRatio,  //< size of the inner blob radius, in pixels
                           const double lineThicknessRatio //< size of the line, in pixels
                             )
{
    double score = 0;
    //first do a size check
    const double averageArea = (conics[op1.o1].bbox.Area() +conics[op1.o2].bbox.Area() +
                                conics[op2.o1].bbox.Area() +conics[op2.o2].bbox.Area())/4.0;
    const Conic& centerConic = conics[op1.c];
    if( centerConic.bbox.Area() < averageArea*areaThreshold ||
        centerConic.bbox.Area() > averageArea*maxAreaThreshold ) {
        score = std::numeric_limits<double>::max();
    }else{
        //go through every pixel, calculate background and foreground from min/max
        unsigned char background = 255, foreground = 0;
        for(int ii = centerConic.bbox.x1 ; ii < centerConic.bbox.x2 ; ii++){
            for(int jj = centerConic.bbox.y1 ; jj < centerConic.bbox.y2 ; jj++){
                unsigned char pval = *(img + jj*w + ii);
                background = std::min(background,pval);
                foreground = std::max(foreground,pval);
            }
        }        
        //std::cout << "Background: " << (int)background << " foreground: " << (int)foreground << std::endl;
        
        score = 0;
        int pixelCount = 0;
        for(int ii = centerConic.bbox.x1 ; ii < centerConic.bbox.x2 ; ii++){
            for(int jj = centerConic.bbox.y1 ; jj < centerConic.bbox.y2 ; jj++){
                pixelCount++;
                unsigned char cVal = background;
                //pixel position relative to center
                const Eigen::Vector2d relativePos(ii - centerConic.center[0],jj - centerConic.center[1]);
                const Eigen::Vector2d vecOp1 = conics[op1.o1].center - conics[op1.o2].center;
                const Eigen::Vector2d vecOp2 = conics[op2.o1].center - conics[op2.o2].center;

                //calculate whether this pixel should be white or black.
                //step 1 radius check
                const double rad = innerRadiusRatio*(vecOp1.norm() + vecOp2.norm())/2.0;
                if(relativePos.norm() <= rad){
                    cVal = foreground;
                }else{
                    //check the distance from the vertical line
                    const double thickness1 = vecOp1.norm()*lineThicknessRatio;
                    const double thickness2 = vecOp2.norm()*lineThicknessRatio;

                    const double cosTheta1 = vecOp1.normalized().dot(relativePos.normalized());
                    const double cosTheta2 = vecOp2.normalized().dot(relativePos.normalized());

                    const double perpDistance1 = fabs(relativePos.norm()*sin(acos(cosTheta1)));
                    const double perpDistance2 = fabs(relativePos.norm()*sin(acos(cosTheta2)));

                    if(perpDistance1 <= thickness1/2.0 || perpDistance2 <= thickness2/2.0){
                        cVal = foreground;
                    }
                }
                const unsigned char pval = *(img + jj*w + ii);
                const double pdiff = pval-cVal;
                score += pdiff*pdiff;
            }
        }
        //normalize the score by the number of pixels
        score /= (double)pixelCount;
    }

    return score;
}

bool TargetGridDot::Find(std::vector<Eigen::Vector2d>& pts, double thresh_dist, double thresh_area ) const
{
    pts_distance = ClosestPoints(pts);
    
    pts_neighbours.resize(pts_distance.size());
    for(size_t i=0; i < pts_neighbours.size(); ++i) {
        pts_neighbours[i] = FindOpposites(pts, pts_distance[i], thresh_dist, thresh_area );
    }
    
    // Fill initial line group structure
    for(size_t c=0; c<pts_neighbours.size(); ++c) {
        for(size_t n=0; n<pts_neighbours[c].size(); ++n) {
            Opposite op = pts_neighbours[c][n];
            line_groups.push_back( LineGroup(op) );
        }
    }
    
    // Try to merge line groups            
    bool something_added;
    do {
        something_added = false;
        for(std::list<LineGroup>::iterator i1 = line_groups.begin(); i1 != line_groups.end(); ++i1)
        {
            for(std::list<LineGroup>::iterator i2 = std::next(i1); i2 != line_groups.end(); ++i2)
            {                    
                if( i1->Merge(*i2) ) {
                    line_groups.erase(i2);
                    something_added = true;
                    break;
                }
            }
        }
    } while(something_added);  
    
    // Work out angle for each line group based on ends
    for(std::list<LineGroup>::iterator i = line_groups.begin(); i != line_groups.end(); ++i)
    {
        LineGroup& lg = *i;
        const Eigen::Vector2d diff = pts[lg.last()] - pts[lg.first()];                
        lg.theta = atan2(diff(1),diff(0));
        while(lg.theta < 0) lg.theta += 2*M_PI;
    }
        
    // Naive K-Means to find two principle directions. Initialise at 90 deg.
    Eigen::Array2d k(0.0, M_PI/2);
    Eigen::Array2d kvar(0.0, M_PI/2);
    
    for(int its=0; its < 10; ++its)
    {
        Eigen::Array2d knum(0, 0);
        Eigen::Array2d ksum(0.0, 0.0);
        Eigen::Array2d ksum_sq(0.0, 0.0);

        for(std::list<LineGroup>::iterator ilg = line_groups.begin(); ilg != line_groups.end(); ++ilg)
        {
            LineGroup& lg = *ilg;
            Eigen::Vector2d kdist;
            
            // distance to k mean
            for(int ik=0; ik<2; ++ik) {
                kdist[ik] = LineAngleDist(lg.theta, k[ik] );
            }
            
            // assign to closest kmean
            lg.k = std::abs(kdist[0]) < std::abs(kdist[1]) ? 0 : 1;
            
            // update stats for close k-mean
            knum[lg.k]++;
            ksum[lg.k] += kdist[lg.k];
            ksum_sq[lg.k] += kdist[lg.k] * kdist[lg.k];
        }
        
        // update k-mean
        for(int ik=0; ik<2; ++ik) {
            k[ik] += ksum[ik] / knum[ik];
            while(k[ik] < 0) k[ik] += M_PI;
            while(k[ik] > M_PI) k[ik] -= M_PI;
        }       
        
        // Compute variance
        kvar = (ksum_sq - ((ksum*ksum)/knum)) / knum;
    }

    //find the max size of each group
    int kMaxSize[2] = {0,0};
    for(const LineGroup& group: line_groups) {
        kMaxSize[group.k] = std::max(kMaxSize[group.k],(int)group.ops.size());
    }
    
    //flip ks if necessary
    const bool flipk = kMaxSize[0] < kMaxSize[1];
    if(flipk) {
        std::swap(k[0],k[1]);
        std::swap(kvar[0],kvar[1]);
    }
            
    // Normalise line directions (so they face same way) and remove bad directions
    for(std::list<LineGroup>::iterator ilg = line_groups.begin(); ilg != line_groups.end(); ++ilg)
    {
        LineGroup& lg = *ilg;
        if(flipk) lg.k = 1-lg.k;
        
        const Eigen::Vector2d dp = pts[lg.last()] - pts[lg.first()];
        
        const double dk = dp(lg.k);
        if( dk <0 ) {
            lg.Reverse();                
            lg.theta = lg.theta - M_PI;
            while(lg.theta < 0) lg.theta += 2*M_PI;                    
        }
        
        const double absdist = std::abs(LineAngleDist(lg.theta, k[lg.k]));
        if( absdist > params.max_line_group_k_sigma * sqrt(kvar[lg.k]) ) {
            lg.k = 2;
        }
        if( absdist > M_PI/16.0 ) {
            lg.k = 2;
        }
        
        // ignore if too short
        if(lg.ops.size() < 4) {
            lg.k = 2;
        }
    }       
    
//    line_groups.erase(
//        std::remove_if(line_groups.begin(), line_groups.end(), LineGroupIsInvalid),
//        line_groups.end()
//    );
    
    // We need at least two line groups to get points which aren't all colinear
    if(line_groups.size() < 2)
        return false;
    
    return true;
}

void TargetGridDot::PropagateGrid(const std::vector<Eigen::Vector2d>& pts,const int idxCross) const
{
    // Find grid coords
    grid.clear();
    grid.resize(pts.size(), Eigen::Vector2i(-100000,-100000));
    grid[idxCross] = Eigen::Vector2i(0,0);

    // Repeat a few times for good measure.
    for(int its=0; its < 10; its++) {
        // For each line, run up and down enforcing local distance
        for(std::list<LineGroup>::iterator ilg = line_groups.begin(); ilg != line_groups.end(); ++ilg)
        {
            // go through in sequence
            LineGroup& lg = *ilg;
            const int ix = lg.k;
            const int io = (lg.k+1)%2;

            if(ix < 2) { // inlier

                // travel up line
                Eigen::Vector2i last = grid[lg.first()];
                for(std::list<size_t>::iterator ie = std::next(lg.ops.begin()); ie != lg.ops.end(); ++ie)
                {
                    Eigen::Vector2i& curr = grid[*ie];
                    curr[ix] = std::max(curr[ix], last[ix]+1);
                    curr[io] = std::max(curr[io], last[io]);
                    last = curr;
                }

                // travel down line
                last = grid[lg.last()];
                for(std::list<size_t>::reverse_iterator ie = std::next(lg.ops.rbegin()); ie != lg.ops.rend(); ++ie)
                {
                    Eigen::Vector2i& curr = grid[*ie];
                    curr[ix] = std::max(curr[ix], last[ix]-1);
                    curr[io] = std::max(curr[io], last[io]);
                    last = curr;
                }
            }
        }
    }
}

bool TargetGridDot::FindTarget(
  const Sophus::SE3d& T_cw,
  const CameraModelBase& cam,
  const ImageProcessing& images,
  const std::vector<Conic>& conics,
  std::vector<int>& ellipse_target_map
) const {
    // This target doesn't use position or camera information
    return FindTarget(images,conics,ellipse_target_map);
}

bool TargetGridDot::FindTarget(
  const CameraModelBase& cam,
  const ImageProcessing& images,
  const std::vector<Conic>& conics,
  std::vector<int>& ellipse_target_map
) const {
    // This target doesn't use position or camera information
    return FindTarget(images,conics,ellipse_target_map);
}

void TargetGridDot::Clear() const
{
    pts_distance.clear();
    pts_neighbours.clear();
    line_groups.clear();
    grid.clear();
}

bool TargetGridDot::FindTarget(
  const ImageProcessing& images,
  const std::vector<Conic>& conics,
  std::vector<int>& ellipse_target_map
) const {
    
    // Clear cached data structures
    Clear();
    
    // Generate map and point structures
    std::vector<Eigen::Vector2d> ellipses;
    for( size_t i=0; i < conics.size(); ++i ) {
      ellipses.push_back(Eigen::Vector2d(conics[i].center.x(),conics[i].center.y()));
    }
    
    bool tracking_good =
        Find(ellipses, params.max_line_dist_ratio, params.max_norm_triple_area);

    //calcualte the score for each conic and keep a tally
    double bestScore = std::numeric_limits<double>::max();
    
    idxCrossConic = -1;
    for(size_t jj = 0 ; jj < conics.size() ; jj++){
        if(pts_neighbours[jj].size() == 2){
            const double score = GetCenterCrossScore(
                pts_neighbours[jj][0], pts_neighbours[jj][1],
                conics, images.Img(), images.Width(), images.Height(),
                params.min_cross_area, params.max_cross_area,
                params.cross_radius_ratio, params.cross_line_ratio
            );

            if(score < bestScore){
                bestScore = score;
                idxCrossConic = jj;
            }
        }
    }
    
    tracking_good = tracking_good && (idxCrossConic != -1);

    ellipse_target_map.clear();

    if(tracking_good) {
        PropagateGrid(ellipses,idxCrossConic);
        
        ellipse_target_map.resize(ellipses.size(), -1);
        for(size_t p=0; p < ellipses.size(); ++p) {
            const Eigen::Vector2i pgz = grid[p] + grid_center;
            if( 0<= pgz(0) && pgz(0) < grid_size(0) &&  0<= pgz(1) && pgz(1) < grid_size(1) )
            {
                ellipse_target_map[p] = pgz(1)*grid_size(0) + pgz(0);
            }
        }
    }
    
    return tracking_good;
}


}
