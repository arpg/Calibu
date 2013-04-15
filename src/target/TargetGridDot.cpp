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

#include <calibu/target/TargetGridDot.h>
#include <calibu/target/RandomGrid.h>

#include <map>
#include <set>
#include <algorithm>
#include <iostream>
#include <deque>

namespace calibu {

TargetGridDot::TargetGridDot(double grid_spacing, Eigen::Vector2i grid_size, Eigen::Vector2i grid_center, uint32_t seed)
    : grid_spacing(grid_spacing), grid_size(grid_size), grid_center(grid_center)
{
    // Create binary pattern (and rotated pattern) from seed
    PG = MakePatternGroup(grid_size(1), grid_size(0), seed);
    
    // Create cached grid coordinates
    tpts2d.resize(grid_size(0) * grid_size(1));
    tpts3d.resize(grid_size(0) * grid_size(1));
    
    for(int r=0; r< grid_size(1); ++r) {
        for(int c=0; c< grid_size(0); ++c) {
            Eigen::Vector2i p = Eigen::Vector2i(c,r);
            tpts2d[r*grid_size(0)+c] = grid_spacing * Eigen::Vector2d(p(0), p(1));
            tpts3d[r*grid_size(0)+c] = grid_spacing * Eigen::Vector3d(p(0), p(1), 0);
        }
    }    
}

std::vector<std::vector<Dist> > ClosestPoints( std::vector<Vertex>& pts)
{
    std::vector<std::vector<Dist> > ret;
    
    // Set size of arrays
    ret.resize(pts.size());
    for(size_t p1=0; p1 < pts.size(); ++p1)  ret[p1].resize(pts.size());
    
    // Compute distances between all points
    for(size_t p1=0; p1 < pts.size(); ++p1)
    {
        ret[p1][p1] = Dist{ &pts[p1],0};
        // Distance relation is symmetric
        for(size_t p2=p1+1; p2 < pts.size(); ++p2 )
        {
            const double dist = Distance(pts[p1], pts[p2]);
            ret[p1][p2] = Dist{ &pts[p2], dist};
            ret[p2][p1] = Dist{ &pts[p1], dist};
        }
    }
    
    // sort distances
    for(size_t p1=0; p1 < pts.size(); ++p1) {
        std::sort(ret[p1].begin(), ret[p1].end() );
    }
    
    return ret;
}

std::vector<Dist> MostCentral( std::vector<std::vector<Dist> >& distances )
{
    std::vector<Dist> sum_sq;
    
    for(size_t i=0; i < distances.size(); ++i) {
        Vertex* v = distances[i][0].v;
        Dist dist{v,0};
        for(size_t j=0; j < distances[i].size(); ++j) {
            dist.dist += distances[i][j].dist * distances[i][j].dist;
        }
        sum_sq.push_back( dist );
    }
    
    std::sort(sum_sq.begin(), sum_sq.end());
    return sum_sq;
}

std::vector<Triple*> PrincipleDirections( Vertex& v)
{
    // Find principle directions by observing that neighbours from princple
    // directions are central within triple that is also formed from these
    // neighbours.
    std::set<Triple*> pd;
    for(size_t i=0; i<v.triples.size(); ++i) {
        Triple& t = v.triples[i];
        for(size_t j=0; j<2; ++j) {
            Vertex& n = t.Neighbour(j);
            for(size_t k=0; k< n.triples.size(); ++k) {
                Triple& a = n.triples[k];
                if(a.In(v.neighbours))  {
                    // a is parallel to principle direction
                    // t is a parallel direction.
                    pd.insert(&t);
                    break;
                }
            }
        }
    }
    
    // convert to vector
    std::vector<Triple*> ret;
    ret.insert(ret.begin(), pd.begin(), pd.end());
    
    // find most x-ily and y-ily
    if(ret.size() == 2) {
        Eigen::Vector2d d[2] = { ret[0]->Dir(), ret[1]->Dir() };
        if(abs(d[1][0]) > abs(d[0][0]) ) {
            std::swap(ret[0], ret[1]);
            std::swap(d[0], d[1]);
        }
        
        // place in axis ascending order.
        if(d[0][0] < 0) ret[0]->Reverse();
        if(d[1][1] < 0) ret[1]->Reverse();
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

double Area(const Conic& c)
{
    // http://en.wikipedia.org/wiki/Matrix_representation_of_conic_sections
    const Eigen::Matrix2d A33 = c.C.block<2,2>(0,0);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eigensolver(A33);
    if (eigensolver.info() == Eigen::Success) {
        const double detA = c.C.determinant();
        const double detA33 = A33.determinant();        
        const double fac = -detA / (detA33);
        const Eigen::Vector2d l = eigensolver.eigenvalues();
        
        const double a = sqrt(fac/l[0]);
        const double b = sqrt(fac/l[1]);
        
        return M_PI * a * b;
    }else{
        return 0.0;
    }
}

std::set<Vertex*> Neighbours(std::map<Eigen::Vector2i, Vertex*>& map, const Vertex& v)
{
    std::set<Vertex*> neighbours;
    for(int r=-1; r <=1; ++r) {
        for(int c=-1; c<=1; ++c) {
            Eigen::Vector2i pg(v.pg[0]+c, v.pg[1]+r);
            std::map<Eigen::Vector2i, Vertex*>::const_iterator i = map.find(pg);
            if(i!= map.end()) {
                neighbours.insert(i->second);
            }
        }
    }
    return neighbours;
}

void FindTriples( Vertex& v, std::vector<Dist>& closest, double thresh_dist, double thresh_area)
{
    // Consider 9 closests points (including itself)
    const size_t NEIGHBOURS = 9;
    const size_t max_neigh = std::min(closest.size(), NEIGHBOURS);
    
    // We need at least 3 points for a single collinear triple.
    if(max_neigh < 3) return;
    
    // Ignore points too much furthar than closest.
    // We are interested in 8-connected region
    const double max_dist = 5 * closest[1].dist;
    
    std::array<bool,NEIGHBOURS> used;
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
                if( 2.0 * fabs(d2-d1) / (fabs(d1)+fabs(d2)) < thresh_dist )
                {
                    // Check points are colinear with center
                    Vertex& c1 = *closest[n1].v;
                    Vertex& c2 = *closest[n2].v;
                    
                    if( NormArea(c1.pc,v.pc,c2.pc) < thresh_area )
                    {
                        // Check no aliasing exists between matched
                        if(used[n1] || used[n2]) {
                            v.triples.clear();
                            v.neighbours.clear();
                            return;
                        }else{
                            used[n1] = true;
                            used[n2] = true;
                            v.neighbours.insert(&c1);
                            v.neighbours.insert(&c2);
                            v.triples.push_back( Triple(c1, v, c2) );
                        }
                    }
                }
            }
        }
    }
}

bool TargetGridDot::FindTarget(
        const Sophus::SE3d& T_cw,
        const CameraModelInterface& cam,
        const ImageProcessing& images,
        const std::vector<Conic>& conics,
        std::vector<int>& ellipse_target_map
        )
{
    // This target doesn't use position or camera information
    return FindTarget(images,conics,ellipse_target_map);
}

bool TargetGridDot::FindTarget(
        const CameraModelInterface& cam,
        const ImageProcessing& images,
        const std::vector<Conic>& conics,
        std::vector<int>& ellipse_target_map
        )
{
    // This target doesn't use position or camera information
    return FindTarget(images,conics,ellipse_target_map);
}

void TargetGridDot::Clear()
{
    vs.clear();
    line_groups.clear();
    map_grid_ellipse.clear();
}

void TargetGridDot::SetGrid(Vertex& v, const Eigen::Vector2i& g)
{
    v.pg = g;
    map_grid_ellipse[g] = &v;
}

bool TargetGridDot::Match(std::map<Eigen::Vector2i, Vertex*>& obs, const std::array<Eigen::MatrixXi,4>& PG)
{
    Eigen::Vector2i omin(std::numeric_limits<int>::max(),std::numeric_limits<int>::max());
    Eigen::Vector2i omax(std::numeric_limits<int>::min(),std::numeric_limits<int>::min());
    
    // find max and min
    for(std::map<Eigen::Vector2i, Vertex*>::const_iterator i = obs.begin(); i != obs.end(); ++i) {
        omin[0] = std::min(omin[0], i->first[0]);
        omin[1] = std::min(omin[1], i->first[1]);
        omax[0] = std::max(omax[0], i->first[0]);
        omax[1] = std::max(omax[1], i->first[1]);
    }
    
    // Create sample matrix
    Eigen::Vector2i osize = (omax + Eigen::Vector2i(1,1)) - omin;
    
    if( osize[0] > 2 && osize[1] > 2) {
        Eigen::MatrixXi m = Eigen::MatrixXi::Constant( osize(1), osize(0), -1);
        int num_valid = 0;
        
        for(std::map<Eigen::Vector2i, Vertex*>::iterator i = obs.begin(); i != obs.end(); ++i) {
            const Eigen::Vector2i pg = i->first - omin;
            i->second->pg = pg;            
            const int val = i->second->value;
            m(pg(1),pg(0)) = val;
            if(val >= 0) ++num_valid;
        }
        
        int bs,bg,br,bc;
        const int num_matches = NumExactMatches(PG,m,bs,bg,br,bc);
        if( num_matches <= 1 && bs < num_valid / 16 )
//        if( num_matches == 1 )
        {
            // Found unique match
            Sophus::SE2Group<int> T_0x[4] = {
                Sophus::SE2Group<int>(Sophus::SO2Group<int>(1,0), Eigen::Vector2i(0,0) ),
                Sophus::SE2Group<int>(Sophus::SO2Group<int>(0,1), Eigen::Vector2i(grid_size[0]-1,0) ),
                Sophus::SE2Group<int>(Sophus::SO2Group<int>(-1,0), Eigen::Vector2i(grid_size[0]-1,grid_size[1]-1) ),
                Sophus::SE2Group<int>(Sophus::SO2Group<int>(0,-1), Eigen::Vector2i(0,grid_size[1]-1) )
            };
            
            Sophus::SE2Group<int> T_xm(Sophus::SO2Group<int>(), Eigen::Vector2i(bc,br));
            
            Sophus::SE2Group<int> T_0m = T_0x[bg] * T_xm;
            
            for(std::map<Eigen::Vector2i, Vertex*>::iterator i = obs.begin(); i != obs.end(); ++i) {
                i->second->pg = T_0m * i->second->pg;
            }     
            return true;
        }else{
            PrintPattern(m);
            std::cerr << "bestscore: " << bs << ", num_matches: " << num_matches << ", ";
        }
    }else{
        std::cerr << "Grid too small, ";
    }
    return false;
}

bool TargetGridDot::FindTarget(
        const ImageProcessing& images,
        const std::vector<Conic>& conics,
        std::vector<int>& ellipse_target_map
        )
{
    
    // Clear cached data structures
    Clear();
    ellipse_target_map.clear();    
    
    // Generate vertex and point structures
    for( size_t i=0; i < conics.size(); ++i ) {
        vs.push_back( Vertex(i, conics[i]) );
    }
    
    // Compute closest points for each ellipse
    std::vector<std::vector<Dist> > vs_distance = ClosestPoints(vs);
    std::vector<Dist> vs_central = MostCentral(vs_distance);
    
    // Find colinear neighbours for each ellipse
    for(size_t i=0; i < vs.size(); ++i) {
        FindTriples(vs[i], vs_distance[i], params.max_line_dist_ratio, params.max_norm_triple_area );
        for(Triple& t : vs[i].triples) line_groups.push_back( LineGroup(t)  );            
    }    
    
    // Find central, well connected vertex
    Vertex* central = nullptr;
    std::vector<Triple*> principle;
    
    for(size_t i=0; i < vs_central.size(); ++i) {
        Vertex* v = vs_central[i].v;
        if(v->triples.size() >= 2) {
            principle = PrincipleDirections(*v);
            if(principle.size() == 2) {
                central = v;
                break;
            }
        }
    }
    
    if(!central) {
        std::cerr << "No central point found" << std::endl;
        return false;
    }
        
    for(auto* t: principle) {
        line_groups.push_back( LineGroup(*t) );
    }
    
    // Search structures
    std::deque<Vertex*> fringe;
    std::deque<Vertex*> available;
    for(size_t i=0; i< vs.size(); ++i) {
        available.push_back(&vs[i]);
    }
    
    // Setup central as center of grid
    SetGrid(*central, Eigen::Vector2i(0,0));
    available.erase(std::find(available.begin(), available.end(), central));
    
    // add neighbours of central to form basis
    for(int i=0; i<2; ++i)
    {
        Triple& t = *principle[i];
        Eigen::Vector2i g(0,0);
        
        for(int j=0; j < 2; ++j) {
            Vertex& n = t.Neighbour(j);
            g[i] = 2*j-1;
            SetGrid(n, g);
            available.erase(std::find(available.begin(), available.end(), &n));
            fringe.push_back(&n);
        }
//        line_groups.push_back( LineGroup(t) );        
    }
    
    // depth first search extending 'fringe' set by adding colinear vertices
    while(fringe.size() > 0) {
        Vertex& f = *fringe.front();
        for(size_t i=0; i<f.triples.size(); ++i) {
            Triple& t = f.triples[i];
            for(size_t j=0; j < 2; ++j) {
                Vertex& n = t.Neighbour(j);
                Vertex& no = t.OtherNeighbour(j);
                if( n.HasGridPosition() ) {
                    // expected other-neighbour grid position
                    const Eigen::Vector2i step = f.pg - n.pg;
                    const Eigen::Vector2i go = f.pg + step; 
                    
                    // Only accept local neighbours.                    
                    if( abs(step[0]) > 1 || abs(step[1]) > 1 ) {
                        continue;
                    }
                    
                    // Either check consistent or complete
                    if( no.HasGridPosition() ) {
                        // check
                        if(no.pg != go) {
                            // tracking bad!
                            std::cerr << "fringe: Not consistent" << std::endl;                            
                            return false;
                        }
                    }else{
                        // add
                        SetGrid(no, go);
                        fringe.push_back(&no);
//                        line_groups.push_back( LineGroup(t)  );
                    }
                    
                    // no need to check other neighbour
                    break;
                }
            }
        }
        
        // Remove from fringe
        fringe.pop_front();
    }
    
    // Try to add any that we've missed by 'filling in'
    while(available.size() > 0) {
        Vertex& f = *available.front();
        for(size_t i=0; i<f.triples.size(); ++i) {
            Triple& t = f.triples[i];
            Vertex& n = t.Neighbour(0);
            Vertex& no = t.OtherNeighbour(0);
            if( n.HasGridPosition() && no.HasGridPosition()) {
                const Eigen::Vector2i step = no.pg - n.pg;
                if(step[0]%2 == 0 && step[1]%2 ==0) {
                    const Eigen::Vector2i g = (no.pg + n.pg) / 2;
                    if(f.HasGridPosition()) {
                        // check
                        if(f.pg != g) {
                            // tracking bad.
                            std::cerr << "fill: Not consistent" << std::endl;                                                        
                            return false;
                        }
                    }else{
                        // add
                        SetGrid(f, g);
//                        line_groups.push_back( LineGroup(t)  );
                    }
                }
            }
        }
        available.pop_front();
    }
    
    // Compute area and grid neighbours for all ellipses in grid
    for(std::map<Eigen::Vector2i, Vertex*>::const_iterator i = map_grid_ellipse.begin(); i != map_grid_ellipse.end(); ++i) {
        Vertex& v = *i->second;
        v.area = Area(v.conic);
        v.neighbours = Neighbours(map_grid_ellipse,v);
    }

    // Determine binary value from neighbours area
    for(std::map<Eigen::Vector2i, Vertex*>::const_iterator i = map_grid_ellipse.begin(); i != map_grid_ellipse.end(); ++i) {
        Vertex& v = *i->second;
        
        if(v.neighbours.size() > 0) {
            // TODO: just take min/max - no need to sort
            // Sort neightbours by circle area
            std::vector<Dist> vecrad;
            vecrad.push_back( Dist{ &v, v.area });
            for(Vertex* n : v.neighbours)  {
                vecrad.push_back( Dist{n, n->area } );
            }
            std::sort(vecrad.begin(), vecrad.end());
            
            const double _area0 = vecrad.front().dist;
            const double _area1 = vecrad.back().dist;
            
            // TODO: determine these values from pattern
            const double area0 = 2*2;
            const double area1 = 3*3;
            if(area1*_area0 / area0 <= _area1 * 1.2)
            {
                // is difference
                const double d0 = std::abs(v.area-_area0);
                const double d1 = std::abs(v.area-_area1);
                v.value = ( d0 < d1 ) ? 0 : 1;
            }
        }else{
            v.value = -1;
        }  
    }    
    
    // Correlation of what we have with binary pattern
    const bool found = Match(map_grid_ellipse, PG);
    
    if(!found) {
        std::cerr << "Pattern not found" << std::endl;                            
        return false;
    }
    
    // output map
    ellipse_target_map.resize(vs.size(), -1);
    for(size_t p=0; p < vs.size(); ++p) {
        Vertex& v = vs[p];
        if( 0<= v.pg(0) && v.pg(0) < grid_size(0) &&  0<= v.pg(1) && v.pg(1) < grid_size(1) )
        {
            ellipse_target_map[p] = v.pg(1)*grid_size(0) + v.pg(0);
        }
    }
    
    return true;
}


}
