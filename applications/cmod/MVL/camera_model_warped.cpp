/**
 *  @file camera_model_warped.c
 *
 *  Warped camera model 2d-to-3d and 3d-to-2d functions.
 *
 *  $Id: camera_model_warped.cpp 373 2008-02-26 13:55:20Z gsibley $
 */
#include <stdio.h>
#include <math.h>
#include <assert.h>
#include <stdlib.h>
#include <string.h>

#include "camera.h"
#include "math.h"

//#include <mvl/mvl.h> // why?

/****************************************************************************/
void mvl_camera_warped_2d_to_3d(
                                const double ifx,
                                const double icx,
                                const double ify,
                                const double icy,
                                const double isx,
                                const double kappa1inv,
                                const double kappa2inv,
                                const double kappa3inv,
                                const double tau1inv,
                                const double tau2inv,
                                const double *px,
                                double *ray,
                                double *dray )
{
    double m_d[2];
    if(dray==NULL) {
        /* Use inverse of K */
        m_d[0] = ifx*px[0] + isx*px[1] + icx;
        m_d[1] =             ify*px[1] + icy;

        /* Apply distortion with inverse model parameters. */
        mvl_camera_warped_distortion( kappa1inv,kappa2inv,kappa3inv,
                                      tau1inv,tau2inv,m_d,ray,NULL);
        ray[2] = 1;

    } else {
        /* Use inverse of K */
        m_d[0] = ifx*px[0] + isx*px[1] + icx;
        m_d[1] =             ify*px[1] + icy;

        double dm_u[4];

        /* Apply distortion with inverse model parameters. */
        mvl_camera_warped_distortion( kappa1inv,kappa2inv,kappa3inv,
                                      tau1inv,tau2inv,m_d,ray,dm_u);

        ray[2] = 1;

        dray[0] = ifx*dm_u[0];
        dray[1] = isx*dm_u[0] + ify*dm_u[1];
        dray[2] = 0;
        dray[3] = ifx*dm_u[2];
        dray[4] = isx*dm_u[2] + ify*dm_u[3];
        dray[5] = 0;
    }
}

/****************************************************************************/
void mvl_camera_warped_3d_to_2d(
        const double fx,
        const double cx,
        const double fy,
        const double cy,
        const double sx,
        const double kappa1,
        const double kappa2,
        const double kappa3,
        const double tau1,
        const double tau2,
        const double *pt,
        double *px,
        double *dpx )
{
    double m_u[2],m_d[2];

    // Project 3D point to the normalised plane.
    m_u[0] = pt[0]/pt[2];
    m_u[1] = pt[1]/pt[2];

    if(dpx==NULL) {
        // Apply distortion
        mvl_camera_warped_distortion( kappa1,kappa2,kappa3,
                                      tau1,tau2,m_u,m_d,NULL);

        // Project using K matrix
        px[0] = fx*m_d[0] + sx*m_d[1] + cx;
        px[1] =             fy*m_d[1] + cy;
    } else {
        double dm_u[6];
        dm_u[0] = 1/pt[2];
        dm_u[1] = 0;
        dm_u[2] = -pt[0]/(pt[2]*pt[2]);
        dm_u[3] = 0;
        dm_u[4] = 1/pt[2];
        dm_u[5] = -pt[1]/(pt[2]*pt[2]);

        double dm_d[4];
        /* Apply distortion */
        mvl_camera_warped_distortion( kappa1,kappa2,kappa3,
                                      tau1,tau2,m_u,m_d,dm_d);

        /* Project using K matrix */
        px[0] = fx*m_d[0] + sx*m_d[1] + cx;
        px[1] =             fy*m_d[1] + cy;

        /* Calculate Jacobian by chain rule dpx = dK*dm_d*dm_u */
        dpx[0] = fx*(dm_u[0]*dm_d[0]+dm_u[3]*dm_d[1]) + sx*(dm_u[0]*dm_d[2]+dm_u[3]*dm_d[3]);
        dpx[1] = fx*(dm_u[1]*dm_d[0]+dm_u[4]*dm_d[1]) + sx*(dm_u[1]*dm_d[2]+dm_u[4]*dm_d[3]);
        dpx[2] = fx*(dm_u[2]*dm_d[0]+dm_u[5]*dm_d[1]) + sx*(dm_u[2]*dm_d[2]+dm_u[5]*dm_d[3]);

        dpx[3] = fy*(dm_u[0]*dm_d[2]+dm_u[3]*dm_d[3]);
        dpx[4] = fy*(dm_u[1]*dm_d[2]+dm_u[4]*dm_d[3]);
        dpx[5] = fy*(dm_u[2]*dm_d[2]+dm_u[5]*dm_d[3]);
    }
}

/****************************************************************************/
// This matches the Matlab camera calibration toolbox:
// http://www.vision.caltech.edu/bouguetj/calib_doc/htmls/parameters.html
// One thing, m_d is already on the z=1 plane.
void mvl_camera_warped_distortion(
                                  const double kappa1,
                                  const double kappa2,
                                  const double kappa3,
                                  const double tau1,
                                  const double tau2,
                                  const double *m_u,
                                  double *m_d,
                                  double *dm_d
                                  ) {
    double mx_u,my_u,mx_d,my_d,mx2_u, my2_u, mxy_u, rho2_u, rad_dist_u;
    mx_u = m_u[0];
    my_u = m_u[1];

    mx2_u = mx_u*mx_u;
    my2_u = my_u*my_u;
    mxy_u = mx_u*my_u;
    rho2_u = mx2_u+my2_u;
    rad_dist_u = kappa1*rho2_u + kappa2*rho2_u*rho2_u + kappa3*rho2_u*rho2_u*rho2_u;
    mx_d = mx_u + mx_u*rad_dist_u + 2*tau1*mxy_u + tau2*(rho2_u+2*mx2_u);
    my_d = my_u + my_u*rad_dist_u + 2*tau2*mxy_u + tau1*(rho2_u+2*my2_u);

    m_d[0] = mx_d;
    m_d[1] = my_d;

    /* l= k1*2*x*y + k2*4*r2*x*y + k3*2*x + k4*2*y + k5*6*r2^2*x*y;
       dPdX=[1 + k1*(r2+2*x^2) + k2*r2*(r2+4*x^2) + k3*2*y + k4*6*x + k5*r2^2*(r2+6*x^2), l;
             l, 1 + k1*(r2+2*y^2) + k2*r2*(r2+4*y^2) + k3*6*y + k4*2*x + k5*r2^2*(r2+6*y^2)];
    */

    if (dm_d!=NULL) {
        dm_d[0] = 1 + rad_dist_u + kappa1*2*mx2_u + kappa2*rho2_u*4*mx2_u + 2*tau1*my_u + 6*tau2*mx_u + kappa3*rho2_u*rho2_u*6*mx_u*mx_u;
        dm_d[1] = kappa1*2*mx_u*my_u + kappa2*4*rho2_u*mx_u*my_u + tau1*2*mx_u + 2*tau2*my_u + kappa3*6*rho2_u*rho2_u*mx_u*my_u;
        dm_d[2] = dm_d[1];
        dm_d[3] = 1 + rad_dist_u + kappa1*2*my2_u + kappa2*rho2_u*4*my2_u + 6*tau1*my_u + 2*tau2*mx_u + kappa3*rho2_u*rho2_u*6*my_u*my_u;
    }
}


#ifdef HAVE_NETLIB // this guy depends on blas/lapack
int mvl_camera_warped_2d_to_3d_gauss(
                                     const double fx,
                                     const double cx,
                                     const double fy,
                                     const double cy,
                                     const double sx,
                                     const double kappa1,
                                     const double kappa2,
                                     const double kappa3,
                                     const double tau1,
                                     const double tau2,
                                     const double *px,
                                     const int max_iter,
                                     const double max_row_error,
                                     const double max_col_error,
                                     double *ray,
                                     double *iray
                                     ) {
    double reproject_pt[2];
    double dpx[6],dpx_simpl[4];
    double ifx,icx,ify,icy,isx;
    double error[2];
    double norm_error;
    int iter = 0;
    int verbose = 0;

    // We are looking for the ray that
    // reprojects in px

    // Initialise assuming no distortion
    if(1) { //iray==NULL) {
        mvl_camera_inv_projmat( fx,cx,fy,cy,sx,&ifx,&icx,&ify,&icy,&isx );
        mvl_camera_warped_2d_to_3d( ifx,icx,ify,icy,isx,0,0,0,0,0,px,ray,NULL );
    } else {
        ray[0] = iray[0];
        ray[1] = iray[1];
        ray[2] = iray[2];
    }

    mvl_camera_warped_3d_to_2d( fx,cx,fy,cy,sx,kappa1,kappa2,kappa3,tau1,tau2,
                                ray,reproject_pt,dpx );

    if(verbose) {
        printf("reproject_pt: [%f,%f]\n",reproject_pt[0],reproject_pt[1]);
        printf("image_pt (px): [%f,%f]\n",px[0],px[1]);
    }

    // Update using Jacobian
    // f(ray_x,ray_y) = 3d_to_2d(ray_x,ray_y)-image_pt
    error[0] = reproject_pt[0]-px[0];
    error[1] = reproject_pt[1]-px[1];

    norm_error = sqrt(error[0]*error[0]+error[1]*error[1]);

    while(((fabs(error[0])>max_col_error)||
           (fabs(error[1])>max_row_error))&&
          (iter++<max_iter)) {

        if(verbose) {
            printf("error: [%f,%f]\n",error[0],error[1]);
        }

        dpx_simpl[0] = dpx[0];
        dpx_simpl[1] = dpx[1];
        dpx_simpl[2] = dpx[3];
        dpx_simpl[3] = dpx[4];

        solveAxb_LU_alloc_nd(dpx_simpl,error,2);

        ray[0] = ray[0] - error[0];
        ray[1] = ray[1] - error[1];

        mvl_camera_warped_3d_to_2d( fx,cx,fy,cy,sx,kappa1,kappa2,kappa3,tau1,tau2,
                                    ray,reproject_pt,dpx );

        if(verbose) {
            printf("reproject_pt: [%f,%f]\n",reproject_pt[0],reproject_pt[1]);
        }
        error[0] = reproject_pt[0]-px[0];
        error[1] = reproject_pt[1]-px[1];
    }

    if( (fabs(error[0])>max_row_error)|| (fabs(error[1])>max_col_error) )
        return 0;

    return 1;
}

int mvl_camera_build_lut_2d_to_3d(const double fx,
                                  const double cx,
                                  const double fy,
                                  const double cy,
                                  const double sx,
                                  const double kappa1,
                                  const double kappa2,
                                  const double kappa3,
                                  const double tau1,
                                  const double tau2,
                                  const int image_width,
                                  const int image_height,
                                  const int max_iter,
                                  const double max_row_error,
                                  const double max_col_error,
                                  Point2D_d ***lut
                                  )
{
    int r,c;
    int int_cx = ceil(cx);
    int int_cy = ceil(cy);
    double ray[3], image_pt[2];

    int ret;

    Point2D_d* lut_array = (Point2D_d*) malloc( sizeof(Point2D_d)*image_width*image_height );

    // Linear memory but 'double' access
    *lut = (Point2D_d** ) malloc( sizeof(Point2D_d*)*image_height );
    for(r=0;r<image_height;r++) {
        (*lut)[r] = lut_array;
        lut_array += image_width;
    }

    /* Build lookup table starting at the center where the distortion
       is lowest and build the four quadrants separately.  This
       ensures faster convergence and reduces the chance of falling into
       a local minima. */
    image_pt[0] = int_cx;
    image_pt[1] = int_cy;
    // Lift center of projection
    mvl_camera_warped_2d_to_3d_gauss( fx,cx,fy,cy,sx,kappa1,kappa2,kappa3,tau1,tau2,
                                      image_pt,max_iter,max_col_error,max_row_error,ray,NULL );

    // Bottom right quadrant
    for( r=int_cx;r<image_height;r++ ) {
        for( c=int_cy;c<image_width;c++ ) {
            image_pt[0] = c;
            image_pt[1] = r;

            ret = mvl_camera_warped_2d_to_3d_gauss( fx,cx,fy,cy,sx,kappa1,kappa2,kappa3,tau1,tau2,
                                                    image_pt,max_iter,max_col_error,max_row_error,ray, ray);

            if(!ret) {
                printf("Out at (%i,%i)\n",r,c);
                return ret;
            }
            (*lut)[r][c].x = ray[0];
            (*lut)[r][c].y = ray[1];
        }
        ray[0] = (*lut)[r][int_cy].x;
        ray[1] = (*lut)[r][int_cy].y;
    }

    // Lift center of projection
    mvl_camera_warped_2d_to_3d_gauss( fx,cx,fy,cy,sx,kappa1,kappa2,kappa3,tau1,tau2,
                                      image_pt,max_iter,max_col_error,max_row_error,ray,NULL );

    // Bottom left quadrant
    for(r=int_cx;r<image_height;r++) {
        for(c=int_cy;c>=0;c--) {
            image_pt[0] = c;
            image_pt[1] = r;

            ret = mvl_camera_warped_2d_to_3d_gauss( fx,cx,fy,cy,sx,kappa1,kappa2,kappa3,tau1,tau2,
                                                    image_pt,max_iter,max_col_error,max_row_error,ray,ray);

            if(!ret) {
                printf("Out at (%i,%i)\n",c,r);
                return ret;
            }
            (*lut)[r][c].x = ray[0];
            (*lut)[r][c].y = ray[1];
        }
        ray[0] = (*lut)[r][int_cy].x;
        ray[1] = (*lut)[r][int_cy].y;
    }

    // Lift center of projection
    mvl_camera_warped_2d_to_3d_gauss( fx,cx,fy,cy,sx,kappa1,kappa2,kappa3,tau1,tau2,
                                      image_pt,max_iter,max_col_error,max_row_error,ray,NULL );

    // Top left quadrant
    for(r=int_cx;r>=0;r--) {
        for(c=int_cy;c>=0;c--) {
            image_pt[0] = c;
            image_pt[1] = r;

            ret = mvl_camera_warped_2d_to_3d_gauss( fx,cx,fy,cy,sx,kappa1,kappa2,kappa3,tau1,tau2,
                                                    image_pt,max_iter,max_col_error,max_row_error,ray,ray);

            if(!ret) {
                printf("Out at (%i,%i)\n",r,c);
                return ret;
            }
            (*lut)[r][c].x = ray[0];
            (*lut)[r][c].y = ray[1];
        }
        ray[0] = (*lut)[r][int_cy].x;
        ray[1] = (*lut)[r][int_cy].y;
    }

    // Lift center of projection
    mvl_camera_warped_2d_to_3d_gauss( fx,cx,fy,cy,sx,kappa1,kappa2,kappa3,tau1,tau2,
                                      image_pt,max_iter,max_col_error,max_row_error,ray,NULL );

    // Top left quadrant
    for(r=int_cx;r>=0;r--) {
        for(c=int_cy;c<image_width;c++) {
            image_pt[0] = c;
            image_pt[1] = r;

            ret = mvl_camera_warped_2d_to_3d_gauss( fx,cx,fy,cy,sx,kappa1,kappa2,kappa3,tau1,tau2,
                                                    image_pt,max_iter,max_col_error,max_row_error,ray,ray );
            if(!ret) {
                printf("Out at (%i,%i)\n",r,c);
                return ret;
            }
            (*lut)[r][c].x = ray[0];
            (*lut)[r][c].y = ray[1];
        }
        ray[0] = (*lut)[r][int_cy].x;
        ray[1] = (*lut)[r][int_cy].y;
    }

    return 1;
}

#endif // have netlib
