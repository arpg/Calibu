#include <iostream>
#include <vector>

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <calibu/cam/CameraXml.h>
#include <calibu/cam/CameraModelT.h>

#include <calibu/cam/camera_crtp.h>
#include <calibu/cam/camera_models_crtp.h>
#include <calibu/cam/camera_crtp_interop.h>

#include <sophus/se3.hpp>
#include <ceres/ceres.h>

#include "ceres_cost_functions.h"

DEFINE_string(cmod,
    "-cmod",
    "calibu camera model xml file.");

DEFINE_string(o,
    "-o",
    "Output file name.");

namespace Eigen
{
  typedef Matrix<double,6,1>  Vector6d;
}

using namespace std;
using namespace ceres;
using namespace Eigen;

/////////////////////////////////////////////////////////////////////////
class LocalizationProblem 
{
  public:
    int num_observations() const 
    {
      return num_observations_;               
    }

    const Eigen::Vector6d& pose_for_observation( int i )
    {
      return poses_[pose_index_[i]];
    }

    double* pose_data_for_observation( int i ) 
    {
      return poses_[pose_index_[i]].data();
    }

    const Eigen::Vector3d& landmark_for_observation(int i) {
      return landmarks_[landmark_index_[i]];
    }

    const Eigen::Vector2d& observation(int i) {
      return observations_[i];
    }

    void PrintData( void )
    {
      fprintf( stdout, "Number of poses loaded: %d\n", num_poses_ );
      fprintf( stdout, "Number of landmarks: %d\n", num_landmarks_ );
      fprintf( stdout, "Number of observations: %d\n", num_observations_ );

      num_parameters_ = 6 * num_poses_; // just a localization problem

      for (int i = 0; i < num_observations_; ++i) {
        fprintf( stdout, "%d, %d, %lf, %lf\n",
                 pose_index_[i], landmark_index_[i], observations_[i](0), observations_[i](1) );
    }

      // poses
      for (int i = 0; i < num_poses_; ++i) {
        fprintf( stdout, "%lf, %lf, %lf, %lf, %lf, %lf\n",
                 poses_[i](0), poses_[i](1), poses_[i](2), poses_[i](3), poses_[i](4), poses_[i](5));
      }

      // landmarks
      for (int i = 0; i < num_landmarks_; ++i) {
        fprintf( stdout, "%d, %lf, %lf, %lf\n", i, landmarks_[i](0), landmarks_[i](1), landmarks_[i](2));
      }

    }

    bool LoadFile( const char* filename )
    {
      FILE* fptr = fopen(filename, "r");
      if (fptr == NULL) {
        return false;
      };

      fscanf( fptr, "%d", &num_poses_ );
      fscanf( fptr, "%d", &num_landmarks_ );
      fscanf( fptr, "%d", &num_observations_ );
      fscanf( fptr, "%d", &num_tags_);

      landmark_index_.resize( num_observations_ );
      pose_index_.resize( num_observations_ );
      observations_.resize( num_observations_ );
      poses_.resize( num_poses_ );
      landmarks_.resize( num_landmarks_ );

      num_parameters_ = 6 * num_poses_; // just a localization problem

      for (int i = 0; i < num_observations_; ++i) {
        double u, v;
        int n = fscanf( fptr, "%d, %d, %lf, %lf",
            &pose_index_[i], &landmark_index_[i], &u, &v );
        if( n != 4 ){
          LOG(FATAL) << "Error reading measurement " << i << ".";
        }


        observations_[i] = Eigen::Vector2d(u,v);
    }

      // poses
      for (int i = 0; i < num_poses_; ++i) {
        double x,y,z,p,q,r;
        int n = fscanf( fptr, "%lf, %lf, %lf, %lf, %lf, %lf", 
            &x, &y , &z , &p , &q , &r );
        if( n != 6 ){
          LOG(FATAL) << "Error reading pose " << i << ".";
        }
        poses_[i] << x, y, z, p, q, r;
      }

      // landmarks
      for (int i = 0; i < num_landmarks_; ++i) {
        double x,y,z;
        int uid;
        int n = fscanf( fptr, "%d, %lf, %lf, %lf", &uid, &x, &y , &z );
        if( n != 4 ){
          LOG(FATAL) << "Error reading landmark " << i << ".";
        }
        landmarks_[i] << x, y, z;
      }
      PrintData();
      return true;
    }

  private:

    int num_poses_;
    int num_landmarks_;
    int num_observations_;
    int num_tags_;
    int num_parameters_;

    std::vector<int> landmark_index_;
    std::vector<int> pose_index_;
    std::vector<Eigen::Vector6d> poses_; // parameters
    std::vector<Eigen::Vector3d> landmarks_;
    std::vector<Eigen::Vector2d> observations_;
};


/////////////////////////////////////////////////////////////////////////
int main( int argc, char** argv )
{
  google::InitGoogleLogging(argv[0]);

  if ((argc < 3) || (argc > 4)) {
    std::cerr << "usage: gettruth <measurement_file> <camera file> [<output file>]\n";
    return 1;
  }

  FILE* file;
  if (argc == 3) {
    file = stdout;
  } else {
    file = fopen(argv[3], "w");
  }

  // get camera model
  calibu::Rig<double> rig;
  calibu::LoadRig( std::string(argv[2]), &rig );
  if (rig.cameras_.size() == 0) {
    fprintf(stderr, "No cameras in this rig or no camera file provided\n");
    exit(0);
  }
  calibu::CameraInterface<double>* cam = rig.cameras_[0];

  LocalizationProblem ctx;
  if (!ctx.LoadFile(argv[1])) {
    std::cerr << "ERROR: unable to open file " << argv[1] << "\n";
    return 1;
  }

  // Build the problem.
  Problem problem;

  for (int i = 0; i < ctx.num_observations(); ++i) {
    const Vector2d& measurement = ctx.observation(i);
    const Vector3d& landmark = ctx.landmark_for_observation(i);
    ceres::CostFunction* cost_function = ProjectionCost( landmark, measurement, cam );
    problem.AddResidualBlock( cost_function, NULL, ctx.pose_data_for_observation(i) );
  }

  fprintf(stdout, "\n\nThe problem has been set up with %d residuals and %d residual blocks\n", problem.NumResiduals(), problem.NumResidualBlocks());

  // Make Ceres automatically detect the bundle structure. Note that the
  // standard solver, SPARSE_NORMAL_CHOLESKY, also works fine but it is slower
  // for standard bundle adjustment problems.
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_SCHUR;
  options.minimizer_progress_to_stdout = true;  

  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  for (int ii = 0; ii < ctx.num_observations(); ii += 4) {
    fprintf(file, "%f, %f, %f, %f, %f, %f\n", ii + 1, ctx.pose_for_observation(ii)(0), ctx.pose_for_observation(ii)(1),
            ctx.pose_for_observation(ii)(2), ctx.pose_for_observation(ii)(3), ctx.pose_for_observation(ii)(4), ctx.pose_for_observation(ii)(5));
  }
  fclose(file);

  return 0;
}


