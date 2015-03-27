#include <stdio.h>
#include <HAL/Utils/GetPot>
#include <HAL/Posys/PosysDevice.h>
#include <HAL/Camera/CameraDevice.h>
#include <PbMsgs/ImageArray.h>
#include <eigen3/Eigen/Eigen>

typedef Eigen::Matrix< double, 6, 1 > Vector6d;

class PoseHandler{
public:
  PoseHandler( void ) {}

  void printdata( pb::PoseMsg& pose_data_  )
  {
    Vector6d pose;
    pose << pose_data_.pose().data(0), pose_data_.pose().data(1), pose_data_.pose().data(2),
            pose_data_.pose().data(3), pose_data_.pose().data(4), pose_data_.pose().data(5);
    if (poses.size() == 5000) {
      poses.pop_front();
      times.pop_front();
    }
    poses.push_back(pose);
    times.push_back( pose_data_.system_time());
  }

  void init( std::string URI )
  {
    vicon = new hal::Posys(URI);
    vicon->RegisterPosysDataCallback(std::bind(&PoseHandler::printdata, this, std::placeholders::_1));
  }

  void clear( void )
  {
    poses.clear();
    times.clear();
  }

  Vector6d get_pose( double time )
  {
    if (times.size() == 0)  {
      Vector6d p;
      std::cout << "No poses to check against."<<std::endl;
      p << 0, 0, 0, 0, 0, 0;
      return p;
    }
    long id = 0;
    while ((times[id] < time) && (id < times.size())) { id++; }
    if (id != 0) id--;
    if (id < times.size() - 1) {
      double t1 = time - times[id];
      double dt = times[id + 1] - times[id];
      dt = t1 / dt;
      std::cout << "[" << id << "]: " << std::fixed << times[id] << " -- " << time << " -- " << times[id + 1] << std::endl;
      return (1 - dt) * poses[id] + dt * poses[id + 1];
    }
    else {
      std::cout << "[" << id << "]: " << std::fixed << times[id] << " -- " << time << std::endl;
      return poses[id];
    }
  }

private:
  std::deque< Vector6d > poses;
  std::deque< double >   times;
  hal::Posys* vicon;
};

int main( int argc, char** argv )
{  
  GetPot cl(argc, argv);
  hal::Camera* cam = 0;
  PoseHandler ph;
  if (cl.search("-posys")) {
    ph.init(cl.follow("", "-posys"));
  }

  if (cl.search("-cam")) {
    cam = new hal::Camera(cl.follow("", "-cam"));
  }


  std::shared_ptr< pb::ImageArray > images_ = pb::ImageArray::Create();

  FILE* f = fopen("poses.txt", "w");

  int frame = 0;
  if (cam) {
    while ( cam->Capture(*images_) ) {
      frame++;
      double time = images_->system_time();
      Vector6d p = ph.get_pose(time);
      fprintf(f, "%f\t%f\t%f\t%f\t%f\t%f\n", p(0), p(1), p(2), p(3), p(4), p(5));
    }
  }
  fprintf(stderr, "Read %d frames\n", frame);
  fclose(f);
  return 0;
}
