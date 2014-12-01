#include <iomanip>
#include <unistd.h>

#include <miniglog/logging.h>
#include <opencv.hpp>

#include <PbMsgs/Image.h>
#include <PbMsgs/Logger.h>
#include <PbMsgs/Reader.h>
#include <CVars/CVar.h>

#include <HAL/Camera/CameraDevice.h>
#include <HAL/Utils/Uri.h>
#include <calibu/image/ImageProcessing.h>
#include <calibu/cam/CameraRig.h>
#include <calibu/cam/CameraXml.h>
#include <calibu/cam/Rectify.h>

#include "GetPot.cpp"

using namespace hal;
using namespace std;
using namespace calibu;

const char* sUriInfo =
"Usage:"
"\trectify <options>\n"
"Options:\n"
"\t-input, -i <file      Input logfile file to read unrectified images from\n"
"\t-output,-o <file>        Output logfile file to write rectified images, default: rectify.log" 
  "\t-cmod,-cam <file>       Input XML file to read camera model from.\n";



int main(int argc, char** argv)
{
  //Goal: Open a given logfile with camera parameters, run through the HAL Rectify driver
  //      Save imagery to a new logfile

    ////////////////////////////////////////////////////////////////////
    // Create command line options. Check if we should print usage.

    GetPot cl(argc,argv);

    if(cl.search(3, "-help", "-h", "?") || argc < 2) {
        std::cout << sUriInfo << std::endl;
        return -1;
    }

    ////////////////////////////////////////////////////////////////////
    // Default configuration values
    std::string src_log = cl.follow("", 2, "-i", "-input");
    if (src_log.empty())
      {
	cerr << "Please provide a source log to read from" << endl;
	return -1;
      }
    std::string dest_log = cl.follow("", 2, "-o", "-output");
    if (dest_log.empty())
      {
	dest_log = "rectified.log";
      }

    ////////////////////////////////////////////////////////////////////
    // Setup Video Source
    std::string cam_model = cl.follow("", 2, "-cmod", "-cam");
    if (cam_model.empty())
      {
	std::cerr << "Please provide a camera model XML file: "
                    << std::endl;
        return -1;
      }

    std::string filename = ExpandTildePath(cam_model);
 
    if(!FileExists(filename))
      {
	std::cerr << "Please provide a camera model XML file, " << filename << " not found"
		  << std::endl;
        return -1;
      }

    CameraRig rig = ReadXmlRig( filename );
    cout << "Found " << rig.cameras.size() << " camera models" << std::endl;
    cout << "Writing to " << dest_log << endl;

    calibu::LookupTable m_vLut;
    const calibu::CameraModel& cam = rig.cameras[0].camera;
    m_vLut = calibu::LookupTable(cam.Width(), cam.Height());

    // From HAL/UndistortDriver:
    // Setup new camera model
    // For now, assume no change in scale so return same params with
    // no distortion.
    calibu::CameraModelT<calibu::Pinhole> new_cam(cam.Width(), cam.Height());
    new_cam.Params() << cam.K()(0,0), cam.K()(1,1), cam.K()(0,2), cam.K()(1,2);
    calibu::CreateLookupTable(rig.cameras[0].camera, new_cam.Kinv(), m_vLut);

    pb::Logger logger;
    logger.LogToFile(dest_log);

    pb::Reader reader(src_log);
    reader.EnableAll();
    std::unique_ptr<pb::Msg> msg;
    while ((msg = reader.ReadMessage()))
      {
	//Pass through all messages if they're not camera messages directly
	if (msg->has_camera())
	  {
	    //cout << "Got camera msg" << endl;
	    //Rectify all camera images
	    const pb::CameraMsg& cam_msg = msg->camera();
	    pb::CameraMsg vImages;
	    for (int ii = 0; ii < cam_msg.image_size(); ++ii)
	      {
		const pb::ImageMsg& img_msg = cam_msg.image(ii);
		pb::Image inimg = pb::Image(cam_msg.image(ii));
	
		pb::ImageMsg* pimg = vImages.add_image();
		pimg->set_width(inimg.Width());
		pimg->set_height(inimg.Height());
		pimg->set_type( (pb::Type)inimg.Type());
		pimg->set_format( (pb::Format)inimg.Format());
		pimg->mutable_data()->resize(inimg.Width()*inimg.Height());
		pb::Image img = pb::Image(*pimg);

		calibu::Rectify(	m_vLut, inimg.data(),
					reinterpret_cast<unsigned char*>(&pimg->mutable_data()->front()),
					img.Width(), img.Height());
	      }
	    msg->mutable_camera()->Swap(&vImages);
	    logger.LogMessage(*msg);
	  }
	else
	  {
	    logger.LogMessage(*msg);
	  }
      }

    cout << "Shutting down" << endl;
    logger.StopLogging();

}

