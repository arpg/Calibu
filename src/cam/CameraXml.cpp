/*
   This file is part of the Calibu Project.
   https://github.com/arpg/Calibu

   Copyright (C) 2015 University of Colorado at Boulder,
                      Steven Lovegrove,
                      Christoffer Heckman,
                      Gabe Sibley

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

#include <tinyxml2.h>
#include <calibu/cam/camera_xml.h>
#include <calibu/cam/camera_crtp.h>
#include <calibu/cam/camera_crtp_impl.h>
#include <calibu/cam/camera_models_crtp.h>
#include <fstream>

namespace calibu
{

///////////////////////////////////////////////////////////////////////////////

std::string IndentStr(int indent)
{
  return std::string(indent, ' ');
}

std::string AttribOpen(const std::string& attrib)
{
  return "<" + attrib + ">";
}

std::string AttribClose(const std::string& attrib)
{
  return "</" + attrib + ">";
}

///////////////////////////////////////////////////////////////////////////////

std::string CameraType( const std::string& sType )
{
  // Translate non-standard types
  if( sType == "MVL_CAMERA_WARPED" ) {
    return "calibu_fu_fv_u0_v0_k1_k2";
  }
  else if( sType == "MVL_CAMERA_LINEAR" ) {
    return "calibu_fu_fv_u0_v0";
  }
  else if( sType == "MVL_CAMERA_LUT" ) {
    return "calibu_lut";
  }
  else {
    return sType;
  }
}

void WriteXmlCamera(std::ostream& out,
                    const std::shared_ptr<CameraInterfaced> cam,
                    int indent)
{
  const std::string dd1 = IndentStr(indent);
  const std::string dd2 = IndentStr(indent+4);

  out << dd1
      << "<" << NODE_CAM << " name=\"" << cam->Name() << "\" "
      << "index=\"" << cam->Index() << "\" "
      << "serialno=\"" << cam->SerialNumber() << "\" "
      << "type=\"" << cam->Type() << "\" "
      << "version=\"" << cam->Version() << "\">\n";

  out << dd2 << "<width> " << cam->Width() << " </width>\n";
  out << dd2 << "<height> " << cam->Height() << " </height>\n";

  // hmm, is RDF a model parameter or should it be outside thie model, like the pose is? GTS
  out << dd2 << "<!-- Use RDF matrix, [right down forward], to define the coordinate frame convention -->\n";
  out << dd2 << "<right> " << (Eigen::Vector3d)cam->RDF().col(0) << " </right>\n";
  out << dd2 << "<down> " << (Eigen::Vector3d)cam->RDF().col(1) << " </down>\n";
  out << dd2 << "<forward> " << (Eigen::Vector3d)cam->RDF().col(2) << " </forward>\n";
  out.precision(7);

  out << dd2 << "<!-- Camera parameters ordered as per type name. -->\n";
  out << dd2 << "<params> " << (Eigen::VectorXd)cam->GetParams() << " </params>\n";

  out << dd1 << AttribClose(NODE_CAM) << std::endl;
}

void WriteXmlCamera(const std::string& filename,
                    const std::shared_ptr<CameraInterfaced> cam)
{
  std::ofstream of(filename);
  int indent = 0; // added for crtp classes.
  WriteXmlCamera(of, cam, indent);
}

std::shared_ptr<CameraInterfaced> ReadXmlCamera(tinyxml2::XMLElement* pEl)
{    
  std::string sType = CameraType( pEl->Attribute("type"));
  std::shared_ptr<CameraInterfaced> rCam;

    if (sType == "calibu_fu_fv_u0_v0_w") {
    rCam.reset(new calibu::FovCamera<double>());
    rCam->SetParams(Eigen::VectorXd::Constant(FovCamera<double>::NumParams,1));
  } else if (sType == "calibu_fu_fv_u0_v0") {
    rCam.reset(new calibu::LinearCamera<double>());
    rCam->SetParams(Eigen::VectorXd::Constant(LinearCamera<double>::NumParams,1));
  } else if (sType == "calibu_fu_fv_u0_v0_k1_k2") {
    rCam.reset(new calibu::Poly2Camera<double>());
    rCam->SetParams(Eigen::VectorXd::Constant(Poly2Camera<double>::NumParams,1));
  } else if (sType == "calibu_fu_fv_u0_v0_kb4") {
    rCam.reset(new calibu::KannalaBrandtCamera<double>());
    rCam->SetParams(Eigen::VectorXd::Constant(KannalaBrandtCamera<double>::NumParams,1));
  } else if (sType == "calibu_fu_fv_u0_v0_k1_k2_k3") {
    rCam.reset(new calibu::Poly3Camera<double>());
    rCam->SetParams(Eigen::VectorXd::Constant(Poly3Camera<double>::NumParams,1));
  } else if (sType == "calibu_fu_fv_u0_v0_rational6") {
	rCam.reset(new calibu::Rational6Camera<double>());
	rCam->SetParams(Eigen::VectorXd::Constant(Rational6Camera<double>::NumParams,1));
  } else {
    std::cerr << "Unknown old camera type " << sType << " please implement this"
                 " camera before initializing it. " << std::endl;
    throw 0;
  }

  if(rCam->IsInitialized()) {
    std::string sVer    = pEl->Attribute("version");
    std::string sName   = pEl->Attribute("name");
    std::string sIndex  = pEl->Attribute("index");
    std::string sSerial = pEl->Attribute("serialno");

    tinyxml2::XMLElement* xmlp = pEl->FirstChildElement("params");
    tinyxml2::XMLElement* xmlw = pEl->FirstChildElement("width");
    tinyxml2::XMLElement* xmlh = pEl->FirstChildElement("height");
    tinyxml2::XMLElement* xmlRight = pEl->FirstChildElement("right");
    tinyxml2::XMLElement* xmlDown = pEl->FirstChildElement("down");
    tinyxml2::XMLElement* xmlForward = pEl->FirstChildElement("forward");

    std::string sParams = xmlp ? xmlp->GetText() : "";
    std::string sWidth  = xmlw ? xmlw->GetText() : "";
    std::string sHeight = xmlh ? xmlh->GetText() : "";
    std::string sR = xmlRight ? xmlRight->GetText() : "[ 1; 0; 0 ]";
    std::string sD = xmlDown ? xmlDown->GetText() : "[0; 1; 0 ]";
    std::string sF = xmlForward ? xmlForward->GetText() : "[ 0; 0; 1 ]";

    rCam->SetVersion( StrToVal<int>(sVer,0) );
    rCam->SetIndex( StrToVal<int>(sIndex,0) );
    rCam->SetSerialNumber( StrToVal<long int>(sSerial,-1) );
    rCam->SetParams( StrToVal<Eigen::VectorXd>(sParams, rCam->GetParams()) );
    rCam->SetImageDimensions( StrToVal<int>(sWidth), StrToVal<int>(sHeight) );
    rCam->SetName(sName);
    rCam->SetType(sType);
    Eigen::Matrix3d rdf;
    rdf.col(0) = StrToVal<Eigen::Vector3d>(sR);
    rdf.col(1) = StrToVal<Eigen::Vector3d>(sD);
    rdf.col(2) = StrToVal<Eigen::Vector3d>(sF);
    rCam->SetRDF( rdf );
  }

  return rCam;
}

std::shared_ptr<CameraInterfaced> ReadXmlCamera(const std::string& filename)
{
  tinyxml2::XMLDocument doc;
  if(tinyxml2::XML_NO_ERROR == doc.LoadFile(filename.c_str())) {
    tinyxml2::XMLElement* pEl = doc.FirstChildElement(NODE_CAM.c_str());
    if(pEl) {
      return ReadXmlCamera(pEl);
    }
  }
  return std::shared_ptr<CameraInterfaced>(NULL);
}

///////////////////////////////////////////////////////////////////////////////

void WriteXmlSE3(std::ostream& out, const Sophus::SE3d& t_rc, int indent)
{
  const std::string dd1 = IndentStr(indent);
  const std::string dd2 = IndentStr(indent+4);

  out << dd1 << AttribOpen(NODE_POSE) << std::endl;
  out << dd2 << "<!-- Camera pose. World from Camera point transfer. 3x4 matrix, in the RDF frame convention defined above -->\n";
  /// Actually now T_rc (rig->camera), but preserving backward compatibility.
  out << dd2 << "<T_wc> " << t_rc.matrix3x4() << " </T_wc>\n";
  out << dd1 << AttribClose(NODE_POSE) << std::endl;
}

void WriteXmlSE3(const std::string& filename, const Sophus::SE3d& t_rc)
{
  std::ofstream of(filename);
  WriteXmlSE3(of, t_rc);
}

Sophus::SE3d ReadXmlSE3(tinyxml2::XMLNode* xmlcampose)
{
  /// Actually now T_rc (rig->camera), but preserving backward compatibility.
  const std::string val = xmlcampose->FirstChildElement("T_wc")->GetText();
  Eigen::Matrix4d m = StrToVal<Eigen::Matrix4d>(val);
  return Sophus::SE3d(m);
}

Sophus::SE3d ReadXmlSE3(const std::string& filename)
{
  tinyxml2::XMLDocument doc;
  if(tinyxml2::XML_NO_ERROR == doc.LoadFile(filename.c_str())) {
    tinyxml2::XMLNode* pNode = doc.FirstChildElement(NODE_POSE.c_str());
    if(pNode) {
      return ReadXmlSE3(pNode);
    }
  }
  return Sophus::SE3d();
}

///////////////////////////////////////////////////////////////////////////////

void WriteXmlCameraAndTransform(
    std::ostream& out, const std::shared_ptr<CameraInterfaced> cop, int indent)
{
  const std::string dd = IndentStr(indent);
  out << dd << AttribOpen(NODE_CAM_POSE) << std::endl;
  WriteXmlCamera(out, cop, indent+4);
  WriteXmlSE3(out, cop->Pose(), indent+4);
  out << dd << AttribClose(NODE_CAM_POSE) << std::endl;
}

void WriteXmlCameraAndTransform(const std::string& filename,
                                const std::shared_ptr<CameraInterfaced> cop)
{
  std::ofstream of(filename);
  int indent = 0; // added for crtp classes.
  WriteXmlCameraAndTransform(of, cop, indent);
}

std::shared_ptr<CameraInterfaced> ReadXmlCameraAndTransform(tinyxml2::XMLNode* xmlcampose)
{
  std::shared_ptr<CameraInterfaced> cop;

  tinyxml2::XMLElement* xmlcam  = xmlcampose->FirstChildElement(NODE_CAM.c_str());
  if(xmlcam) {
    cop = ReadXmlCamera(xmlcam);
  }

  tinyxml2::XMLNode* xmlpose = xmlcampose->FirstChildElement(NODE_POSE.c_str());
  if(xmlpose) {
    cop->SetPose(ReadXmlSE3(xmlpose));
  }

  return cop;
}

std::shared_ptr<CameraInterfaced> ReadXmlCameraAndTransform(
    const std::string& filename)//, const std::shared_ptr<CameraInterfaced> cam)
{
  tinyxml2::XMLDocument doc;
  if(tinyxml2::XML_NO_ERROR == doc.LoadFile(filename.c_str())) {
    tinyxml2::XMLNode* pNode = doc.FirstChildElement(NODE_CAM_POSE.c_str());
    if(pNode) {
      return ReadXmlCameraAndTransform(pNode);
    }
  }
  return std::shared_ptr<CameraInterfaced>(NULL);
}


///////////////////////////////////////////////////////////////////////////////

void WriteXmlRig(std::ostream& out, const std::shared_ptr<Rigd> rig, int indent )
{
  const std::string dd = IndentStr(indent);

  out << dd << AttribOpen(NODE_RIG) << std::endl;
  for(const std::shared_ptr<CameraInterfaced>	cop : rig->cameras_) {
    WriteXmlCameraAndTransform(out, cop, indent + 4);
  }
  out << dd << AttribClose(NODE_RIG) << std::endl;
}

void WriteXmlRig(const std::string& filename, const std::shared_ptr<Rigd> rig)
{
  std::ofstream of(filename);
  WriteXmlRig(of, rig, 0);
}

std::shared_ptr<Rigd> ReadXmlRig(tinyxml2::XMLNode* xmlrig)
{
  std::shared_ptr<Rigd> rig(new Rigd());

  for( tinyxml2::XMLNode* child = xmlrig->FirstChildElement(NODE_CAM_POSE.c_str()); child; child = child->NextSiblingElement(NODE_CAM_POSE.c_str()) )
  {
    std::shared_ptr<CameraInterfaced> cap = ReadXmlCameraAndTransform(child);
    if(cap->IsInitialized()) {
      rig->AddCamera(cap);
    }
  }

  return rig;
}

std::shared_ptr<Rigd> ReadXmlRig(const std::string& filename)//, const std::shared_ptr<Rigd> rig)
{
  tinyxml2::XMLDocument doc;
  if(tinyxml2::XML_NO_ERROR == doc.LoadFile(filename.c_str())) {
    tinyxml2::XMLNode* pNode = doc.FirstChildElement(NODE_RIG.c_str());
    if(pNode) {
      return ReadXmlRig(pNode);
    }
  }else{
    std::cerr << doc.GetErrorStr1() << ": '" << filename << "'" << std::endl;
  }
  return std::shared_ptr<Rigd>(NULL);
}
  
std::shared_ptr<Rigd> ReadXmlRigFromString(const std::string& modelXML)
{
  tinyxml2::XMLDocument doc;
  if(tinyxml2::XML_NO_ERROR == doc.Parse(modelXML.c_str())) {
    tinyxml2::XMLNode* pNode = doc.FirstChildElement(NODE_RIG.c_str());
    if(pNode) {
      return ReadXmlRig(pNode);
    }
  }else{
    std::cerr << doc.GetErrorStr1() << ": Error parsing XML from string: '" << modelXML << "'" << std::endl;
  }
  return std::shared_ptr<Rigd>(NULL);
}
  
}
