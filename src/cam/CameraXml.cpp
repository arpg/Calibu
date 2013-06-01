/* 
   This file is part of the Calibu Project.
   https://robotics.gwu.edu/git/calibu

   Copyright (C) 2013 George Washington University,
                      Steven Lovegrove,
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

#include <calibu/cam/CameraXml.h>
#include <calibu/utils/Xml.h>
#include <calibu/utils/StreamOperatorsEigen.h>
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

std::string CameraModelType( const std::string& sType )
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

void WriteXmlCameraModel(std::ostream& out, const CameraModelInterface& cam, int indent)
{
    const std::string dd1 = IndentStr(indent);
    const std::string dd2 = IndentStr(indent+4);
    
    out << dd1
        << "<" << NODE_CAMMODEL << " name=\"" << cam.Name() << "\" "
        << "index=\"" << cam.Index() << "\" "
        << "serialno=\"" << cam.SerialNumber() << "\" "
        << "type=\"" << cam.Type() << "\" "
        << "version=\"" << cam.Version() << "\">\n";
    
    out << dd2 << "<width> " << cam.Width() << " </width>\n";
    out << dd2 << "<height> " << cam.Height() << " </height>\n";

    // hmm, is RDF a model parameter or should it be outside thie model, like the pose is? GTS
    out << dd2 << "<!-- [right'; down'; forward'] camera convention -->\n";
    out << dd2 << "<RDF> " << cam.RDF() << " </RDF>\n";
    out.precision(7);
    
    out << dd2 << "<!-- Camera parameters ordered as per type name. -->\n";    
    out << dd2 << "<params> " << cam.GenericParams() << " </params>\n";
    
    out << dd1 << AttribClose(NODE_CAMMODEL) << std::endl;
}

void WriteXmlCameraModel(const std::string& filename, const CameraModelInterface& cam)
{
    std::ofstream of(filename);
    WriteXmlCameraModel(of, cam);
}

CameraModel ReadXmlCameraModel(TiXmlElement* pEl)
{    
    std::string sType = CameraModelType( pEl->Attribute("type"));
    
    CameraModel rCam( sType );
    if(rCam.IsInitialised()) {
        std::string sVer    = pEl->Attribute("version");
        std::string sName   = pEl->Attribute("name");
        std::string sIndex  = pEl->Attribute("index");
        std::string sSerial = pEl->Attribute("serialno");
        
        TiXmlElement* xmlp = pEl->FirstChildElement("params");
        TiXmlElement* xmlw = pEl->FirstChildElement("width");
        TiXmlElement* xmlh = pEl->FirstChildElement("height");
        TiXmlElement* xmlrdf = pEl->FirstChildElement("RDF");

        std::string sParams = xmlp ? xmlp->GetText() : "";
        std::string sWidth  = xmlw ? xmlw->GetText() : "";
        std::string sHeight = xmlh ? xmlh->GetText() : "";
        std::string sRdf = xmlrdf ? xmlrdf->GetText() : "";
        
        rCam.SetVersion( StrToVal<int>(sVer,0) );
        rCam.SetName( sName );
        rCam.SetIndex( StrToVal<int>(sIndex,0) );
        rCam.SetSerialNumber( StrToVal<int>(sSerial,-1) );
        rCam.SetGenericParams( StrToVal(sParams, rCam.GenericParams()) );
        rCam.SetImageDimensions( StrToVal<int>(sWidth), StrToVal<int>(sHeight) );
        rCam.SetRDF( StrToVal<Eigen::Matrix3d>(sRdf, RdfVision.matrix() ) );
    }
    
    return rCam;
}

CameraModel ReadXmlCameraModel(const std::string& filename)
{
    TiXmlDocument doc;
    if(doc.LoadFile(filename)) {
        TiXmlElement* pEl = doc.FirstChildElement(NODE_CAMMODEL);
        if(pEl) {
            return ReadXmlCameraModel(pEl);
        }
    }
    return CameraModel();
}

///////////////////////////////////////////////////////////////////////////////

void WriteXmlSE3(std::ostream& out, const Sophus::SE3d& T_wc, int indent)
{
    const std::string dd1 = IndentStr(indent);
    const std::string dd2 = IndentStr(indent+4);
    
    out << dd1 << AttribOpen(NODE_POSE) << std::endl;
    out << dd2 << "<!-- Camera pose. World from Camera point transfer. 3x4 matrix -->\n";
    out << dd2 << "<T_wc> " << T_wc.matrix3x4() << " </T_wc>\n";
    out << dd1 << AttribClose(NODE_POSE) << std::endl;
}

void WriteXmlSE3(const std::string& filename, const Sophus::SE3d& T_wc)
{
    std::ofstream of(filename);
    WriteXmlSE3(of, T_wc);
}

Sophus::SE3d ReadXmlSE3(TiXmlNode* xmlcampose)
{
    const std::string val = xmlcampose->FirstChildElement("T_wc")->GetText();
    Eigen::Matrix4d m = StrToVal<Eigen::Matrix4d>(val);
    return Sophus::SE3d(m);
}

Sophus::SE3d ReadXmlSE3(const std::string& filename)
{
    TiXmlDocument doc;
    if(doc.LoadFile(filename)) {
        TiXmlNode* pNode = doc.FirstChild(NODE_CAMMODEL);
        if(pNode) {
            return ReadXmlSE3(pNode);
        }
    }
    return Sophus::SE3d();
}

///////////////////////////////////////////////////////////////////////////////

void WriteXmlCameraModelAndPose(std::ostream& out, const CameraModelAndPose& cop, int indent)
{
    const std::string dd = IndentStr(indent);
    out << dd << AttribOpen(NODE_CAMMODEL_POSE) << std::endl;
    WriteXmlCameraModel(out, cop.camera, indent+4);
    WriteXmlSE3(out, cop.T_wc, indent+4);
    out << dd << AttribClose(NODE_CAMMODEL_POSE) << std::endl;
}

void WriteXmlCameraModelAndPose(const std::string& filename, const CameraModelAndPose& cop)
{
    std::ofstream of(filename);
    WriteXmlCameraModelAndPose(of, cop);
}


CameraModelAndPose ReadXmlCameraModelAndPose(TiXmlNode* xmlcampose)
{
    CameraModelAndPose cop;
    
    TiXmlElement* xmlcam  = xmlcampose->FirstChildElement(NODE_CAMMODEL);
    if(xmlcam) {
        cop.camera = ReadXmlCameraModel(xmlcam);
    }
    
    TiXmlNode* xmlpose = xmlcampose->FirstChild(NODE_POSE);
    if(xmlpose) {
        cop.T_wc = ReadXmlSE3(xmlpose);
    }   
 
    return cop;
}

CameraModelAndPose ReadXmlCameraModelAndPose(const std::string& filename)
{
    TiXmlDocument doc;
    if(doc.LoadFile(filename)) {
        TiXmlNode* pNode = doc.FirstChild(NODE_CAMMODEL_POSE);
        if(pNode) {
            return ReadXmlCameraModelAndPose(pNode);
        }
    }
    return CameraModelAndPose();
}


///////////////////////////////////////////////////////////////////////////////

void WriteXmlRig(std::ostream& out, const CameraRig& rig, int indent )
{
    const std::string dd = IndentStr(indent);
    
    out << dd << AttribOpen(NODE_RIG) << std::endl;    
    for(const CameraModelAndPose& cop : rig.cameras) {
        WriteXmlCameraModelAndPose(out, cop, indent + 4);
    }
    out << dd << AttribClose(NODE_RIG) << std::endl;
}

void WriteXmlRig(const std::string& filename, const CameraRig& rig)
{
    std::ofstream of(filename);
    WriteXmlRig(of, rig);
}


CameraRig ReadXmlRig(TiXmlNode* xmlrig)
{
    CameraRig rig;
    
    for( TiXmlNode* child = xmlrig->FirstChild(NODE_CAMMODEL_POSE); child; child = child->NextSibling(NODE_CAMMODEL_POSE) )
    {
        CameraModelAndPose cap = ReadXmlCameraModelAndPose(child);
        if(cap.camera.IsInitialised()) {
            rig.cameras.push_back(cap);
        }
    }    
    
    return rig;
}

CameraRig ReadXmlRig(const std::string& filename)
{
    TiXmlDocument doc;
    if(doc.LoadFile(filename)) {
        TiXmlNode* pNode = doc.FirstChild(NODE_RIG);
        if(pNode) {
            return ReadXmlRig(pNode);
        }
    }else{
        std::cerr << doc.ErrorDesc() << std::endl;
    }
    return CameraRig();
}

}
