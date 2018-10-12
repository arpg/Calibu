#include <calibu/pcalib/pcalib_xml.h>
#include <cmath>
#include <iostream>
#include <calibu/pcalib/base64.h>
#include <calibu/pcalib/pcalib.h>
#include <calibu/pcalib/response_linear.h>
#include <calibu/pcalib/response_poly.h>
#include <calibu/pcalib/vignetting_dense.h>
#include <calibu/pcalib/vignetting_poly.h>
#include <calibu/pcalib/vignetting_uniform.h>

#define NUM_SHORT_PARAMS 25

namespace calibu
{

////////////////////////////////////////////////////////////////////////////////

PhotoRigReader::PhotoRigReader(const std::string& filename) :
  filename_(filename),
  root_(nullptr)
{
}

void PhotoRigReader::Read(PhotoRigd& rig)
{
  rig.Clear();
  PrepareRead();
  ReadCameras(rig);
  FinishRead();
}

void PhotoRigReader::PrepareRead()
{
  document_.LoadFile(filename_.c_str());
  root_ = document_.FirstChildElement("pcalib");
  CALIBU_ASSERT_DESC(root_, "missing 'pcalib' element");
}

void PhotoRigReader::ReadCameras(PhotoRigd& rig)
{
  // get first camera
  const tinyxml2::XMLElement* child = root_->FirstChildElement("camera");

  // process until no more cameras
  while (child)
  {
    // process current resposne
    std::shared_ptr<PhotoCamera<double>> camera = ReadCamera(child);
    if (camera) rig.cameras.push_back(camera);

    // get next camera
    child = child->NextSiblingElement("camera");
  }
}

std::shared_ptr<PhotoCamerad> PhotoRigReader::ReadCamera(
    const tinyxml2::XMLElement* element)
{
  std::shared_ptr<PhotoCamerad> camera;
  camera = std::make_shared<PhotoCamerad>();
  ReadResponses(element, *camera);
  ReadVignettings(element, *camera);
  return camera;
}

void PhotoRigReader::ReadResponses(const tinyxml2::XMLElement* parent,
    PhotoCamerad& camera)
{
  // get first response
  const tinyxml2::XMLElement* child = parent->FirstChildElement("response");

  // process until no more responses
  while (child)
  {
    // process current resposne
    std::shared_ptr<Response<double>> response = ReadResponse(child);
    if (response) camera.responses.push_back(response);

    // get next response
    child = child->NextSiblingElement("response");
  }
}

void PhotoRigReader::ReadVignettings(const tinyxml2::XMLElement* parent,
    PhotoCamerad& camera)
{
  // get first vignetting
  const tinyxml2::XMLElement* child = parent->FirstChildElement("vignetting");

  // process until no more vignettings
  while (child)
  {
    // process current vignetting
    std::shared_ptr<Vignetting<double>> vignetting = ReadVignetting(child);
    if (vignetting) camera.vignettings.push_back(vignetting);

    // get next vignetting
    child = child->NextSiblingElement("vignetting");
  }
}

std::shared_ptr<Response<double> > PhotoRigReader::ReadResponse(
    const tinyxml2::XMLElement* element)
{
  using namespace tinyxml2;
  std::shared_ptr<Response<double>> response;

  //read type
  const std::string type(element->Attribute("type"));
  response = CreateResponse(type);

  // read range
  Eigen::MatrixXd range;
  range = response->GetRange();

  const XMLElement* range_elem = element->FirstChildElement("range");
  CALIBU_ASSERT_DESC(range_elem, "response missing range element");
  GetMatrix(range_elem->GetText(), range);
  response->SetRange(range);

  // read params
  Eigen::MatrixXd params;
  params = response->GetParams();

  const XMLElement* params_elem = element->FirstChildElement("params");
  if (params_elem) GetMatrix(params_elem->GetText(), params);
  response->SetParams(params);

  return response;
}

std::shared_ptr<Vignetting<double>> PhotoRigReader::ReadVignetting(
    const tinyxml2::XMLElement* element)
{
  using namespace tinyxml2;
  std::shared_ptr<Vignetting<double>> vignetting;

  // read size
  Eigen::MatrixXd size = Eigen::Vector2d(0, 0);
  const XMLElement* size_elem = element->FirstChildElement("size");
  CALIBU_ASSERT_DESC(size_elem, "vignetting missing size element");

  GetMatrix(size_elem->GetText(), size);
  const int w = size(0, 0);
  const int h = size(1, 0);

  CALIBU_ASSERT_DESC(w > 0 && h > 0, "invalid vignetting size");

  // read type
  const std::string type(element->Attribute("type"));
  vignetting = CreateVignetting(type, w, h);

  // read params
  Eigen::MatrixXd params;
  params = vignetting->GetParams();

  const XMLElement* params_elem = element->FirstChildElement("params");
  if (params_elem) GetMatrix(params_elem->GetText(), params);
  vignetting->SetParams(params);

  return vignetting;
}

void PhotoRigReader::FinishRead()
{
  document_.Clear();
  root_ = nullptr;
}

std::shared_ptr<Response<double>> PhotoRigReader::CreateResponse(
    const std::string& type)
{
  if (type.compare(Poly3Response<double>::type) == 0)
  {
    return std::make_shared<Poly3Response<double>>();
  }
  else if (type.compare(Poly4Response<double>::type) == 0)
  {
    return std::make_shared<Poly4Response<double>>();
  }
  else if (type.compare(LinearResponse<double>::type) == 0)
  {
    return std::make_shared<LinearResponse<double>>();
  }

  CALIBU_THROW("unsupported response type: " + type);
}

std::shared_ptr<Vignetting<double>> PhotoRigReader::CreateVignetting(
    const std::string& type, int width, int height)
{
  if (type.compare(DenseVignetting<double>::type) == 0)
  {
    return std::make_shared<DenseVignetting<double>>(width, height);
  }
  else if (type.compare(EvenPoly6Vignetting<double>::type) == 0)
  {
    return std::make_shared<EvenPoly6Vignetting<double>>(width, height);
  }
  else if (type.compare(UniformVignetting<double>::type) == 0)
  {
    return std::make_shared<UniformVignetting<double>>(width, height);
  }

  CALIBU_THROW("unsupported vignetting type: " + type);
}

void PhotoRigReader::GetMatrix(const char* text, Eigen::MatrixXd& matrix)
{
  (matrix.size() > NUM_SHORT_PARAMS) ?
      GetLongMatrix(text, matrix) : GetShortMatrix(text, matrix);
}

void PhotoRigReader::GetShortMatrix(const char* text, Eigen::MatrixXd& matrix)
{
  char token;
  std::stringstream tokens(text);

  tokens >> token;

  if (token != '[')
  {
    std::cerr << "invalid matrix data" << std::endl;
  }

  for (int row = 0; row < matrix.rows(); ++row)
  {
    for (int col = 0; col < matrix.cols(); ++col)
    {
      tokens >> matrix(row, col);

      if (col < matrix.cols() - 1)
      {
        tokens >> token;

        if (token != ',')
        {
          std::cerr << "invalid matrix data" << std::endl;
        }
      }
    }

    if (row < matrix.rows() - 1)
    {
      tokens >> token;

      if (token != ';')
      {
        std::cerr << "invalid matrix data" << std::endl;
      }
    }
  }

  tokens >> token;

  if (token != ']')
  {
    std::cerr << "invalid matrix data" << std::endl;
  }
}

void PhotoRigReader::GetLongMatrix(const char* text, Eigen::MatrixXd& matrix)
{
  Base64Decoder decoder(text);
  decoder >> matrix;
}

////////////////////////////////////////////////////////////////////////////////

PhotoRigWriter::PhotoRigWriter(const std::string& filename) :
  filename_(filename)
{
}

void PhotoRigWriter::Write(const PhotoRigd& rig)
{
  PrepareWrite();
  WriteCameras(rig);
  FinishWrite();
}

void PhotoRigWriter::PrepareWrite()
{
  document_.Clear();
  tinyxml2::XMLElement* root = document_.NewElement("pcalib");
  document_.InsertEndChild(root);
}

void PhotoRigWriter::WriteCameras(const PhotoRigd& rig)
{
  for (auto camera : rig.cameras)
  {
    if (camera) WriteCamera(*camera);
  }
}

void PhotoRigWriter::WriteCamera(const PhotoCamerad& camera)
{
  tinyxml2::XMLElement* root = document_.FirstChildElement("pcalib");
  tinyxml2::XMLElement* element = document_.NewElement("camera");
  WriteResponses(element, camera);
  WriteVignettings(element, camera);
  root->InsertEndChild(element);
}

void PhotoRigWriter::WriteResponses(tinyxml2::XMLElement* parent,
    const PhotoCamerad& camera)
{
  for (auto response : camera.responses)
  {
    if (response) WriteResponse(parent, *response);
  }
}

void PhotoRigWriter::WriteVignettings(tinyxml2::XMLElement* parent,
    const PhotoCamerad& camera)
{
  for (auto vignetting : camera.vignettings)
  {
    if (vignetting) WriteVignetting(parent, *vignetting);
  }
}

void PhotoRigWriter::WriteResponse(tinyxml2::XMLElement* parent,
    const Response<double>& response)
{
  tinyxml2::XMLElement* element = document_.NewElement("response");

  // set response type
  element->SetAttribute("type", response.Type().c_str());

  // set response range
  tinyxml2::XMLElement* range = document_.NewElement("range");
  range->SetText(GetText(response.GetRange()).c_str());
  element->InsertEndChild(range);

  if (response.GetParams().size() > 0)
  {
    // set response params
    tinyxml2::XMLElement* params = document_.NewElement("params");
    params->SetText(GetText(response.GetParams()).c_str());
    element->InsertEndChild(params);
  }

  parent->InsertEndChild(element);
}

void PhotoRigWriter::WriteVignetting(tinyxml2::XMLElement* parent,
    const Vignetting<double>& vignetting)
{
  tinyxml2::XMLElement* element = document_.NewElement("vignetting");

  // set vignetting type
  element->SetAttribute("type", vignetting.Type().c_str());

  // set vignetting range
  Eigen::Vector2d dims;
  dims[0] = vignetting.Width();
  dims[1] = vignetting.Height();
  tinyxml2::XMLElement* size = document_.NewElement("size");
  size->SetText(GetText(dims).c_str());
  element->InsertEndChild(size);

  if (vignetting.GetParams().size() > 0)
  {
    // set vignetting params
    tinyxml2::XMLElement* params = document_.NewElement("params");
    params->SetText(GetText(vignetting.GetParams()).c_str());
    element->InsertEndChild(params);
  }

  parent->InsertEndChild(element);
}

void PhotoRigWriter::FinishWrite()
{
  document_.SaveFile(filename_.c_str());
}

std::string PhotoRigWriter::GetText(const Eigen::MatrixXd& matrix)
{
  return (matrix.size() > NUM_SHORT_PARAMS) ?
      GetLongText(matrix) : GetShortText(matrix);
}

std::string PhotoRigWriter::GetShortText(const Eigen::MatrixXd& matrix)
{
  std::stringstream text;
  text << " [ ";

  for (int row = 0; row < matrix.rows(); ++row)
  {
    for (int col = 0; col < matrix.cols(); ++col)
    {
      text << matrix(row, col);
      if (col < matrix.cols() - 1) text << ", ";
    }

    if (row < matrix.rows() - 1) text << "; ";
  }

  text << " ] ";
  return text.str();
}

std::string PhotoRigWriter::GetLongText(const Eigen::MatrixXd& matrix)
{
  Base64Encoder encoder;
  encoder << matrix;
  return encoder.str();
}

////////////////////////////////////////////////////////////////////////////////

std::shared_ptr<PhotoRigd> ReadXmlPhotoRig(const std::string& filename)
{
  std::shared_ptr<PhotoRigd> rig;
  rig = std::make_shared<PhotoRigd>();
  PhotoRigReader reader(filename);
  reader.Read(*rig);
  return rig;
}

////////////////////////////////////////////////////////////////////////////////

void WriteXmlPhotoRig(const std::string& filename, const PhotoRigd& rig)
{
  PhotoRigWriter writer(filename);
  writer.Write(rig);
}

////////////////////////////////////////////////////////////////////////////////

} // namespace calibu