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

#define NUM_SHORT_PARAMS 10

namespace calibu
{

////////////////////////////////////////////////////////////////////////////////

PhotoCalibReader::PhotoCalibReader(const std::string& filename) :
  filename_(filename),
  root_(nullptr)
{
}

void PhotoCalibReader::Read(PhotoCalibd& calib)
{
  calib.Clear();
  PrepareRead();

  if (root_)
  {
    ReadResponses(calib);
    ReadVignettings(calib);
  }

  FinishRead();
}

void PhotoCalibReader::PrepareRead()
{
  document_.LoadFile(filename_.c_str());
  root_ = document_.FirstChildElement("pcalib");
  if (!root_) std::cerr << "missing 'pcalib' element" << std::endl;
}

void PhotoCalibReader::ReadResponses(PhotoCalibd& calib)
{
  const tinyxml2::XMLElement* child = root_->FirstChildElement("response");

  while (child)
  {
    std::shared_ptr<Response<double>> response = ReadResponse(child);
    if (response) calib.responses.push_back(response);
    child = child->NextSiblingElement("response");
  }
}

void PhotoCalibReader::ReadVignettings(PhotoCalibd& calib)
{
  const tinyxml2::XMLElement* child = root_->FirstChildElement("vignetting");

  while (child)
  {
    std::shared_ptr<Vignetting<double>> vignetting = ReadVignetting(child);
    if (vignetting) calib.vignettings.push_back(vignetting);
    child = child->NextSiblingElement("vignetting");
  }
}

std::shared_ptr<Response<double> > PhotoCalibReader::ReadResponse(
    const tinyxml2::XMLElement* element)
{
  using namespace tinyxml2;
  std::shared_ptr<Response<double>> response;
  const std::string type(element->Attribute("type"));

  if (type.compare(Poly3Response<double>::type) == 0)
  {
    response = std::make_shared<Poly3Response<double>>();
  }
  else if (type.compare(Poly4Response<double>::type) == 0)
  {
    response = std::make_shared<Poly4Response<double>>();
  }
  else if (type.compare(LinearResponse<double>::type) == 0)
  {
    response = std::make_shared<LinearResponse<double>>();
  }
  else
  {
    std::cerr << "unknown response type '" << type << "'" << std::endl;
  }

  if (response)
  {
    // read range
    Eigen::MatrixXd range;
    range = response->GetRange();

    const XMLElement* range_elem = element->FirstChildElement("range");
    if (range_elem) GetMatrix(range_elem->GetText(), range);
    response->SetRange(range);

    // read params
    Eigen::MatrixXd params;
    params = response->GetParams();

    const XMLElement* params_elem = element->FirstChildElement("params");
    if (params_elem) GetMatrix(params_elem->GetText(), params);
    response->SetParams(params);
  }

  return response;
}

std::shared_ptr<Vignetting<double>> PhotoCalibReader::ReadVignetting(
    const tinyxml2::XMLElement* element)
{
  using namespace tinyxml2;
  std::shared_ptr<Vignetting<double>> vignetting;
  const std::string type(element->Attribute("type"));

  Eigen::MatrixXd size = Eigen::Vector2d(0, 0);
  const XMLElement* size_elem = element->FirstChildElement("size");
  if (size_elem) GetMatrix(size_elem->GetText(), size);
  const int w = size(0, 0);
  const int h = size(1, 0);

  if (type.compare(DenseVignetting<double>::type) == 0)
  {
    vignetting = std::make_shared<DenseVignetting<double>>(w, h);
  }
  else if (type.compare(EvenPoly6Vignetting<double>::type) == 0)
  {
    vignetting = std::make_shared<EvenPoly6Vignetting<double>>(w, h);
  }
  else if (type.compare(UniformVignetting<double>::type) == 0)
  {
    vignetting = std::make_shared<UniformVignetting<double>>(w, h);
  }
  else
  {
    std::cerr << "unknown vignetting type '" << type << "'" << std::endl;
  }

  if (vignetting)
  {
    Eigen::MatrixXd params;
    params = vignetting->GetParams();

    const XMLElement* params_elem = element->FirstChildElement("params");
    if (params_elem) GetMatrix(params_elem->GetText(), params);
    vignetting->SetParams(params);
  }

  return vignetting;
}

void PhotoCalibReader::FinishRead()
{
  document_.Clear();
  root_ = nullptr;
}

void PhotoCalibReader::GetMatrix(const char* text, Eigen::MatrixXd& matrix)
{
  (matrix.size() > NUM_SHORT_PARAMS) ?
      GetLongMatrix(text, matrix) : GetShortMatrix(text, matrix);
}

void PhotoCalibReader::GetShortMatrix(const char* text, Eigen::MatrixXd& matrix)
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

void PhotoCalibReader::GetLongMatrix(const char* text, Eigen::MatrixXd& matrix)
{
  Base64Decoder decoder(text);
  decoder >> matrix;
}

////////////////////////////////////////////////////////////////////////////////

PhotoCalibWriter::PhotoCalibWriter(const std::string& filename) :
  filename_(filename)
{
}

void PhotoCalibWriter::Write(const PhotoCalibd& calib)
{
  PrepareWrite();
  WriteResponses(calib);
  WriteVignettings(calib);
  FinishWrite();
}

void PhotoCalibWriter::PrepareWrite()
{
  document_.Clear();
  tinyxml2::XMLElement* root = document_.NewElement("pcalib");
  document_.InsertEndChild(root);
}

void PhotoCalibWriter::WriteResponses(const PhotoCalibd& calib)
{
  for (auto response : calib.responses)
  {
    if (response) WriteResponse(*response);
  }
}

void PhotoCalibWriter::WriteVignettings(const PhotoCalibd& calib)
{
  for (auto vignetting : calib.vignettings)
  {
    if (vignetting) WriteVignetting(*vignetting);
  }
}

void PhotoCalibWriter::WriteResponse(const Response<double>& response)
{
  using namespace tinyxml2;
  XMLElement* root = document_.FirstChildElement("pcalib");
  XMLElement* element = document_.NewElement("response");

  // set response type
  element->SetAttribute("type", response.Type().c_str());

  // set response range
  XMLElement* range = document_.NewElement("range");
  range->SetText(GetText(response.GetRange()).c_str());
  element->InsertEndChild(range);

  if (response.GetParams().size() > 0)
  {
    // set response params
    XMLElement* params = document_.NewElement("params");
    params->SetText(GetText(response.GetParams()).c_str());
    element->InsertEndChild(params);
  }

  root->InsertEndChild(element);
}

void PhotoCalibWriter::WriteVignetting(const Vignetting<double>& vignetting)
{
  using namespace tinyxml2;
  XMLElement* root = document_.FirstChildElement("pcalib");
  XMLElement* element = document_.NewElement("vignetting");

  // set vignetting type
  element->SetAttribute("type", vignetting.Type().c_str());

  // set vignetting range
  Eigen::Vector2d dims;
  dims[0] = vignetting.Width();
  dims[1] = vignetting.Height();
  XMLElement* size = document_.NewElement("size");
  size->SetText(GetText(dims).c_str());
  element->InsertEndChild(size);

  if (vignetting.GetParams().size() > 0)
  {
    // set vignetting params
    XMLElement* params = document_.NewElement("params");
    params->SetText(GetText(vignetting.GetParams()).c_str());
    element->InsertEndChild(params);
  }

  root->InsertEndChild(element);
}

void PhotoCalibWriter::FinishWrite()
{
  document_.SaveFile(filename_.c_str());
}

std::string PhotoCalibWriter::GetText(const Eigen::MatrixXd& matrix)
{
  return (matrix.size() > NUM_SHORT_PARAMS) ?
      GetLongText(matrix) : GetShortText(matrix);
}

std::string PhotoCalibWriter::GetShortText(const Eigen::MatrixXd& matrix)
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

std::string PhotoCalibWriter::GetLongText(const Eigen::MatrixXd& matrix)
{
  Base64Encoder encoder;
  encoder << matrix;
  return encoder.str();
}

////////////////////////////////////////////////////////////////////////////////

std::shared_ptr<PhotoCalibd> ReadXmlPhotoCalib(const std::string& filename)
{
  std::shared_ptr<PhotoCalibd> calib;
  calib = std::make_shared<PhotoCalibd>();
  PhotoCalibReader reader(filename);
  reader.Read(*calib);
  return calib;
}

void WriteXmlPhotoCalib(const std::string& filename, const PhotoCalibd& calib)
{
  PhotoCalibWriter writer(filename);
  writer.Write(calib);
}

////////////////////////////////////////////////////////////////////////////////

} // namespace calibu