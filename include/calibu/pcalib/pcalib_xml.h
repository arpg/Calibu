#pragma once

#include <memory>
#include <sstream>
#include <tinyxml2.h>
#include <calibu/Platform.h>
#include <calibu/pcalib/pcalib.h>

namespace calibu
{

class PhotoCalibReader
{
  public:

    PhotoCalibReader(const std::string& filename);

    void Read(PhotoCalibd& calib);

  protected:

    void PrepareRead();

    void ReadResponses(PhotoCalibd& calib);

    void ReadVignettings(PhotoCalibd& calib);

    std::shared_ptr<Response<double>> ReadResponse(
        const tinyxml2::XMLElement* element);

    std::shared_ptr<Vignetting<double>> ReadVignetting(
        const tinyxml2::XMLElement* element);

    void FinishRead();

    static void GetMatrix(const char* text, Eigen::MatrixXd& matrix);

    static void GetShortMatrix(const char* text, Eigen::MatrixXd& matrix);

    static void GetLongMatrix(const char* text, Eigen::MatrixXd& matrix);

  protected:

    std::string filename_;

    tinyxml2::XMLDocument document_;

    const tinyxml2::XMLElement* root_;
};

class PhotoCalibWriter
{
  public:

    PhotoCalibWriter(const std::string& filename);

    void Write(const PhotoCalibd& calib);

  protected:

    void PrepareWrite();

    void WriteResponses(const PhotoCalibd& calib);

    void WriteVignettings(const PhotoCalibd& calib);

    void WriteResponse(const Response<double>& response);

    void WriteVignetting(const Vignetting<double>& vignetting);

    void FinishWrite();

    static std::string GetText(const Eigen::MatrixXd& matrix);

    static std::string GetShortText(const Eigen::MatrixXd& matrix);

    static std::string GetLongText(const Eigen::MatrixXd& matrix);

  protected:

    std::string filename_;

    tinyxml2::XMLDocument document_;
};

CALIBU_EXPORT
std::shared_ptr<PhotoCalibd> ReadXmlPhotoCalib(const std::string& filename);

CALIBU_EXPORT
void WriteXmlPhotoCalib(const std::string& filename, const PhotoCalibd& calib);

} // namespace calibu