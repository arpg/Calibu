#include <gtest/gtest.h>
#include <calibu/pcalib/pcalib_xml.h>
#include <calibu/pcalib/response_poly.h>
#include <calibu/pcalib/response_linear.h>
#include <calibu/pcalib/vignetting_dense.h>
#include <calibu/pcalib/vignetting_poly.h>
#include <calibu/pcalib/vignetting_uniform.h>

namespace calibu
{
namespace testing
{

TEST(PhotoCalibWriter, Write)
{
  PhotoCalibd found;
  PhotoCalibd expected;

  {
    std::shared_ptr<Poly3Response<double>> response;
    response = std::make_shared<Poly3Response<double>>();
    response->SetParams(Eigen::Vector3d(+0.1, -0.2, +0.3));
    response->SetRange(0, 255);
    expected.responses.push_back(response);
  }

  {
    std::shared_ptr<Poly4Response<double>> response;
    response = std::make_shared<Poly4Response<double>>();
    response->SetParams(Eigen::Vector4d(0.0, +0.1, -0.2, +0.3));
    response->SetRange(0, 65535);
    expected.responses.push_back(response);
  }

  {
    std::shared_ptr<LinearResponse<double>> response;
    response = std::make_shared<LinearResponse<double>>();
    response->SetRange(0, 1);
    expected.responses.push_back(response);
  }

  {
    std::shared_ptr<UniformVignetting<double>> vignetting;
    vignetting = std::make_shared<UniformVignetting<double>>(640, 480);
    expected.vignettings.push_back(vignetting);
  }

  {
    std::shared_ptr<EvenPoly6Vignetting<double>> vignetting;
    vignetting = std::make_shared<EvenPoly6Vignetting<double>>(640, 480);
    vignetting->SetParams(Eigen::Vector3d(+0.1, -0.2, +0.3));
    expected.vignettings.push_back(vignetting);
  }

  {
    Eigen::VectorXd params(640 * 480);

    for (int i = 0; i < params.size(); ++i)
    {
      params[i] = 0.01 * i;
    }

    std::shared_ptr<DenseVignetting<double>> vignetting;
    vignetting = std::make_shared<DenseVignetting<double>>(640, 480);
    vignetting->SetParams(params);
    expected.vignettings.push_back(vignetting);
  }

  const std::string filename = "test_pcalib.xml";
  PhotoCalibWriter writer(filename);
  writer.Write(expected);

  PhotoCalibReader reader(filename);
  reader.Read(found);

  std::remove(filename.c_str());

  ASSERT_EQ(expected.responses.size(), found.responses.size());

  for (size_t i = 0; i < expected.responses.size(); ++i)
  {
    std::shared_ptr<const Response<double>> e = expected.responses[i];
    std::shared_ptr<const Response<double>> f = found.responses[i];

    ASSERT_NEAR(e->GetRange()[0], f->GetRange()[0], 1E-10);
    ASSERT_NEAR(f->GetRange()[1], f->GetRange()[1], 1E-10);

    const Eigen::VectorXd& eparams = e->GetParams();
    const Eigen::VectorXd& fparams = f->GetParams();

    ASSERT_EQ(eparams.size(), fparams.size());

    for (int j = 0; j < eparams.size(); ++j)
    {
      ASSERT_NEAR(eparams[j], fparams[j], 1E-10);
    }
  }
}

} // namespace testing

} // namespace calibu