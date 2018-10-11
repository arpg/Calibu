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

TEST(PhotoRigXml, ReadWrite)
{
  PhotoRigd found;
  PhotoRigd expected;

  for (int i = 0; i < 3; ++i)
  {
    std::shared_ptr<PhotoCamerad> camera;
    camera = std::make_shared<PhotoCamerad>();
    expected.cameras.push_back(camera);

    {
      std::shared_ptr<Poly3Response<double>> response;
      response = std::make_shared<Poly3Response<double>>();
      response->SetParams(Eigen::Vector3d(+0.1, -0.2, +0.3));
      response->SetRange(0, 255);
      camera->responses.push_back(response);
    }

    {
      std::shared_ptr<Poly4Response<double>> response;
      response = std::make_shared<Poly4Response<double>>();
      response->SetParams(Eigen::Vector4d(0.0, +0.1, -0.2, +0.3));
      response->SetRange(0, 65535);
      camera->responses.push_back(response);
    }

    {
      std::shared_ptr<LinearResponse<double>> response;
      response = std::make_shared<LinearResponse<double>>();
      response->SetRange(0, 1);
      camera->responses.push_back(response);
    }

    {
      std::shared_ptr<UniformVignetting<double>> vignetting;
      vignetting = std::make_shared<UniformVignetting<double>>(640, 480);
      camera->vignettings.push_back(vignetting);
    }

    {
      std::shared_ptr<EvenPoly6Vignetting<double>> vignetting;
      vignetting = std::make_shared<EvenPoly6Vignetting<double>>(640, 480);
      vignetting->SetParams(Eigen::Vector3d(+0.1, -0.2, +0.3));
      camera->vignettings.push_back(vignetting);
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
      camera->vignettings.push_back(vignetting);
    }
  }

  const std::string filename = "test_pcalib.xml";
  PhotoRigWriter writer(filename);
  writer.Write(expected);

  PhotoRigReader reader(filename);
  reader.Read(found);

  // std::remove(filename.c_str());

  ASSERT_EQ(expected.cameras.size(), found.cameras.size());

  for (size_t j = 0; j < expected.cameras.size(); ++j)
  {
    std::shared_ptr<const PhotoCamerad> fcamera = found.cameras[j];
    std::shared_ptr<const PhotoCamerad> ecamera = expected.cameras[j];

    ASSERT_EQ(ecamera->responses.size(), fcamera->responses.size());

    for (size_t i = 0; i < ecamera->responses.size(); ++i)
    {
      std::shared_ptr<const Response<double>> e = ecamera->responses[i];
      std::shared_ptr<const Response<double>> f = fcamera->responses[i];

      ASSERT_EQ(e->Type(), f->Type());
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

    ASSERT_EQ(ecamera->vignettings.size(), fcamera->vignettings.size());

    for (size_t i = 0; i < ecamera->vignettings.size(); ++i)
    {
      std::shared_ptr<const Vignetting<double>> e = ecamera->vignettings[i];
      std::shared_ptr<const Vignetting<double>> f = fcamera->vignettings[i];

      ASSERT_EQ(e->Type(), f->Type());
      ASSERT_EQ(e->Width(), f->Width());
      ASSERT_EQ(e->Height(), f->Height());

      const Eigen::VectorXd& eparams = e->GetParams();
      const Eigen::VectorXd& fparams = f->GetParams();

      ASSERT_EQ(eparams.size(), fparams.size());

      for (int j = 0; j < eparams.size(); ++j)
      {
        ASSERT_NEAR(eparams[j], fparams[j], 1E-10);
      }
    }
  }
}

} // namespace testing

} // namespace calibu