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

#pragma once

#include <thread>
#include <mutex>
#include <memory>
#include <system_error>

#include <sophus/se3.hpp>

#include <calibu/cam/CameraModel.h>
#include <calibu/cam/CameraXml.h>
#include <calibu/calib/CostFunctionAndParams.h>

#include <ceres/ceres.h>
#include <calibu/calib/CeresExtra.h>
#include <calibu/calib/LocalParamSe3.h>

#include <calibu/calib/ReprojectionCostFunctor.h>
#include <calibu/calib/CostFunctionAndParams.h>

#include <Eigen/Sparse>

namespace calibu {

template<typename T, typename ...Args>
std::unique_ptr<T> make_unique( Args&& ...args )
{
    return std::unique_ptr<T>( new T( std::forward<Args>(args)... ) );
}

template<typename ProjModel>
struct CameraAndPose
{
    CameraAndPose( const CameraModelT<ProjModel>& camera, const Sophus::SE3d& T_ck)
        : camera(camera), T_ck(T_ck)
    {
    }
    
    CameraModelT<ProjModel> camera;
    Sophus::SE3d T_ck;
};

// https://svn.blender.org/svnroot/bf-blender/branches/soc-2011-tomato/extern/libmv/libmv/simple_pipeline/bundle.cc
Eigen::MatrixXd ToEigenMatrix(const ceres::CRSMatrix &crs_matrix) {
    Eigen::MatrixXd eigen_matrix(crs_matrix.num_rows, crs_matrix.num_cols);
    eigen_matrix.setZero();
    
    for (int row = 0; row < crs_matrix.num_rows; ++row) {
        const int start = crs_matrix.rows[row];
        const int end = crs_matrix.rows[row + 1];
        
        for (int i = start; i < end; i++) {
            const int col = crs_matrix.cols[i];
            const double value = crs_matrix.values[i];
            eigen_matrix(row, col) = value;
        }
    }
    
    return eigen_matrix;
}

// https://svn.blender.org/svnroot/bf-blender/branches/soc-2011-tomato/extern/libmv/libmv/simple_pipeline/bundle.cc
Eigen::SparseMatrix<double,Eigen::RowMajor> ToSparseEigenMatrix(const ceres::CRSMatrix &crs_matrix) {
    Eigen::SparseMatrix<double,Eigen::RowMajor> eigen_matrix(crs_matrix.num_rows, crs_matrix.num_cols);

    for (int row = 0; row < crs_matrix.num_rows; ++row) {
        const int row_start = crs_matrix.rows[row];
        const int row_end = crs_matrix.rows[row + 1];
        
        eigen_matrix.startVec(row);
        for (int i = row_start; i < row_end; i++) {
            const int col = crs_matrix.cols[i];
            const double value = crs_matrix.values[i];
            eigen_matrix.insertBack(row,col) = value;
        }
    }

    return eigen_matrix;
}

// https://svn.blender.org/svnroot/bf-blender/branches/soc-2011-tomato/extern/libmv/libmv/simple_pipeline/keyframe_selection.cc
// Based on code from http://eigen.tuxfamily.org/index.php?title=FAQ
Eigen::MatrixXd pseudoInverse(const Eigen::MatrixXd &matrix) {
  typedef Eigen::JacobiSVD<Eigen::MatrixXd> JacobiSVD;
  typedef JacobiSVD::SingularValuesType SingularValuesType;

  JacobiSVD jacobiSvd(matrix, Eigen::ComputeThinU | Eigen::ComputeThinV);
  const SingularValuesType singularValues = jacobiSvd.singularValues();
  SingularValuesType singularValues_inv = singularValues;

  double epsilon = std::numeric_limits<double>::epsilon();

  for (int i = 0; i < singularValues.rows(); i++) {
    if (singularValues(i) > epsilon)
      singularValues_inv(i) = 1.0 / singularValues(i);
    else
      singularValues_inv(i) = 0.0;
  }

  return jacobiSvd.matrixV() *
         singularValues_inv.asDiagonal() *
         jacobiSvd.matrixU().transpose();
}

template<typename ProjModel>
class Calibrator
{
public:
    
    Calibrator() :
        m_running(false),
        m_LossFunction( new ceres::SoftLOneLoss(0.5), ceres::TAKE_OWNERSHIP )
    {
        m_prob_options.cost_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
        m_prob_options.local_parameterization_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
        m_prob_options.loss_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
        
        m_solver_options.num_threads = 4;
        m_solver_options.update_state_every_iteration = true;
        m_solver_options.max_num_iterations = 10;
        
        Clear();
    }
    
    ~Calibrator()
    {
        std::cout << "------------------------------------------" << std::endl;
        for(size_t c=0; c<m_camera.size(); ++c) {
            std::cout << "Camera: " << c << std::endl;
            std::cout << m_camera[c]->camera.Params().transpose() << std::endl;
            std::cout << m_camera[c]->T_ck.matrix3x4() << std::endl << std::endl;
        }        
    }

    void WriteCameraModels()
    {
        CameraRig rig;
        
        for(size_t c=0; c<m_camera.size(); ++c) {
            rig.Add(m_camera[c]->camera, m_camera[c]->T_ck);
        }
        
        WriteXmlRig("cameras.xml", rig);
    }
    
    void Clear()
    {
        Stop();
        m_T_kw.clear();
        m_camera.clear();
        m_costs.clear();
        m_mse = 0;
    }
    
    void Start()
    {
        if(!m_running) {
            m_should_run = true;
            m_thread = std::thread(std::bind( &Calibrator::SolveThread, this )) ;
        }else{
            std::cerr << "Already Running." << std::endl;
        }        
    }
    
    void Stop()
    {
        if(m_running) {
            m_should_run = false;
            try {
                m_thread.join();
            }catch(std::system_error) {
                // thread already died.
            }
        }
    }
 
    int AddCamera(const CameraModelT<ProjModel>& cam, const Sophus::SE3d T_ck = Sophus::SE3d() )
    {
        int id = m_camera.size();
        m_camera.push_back( make_unique<CameraAndPose<ProjModel> >(cam,T_ck) );
        m_camera.back()->camera.SetIndex(id);
        return id;
    }
 
    int AddFrame(Sophus::SE3d T_kw = Sophus::SE3d())
    {
        int id = m_T_kw.size();
        m_T_kw.push_back( make_unique<Sophus::SE3d>(T_kw) );
        return id;
    }
 
    void AddObservation(
            size_t frame, size_t camera,
            const Eigen::Vector3d& P_c,
            const Eigen::Vector2d& p_c
            ) {
        m_update_mutex.lock();
 
        // Ensure index is valid
        while( NumFrames() < frame) { AddFrame(); }
        while( NumCameras() < camera ) { AddCamera(CameraModelT<ProjModel>()); }
        
        // new camera pose to bundle adjust
        
        CameraAndPose<ProjModel>& cp = *m_camera[camera];
        Sophus::SE3d& T_kw = *m_T_kw[frame];
        
        // Create cost function
//        CostFunctionAndParams* cost = new ReprojectionCost<ProjModel>(P_c, p_c);
        
        CostFunctionAndParams* cost = new CostFunctionAndParams();
        ReprojectionCostFunctor<ProjModel>* c =
                new ReprojectionCostFunctor<ProjModel>(P_c, p_c);
        cost->Cost() =  new ceres::AutoDiffCostFunction<ReprojectionCostFunctor<ProjModel>,
                2, Sophus::SE3d::num_parameters, Sophus::SE3d::num_parameters,
                ProjModel::NUM_PARAMS>(c);

        cost->Params() = std::vector<double*>{
                T_kw.data(), cp.T_ck.data(), cp.camera.data()
        };
        cost->Loss() = &m_LossFunction;
        m_costs.push_back(std::unique_ptr<CostFunctionAndParams>(cost));
        
        m_update_mutex.unlock();
    }
    
    size_t NumFrames() const
    {
        return m_T_kw.size();
    }
    
    Sophus::SE3d& GetFrame(size_t i)
    {
        return *m_T_kw[i];
    }
    
    size_t NumCameras() const
    {
        return m_camera.size();
    }
    
    CameraAndPose<ProjModel> GetCamera(size_t i)
    {
        return *m_camera[i];
    }
    
    double MeanSquareError() const
    {
        return m_mse;
    }
    
    void PrintResults()
    {
        std::cout << "----------------------------------------" << std::endl;
        
        ceres::Problem problem(m_prob_options);
        SetupProblem(problem);

        ceres::Problem::EvaluateOptions eval_options;
//        for(size_t c=0; c < m_camera.size(); ++c) {
//            eval_options.parameter_blocks.push_back(m_camera[c]->camera.data() );
//            if(c==1) eval_options.parameter_blocks.push_back(m_camera[c]->T_ck.data() );            
//        }
        
        double cost;
        ceres::CRSMatrix crs_jacobian;        
        problem.Evaluate(eval_options, &cost, nullptr, nullptr, &crs_jacobian);
        std::cout << "Evaluated jacobian" << std::endl;
        
        Eigen::SparseMatrix<double,Eigen::RowMajor> J =
                ToSparseEigenMatrix(crs_jacobian);
        std::cout << "Converted to Eigen" << std::endl;
        
        Eigen::SparseMatrix<double> JTJ = J.transpose() * J;
        std::cout << "Computed JTJ " << JTJ.rows() << "x" << JTJ.cols() << std::endl;
        
        
//        Eigen::SimplicialCholesky<Eigen::SparseMatrix<double> > chol(JTJ);
//        Eigen::MatrixXd invJTJ = chol.solve(Eigen::MatrixXd::Identity(JTJ.rows(), JTJ.cols()));
                
        Eigen::MatrixXd dJTJ = JTJ;
        Eigen::MatrixXd invJTJ = pseudoInverse(dJTJ);
        std::cout << "Inverted" << std::endl;
        
        std::cout << cost << std::endl << std::endl;
        std::cout << invJTJ.squaredNorm() << std::endl;
    }
    
protected:
    
    void SetupProblem(ceres::Problem& problem)
    {
        m_update_mutex.lock();
        
        // Add parameters
        for(size_t c=0; c<m_camera.size(); ++c) {
            problem.AddParameterBlock(m_camera[c]->T_ck.data(), 7, &m_LocalParamSe3 );
            if(c==0) {
                problem.SetParameterBlockConstant(m_camera[c]->T_ck.data());
            }
        }
        for(size_t p=0; p<m_T_kw.size(); ++p) {
            problem.AddParameterBlock(m_T_kw[p]->data(), 7, &m_LocalParamSe3 );
        }
        
        // Add costs
        for(size_t c=0; c<m_costs.size(); ++c) {
            CostFunctionAndParams& cost = *m_costs[c];
            problem.AddResidualBlock(cost.Cost(), cost.Loss(), cost.Params());
        }
        
        m_update_mutex.unlock();        
    }
    
    void SolveThread()
    {
        m_running = true;
        while( m_should_run ){
            ceres::Problem problem(m_prob_options);
            SetupProblem(problem);            
            
            // Crank optimisation
            if(problem.NumResiduals() > 0) {
                try {
                    ceres::Solver::Summary summary;
                    ceres::Solve(m_solver_options, &problem, &summary);
                    std::cout << summary.BriefReport() << std::endl;     
                    m_mse = summary.final_cost / summary.num_residuals;
                    std::cout << "Frames: " << m_T_kw.size() << "; Observations: " << summary.num_residuals << "; mse: " << m_mse << std::endl;
                }catch(std::exception e) {
                    std::cerr << e.what() << std::endl;
                }
            }
        }
        m_running = false;
    }
 
    std::mutex m_update_mutex;
    std::thread m_thread;
    bool m_should_run;
    bool m_running;
 
    std::vector< std::unique_ptr<Sophus::SE3d> > m_T_kw;
    std::vector< std::unique_ptr<CameraAndPose<ProjModel> > > m_camera;
    std::vector< std::unique_ptr<CostFunctionAndParams > > m_costs;
 
    ceres::Problem::Options m_prob_options;
    ceres::Solver::Options  m_solver_options;
    ceres::LossFunctionWrapper m_LossFunction;
    LocalParameterizationSe3  m_LocalParamSe3; 

    double m_mse;
};

}
