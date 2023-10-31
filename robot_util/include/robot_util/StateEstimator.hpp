//
// Created by hs on 22. 10. 14.
//

#ifndef RAISIM_STATEESTIMATOR_HPP
#define RAISIM_STATEESTIMATOR_HPP

#include <raisim/World.hpp>

#include "RobotDescription.hpp"
#include "SharedMemory.hpp"
#include "RobotMath.hpp"
#include "Filter.hpp"

#include <rbdl/rbdl.h>
#include <rbdl/rbdl_utils.h>
#include <rbdl/addons/urdfreader/urdfreader.h>

class StateEstimator{
public:
    StateEstimator(RigidBodyDynamics::Model* model);

    void StateEstimatorFunction();
private:
    void getContactState();
    void getResidual(double* residual);
    void getRobotLinearState();
    void getRobotFootPosition();
    void updateRBDL();
    void updateState();
    void doEKF();
    void showOutputs();


private:
    double testBias[3];
    int testIter;

    const double mDt;
    bool mbContactState[4];

    Vec3<double> mAcceleration;
    Vec4<double> mQuaternion;
    Mat4<double> mTransMat[4];

    Eigen::Matrix<double, 18, 18> mGain;

    Eigen::VectorXd mBeta;
    Eigen::VectorXd mCalcTorque;
    Eigen::VectorXd mMomentum;
    Eigen::VectorXd mPrevQ;
    Eigen::VectorXd mPrevQd;
    Eigen::VectorXd mQ;
    Eigen::VectorXd mQd;
    Eigen::VectorXd mResidual;
    Eigen::VectorXd mTau;

    raisim::ArticulatedSystem* mRobot;
    raisim::VecDyn mPosition;
    raisim::VecDyn mVelocity;

    RigidBodyDynamics::Math::MatrixNd mH;
    RigidBodyDynamics::Math::MatrixNd mPrevH;
    RigidBodyDynamics::Model* mModel;

    ///for EKF
    Eigen::VectorXd mEstState; /// the output of EKF
    Eigen::VectorXd mMeasurements;
    Eigen::VectorXd mState;
    Eigen::VectorXd tempIMU;
    Eigen::VectorXd tempPrevIMU;
    Eigen::VectorXd tempPrevV;
    Eigen::Matrix<double, 12,12> mPredictMat; /// x(t) = A * x(t) + B * u(t)
    Eigen::Matrix<double, 12,3> mInputMat;
    Eigen::Matrix<double, 6,12> mMeasureMat;
    Eigen::Matrix<double, 12,12> mNoiseQ;
    Eigen::Matrix<double, 6,6> mNoiseR;
    Eigen::Matrix<double, 12,12> mErrorCov;
    Eigen::Matrix<double, 12,6> mKalmanGain;
    bool mbIsFirstFiltering;
    bool mbIsFirstCon[4] ;
};

#endif //RAISIM_STATEESTIMATOR_HPP
