//
// Created by hs on 22. 10. 27.
//

#ifndef RAISIM_SIMULSTATEESTIMATOR_HPP
#define RAISIM_SIMULSTATEESTIMATOR_HPP

#include <raisim/World.hpp>

#include <robot_util/SharedMemory.hpp>
#include <robot_util/RobotMath.hpp>
#include <robot_util/Filter.hpp>

#include <rbdl/rbdl.h>
#include <rbdl/rbdl_utils.h>
#include <rbdl/addons/urdfreader/urdfreader.h>


class SimulStateEstimator{
public:
    SimulStateEstimator(raisim::ArticulatedSystem* robot, RigidBodyDynamics::Model* model);

    void StateEstimatorFunction();
private:
    void getContactState();
    void getJointState();
    void getRobotAngulerState();
    void getRobotLinearState();
    void getRobotFootPosition();
    void updateState();
    void updateRBDL();
    void setOffset();
    void doEKF();
    void doLegKinematics();

    void tempImuUpdate();
    void showOutputs();


private:
    const double mDt;
    bool mbContactState[4] ;

    double mPrevBasePos[3];

    Eigen::Matrix<double, 23, 23> mGain;

    Eigen::VectorXd mBeta;
    Eigen::VectorXd mCalcTorque;
    Eigen::VectorXd mMomentum;
    Eigen::VectorXd mPrevQ;
    Eigen::VectorXd mPrevQd;
    Eigen::VectorXd mQ;
    Eigen::VectorXd mQd;
    Eigen::VectorXd mQdd;
    Eigen::VectorXd mResidual;
    Eigen::VectorXd mTau;
    Eigen::VectorXd mGeneralizedForce;

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

    double testZ;
    double testY;
    double testX;

    double tempV[3];
    int mMulti;
    double mRawEulerAngle[3];

    ///

    Mat4<double> mTransMat[4];
    Vec3<double> mOffset[4];

    raisim::ArticulatedSystem* mRobot;
    raisim::VecDyn mPosition;
    raisim::VecDyn mVelocity;

    RigidBodyDynamics::Math::MatrixNd mH;
    RigidBodyDynamics::Math::MatrixNd mPrevH;
    RigidBodyDynamics::Model* mModel;

    /// leg kinematics
    Vec3<double> mPos;
    Vec3<double> mPrevPos;
    Vec3<double> mGlobalFootPos[4];
    bool mbIsFirstRun;

};

#endif //RAISIM_SIMULSTATEESTIMATOR_HPP
